//! examples/smallest.rs

#![no_main]
#![no_std]

mod protocol;

use panic_semihosting as _; // panic handler

// TODO: add in GPIO buttons
//     let pin_switch = gpiob.pb1.into_pull_up_input(&mut gpiob.crl);

// from: https://github.com/kalkyl/f103-rtic/blob/main/src/bin/serial.rs
mod app {
    #[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [EXTI1])]
    mod app {
        use stm32f1xx_hal::{
            adc,
            adc::Continuous,
            dma::{
                dma1::{C1, C2, C3},
                Event, RxDma, Transfer, TxDma, R, W,
            },
            gpio::{gpioa, gpiob, Alternate, Analog, OpenDrain},
            i2c,
            pac::{I2C1, USART3},
            prelude::*,
            serial::{Config, Rx, Serial, Tx},
        };

        use adafruit_7segment::{Index, SevenSegment};
        use cortex_m::asm;
        use ht16k33::{Dimming, Display, HT16K33};

        use cortex_m_semihosting::hprintln;
        use stm32f1xx_hal::adc::{AdcPayload, SampleTime};

        use crate::protocol::*;

        // Should be a multiple of DATA_FRAME_SIZE to ensure Tx is always a whole number of frames
        const BUF_SIZE: usize = DATA_FRAME_SIZE * 5;

        // Should be large enough to allow average (de-noising) and reduce interrupt frequency,
        // while also small enough not to negatively impact response.
        // Empirically, around 32 feels response enough without any noticeable noise.
        const ADC_BUF_SIZE: usize = 32;

        // The total range is 127-65.5 = 62.5 and each 0.5 cm is a step so 62.5 * 2 = 125 steps.
        // The ADC is 12-bits (i.e. 2^12) but we only really need 125 (i.e. a bit under 2^7),
        // therefore the max threshold can be a bit over 2^(12-7) = 2^5 = 32.
        // Set the threshold to 16 to provide good noise reduction
        // while also not losing too much signal from small (i.e. slow) movements.
        // We observe that lower than around 16 tends to result in noticeable noise,
        // and more than around 16 tends to result in noticeable loss of precision,
        // especially at the max value.
        const ADC_SMOOTH_THRESHOLD: u16 = 16;

        // 12-bit ADC i.e. 2^12
        // However, due to intrinsic resistance, we can't ever get to the max value.
        // We empirically observe a typical max value is about 4090.
        // We also want to set this lower to account for de-noising (averaging)
        const ADC_MAX_VALUE: u16 = 4070;

        pub enum TxTransfer {
            Running(Transfer<R, &'static mut [u8; BUF_SIZE], TxDma<Tx<USART3>, C2>>),
            Idle(&'static mut [u8; BUF_SIZE], TxDma<Tx<USART3>, C2>),
        }

        #[shared]
        struct Shared {
            #[lock_free]
            send: Option<TxTransfer>,

            #[lock_free]
            disp: HT16K33<
                i2c::BlockingI2c<
                    I2C1,
                    (
                        gpiob::PB6<Alternate<OpenDrain>>,
                        gpiob::PB7<Alternate<OpenDrain>>,
                    ),
                >,
            >,
        }

        #[local]
        struct Local {
            recv: Option<Transfer<W, &'static mut [u8; BUF_SIZE], RxDma<Rx<USART3>, C3>>>,

            adc_recv: Option<
                Transfer<
                    W,
                    &'static mut [u16; ADC_BUF_SIZE],
                    RxDma<AdcPayload<gpioa::PA0<Analog>, Continuous>, C1>,
                >,
            >,

            current_adc_pos: u16,
        }

        #[init(local = [rx_buf: [u8; BUF_SIZE] = [0; BUF_SIZE], tx_buf: [u8; BUF_SIZE] = [0; BUF_SIZE], adc_buf: [u16; ADC_BUF_SIZE] = [0; ADC_BUF_SIZE]])]
        fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
            let mut rcc = ctx.device.RCC.constrain();
            let mut flash = ctx.device.FLASH.constrain();

            let clocks = rcc
                .cfgr
                .use_hse(8.mhz())
                .sysclk(72.mhz())
                .pclk1(36.mhz())
                .freeze(&mut flash.acr);

            let mut afio = ctx.device.AFIO.constrain(&mut rcc.apb2);
            let mut gpioa = ctx.device.GPIOA.split(&mut rcc.apb2);
            let mut gpiob = ctx.device.GPIOB.split(&mut rcc.apb2);

            let mut dma_channels = ctx.device.DMA1.split(&mut rcc.ahb);
            dma_channels.1.listen(Event::TransferComplete);

            // Setup ADC on pin PA0 (potentiometer input)
            let mut adc1 = adc::Adc::adc1(ctx.device.ADC1, &mut rcc.apb2, clocks);
            adc1.set_sample_time(SampleTime::T_239); // Slow down the ADC as much as possible
            let mut adc_ch0 = gpioa.pa0.into_analog(&mut gpioa.crl);
            let adc_dma = adc1.with_dma(adc_ch0, dma_channels.1);

            // Set up the I2C bus on pins PB6 and PB7 (display)
            let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
            let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
            let mode = i2c::Mode::Standard {
                frequency: 100_000.hz(),
            };

            let start_timeout_us: u32 = 10000;
            let start_retries: u8 = 5;
            let addr_timeout_us: u32 = 10000;
            let data_timeout_us: u32 = 10000;

            let i2c = i2c::BlockingI2c::i2c1(
                ctx.device.I2C1,
                (scl, sda),
                &mut afio.mapr,
                mode,
                clocks,
                &mut rcc.apb1,
                start_timeout_us,
                start_retries,
                addr_timeout_us,
                data_timeout_us,
            );

            const DISP_I2C_ADDR: u8 = 112;

            let mut ht16k33 = HT16K33::new(i2c, DISP_I2C_ADDR);

            ht16k33.initialize().expect("Failed to initialize ht16k33");

            ht16k33
                .set_display(Display::ON)
                .expect("Could not turn on the display!");

            ht16k33
                .set_dimming(Dimming::BRIGHTNESS_MAX)
                .expect("Could not set dimming!");

            let tx = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
            let rx = gpiob.pb11;
            let serial = Serial::usart3(
                ctx.device.USART3,
                (tx, rx),
                &mut afio.mapr,
                Config::default().baudrate(9_600.bps()),
                clocks,
                &mut rcc.apb1,
            );
            dma_channels.2.listen(Event::TransferComplete);
            dma_channels.3.listen(Event::TransferComplete);
            let (tx_serial, rx_serial) = serial.split();
            let tx = tx_serial.with_dma(dma_channels.2);
            let rx = rx_serial.with_dma(dma_channels.3);

            (
                Shared {
                    send: Some(TxTransfer::Idle(ctx.local.tx_buf, tx)),
                    disp: ht16k33,
                },
                Local {
                    recv: Some(rx.read(ctx.local.rx_buf)),
                    adc_recv: Some(adc_dma.read(ctx.local.adc_buf)),
                    current_adc_pos: 0,
                },
                init::Monotonics(),
            )
        }

        #[idle]
        fn idle(_: idle::Context) -> ! {
            loop {
                // hprintln!("i").unwrap();
                asm::wfi();
            }
        }

        // Triggers on RX transfer completed
        #[task(binds = DMA1_CHANNEL3, local = [recv], priority = 2)]
        fn on_rx(ctx: on_rx::Context) {
            let (rx_buf, rx) = ctx.local.recv.take().unwrap().wait();
            read_height::spawn(*rx_buf).ok();
            ctx.local.recv.replace(rx.read(rx_buf));
        }

        #[task(shared = [disp], priority = 1, capacity = 8)]
        fn read_height(ctx: read_height::Context, data: [u8; BUF_SIZE]) {
            // hprintln!("{:?}", data[0]).unwrap();
            let mut height = 0.;
            let frame = find_first_frame(&data);
            if validate_frame(&frame) {
                match DeskToPanelMessage::from_frame(&frame) {
                    DeskToPanelMessage::Height(x) => {
                        height = x;
                    }
                    DeskToPanelMessage::Unknown(a, b, c, d, e) => {
                        // do nothing for now
                    }
                }
            }

            // ctx.local
            //     .disp
            //     .update_buffer_with_float(Index::One, height, 1, 10);
            // ctx.local.disp.write_display_buffer().unwrap();
            //
            // if height < 75.0 {
            //     send_message::spawn(PanelToDeskMessage::Up);
            // } else if height >= 65.0 {
            //     send_message::spawn(PanelToDeskMessage::Down);
            // } else {
            //     send_message::spawn(PanelToDeskMessage::NoKey);
            // }
        }

        fn find_first_frame(buf: &[u8; BUF_SIZE]) -> [u8; DATA_FRAME_SIZE] {
            let mut frame = [0; DATA_FRAME_SIZE];
            for (i, x) in buf.iter().enumerate() {
                if is_start_byte(*x) {
                    frame[0..DATA_FRAME_SIZE].copy_from_slice(&buf[i..i + DATA_FRAME_SIZE]);
                    return frame;
                }
            }
            return [0; DATA_FRAME_SIZE];
        }

        #[task(shared = [send], priority = 1, capacity = 1)]
        fn send_message(ctx: send_message::Context, message: PanelToDeskMessage) {
            // defmt::info!("Received {:?}", data);
            let send = ctx.shared.send;
            let (tx_buf, tx) = match send.take().unwrap() {
                TxTransfer::Idle(buf, tx) => (buf, tx),
                TxTransfer::Running(transfer) => transfer.wait(),
            };
            let b = &fill_buffer_with_message(message);
            //hprintln!("{:?}", b).unwrap();
            //asm::bkpt();
            tx_buf.copy_from_slice(&fill_buffer_with_message(message));
            send.replace(TxTransfer::Running(tx.write(tx_buf)));
        }

        fn fill_buffer_with_message(message: PanelToDeskMessage) -> [u8; BUF_SIZE] {
            let mut buf = [0; BUF_SIZE];
            for i in 0..(BUF_SIZE / DATA_FRAME_SIZE) - 1 {
                let start = i * DATA_FRAME_SIZE;
                let end = (i + 1) * DATA_FRAME_SIZE - 1;
                buf[start..end + 1].copy_from_slice(&message.as_frame());
                //hprintln!("buf[{:?}..{:?}]={:?}", start, end, &buf[start..end + 1],).unwrap();
            }
            buf
        }

        // Triggers on TX transfer completed
        #[task(binds = DMA1_CHANNEL2, shared = [send], priority = 1)]
        fn on_tx(ctx: on_tx::Context) {
            // hprintln!("s").unwrap();
            let send = ctx.shared.send;
            let (tx_buf, tx) = match send.take().unwrap() {
                TxTransfer::Idle(buf, tx) => (buf, tx),
                TxTransfer::Running(transfer) => transfer.wait(),
            };
            // defmt::info!("Sent {:?}", tx_buf);
            send.replace(TxTransfer::Idle(tx_buf, tx));
        }

        // Triggers on ADC read completed
        #[task(binds = DMA1_CHANNEL1, local = [adc_recv,current_adc_pos], shared = [disp], priority = 1)]
        fn on_adc_read(ctx: on_adc_read::Context) {
            let (rx_buf, rx) = ctx.local.adc_recv.take().unwrap().wait();

            // Cast to u32 to enable summation as u32 and avoid overrun which would often occur
            // if we tried to sum as u16.
            // The max value in the buffer is 2^12 - 1, and the max u16 value is 2^16 - 1,
            // and the buffer size is 35.
            // 35 * (2^12 - 1) > 2^16-1, hence overrun
            let avg =
                (&rx_buf.iter().map(|&x| x as u32).sum::<u32>() / (rx_buf.len() as u32)) as u16;

            let new_position = threshold_smooth_adc(avg, *ctx.local.current_adc_pos);

            let height = normalize_input_height(new_position);
            *ctx.local.current_adc_pos = new_position;

            ctx.shared
                .disp
                .update_buffer_with_float(Index::One, height, 1, 10)
                .unwrap();
            ctx.shared.disp.write_display_buffer().unwrap();

            ctx.local.adc_recv.replace(rx.read(rx_buf));
        }

        fn threshold_smooth_adc(val: u16, prev_val: u16) -> u16 {
            let h = if abs_diff(val, prev_val) > ADC_SMOOTH_THRESHOLD {
                val
            } else {
                prev_val
            };

            return h;
        }

        fn abs_diff(a: u16, b: u16) -> u16 {
            if a > b {
                return a - b;
            } else {
                return b - a;
            }
        }

        fn normalize_input_height(adc_pos: u16) -> f32 {
            let min_pos = 65.0;
            let max_pos = 127.5;
            let step_size = 0.5;

            let ratio = adc_pos as f32 / ADC_MAX_VALUE as f32;

            let mut pos: f32 = ratio * (max_pos - min_pos) + min_pos;

            pos = round_to_nearest(pos, step_size);

            return pos;
        }

        fn round_to_nearest(val: f32, round_to: f32) -> f32 {
            let rounded_multiple = (val / round_to) as u16;
            let x = rounded_multiple as f32 * round_to;
            return x;
        }
    }
}
