//! examples/smallest.rs

#![no_main]
#![no_std]

mod protocol;

use panic_semihosting as _; // panic handler

// TODO: add in GPIO buttons
//     let pin_switch = gpiob.pb1.into_pull_up_input(&mut gpiob.crl);

// from: https://github.com/kalkyl/f103-rtic/blob/main/src/bin/serial.rs
mod app {
    #[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [EXTI1,EXTI2])]
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

        // Typically can go as low as 64 bytes as long as there are no delays (e.g. hprintln! calls)
        // Simple hprintln! calls require 512 or so
        const RX_BUF_SIZE: usize = 64;
        // const RX_BUF_SIZE: usize = 256;
        // const RX_BUF_SIZE: usize = 1024;

        // Should be a multiple of DATA_FRAME_SIZE to ensure Tx is always a whole number of frames
        const TX_BUF_SIZE: usize = DATA_FRAME_SIZE * 8;
        // const TX_BUF_SIZE: usize = DATA_FRAME_SIZE * 35;
        // const TX_BUF_SIZE: usize = 256;

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

        // Set to around 4 when using hprintln statements and larger tx/rx buffers
        const TX_REPEAT_TIMES: u16 = 1;

        // TODO: move to protocol module
        // Set to a lower number when larger buffers are being used
        const NO_KEY_ITERATION_COUNT: u16 = 5;
        // const NO_KEY_ITERATION_COUNT: u16 = 4;
        // const NO_KEY_ITERATION_COUNT: u16 = 1;

        const NO_KEY_SPAWN_COUNT: u16 = 20;

        pub enum TxTransfer {
            Running(Transfer<R, &'static mut [u8; TX_BUF_SIZE], TxDma<Tx<USART3>, C2>>),
            Idle(&'static mut [u8; TX_BUF_SIZE], TxDma<Tx<USART3>, C2>),
        }

        #[derive(Clone, Copy)]
        pub enum Direction {
            Up,
            Down,
            Reversing,
        }

        #[shared]
        struct Shared {
            // TODO: ensure that this doesn't actually result in a deadlock due to the use of
            // transfer.wait() after getting the lock.
            send: Option<TxTransfer>,

            disp: HT16K33<
                i2c::BlockingI2c<
                    I2C1,
                    (
                        gpiob::PB6<Alternate<OpenDrain>>,
                        gpiob::PB7<Alternate<OpenDrain>>,
                    ),
                >,
            >,

            target_height: Option<f32>,
            current_height: f32,
        }

        #[local]
        struct Local {
            no_key_send_count: u16,
            current_direction: Option<Direction>,

            recv: Option<Transfer<W, &'static mut [u8; RX_BUF_SIZE], RxDma<Rx<USART3>, C3>>>,

            adc_recv: Option<
                Transfer<
                    W,
                    &'static mut [u16; ADC_BUF_SIZE],
                    RxDma<AdcPayload<gpioa::PA0<Analog>, Continuous>, C1>,
                >,
            >,

            current_adc_pos: u16,
        }

        #[init(local = [rx_buf: [u8; RX_BUF_SIZE] = [0; RX_BUF_SIZE], tx_buf: [u8; TX_BUF_SIZE] = [0; TX_BUF_SIZE], adc_buf: [u16; ADC_BUF_SIZE] = [0; ADC_BUF_SIZE]])]
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

            for _ in 0..NO_KEY_SPAWN_COUNT {
                send_message::spawn(PanelToDeskMessage::NoKey).unwrap();
            }

            let temp_target_height = 71.5;
            (
                Shared {
                    send: Some(TxTransfer::Idle(ctx.local.tx_buf, tx)),
                    disp: ht16k33,
                    target_height: Some(temp_target_height),
                    current_height: 0.0,
                },
                Local {
                    recv: Some(rx.read(ctx.local.rx_buf)),
                    adc_recv: Some(adc_dma.read(ctx.local.adc_buf)),
                    current_adc_pos: 0,
                    no_key_send_count: 0,
                    current_direction: None,
                },
                init::Monotonics(),
            )
        }

        #[idle]
        fn idle(_: idle::Context) -> ! {
            loop {
                asm::wfi();
            }
        }

        // Triggers on RX transfer completed
        #[task(binds = DMA1_CHANNEL3, local = [recv], priority = 3)]
        fn on_rx(ctx: on_rx::Context) {
            let (rx_buf, rx) = ctx.local.recv.take().unwrap().wait();
            read_height::spawn(*rx_buf).unwrap();
            ctx.local.recv.replace(rx.read(rx_buf));
        }

        #[task(shared = [disp, current_height], priority = 2, capacity = 2)]
        fn read_height(mut ctx: read_height::Context, data: [u8; RX_BUF_SIZE]) {
            let frame = find_first_frame(&data);
            if validate_frame(&frame) {
                match DeskToPanelMessage::from_frame(&frame) {
                    DeskToPanelMessage::Height(h) => {
                        ctx.shared.current_height.lock(|current_height| {
                            *current_height = h;
                        });

                        ctx.shared.disp.lock(|disp| {
                            disp.update_buffer_with_float(Index::One, h, 1, 10).unwrap();
                            disp.write_display_buffer().unwrap();
                        });

                        // Explicitly ignore a failed spawn attempt.
                        // We do not care if we miss an opportunity to potentially send a message
                        // as we will catch it next time we enter this function.
                        let _ = compare_height::spawn();
                    }
                    DeskToPanelMessage::Unknown(a, b, c, d, e) => {
                        // do nothing for now
                    }
                }
            }
        }

        fn find_first_frame(buf: &[u8; RX_BUF_SIZE]) -> [u8; DATA_FRAME_SIZE] {
            let mut frame = [0; DATA_FRAME_SIZE];
            for (i, x) in buf.iter().enumerate() {
                if is_start_byte(*x) {
                    frame[0..DATA_FRAME_SIZE].copy_from_slice(&buf[i..i + DATA_FRAME_SIZE]);
                    return frame;
                }
            }
            return [0; DATA_FRAME_SIZE];
        }

        #[task(local = [no_key_send_count,current_direction], shared = [current_height, target_height], priority = 1, capacity = 1)]
        fn compare_height(mut ctx: compare_height::Context) {
            let mut current_direction = *ctx.local.current_direction;
            let mut no_key_send_count = *ctx.local.no_key_send_count;
            let mut message = None;

            (ctx.shared.current_height, ctx.shared.target_height).lock(
                |current_height, ctx_target_height| match current_direction {
                    Some(Direction::Reversing) => {
                        // if no_key_send_count < NO_KEY_ITERATION_COUNT {
                        //     message = Some(PanelToDeskMessage::NoKey);
                        // } else {
                        current_direction = None;
                        // }
                    }
                    _ => match *ctx_target_height {
                        Some(target_height) => {
                            if *current_height < target_height {
                                match current_direction {
                                    Some(Direction::Down) => {
                                        current_direction = Some(Direction::Reversing);
                                        message = Some(PanelToDeskMessage::NoKey);
                                    }
                                    _ => {
                                        current_direction = Some(Direction::Up);
                                        message = Some(PanelToDeskMessage::Up);
                                    }
                                }
                            } else if *current_height > target_height {
                                match current_direction {
                                    Some(Direction::Up) => {
                                        current_direction = Some(Direction::Reversing);
                                        message = Some(PanelToDeskMessage::NoKey);
                                    }
                                    _ => {
                                        current_direction = Some(Direction::Down);
                                        message = Some(PanelToDeskMessage::Down);
                                    }
                                }
                            } else {
                                current_direction = None;
                                if no_key_send_count < NO_KEY_ITERATION_COUNT {
                                    message = Some(PanelToDeskMessage::NoKey);
                                } else {
                                    *ctx_target_height = None;
                                }
                            }
                        }
                        None => {
                            panic!("at target height - rest of code not implemented");
                        }
                    },
                },
            );

            // // TODO: remove references to this once we're happy with the multiple spawn behavior
            // no_key_send_count = NO_KEY_SEND_COUNT;

            match message {
                Some(PanelToDeskMessage::NoKey) => {
                    no_key_send_count += 1;
                }
                _ => {
                    no_key_send_count = 0;
                }
            }

            // match message {
            //     Some(PanelToDeskMessage::Up) => {
            //         hprintln!("u").unwrap();
            //     }
            //     Some(PanelToDeskMessage::Down) => {
            //         hprintln!("d").unwrap();
            //     }
            //     Some(PanelToDeskMessage::NoKey) => {
            //         hprintln!("k, {}", no_key_send_count).unwrap();
            //         match current_direction {
            //             Some(Direction::Reversing) => {
            //                 hprintln!("dr").unwrap();
            //             }
            //             Some(Direction::Up) => {
            //                 hprintln!("du").unwrap();
            //             }
            //             Some(Direction::Down) => {
            //                 hprintln!("dd").unwrap();
            //             }
            //             None => {
            //                 hprintln!("dn").unwrap();
            //             }
            //         }
            //     }
            //     None => {
            //         hprintln!("n").unwrap();
            //     }
            //     _ => {}
            // }

            *ctx.local.no_key_send_count = no_key_send_count;
            *ctx.local.current_direction = current_direction;

            match message {
                Some(PanelToDeskMessage::NoKey) => {
                    for _ in 0..NO_KEY_SPAWN_COUNT {
                        send_message::spawn(PanelToDeskMessage::NoKey).unwrap();
                    }
                }
                Some(m) => {
                    send_message::spawn(m).unwrap();
                }
                None => {}
            }
        }

        #[task(shared = [send], priority = 1, capacity = 50)]
        fn send_message(mut ctx: send_message::Context, message: PanelToDeskMessage) {
            ctx.shared.send.lock(|send| {
                let (tx_buf, tx) = match send.take().unwrap() {
                    TxTransfer::Idle(buf, tx) => (buf, tx),
                    TxTransfer::Running(transfer) => transfer.wait(),
                };

                tx_buf.copy_from_slice(&fill_tx_buffer_with_message(message));
                (*send).replace(TxTransfer::Running(tx.write(tx_buf)));
            });
        }

        fn fill_tx_buffer_with_message(message: PanelToDeskMessage) -> [u8; TX_BUF_SIZE] {
            let mut buf = [0; TX_BUF_SIZE];
            for i in 0..(TX_BUF_SIZE / DATA_FRAME_SIZE) {
                let start = i * DATA_FRAME_SIZE;
                let end = (i + 1) * DATA_FRAME_SIZE - 1;
                buf[start..end + 1].copy_from_slice(&message.as_frame());
                //hprintln!("buf[{:?}..{:?}]={:?}", start, end, &buf[start..end + 1],).unwrap();
            }
            buf
        }

        // Triggers on TX transfer completed
        #[task(binds = DMA1_CHANNEL2, shared = [send], priority = 2)]
        fn on_tx(mut ctx: on_tx::Context) {
            // hprintln!("s").unwrap();

            ctx.shared.send.lock(|send| {
                let (tx_buf, tx) = match send.take().unwrap() {
                    TxTransfer::Idle(buf, tx) => (buf, tx),
                    TxTransfer::Running(transfer) => transfer.wait(),
                };
                // defmt::info!("Sent {:?}", tx_buf);
                send.replace(TxTransfer::Idle(tx_buf, tx));
            });
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

            // ctx.shared
            //     .disp
            //     .update_buffer_with_float(Index::One, height, 1, 10)
            //     .unwrap();
            // ctx.shared.disp.write_display_buffer().unwrap();

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
