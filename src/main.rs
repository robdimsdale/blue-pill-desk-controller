//! examples/smallest.rs

#![no_main]
#![no_std]

mod protocol;
// use panic_semihosting as _;
// use panic_reset as _;

use blue_pill_desk_controller as _;
use panic_probe as _;

// from: https://github.com/kalkyl/f103-rtic/blob/main/src/bin/serial.rs
#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [EXTI2, EXTI3, EXTI4])]
mod app {
    use cortex_m::{
        asm,
        peripheral::{syst::SystClkSource, SYST},
    };
    use stm32f1xx_hal::{
        adc,
        adc::{AdcPayload, Continuous, SampleTime},
        dma::{
            dma1::{C1, C2, C3},
            Event, RxDma, Transfer, TxDma, R, W,
        },
        gpio::{
            gpioa, gpiob, gpioc, Alternate, Analog, Edge, ExtiPin, Input, OpenDrain, Output,
            PullUp, PushPull,
        },
        i2c,
        pac::{I2C1, USART3},
        prelude::*,
        serial::{Config, Rx, Serial, Tx},
    };

    use adafruit_7segment::{Index, SevenSegment};
    use embedded_hal::digital::v2::OutputPin;
    use ht16k33::{Dimming, Display, HT16K33};
    use nb::block;
    use rtic::rtic_monotonic::{
        embedded_time::{clock::Error, fraction::Fraction},
        Clock, Instant, Monotonic,
    };
    use rtic::time::duration::*;

    use crate::protocol::*;

    // Buffer size of 32 is the best trade-off between responsiveness and stability.
    // Buffer size of 16 can result in an unnecessarily large number of interrupts and under/overshoots.
    // Buffer size of 64 is a little less responsive and also causes under/overshoots.
    // Buffer size smaller than 16 increases the risk of having a corrupted and/or incomplete frame,
    // rendering the buffer useless.
    // Any inline hprintln! calls require a buffer size of 512 or more,
    // which will also require manually adjusting the TX_BUF_SIZE and NO_KEY_SPAWN_COUNT.
    const RX_BUF_SIZE: usize = 32;

    // Should be a multiple of DATA_FRAME_SIZE to ensure Tx is always a whole number of frames
    // Should always be smaller than RX_BUF_SIZE to avoid trying to send more than fits
    // in the RX-interrupt-driven loop
    // The expression cannot be simplified to TX_BUF_SIZE = RX_BUF_SIZE even though it might look
    // like that is possible, as RX_BUF_SIZE / DATA_FRAME_SIZE will automatically round down
    // to the nearest whole integer as they are both integers.
    const TX_BUF_SIZE: usize = DATA_FRAME_SIZE * (RX_BUF_SIZE / DATA_FRAME_SIZE);

    // This is inversely proportional to TX_BUF_SIZE, as there is a minimum amount of time the
    // "no key" signal must be sent for.
    // Aim for at around 150ms-200ms of "no key" between other messages.
    // At 9600 baud, each byte takes about 1ms, therefore TX_BUF takes about TX_BUF_SIZE milliseconds
    const NO_KEY_SPAWN_COUNT: u16 = 200 / (TX_BUF_SIZE as u16);

    // Controls overshoot/undershoot. Adjust if changing the buffer sizes above.
    // Minimum useful value is 1x step size i.e. 0.5
    // For RX_BUF_SIZE = 64:
    //    TARGET_HEIGHT_STOP_DIFFERENCE = 1.5 occasionally undershoots but converges.
    //    TARGET_HEIGHT_STOP_DIFFERENCE = 1.0 results in occasional undershoot but converges.
    //    TARGET_HEIGHT_STOP_DIFFERENCE = 0.5 results in fairly frequent overshoot but converges.
    // For RX_BUF_SIZE = 32:
    //    TARGET_HEIGHT_STOP_DIFFERENCE = 1.0 works well - no observed overshoot/undershoot.
    //    TARGET_HEIGHT_STOP_DIFFERENCE = 0.5 occasionally overshoots a little bit.
    // For RX_BUF_SIZE = 16:
    //    TARGET_HEIGHT_STOP_DIFFERENCE = 1.0 undershoots and can cause crashes.
    //    TARGET_HEIGHT_STOP_DIFFERENCE = 0.5 occasionally overshoots a little bit.
    // const TARGET_HEIGHT_STOP_DIFFERENCE: f32 = 1.0;
    const TARGET_HEIGHT_STOP_DIFFERENCE: f32 = 0.5;

    // Must be > 1
    const MAX_STABLE_ITERATION_COUNT: u16 = 5;

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
    // We also want to set this lower to account for de-noising (averaging),
    // and to ensure we reliably get to the max even if the input is moving very slowly
    const ADC_MAX_VALUE: u16 = 4070;

    const DISPLAY_TARGET_HEIGHT_RESET_INTERVAL_SECONDS: u32 = 4;
    const DISPLAY_OFF_AFTER_SECONDS: u32 = 10;
    const DISPLAY_FLASH_AFTER_SECONDS: u32 = 1;
    const DISPLAY_REFRESH_INTERVAL_MILLISECONDS: u32 = 10;

    pub enum TxTransfer {
        Running(Transfer<R, &'static mut [u8; TX_BUF_SIZE], TxDma<Tx<USART3>, C2>>),
        Idle(&'static mut [u8; TX_BUF_SIZE], TxDma<Tx<USART3>, C2>),
    }

    #[derive(Clone, Copy, PartialEq)]
    pub enum Direction {
        Up,
        Down,
        Reversing,
    }

    pub enum DisplayMode {
        CurrentHeight,
        InputHeight,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<1000>; // 1000 Hz / 1 ms granularity

    #[shared]
    struct Shared {
        // TODO: ensure that this doesn't actually result in a deadlock due to the use of
        // transfer.wait() after getting the lock.
        send: Option<TxTransfer>,
        disp_mode: DisplayMode,

        target_height: Option<f32>,
        current_height: f32,
        input_height: f32,
        current_direction: Option<Direction>,

        reset_display_handler: Option<reset_display_mode::SpawnHandle>,
    }

    #[local]
    struct Local {
        no_key_send_count: u16,
        stable_iteration_count: u16,
        previous_iteration_height: f32,

        recv: Option<Transfer<W, &'static mut [u8; RX_BUF_SIZE], RxDma<Rx<USART3>, C3>>>,

        button: gpiob::PB1<Input<PullUp>>,

        adc_recv: Option<
            Transfer<
                W,
                &'static mut [u16; ADC_BUF_SIZE],
                RxDma<AdcPayload<gpioa::PA0<Analog>, Continuous>, C1>,
            >,
        >,
        current_adc_pos: u16,

        disp: HT16K33<
            i2c::BlockingI2c<
                I2C1,
                (
                    gpiob::PB6<Alternate<OpenDrain>>,
                    gpiob::PB7<Alternate<OpenDrain>>,
                ),
            >,
        >,
        current_disp_value: f32,
        disp_last_changed: Instant<MyMono>,

        onboard_led: gpioc::PC13<Output<PushPull>>,
    }

    #[init(local = [rx_buf: [u8; RX_BUF_SIZE] = [0; RX_BUF_SIZE], tx_buf: [u8; TX_BUF_SIZE] = [0; TX_BUF_SIZE], adc_buf: [u16; ADC_BUF_SIZE] = [0; ADC_BUF_SIZE]])]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("Hello, world!");

        let mut rcc = ctx.device.RCC.constrain();
        let mut flash = ctx.device.FLASH.constrain();

        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(72.mhz())
            .pclk1(36.mhz())
            .freeze(&mut flash.acr);

        let mono = Systick::new(ctx.core.SYST, 72_000_000);

        let mut afio = ctx.device.AFIO.constrain(&mut rcc.apb2);
        let mut gpioa = ctx.device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = ctx.device.GPIOB.split(&mut rcc.apb2);
        let mut gpioc = ctx.device.GPIOC.split(&mut rcc.apb2);

        let onboard_led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

        let mut dma_channels = ctx.device.DMA1.split(&mut rcc.ahb);
        dma_channels.1.listen(Event::TransferComplete);

        // Setup button
        let mut button = gpiob.pb1.into_pull_up_input(&mut gpiob.crl);
        button.make_interrupt_source(&mut afio);
        button.enable_interrupt(&mut ctx.device.EXTI);
        button.trigger_on_edge(&mut ctx.device.EXTI, Edge::FALLING);

        // Setup ADC on pin PA0 (potentiometer input)
        let mut adc1 = adc::Adc::adc1(ctx.device.ADC1, &mut rcc.apb2, clocks);
        adc1.set_sample_time(SampleTime::T_239); // Slow down the ADC as much as possible
        let adc_ch0 = gpioa.pa0.into_analog(&mut gpioa.crl);
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
        block!(ht16k33.initialize()).expect("Failed to initialize ht16k33");
        ht16k33.set_dimming(Dimming::BRIGHTNESS_MAX).unwrap();

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

        update_display::spawn_after(DISPLAY_REFRESH_INTERVAL_MILLISECONDS.milliseconds()).unwrap();
        for _ in 0..NO_KEY_SPAWN_COUNT {
            send_message::spawn(PanelToDeskMessage::NoKey).unwrap();
        }

        (
            Shared {
                send: Some(TxTransfer::Idle(ctx.local.tx_buf, tx)),
                disp_mode: DisplayMode::InputHeight,
                target_height: None,
                current_height: 0.0,
                input_height: 0.0,
                current_direction: None,
                reset_display_handler: None,
            },
            Local {
                disp: ht16k33,
                current_disp_value: 0.0,
                disp_last_changed: Instant::<MyMono>::new(0),
                recv: Some(rx.read(ctx.local.rx_buf)),
                adc_recv: Some(adc_dma.read(ctx.local.adc_buf)),
                current_adc_pos: 0,
                no_key_send_count: 0,
                stable_iteration_count: 0,
                previous_iteration_height: 0.0,
                button: button,
                onboard_led: onboard_led,
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            asm::wfi();
        }
    }

    // Triggers on RX transfer completed
    #[task(binds = DMA1_CHANNEL3, local = [recv], priority = 4)]
    fn on_rx(ctx: on_rx::Context) {
        let (rx_buf, rx) = ctx.local.recv.take().unwrap().wait();
        read_height::spawn(*rx_buf).unwrap();
        ctx.local.recv.replace(rx.read(rx_buf));
    }

    #[task(shared = [current_height, disp_mode], priority = 2, capacity = 2)]
    fn read_height(mut ctx: read_height::Context, data: [u8; RX_BUF_SIZE]) {
        let frame = find_first_frame(&data);
        if validate_frame(&frame) {
            match DeskToPanelMessage::from_frame(&frame) {
                DeskToPanelMessage::Height(h) => {
                    ctx.shared.current_height.lock(|current_height| {
                        *current_height = h;
                    });

                    // Explicitly ignore a failed spawn attempt.
                    // We do not care if we miss an opportunity to potentially send a message
                    // as we will catch it next time we enter this function.
                    let _ = compare_height::spawn();
                }
                DeskToPanelMessage::Unknown(_, _, _, _, _) => {
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

    #[task(local = [no_key_send_count, previous_iteration_height, stable_iteration_count], shared = [current_direction, current_height, target_height], priority = 1, capacity = 1)]
    fn compare_height(ctx: compare_height::Context) {
        let mut no_key_send_count = *ctx.local.no_key_send_count;
        let mut stable_iteration_count = *ctx.local.stable_iteration_count;
        let mut previous_iteration_height = *ctx.local.previous_iteration_height;
        let mut message = None;

        (
            ctx.shared.current_direction,
            ctx.shared.current_height,
            ctx.shared.target_height,
        )
            .lock(|current_direction, current_height, ctx_target_height| {
                if *current_height == previous_iteration_height {
                    stable_iteration_count += 1;
                } else {
                    stable_iteration_count = 0;
                }
                previous_iteration_height = *current_height;

                match current_direction {
                    Some(Direction::Reversing) => {
                        if no_key_send_count < NO_KEY_SPAWN_COUNT {
                            message = Some(PanelToDeskMessage::NoKey);
                        } else {
                            *current_direction = None;
                        }
                    }
                    _ => match *ctx_target_height {
                        Some(target_height) => {
                            if abs_diff_f32(*current_height, target_height)
                                <= TARGET_HEIGHT_STOP_DIFFERENCE
                                && stable_iteration_count < MAX_STABLE_ITERATION_COUNT
                            {
                                message = Some(PanelToDeskMessage::NoKey);
                                *current_direction = None;
                            } else {
                                if *current_height < target_height {
                                    match current_direction {
                                        Some(Direction::Down) => {
                                            *current_direction = Some(Direction::Reversing);
                                            message = Some(PanelToDeskMessage::NoKey);
                                        }
                                        _ => {
                                            *current_direction = Some(Direction::Up);
                                            message = Some(PanelToDeskMessage::Up);
                                        }
                                    }
                                } else if *current_height > target_height {
                                    match current_direction {
                                        Some(Direction::Up) => {
                                            *current_direction = Some(Direction::Reversing);
                                            message = Some(PanelToDeskMessage::NoKey);
                                        }
                                        _ => {
                                            *current_direction = Some(Direction::Down);
                                            message = Some(PanelToDeskMessage::Down);
                                        }
                                    }
                                } else {
                                    *current_direction = None;

                                    if stable_iteration_count < MAX_STABLE_ITERATION_COUNT {
                                        message = Some(PanelToDeskMessage::NoKey);
                                    } else {
                                        *ctx_target_height = None;
                                    }

                                    // if no_key_send_count < NO_KEY_ITERATION_COUNT {
                                    //     message = Some(PanelToDeskMessage::NoKey);
                                    // } else {
                                    //     *ctx_target_height = None;
                                    // }
                                }
                            }
                        }
                        None => {
                            // panic!("at target height - rest of code not implemented");
                        }
                    },
                }
            });

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

        *ctx.local.no_key_send_count = no_key_send_count;
        *ctx.local.previous_iteration_height = previous_iteration_height;
        *ctx.local.stable_iteration_count = stable_iteration_count;

        match message {
            Some(PanelToDeskMessage::NoKey) => {
                // for _ in 0..NO_KEY_SPAWN_COUNT {
                send_message::spawn(PanelToDeskMessage::NoKey).unwrap();
                // }
            }
            Some(m) => {
                send_message::spawn(m).unwrap();
            }
            None => {}
        }
    }

    #[task(local = [onboard_led], shared = [send], priority = 1, capacity = 20)]
    fn send_message(mut ctx: send_message::Context, message: PanelToDeskMessage) {
        let onboard_led = ctx.local.onboard_led;
        ctx.shared.send.lock(|send| {
            onboard_led.set_low().unwrap();

            let (tx_buf, tx) = match send.take().unwrap() {
                TxTransfer::Idle(buf, tx) => (buf, tx),
                TxTransfer::Running(transfer) => transfer.wait(),
            };

            tx_buf.copy_from_slice(&fill_tx_buffer_with_message(message));
            (*send).replace(TxTransfer::Running(tx.write(tx_buf)));

            onboard_led.set_high().unwrap();
        });
    }

    fn fill_tx_buffer_with_message(message: PanelToDeskMessage) -> [u8; TX_BUF_SIZE] {
        let mut buf = [0; TX_BUF_SIZE];
        for i in 0..(TX_BUF_SIZE / DATA_FRAME_SIZE) {
            let start = i * DATA_FRAME_SIZE;
            let end = (i + 1) * DATA_FRAME_SIZE - 1;
            buf[start..end + 1].copy_from_slice(&message.as_frame());
        }
        buf
    }

    // Triggers on TX transfer completed
    #[task(binds = DMA1_CHANNEL2, shared = [send], priority = 2)]
    fn on_tx(mut ctx: on_tx::Context) {
        ctx.shared.send.lock(|send| {
            let (tx_buf, tx) = match send.take().unwrap() {
                TxTransfer::Idle(buf, tx) => (buf, tx),
                TxTransfer::Running(transfer) => transfer.wait(),
            };
            send.replace(TxTransfer::Idle(tx_buf, tx));
        });
    }

    #[task(local = [disp, current_disp_value, disp_last_changed], shared = [current_height, disp_mode, input_height], priority = 1)]
    fn update_display(ctx: update_display::Context) {
        let disp = ctx.local.disp;
        let current_disp_value = ctx.local.current_disp_value;
        let disp_last_changed = ctx.local.disp_last_changed;

        (
            ctx.shared.current_height,
            ctx.shared.input_height,
            ctx.shared.disp_mode,
        )
            .lock(|current_height, input_height, disp_mode| {
                let now = monotonics::now();

                let (h, disp_flash) = match disp_mode {
                    DisplayMode::CurrentHeight => (*current_height, Display::ON),
                    DisplayMode::InputHeight => (*input_height, Display::TWO_HZ),
                };

                if h != *current_disp_value {
                    *disp_last_changed = now;
                }
                *current_disp_value = h;

                let time_to_flash = *disp_last_changed + DISPLAY_FLASH_AFTER_SECONDS.seconds();
                let time_to_turn_off = *disp_last_changed + DISPLAY_OFF_AFTER_SECONDS.seconds();

                if now < time_to_flash {
                    disp.set_display(Display::ON).unwrap();
                } else if now >= time_to_flash && now < time_to_turn_off {
                    disp.set_display(disp_flash).unwrap();
                } else {
                    disp.set_display(Display::OFF).unwrap();
                }

                disp.update_buffer_with_float(Index::One, h, 1, 10).unwrap();
                disp.write_display_buffer().unwrap();
            });

        update_display::spawn_after(DISPLAY_REFRESH_INTERVAL_MILLISECONDS.milliseconds()).unwrap();
    }

    #[task(shared = [disp_mode,reset_display_handler], priority = 1, capacity = 1)]
    fn reset_display_mode(ctx: reset_display_mode::Context) {
        (ctx.shared.disp_mode, ctx.shared.reset_display_handler).lock(
            |disp_mode, reset_display_handler| {
                *disp_mode = DisplayMode::CurrentHeight;
                *reset_display_handler = None;
            },
        );
    }

    // Triggers on button pressed
    #[task(binds = EXTI1, local = [button], shared = [disp_mode, input_height, target_height], priority = 1)]
    fn on_btn_press(ctx: on_btn_press::Context) {
        let button = ctx.local.button;
        if button.check_interrupt() {
            button.clear_interrupt_pending_bit();
            (
                ctx.shared.disp_mode,
                ctx.shared.input_height,
                ctx.shared.target_height,
            )
                .lock(|disp_mode, input_height, target_height| {
                    // TODO: replace with more useful code when we have multiple buttons
                    // match target_height {
                    //     None => {
                    //         *target_height = Some(*input_height);
                    //     }
                    //     Some(_) => {
                    //         *target_height = None;
                    //         stop_moving::spawn().unwrap();
                    //     }
                    // }

                    *target_height = Some(*input_height);
                    // stop_moving::spawn_after(3.seconds()).unwrap();

                    *disp_mode = DisplayMode::CurrentHeight;
                });
        }
    }

    #[task(shared = [current_direction, target_height], priority = 3, capacity = 5)]
    fn stop_moving(ctx: stop_moving::Context) {
        (ctx.shared.current_direction, ctx.shared.target_height).lock(
            |current_direction, target_height| {
                *current_direction = None;
                *target_height = None;
            },
        );

        send_message::spawn(PanelToDeskMessage::NoKey).unwrap();
    }

    // Triggers on ADC read completed
    #[task(binds = DMA1_CHANNEL1, local = [adc_recv, current_adc_pos], shared = [disp_mode, input_height,reset_display_handler], priority = 1)]
    fn on_adc_read(mut ctx: on_adc_read::Context) {
        let (rx_buf, rx) = ctx.local.adc_recv.take().unwrap().wait();

        // Cast to u32 to enable summation as u32 and avoid overrun which would often occur
        // if we tried to sum as u16.
        // The max value in the buffer is 2^12 - 1, and the max u16 value is 2^16 - 1,
        // so if the buffer size is > 2^4 then we will see overrun for large values:
        // 2^4 * (2^12-1) is close to 2^16
        let avg = (&rx_buf.iter().map(|&x| x as u32).sum::<u32>() / (rx_buf.len() as u32)) as u16;

        let (new_position, above_threshold) = threshold_smooth_adc(avg, *ctx.local.current_adc_pos);

        let height = normalize_input_height(new_position);
        *ctx.local.current_adc_pos = new_position;

        ctx.shared.input_height.lock(|input_height| {
            *input_height = height;
        });

        if above_threshold {
            (ctx.shared.disp_mode, ctx.shared.reset_display_handler).lock(
                |disp_mode, reset_display_handler| {
                    *disp_mode = DisplayMode::InputHeight;

                    *reset_display_handler = Some(match *reset_display_handler {
                        None => reset_display_mode::spawn_after(
                            DISPLAY_TARGET_HEIGHT_RESET_INTERVAL_SECONDS.seconds(),
                        )
                        .unwrap(),

                        // Have to take the original reference as the reschedule_after consumes self
                        Some(_) => reset_display_handler
                            .take()
                            .unwrap()
                            // This could possibly fail for a valid reason if the other task has started execution and this one preempts it.
                            // This won't happen as long as the priority of this task is lower than or equal to the priority of the other task.
                            // And in any case it is an extremely small window as the other task immediately gets a lock on the shared reset_display_handler.
                            .reschedule_after(
                                DISPLAY_TARGET_HEIGHT_RESET_INTERVAL_SECONDS.seconds(),
                            )
                            .unwrap(),
                    });
                },
            );
        }

        ctx.local.adc_recv.replace(rx.read(rx_buf));
    }

    fn threshold_smooth_adc(val: u16, prev_val: u16) -> (u16, bool) {
        if abs_diff_u16(val, prev_val) > ADC_SMOOTH_THRESHOLD {
            (val, true)
        } else {
            (prev_val, false)
        }
    }

    fn abs_diff_u16(a: u16, b: u16) -> u16 {
        if a > b {
            return a - b;
        } else {
            return b - a;
        }
    }

    fn abs_diff_f32(a: f32, b: f32) -> f32 {
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

    /// Systick implementing `embedded_time::Clock` and `rtic_monotonic::Monotonic` which runs at a
    /// settable rate using the `TIMER_HZ` parameter.
    ///
    /// from: https://github.com/rtic-rs/systick-monotonic/blob/master/src/lib.rs
    pub struct Systick<const TIMER_HZ: u32> {
        systick: SYST,
        cnt: u32,
        reload: u32,
    }

    impl<const TIMER_HZ: u32> Systick<TIMER_HZ> {
        /// Provide a new `Monotonic` based on SysTick.
        ///
        /// Note that the `sysclk` parameter should come from e.g. the HAL's clock generation function
        /// so the real speed and the declared speed can be compared.
        pub fn new(mut systick: SYST, sysclk: u32) -> Self {
            systick.disable_counter();

            Systick {
                systick,
                cnt: 0,
                reload: (sysclk + TIMER_HZ / 2) / TIMER_HZ - 1,
            }
        }
    }

    impl<const TIMER_HZ: u32> Clock for Systick<TIMER_HZ> {
        type T = u32;

        const SCALING_FACTOR: Fraction = Fraction::new(1, TIMER_HZ);

        #[inline(always)]
        fn try_now(&self) -> Result<Instant<Self>, Error> {
            // The instant is always valid
            Ok(Instant::new(self.cnt))
        }
    }

    impl<const TIMER_HZ: u32> Monotonic for Systick<TIMER_HZ> {
        const DISABLE_INTERRUPT_ON_EMPTY_QUEUE: bool = false;

        unsafe fn reset(&mut self) {
            self.systick.set_clock_source(SystClkSource::Core);
            self.systick.set_reload(self.reload);
            self.systick.clear_current();
            self.systick.enable_counter();
        }

        fn set_compare(&mut self, _val: &Instant<Self>) {
            // No need to do something here, we get interrupts every tick anyways.
        }

        fn clear_compare_flag(&mut self) {
            // NOOP with SysTick interrupt
        }

        fn on_interrupt(&mut self) {
            // Increase the counter every overflow.
            self.cnt += 1;
        }
    }
}
