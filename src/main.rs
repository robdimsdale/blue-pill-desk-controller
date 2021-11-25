#![no_main]
#![no_std]

use panic_semihosting as _;
// use panic_reset as _;

// from: https://github.com/kalkyl/f103-rtic/blob/main/src/bin/serial.rs
#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [EXTI1, EXTI2, EXTI3])]
mod app {
    use cortex_m::{
        asm,
        peripheral::{syst::SystClkSource, SYST},
    };
    use stm32f1xx_hal::{
        gpio::{
            gpioa::{PA4, PA5, PA6, PA7},
            gpiob::{PB10, PB11},
            gpioc::PC13,
            Alternate, Edge, ExtiPin, Input, OpenDrain, Output, PullUp, PushPull,
        },
        i2c,
        pac::I2C2,
        prelude::*,
    };

    use adafruit_7segment::{Index, SevenSegment};
    use embedded_hal::digital::v2::{InputPin, OutputPin};
    use ht16k33::{Dimming, Display, HT16K33};
    use nb::block;
    use rtic::rtic_monotonic::{
        embedded_time::{clock::Error, fraction::Fraction},
        Clock, Instant, Monotonic,
    };
    use rtic::time::duration::*;

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<1000>; // 1000 Hz / 1 ms granularity

    #[derive(Debug, Clone, Copy, PartialOrd, PartialEq)]
    pub enum SwitchState {
        High,
        Low,
    }

    const DEBOUNCE_DURATION_MS: u32 = 10;

    #[shared]
    struct Shared {
        onboard_led: PC13<Output<PushPull>>,

        entry_count: u16,
        exit_count: u16,

        switch_debouncer_running: bool,

        button_stop: PA4<Input<PullUp>>,
    }

    #[local]
    struct Local {
        button1: PA7<Input<PullUp>>,
        button2: PA6<Input<PullUp>>,
        button3: PA5<Input<PullUp>>,

        disp: HT16K33<
            i2c::BlockingI2c<I2C2, (PB10<Alternate<OpenDrain>>, PB11<Alternate<OpenDrain>>)>,
        >,
    }

    #[init()]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
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

        let mut onboard_led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        onboard_led.set_high().unwrap();

        // Setup buttons
        let mut button_stop = gpioa.pa4.into_pull_up_input(&mut gpioa.crl);
        button_stop.make_interrupt_source(&mut afio);
        button_stop.enable_interrupt(&mut ctx.device.EXTI);
        button_stop.trigger_on_edge(&mut ctx.device.EXTI, Edge::RISING_FALLING);

        let mut button1 = gpioa.pa7.into_pull_up_input(&mut gpioa.crl);
        button1.make_interrupt_source(&mut afio);
        button1.enable_interrupt(&mut ctx.device.EXTI);
        button1.trigger_on_edge(&mut ctx.device.EXTI, Edge::RISING_FALLING);

        let mut button2 = gpioa.pa6.into_pull_up_input(&mut gpioa.crl);
        button2.make_interrupt_source(&mut afio);
        button2.enable_interrupt(&mut ctx.device.EXTI);
        button2.trigger_on_edge(&mut ctx.device.EXTI, Edge::RISING_FALLING);

        let mut button3 = gpioa.pa5.into_pull_up_input(&mut gpioa.crl);
        button3.make_interrupt_source(&mut afio);
        button3.enable_interrupt(&mut ctx.device.EXTI);
        button3.trigger_on_edge(&mut ctx.device.EXTI, Edge::RISING_FALLING);

        // Set up the I2C bus on pins PB6 and PB7 (display)
        let scl = gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh);
        let sda = gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh);
        let mode = i2c::Mode::Standard {
            frequency: 100_000.hz(),
        };

        let start_timeout_us: u32 = 10000;
        let start_retries: u8 = 5;
        let addr_timeout_us: u32 = 10000;
        let data_timeout_us: u32 = 10000;

        let i2c = i2c::BlockingI2c::i2c2(
            ctx.device.I2C2,
            (scl, sda),
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
        ht16k33.set_display(Display::ON).unwrap();
        ht16k33.update_buffer_with_colon(true);

        ht16k33.update_buffer_with_digit(Index::One, 0);
        ht16k33.update_buffer_with_digit(Index::Two, 0);
        ht16k33.update_buffer_with_digit(Index::Three, 0);
        ht16k33.update_buffer_with_digit(Index::Four, 0);

        ht16k33.write_display_buffer().unwrap();

        (
            Shared {
                button_stop: button_stop,

                onboard_led: onboard_led,
                entry_count: 0,
                exit_count: 0,

                switch_debouncer_running: false,
            },
            Local {
                button1: button1,
                button2: button2,
                button3: button3,
                disp: ht16k33,
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

    #[task(shared = [button_stop, entry_count, exit_count, onboard_led, switch_debouncer_running], priority = 1, capacity = 20)]
    fn check_switch(mut ctx: check_switch::Context, prev_state: SwitchState) {
        let mut new_state = SwitchState::Low;

        ctx.shared.button_stop.lock(|button_stop| {
            new_state = if button_stop.is_low().unwrap() {
                SwitchState::Low
            } else {
                SwitchState::High
            }
        });

        if new_state == prev_state {
            (
                ctx.shared.entry_count,
                ctx.shared.exit_count,
                ctx.shared.onboard_led,
                ctx.shared.switch_debouncer_running,
            )
                .lock(|entry_count, exit_count, led, switch_debouncer_running| {
                    *switch_debouncer_running = false;

                    match new_state {
                        SwitchState::Low => {
                            *entry_count += 1;
                            led.set_low().unwrap();
                        }
                        SwitchState::High => {
                            *exit_count += 1;
                            led.set_high().unwrap();
                        }
                    }
                });

            update_display::spawn().unwrap();
        } else {
            check_switch::spawn_after(DEBOUNCE_DURATION_MS.milliseconds(), new_state).unwrap();
        }
    }

    // Triggers on buttons pressed
    #[task(binds = EXTI4, shared = [button_stop, switch_debouncer_running], priority = 2)]
    fn on_btn_stop_press(ctx: on_btn_stop_press::Context) {
        let mut switch_state = SwitchState::Low;
        let mut start_debouncer = false;

        (ctx.shared.button_stop, ctx.shared.switch_debouncer_running).lock(
            |button_stop, switch_debouncer_running| {
                if button_stop.check_interrupt() {
                    button_stop.clear_interrupt_pending_bit();
                    switch_state = if button_stop.is_low().unwrap() {
                        SwitchState::Low
                    } else {
                        SwitchState::High
                    };
                    start_debouncer = !*switch_debouncer_running;
                    *switch_debouncer_running = true;
                } else {
                    panic!("unexpected button press");
                }
            },
        );

        // handle_button_press::spawn(3, switch_state).unwrap();
        if start_debouncer {
            check_switch::spawn_after(DEBOUNCE_DURATION_MS.milliseconds(), switch_state).unwrap();
        }
    }

    // Triggers on buttons pressed
    #[task(binds = EXTI9_5, shared = [onboard_led, entry_count, exit_count], local = [button1, button2, button3], priority = 1)]
    fn on_btn_press(ctx: on_btn_press::Context) {
        let button1 = ctx.local.button1;
        let button2 = ctx.local.button2;
        let button3 = ctx.local.button3;

        let is_low: bool;
        if button1.check_interrupt() {
            button1.clear_interrupt_pending_bit();
            is_low = button1.is_low().unwrap();
        } else if button2.check_interrupt() {
            button2.clear_interrupt_pending_bit();
            is_low = button2.is_low().unwrap();
        } else if button3.check_interrupt() {
            button3.clear_interrupt_pending_bit();
            is_low = button3.is_low().unwrap();
        } else {
            panic!("unexpected button press");
        }

        (
            ctx.shared.entry_count,
            ctx.shared.exit_count,
            ctx.shared.onboard_led,
        )
            .lock(|entry_count, exit_count, led| {
                if is_low {
                    *entry_count += 1;
                    led.set_low().unwrap();
                } else {
                    *exit_count += 1;
                    led.set_high().unwrap();
                }
            });

        update_display::spawn().unwrap();
    }

    #[task(local = [disp], shared = [entry_count, exit_count], priority = 1, capacity = 20)]
    fn update_display(ctx: update_display::Context) {
        let disp = ctx.local.disp;

        (ctx.shared.entry_count, ctx.shared.exit_count).lock(|entry_count, exit_count| {
            let entry = *entry_count % 100;
            let exit = *exit_count % 100;

            disp.update_buffer_with_digit(Index::One, (entry / 10) as u8);
            disp.update_buffer_with_digit(Index::Two, (entry % 10) as u8);
            disp.update_buffer_with_digit(Index::Three, (exit / 10) as u8);
            disp.update_buffer_with_digit(Index::Four, (exit % 10) as u8);
        });
        disp.write_display_buffer().unwrap();
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
