#![no_main]
#![no_std]

// set the panic handler
extern crate panic_semihosting;

use cortex_m_rt::entry;
use embedded_hal::digital::v2::InputPin;
use stm32f1xx_hal::{i2c, prelude::*, stm32};

use adafruit_7segment::{Index, SevenSegment};
use ht16k33::{Dimming, Display, HT16K33};
use rotary_encoder_hal::{Direction, Rotary};

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(72.mhz())
        .pclk1(36.mhz())
        .freeze(&mut flash.acr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);

    // Set up the I2C bus
    let afio = dp.AFIO.constrain(&mut rcc.apb2);
    let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
    let mut mapr = afio.mapr;
    let mode = i2c::Mode::Standard {
        frequency: 100_000.hz(),
    };

    let mut apb = rcc.apb1;
    let start_timeout_us: u32 = 10000;
    let start_retries: u8 = 5;
    let addr_timeout_us: u32 = 10000;
    let data_timeout_us: u32 = 10000;

    // Creates a blocking I2C1 object on pins PB6 and PB7
    let i2c = i2c::BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        &mut mapr,
        mode,
        clocks,
        &mut apb,
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

    let pin_a = gpioa.pa12.into_pull_up_input(&mut gpioa.crh);
    let pin_b = gpioa.pa10.into_pull_up_input(&mut gpioa.crh);

    let mut enc = Rotary::new(pin_a, pin_b);
    let mut pos: f32 = 65.0;

    let min_pos = 65.0;
    let max_pos = 127.5;

    let pin_switch = gpiob.pb11.into_pull_up_input(&mut gpiob.crh);

    loop {
        match enc.update().unwrap() {
            Direction::Clockwise => {
                pos += 0.5;
            }
            Direction::CounterClockwise => {
                pos -= 0.5;
            }
            Direction::None => {}
        }

        if pos < min_pos {
            pos = min_pos
        }

        if pos > max_pos {
            pos = max_pos
        }

        if pin_switch.is_low().unwrap() {
            ht16k33
                .set_dimming(Dimming::BRIGHTNESS_MIN)
                .expect("Could not set dimming!");
        } else {
            ht16k33
                .set_dimming(Dimming::BRIGHTNESS_MAX)
                .expect("Could not set dimming!");
        }

        ht16k33
            .update_buffer_with_float(Index::One, pos as f32, 1, 10)
            .unwrap();

        ht16k33.write_display_buffer().unwrap();
    }
}
