#![no_main]
#![no_std]

// set the panic handler
extern crate panic_semihosting;

use cortex_m_rt::entry;
use embedded_hal::digital::v2::InputPin;
use stm32f1xx_hal::{adc, i2c, prelude::*, stm32};

use adafruit_7segment::{Index, SevenSegment};
use ht16k33::{Dimming, Display, HT16K33};

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
        .adcclk(2.mhz())
        .freeze(&mut flash.acr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);

    // Setup ADC
    let mut adc1 = adc::Adc::adc1(dp.ADC1, &mut rcc.apb2, clocks);
    let mut ch0 = gpioa.pa0.into_analog(&mut gpioa.crl);

    // Set up the I2C bus
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
    let mode = i2c::Mode::Standard {
        frequency: 100_000.hz(),
    };

    let start_timeout_us: u32 = 10000;
    let start_retries: u8 = 5;
    let addr_timeout_us: u32 = 10000;
    let data_timeout_us: u32 = 10000;

    // Creates a blocking I2C1 object on pins PB6 and PB7
    let i2c = i2c::BlockingI2c::i2c1(
        dp.I2C1,
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

    let min_pos = 65.0;
    let max_pos = 127.5;
    let step_size = 0.5;

    let adc_max = 4096.0; // 12-bit ADC i.e. 2^12

    let pin_switch = gpiob.pb11.into_pull_up_input(&mut gpiob.crh);

    let mut current_data: u16 = 0;

    // The total range is 127-65.5 = 62.5 and each 0.5 cm is a step so 62.5 * 2 = 125 steps.
    // The ADC is 12-bits (i.e. 2^12) but we only really need 125 (i.e. a bit under 2^7),
    // therefore the max threshold can be a bit over 2^(12-7) = 2^5 = 32.
    // Set the threshold to 32 to provide a bit of headroom while still providing good noise reduction.
    // Humans will not adjust the value back and forth fast enough for any meaningful signal to be near
    // the same frequency as the noise.
    let threshold: u16 = 32;

    loop {
        let new_data: u16 = adc1
            .read(&mut ch0)
            .expect("Failed to read analog value on ADC1");
        if abs_diff(new_data, current_data) > threshold {
            current_data = new_data;
        }
        let ratio = current_data as f32 / adc_max;

        let mut pos: f32 = ratio * (max_pos - min_pos) + min_pos;

        pos = round_to_nearest(pos, step_size);

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

fn round_to_nearest(val: f32, round_to: f32) -> f32 {
    let rounded_multiple = (val / round_to) as u16;
    return rounded_multiple as f32 * round_to;
}

fn abs_diff(a: u16, b: u16) -> u16 {
    if a > b {
        return a - b;
    } else {
        return b - a;
    }
}
