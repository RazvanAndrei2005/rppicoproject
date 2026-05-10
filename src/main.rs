#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_time::{Delay, Instant, Timer};
use embedded_graphics::{
    mono_font::{ascii::FONT_9X15, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};
use hx711::Hx711;
use st7735_lcd::{Orientation, ST7735};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // --- 1. SETUP DISPLAY (SPI0) ---
    // SCK=GP2 (Pin 4), MOSI=GP3 (Pin 5), CS=GP4 (Pin 6), DC=GP5 (Pin 7), RST=GP6 (Pin 9)
    let dc  = Output::new(p.PIN_5, Level::Low);
    let rst = Output::new(p.PIN_6, Level::Low);
    let _cs = Output::new(p.PIN_4, Level::Low); // held Low = always selected

    let mut spi_config = SpiConfig::default();
    spi_config.frequency = 10_000_000;

    // SPI0: SCK=GP2, MOSI=GP3, MISO=GP0 (unused but required by type)
    let spi = Spi::new_blocking(p.SPI0, p.PIN_2, p.PIN_3, p.PIN_0, spi_config);

    let mut display = ST7735::new(spi, dc, rst, true, false, 128, 128);
    let mut delay = Delay;
    display.init(&mut delay).unwrap();
    display.set_orientation(&Orientation::Portrait).unwrap();
    display.clear(Rgb565::BLACK).unwrap();

    let style = MonoTextStyle::new(&FONT_9X15, Rgb565::WHITE);

    // --- 2. SETUP SERVO (PWM on GP15, Pin 20) ---
    // GP15 = PWM_SLICE7, channel B — 50Hz for standard servos
    let mut pwm_config: PwmConfig = Default::default();
    pwm_config.top = 20000;
    pwm_config.divider = 150.into(); // 125MHz / 150 / 20000 = 50Hz
    let mut pwm = Pwm::new_output_b(p.PWM_SLICE7, p.PIN_15, pwm_config.clone());

    // --- 3. SETUP ULTRASONIC (TRIG=GP14 Pin 19, ECHO=GP13 Pin 17) ---
    let mut trig = Output::new(p.PIN_14, Level::Low);
    let echo     = Input::new(p.PIN_13, Pull::None); // resistor divider on ECHO!

    // --- 4. SETUP HX711 (DT=GP12 Pin 16, SCK=GP11 Pin 15) ---
    // hx711 0.7.0 API: Hx711::new(delay, dout_pin, sck_pin)
    let hx711_dt  = Input::new(p.PIN_12, Pull::None);  // DOUT / DT
    let hx711_sck = Output::new(p.PIN_11, Level::Low); // PD_SCK / SCK
    let mut scale = Hx711::new(Delay, hx711_dt, hx711_sck).unwrap();

    // Tare: average 8 readings at startup as the zero baseline
    info!("Taring scale — keep load cell unloaded...");
    let mut tare_sum: i32 = 0;
    for _ in 0..8 {
        tare_sum += nb::block!(scale.retrieve()).unwrap();
        Timer::after_millis(120).await; // HX711 at 10SPS needs ~100ms between reads
    }
    let tare_offset: i32 = tare_sum / 8;

    // Calibration factor — tune this for your specific load cell:
    // 1. Place a known weight (e.g. 500g)
    // 2. Read the raw value from the terminal (raw - tare)
    // 3. counts_per_gram = (raw - tare) / known_grams
    const COUNTS_PER_GRAM: f32 = 450.0;

    info!("System Online! Tare offset = {}", tare_offset);

    let mut last_motor_move = Instant::now();
    let mut servo_at_max = false;

    loop {
        // --- STEP A: ULTRASONIC SENSING ---
        trig.set_low();
        Timer::after_micros(2).await;
        trig.set_high();
        Timer::after_micros(10).await;
        trig.set_low();

        while echo.is_low() {}
        let start = Instant::now();
        while echo.is_high() {}
        let end = Instant::now();

        let duration = end.duration_since(start).as_micros();
        let distance_cm = (duration as f32 * 0.0343) / 2.0;

        // --- STEP B: HX711 WEIGHT READING ---
        let weight_grams: f32 = match nb::block!(scale.retrieve()) {
            Ok(raw) => (raw - tare_offset) as f32 / COUNTS_PER_GRAM,
            Err(_)  => f32::NAN,
        };

        // --- STEP C: LOG TO TERMINAL ---
        let mut dist_buf = [0u8; 16];
        if let Some(s) = format_no_std::show(&mut dist_buf, format_args!("{:.1}", distance_cm)) {
            info!("Distance: {} cm", s);
        }
        let mut wt_buf = [0u8; 16];
        if let Some(s) = format_no_std::show(&mut wt_buf, format_args!("{:.1}", weight_grams)) {
            info!("Weight:   {} g", s);
        }

        // --- STEP D: SCREEN UPDATE ---
        display.clear(Rgb565::BLACK).unwrap();

        Text::new("RADAR+SCALE", Point::new(2, 15), style)
            .draw(&mut display).unwrap();

        // Distance row
        Text::new("Dist:", Point::new(2, 40), style)
            .draw(&mut display).unwrap();
        let mut scr_dist = [0u8; 16];
        if let Some(s) = format_no_std::show(&mut scr_dist, format_args!("{:.1}cm", distance_cm)) {
            Text::new(s, Point::new(2, 57), style)
                .draw(&mut display).unwrap();
        }

        // Weight row
        Text::new("Wt:", Point::new(2, 80), style)
            .draw(&mut display).unwrap();
        let mut scr_wt = [0u8; 16];
        if let Some(s) = format_no_std::show(&mut scr_wt, format_args!("{:.1}g", weight_grams)) {
            Text::new(s, Point::new(2, 97), style)
                .draw(&mut display).unwrap();
        }

        // --- STEP E: SERVO LOGIC (toggles every 10 seconds) ---
        if last_motor_move.elapsed().as_secs() >= 10 {
            servo_pos_toggle(&mut pwm, &mut pwm_config, &mut servo_at_max);
            last_motor_move = Instant::now();
        }

        Timer::after_millis(500).await;
    }
}

fn servo_pos_toggle(pwm: &mut Pwm<'_>, config: &mut PwmConfig, at_max: &mut bool) {
    if *at_max {
        config.compare_b = 1000; // ~1ms pulse = 0°
        *at_max = false;
    } else {
        config.compare_b = 2000; // ~2ms pulse = 180°
        *at_max = true;
    }
    pwm.set_config(config);
    info!("Servo toggled.");
}

mod format_no_std {
    use core::fmt;
    pub fn show<'a>(buf: &'a mut [u8], args: fmt::Arguments) -> Option<&'a str> {
        let mut w = Wrapper(buf, 0);
        fmt::write(&mut w, args).ok()?;
        core::str::from_utf8(&w.0[..w.1]).ok()
    }
    struct Wrapper<'a>(&'a mut [u8], usize);
    impl<'a> fmt::Write for Wrapper<'a> {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            let bytes = s.as_bytes();
            let rem = &mut self.0[self.1..];
            if bytes.len() > rem.len() {
                return Err(fmt::Error);
            }
            rem[..bytes.len()].copy_from_slice(bytes);
            self.1 += bytes.len();
            Ok(())
        }
    }
}