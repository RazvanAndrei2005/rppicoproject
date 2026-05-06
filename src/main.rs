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
use st7735_lcd::{Orientation, ST7735};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // --- 1. SETUP DISPLAY (SPI0) ---
    let dc = Output::new(p.PIN_5, Level::Low);
    let rst = Output::new(p.PIN_6, Level::Low);
    let mut cs = Output::new(p.PIN_4, Level::Low); // Set CS Low to keep screen selected

    let mut spi_config = SpiConfig::default();
    spi_config.frequency = 10_000_000; // Lowered to 10MHz for better stability during testing

    let spi = Spi::new_blocking(p.SPI0, p.PIN_2, p.PIN_3, p.PIN_0, spi_config);

    let mut display = ST7735::new(spi, dc, rst, true, false, 128, 128);
    
    let mut delay = Delay; 
    display.init(&mut delay).unwrap();
    display.set_orientation(&Orientation::Portrait).unwrap();
    display.clear(Rgb565::BLACK).unwrap();

    let style = MonoTextStyle::new(&FONT_9X15, Rgb565::WHITE);

    // --- 2. SETUP SERVO (PWM) ---
    let mut pwm_config: PwmConfig = Default::default();
    pwm_config.top = 20000;
    pwm_config.divider = 150.into(); 
    let mut pwm = Pwm::new_output_b(p.PWM_SLICE7, p.PIN_15, pwm_config.clone());

    // --- 3. SETUP ULTRASONIC (GP14, GP13) ---
    let mut trig = Output::new(p.PIN_14, Level::Low);
    let echo = Input::new(p.PIN_13, Pull::None);

    info!("System Online. Watch the terminal for distance data!");
let mut backlight = Output::new(p.PIN_7, Level::High); // High = On
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
if distance_cm > 100.0 {
    backlight.set_low();  // Turn off if nothing is nearby
} else {
    backlight.set_high(); // Turn on when something is detected
}
        // --- STEP B: LOG TO PC TERMINAL ---
        // This lets you see the sensor work even if the screen is dead
        let mut log_buf = [0u8; 16];
        if let Some(msg) = format_no_std::show(&mut log_buf, format_args!("{:.1}", distance_cm)) {
            info!("Distance: {} cm", msg); 
        }

        // --- STEP C: SCREEN UPDATE ---
        display.clear(Rgb565::BLACK).unwrap();
        Text::new("RADAR", Point::new(10, 20), style).draw(&mut display).unwrap();

        let mut scr_buf = [0u8; 16];
        if let Some(msg) = format_no_std::show(&mut scr_buf, format_args!("{:.1} cm", distance_cm)) {
            Text::new(msg, Point::new(10, 60), style).draw(&mut display).unwrap();
        }

        // --- STEP D: SERVO LOGIC ---
        if last_motor_move.elapsed().as_secs() >= 10 {
            servo_pos_toggle(&mut pwm, &mut pwm_config, &mut servo_at_max);
            last_motor_move = Instant::now();
        }

        Timer::after_millis(500).await; // Slower refresh to make terminal readable
    }
}

fn servo_pos_toggle(pwm: &mut Pwm<'_>, config: &mut PwmConfig, at_max: &mut bool) {
    if *at_max {
        config.compare_b = 1000;
        *at_max = false;
    } else {
        config.compare_b = 2000;
        *at_max = true;
    }
    pwm.set_config(config);
    info!("Toggling Servo...");
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
            if bytes.len() > rem.len() { return Err(fmt::Error); }
            rem[..bytes.len()].copy_from_slice(bytes);
            self.1 += bytes.len();
            Ok(())
        }
    }
}