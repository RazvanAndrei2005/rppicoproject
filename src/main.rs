#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{Config as I2cConfig, I2c};
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

use ds323x::{Ds323x, NaiveDate, DateTimeAccess, Timelike};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{SdCard, TimeSource, Timestamp, VolumeManager};
use {defmt_rtt as _, panic_probe as _};

// Dummy TimeSource required by embedded-sdmmc for file creation dates
struct Clock;
impl TimeSource for Clock {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp { year_since_1970: 56, zero_indexed_month: 0, zero_indexed_day: 0, hours: 0, minutes: 0, seconds: 0 }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // --- 1. SETUP DISPLAY (SPI0) ---
    let dc  = Output::new(p.PIN_5, Level::Low);
    let rst = Output::new(p.PIN_6, Level::Low);
    let _cs = Output::new(p.PIN_4, Level::Low);

    let mut spi0_config = SpiConfig::default();
    spi0_config.frequency = 10_000_000;
    let spi0 = Spi::new_blocking(p.SPI0, p.PIN_2, p.PIN_3, p.PIN_0, spi0_config);

    let mut display = ST7735::new(spi0, dc, rst, true, false, 128, 128);
    let mut delay = Delay;
    display.init(&mut delay).unwrap();
    display.set_orientation(&Orientation::Portrait).unwrap();
    display.clear(Rgb565::BLACK).unwrap();
    let style = MonoTextStyle::new(&FONT_9X15, Rgb565::WHITE);

    // --- 2. SETUP RTC (I2C0 on GP16=SDA, GP17=SCL) ---
    let i2c = I2c::new_blocking(p.I2C0, p.PIN_17, p.PIN_16, I2cConfig::default());
    let mut rtc = Ds323x::new_ds3231(i2c);
    
    // NOTE: UNCOMMENT THIS ONCE to set the initial time, then comment it out and re-flash.
     //let initial_time = NaiveDate::from_ymd_opt(2026, 5, 17).unwrap().and_hms_opt(22, 49, 0).unwrap();
    // rtc.set_datetime(&initial_time).unwrap();

    // --- 3. SETUP MICROSD (SPI1 on GP26, GP27, GP28, CS=GP9) ---
    let mut spi1_config = SpiConfig::default();
   spi1_config.frequency = 400_000;
    let spi1 = Spi::new_blocking(p.SPI1, p.PIN_26, p.PIN_27, p.PIN_28, spi1_config);
    let sd_cs = Output::new(p.PIN_9, Level::High);
    
    let spi_device = ExclusiveDevice::new_no_delay(spi1, sd_cs).unwrap();
    
    let sd_spi = SdCard::new(spi_device, Delay);
    let mut volume_mgr = VolumeManager::new(sd_spi, Clock);

    // --- 4. SETUP SENSORS & ACTUATORS ---
    let mut pwm_config: PwmConfig = Default::default();
    pwm_config.top = 20000;
    pwm_config.divider = 150.into();
    let mut pwm = Pwm::new_output_b(p.PWM_SLICE7, p.PIN_15, pwm_config.clone());

    let mut trig = Output::new(p.PIN_14, Level::Low);
    let echo     = Input::new(p.PIN_13, Pull::None);

    let hx711_dt  = Input::new(p.PIN_12, Pull::None);
    let hx711_sck = Output::new(p.PIN_11, Level::Low);
    let mut scale = Hx711::new(Delay, hx711_dt, hx711_sck).unwrap();
    
    let pir = Input::new(p.PIN_10, Pull::Down); // PIR Sensor on GP10

    let button = Input::new(p.PIN_18, Pull::Up); 
    let mut led_green  = Output::new(p.PIN_20, Level::Low);
    let mut led_yellow = Output::new(p.PIN_21, Level::Low);
    let mut led_red    = Output::new(p.PIN_22, Level::Low);

    // Tare scale
    info!("Taring scale — keep load cell unloaded...");
    let mut tare_sum: i32 = 0;
    for _ in 0..8 {
        tare_sum += nb::block!(scale.retrieve()).unwrap();
        Timer::after_millis(120).await;
    }
    let tare_offset: i32 = tare_sum / 8;
    const COUNTS_PER_GRAM: f32 = 450.0;

    info!("System Online!");

    // --- STATE VARIABLES ---
    let mut servo_at_max = false;
    let mut weight_grams: f32 = 0.0;
    let mut expected_weight: f32 = 0.0;
    let mut is_error_state = false;
    let mut feed_in_progress = false;
    let mut feed_timer = Instant::now();
    
    let mut fed_morning = false;
    let mut fed_evening = false;
    let mut last_pir_state = false;

    loop {
        let current_time = match rtc.datetime() {
            Ok(dt) => dt,
            Err(_) => {
                error!("RTC Read Error!");
                NaiveDate::from_ymd_opt(2000, 1, 1).unwrap().and_hms_opt(0, 0, 0).unwrap()
            }
        };

        // --- STEP A: AUTOMATED FEEDING LOGIC ---
        if current_time.hour() == 8 && current_time.minute() == 0 && !fed_morning {
            trigger_feed(&mut pwm, &mut pwm_config, &mut servo_at_max, &mut expected_weight, &mut feed_in_progress, &mut feed_timer, weight_grams);
            fed_morning = true;
        }
        if current_time.hour() == 18 && current_time.minute() == 0 && !fed_evening {
            trigger_feed(&mut pwm, &mut pwm_config, &mut servo_at_max, &mut expected_weight, &mut feed_in_progress, &mut feed_timer, weight_grams);
            fed_evening = true;
        }
        
        if current_time.hour() == 0 {
            fed_morning = false;
            fed_evening = false;
        }

        // --- STEP B: PIR SENSOR & SD LOGGING ---
        let current_pir_state = pir.is_high();
        if current_pir_state && !last_pir_state {
            info!("Cat detected! Logging to SD Card...");
            
            let mut log_buf = [0u8; 64];
            if let Some(s) = format_no_std::show(&mut log_buf, format_args!("Cat detected at {:02}:{:02}:{:02}\n", current_time.hour(), current_time.minute(), current_time.second())) {
                
                match volume_mgr.open_volume(embedded_sdmmc::VolumeIdx(0)) {
                    Ok(mut volume) => {
                        if let Ok(mut dir) = volume.open_root_dir() {
                            if let Ok(mut file) = dir.open_file_in_dir("CATLOG.TXT", embedded_sdmmc::Mode::ReadWriteCreateOrAppend) {
                                file.seek_from_end(0).unwrap_or_default();
                                file.write(s.as_bytes()).unwrap_or_default();
                                
                                // FIXED: Using new .close() methods
                                file.close().unwrap_or_default();
                                info!("Log successful!");
                            } else {
                                error!("Failed to open/create CATLOG.TXT");
                            }
                            
                            // FIXED: Using new .close() methods
                            dir.close().unwrap_or_default();
                        }
                    },
                    Err(_) => error!("SD Card Volume Error. Is it FAT32?"),
                }
            }
        }
        last_pir_state = current_pir_state;

        // --- STEP C: READ WEIGHT ---
        weight_grams = match nb::block!(scale.retrieve()) {
            Ok(raw) => (raw - tare_offset) as f32 / COUNTS_PER_GRAM,
            Err(_)  => f32::NAN,
        };

        // --- STEP D: MANUAL FEED BUTTON CHECK ---
        if button.is_low() {
            trigger_feed(&mut pwm, &mut pwm_config, &mut servo_at_max, &mut expected_weight, &mut feed_in_progress, &mut feed_timer, weight_grams);
            Timer::after_millis(300).await; 
        }

        // --- STEP E: VERIFY FEED SUCCESS ---
        if feed_in_progress && feed_timer.elapsed().as_secs() >= 2 {
            feed_in_progress = false;
            if weight_grams < expected_weight {
                info!("ERROR: Weight did not increase by 3g!");
                is_error_state = true;
            } else {
                info!("Feed successful!");
                is_error_state = false;
            }
        }

        if is_error_state && weight_grams >= expected_weight {
            info!("Error Cleared.");
            is_error_state = false;
        }

        // --- STEP F: ULTRASONIC SENSING ---
        trig.set_low();
        Timer::after_micros(2).await;
        trig.set_high();
        Timer::after_micros(10).await;
        trig.set_low();

        while echo.is_low() {}
        let start = Instant::now();
        while echo.is_high() {}
        let end = Instant::now();

        let distance_cm = (end.duration_since(start).as_micros() as f32 * 0.0343) / 2.0;

        // --- STEP G: STATUS LED LOGIC ---
        if is_error_state {
            led_red.set_high(); led_green.set_low(); led_yellow.set_low();
        } else {
            led_red.set_low();
            if distance_cm < 10.0 {
                led_green.set_high(); led_yellow.set_low(); 
            } else {
                led_green.set_low(); led_yellow.set_high();
            }
        }

        // --- STEP H: SCREEN UPDATE ---
        display.clear(Rgb565::BLACK).unwrap();
        
        let mut time_buf = [0u8; 16];
        if let Some(s) = format_no_std::show(&mut time_buf, format_args!("{:02}:{:02}", current_time.hour(), current_time.minute())) {
            Text::new(s, Point::new(2, 15), style).draw(&mut display).unwrap();
        }

        let mut scr_dist = [0u8; 16];
        if let Some(s) = format_no_std::show(&mut scr_dist, format_args!("Dist: {:.1}cm", distance_cm)) {
            Text::new(s, Point::new(2, 40), style).draw(&mut display).unwrap();
        }

        let mut scr_wt = [0u8; 16];
        if let Some(s) = format_no_std::show(&mut scr_wt, format_args!("Wt: {:.1}g", weight_grams)) {
            Text::new(s, Point::new(2, 60), style).draw(&mut display).unwrap();
        }

        Timer::after_millis(200).await;
    }
}

fn trigger_feed(pwm: &mut Pwm<'_>, config: &mut PwmConfig, at_max: &mut bool, expected_wt: &mut f32, in_prog: &mut bool, timer: &mut Instant, current_wt: f32) {
    info!("Feeding triggered!");
    if *at_max {
        config.compare_b = 1000;
        *at_max = false;
    } else {
        config.compare_b = 2000;
        *at_max = true;
    }
    pwm.set_config(config);
    
    *expected_wt = current_wt + 3.0;
    *in_prog = true;
    *timer = Instant::now();
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