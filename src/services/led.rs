//! WS2812 status LED.

use ws2812_esp32_rmt_driver::driver::color::{LedPixelColor, LedPixelColorGrb24};
use ws2812_esp32_rmt_driver::driver::Ws2812Esp32RmtDriver;

pub enum Colors {
    Red,
    Green,
    Blue,
    Yellow,
    White,
    Black,
}

/// Set the on-board status LED to `col`. Blocking; brightness is capped at 30/255.
///
/// The LED is best-effort: a driver error is logged and otherwise ignored.
pub fn status_led(led: &mut Ws2812Esp32RmtDriver, col: Colors) {
    let color = match col {
        Colors::Red => LedPixelColorGrb24::new_with_rgb(30, 0, 0),
        Colors::Green => LedPixelColorGrb24::new_with_rgb(0, 30, 0),
        Colors::Blue => LedPixelColorGrb24::new_with_rgb(0, 0, 30),
        Colors::Yellow => LedPixelColorGrb24::new_with_rgb(30, 30, 0),
        Colors::White => LedPixelColorGrb24::new_with_rgb(30, 30, 30),
        Colors::Black => LedPixelColorGrb24::new_with_rgb(0, 0, 0),
    };
    let Ok(pixel) = <[u8; 3]>::try_from(color.as_ref()) else {
        log::error!("status LED: unexpected pixel length");
        return;
    };
    if let Err(e) = led.write_blocking(pixel.into_iter()) {
        log::warn!("status LED write failed: {:?}", e);
    }
}
