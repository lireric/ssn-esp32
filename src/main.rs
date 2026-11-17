//! `ssn-esp32` firmware entry point.
//!
//! Only module wiring and the reboot loop live here; lifecycle and services are in `app`.

mod app;
mod devices;
mod drivers;
mod platform;
mod protocol;
mod services;

use anyhow::Result;
use esp_idf_sys::{self as _, esp_restart};

fn main() -> Result<()> {
    loop {
        match app::run() {
            Ok(_) => {
                println!("Main logic completed successfully.");
                // Optionally, break or continue based on your logic
                // break; // Exit the loop if main_logic succeeds
            }
            Err(e) => {
                println!("Error occurred: {}", e);
                println!("Rebooting...");

                // Reboot the ESP32 only on error
                unsafe {
                    esp_restart();
                }
            }
        }
    }
}
