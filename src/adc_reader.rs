use anyhow::{Context, Result};
use esp_idf_hal::{
    adc::{
        attenuation::DB_11, oneshot::config::AdcChannelConfig, oneshot::AdcChannelDriver,
        oneshot::AdcDriver, oneshot::*, ADC1,
    },
    gpio::{ADCPin, Gpio0, Gpio1, Gpio2, Gpio3, Gpio4, Gpio5},
};
use std::sync::Arc;
// use esp_idf_hal::gpio::Pins;
// use esp_idf_hal::gpio::{AnyIOPin, Pin};
use esp_idf_hal::peripherals::Peripherals; // Concrete ADC-capable pin
use esp_idf_svc::hal::peripheral::Peripheral;
// use esp_idf_svc::sys::EspError;

pub struct AdcReader<P: ADCPin<Adc = ADC1>> {
    // adc: AdcDriver<'static, ADC1>,
    adc_pin: AdcChannelDriver<'static, P, AdcDriver<'static, ADC1>>,
}

impl<P: ADCPin<Adc = ADC1>> AdcReader<P> {
    pub fn new(gpio_pin: P, adc1: impl Peripheral<P = ADC1> + 'static) -> anyhow::Result<Self> {
        // let pins = peripherals.pins;
        log::info!("Initialize the ADC");
        // let peripherals = Peripherals::take().context("Failed to take peripherals")?;
        log::info!("Initialize the Gpio");
        // let adc_pin = peripherals.pins.gpio4;
        // let adc_pin = *pins.gpio4;
        // let adc = adc;

        log::info!("Initializing the AdcDriver..");
        // let adc = AdcDriver::new(peripherals.adc1).context("Failed to initialize ADC driver")?;
        let adc = AdcDriver::new(adc1).context("Failed to initialize ADC driver")?;
        log::info!("Initializing the AdcChannelConfig..");
        let config = AdcChannelConfig {
            attenuation: DB_11,
            ..Default::default()
        };
        log::info!("Initializing the AdcChannelDriver for selected pin..");
        let adc_pin = AdcChannelDriver::new(adc, gpio_pin, &config)
            .context("Failed to initialize ADC channel")?;

        Ok(Self { adc_pin })
    }

    pub fn read(&mut self) -> Result<(u16, f32)> {
        let raw_value = self.adc_pin.read()?;
        let voltage_mv = (raw_value as f32 * 2500.0) / 4095.0; // Approximate conversion
        Ok((raw_value, voltage_mv))
    }
}
