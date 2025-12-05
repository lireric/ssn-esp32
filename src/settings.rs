use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, EspNvsPartition, NvsDefault};
use log;
use serde::{Deserialize, Serialize};
use std::fmt;
use std::io::{Error, ErrorKind};

// Import the DeviceConfig from ssndevices module
use crate::ssndevices::DeviceConfig;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Config {
    pub account: String,
    pub object: String,
    pub ssn_id: String,
    pub mqtt_host: String,
    pub mqtt_user: String,
    pub mqtt_pass: String,
    pub wifi_ssid: String,
    pub wifi_psk: String,
    pub mqtt_client_id: String,
    pub ssn_wifi_ssid: String,
    pub ssn_wifi_pass: String,
    pub devices: Vec<DeviceConfig>,
}

pub struct Settings {
    nvs: EspNvs<NvsDefault>,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            account: "3".to_string(),
            object: "2".to_string(), // esp32-test
            mqtt_host: "192.168.1.105".to_string(),
            mqtt_user: "mosquitto".to_string(),
            mqtt_pass: "test".to_string(),
            wifi_ssid: "lir".to_string(),
            wifi_psk: "springrain".to_string(),
            mqtt_client_id: "esp_client_id_12345".to_string(),
            ssn_id: "123".to_string(),
            ssn_wifi_ssid: "ssn".to_string(),
            ssn_wifi_pass: "ssn123456".to_string(),
            devices: Vec::new(), // Empty devices array by default
        }
    }
}

impl fmt::Display for Config {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut html = String::from("<ul>");

        // // Use serde_json to serialize and iterate fields
        // if let Ok(value) = serde_json::to_value(self) {
        //     if let Some(obj) = value.as_object() {
        //         for (field, val) in obj {
        //             if let Some(str_val) = val.as_str() {
        //                 html.push_str(&format!("<li>{}: {}</li>", field, str_val));
        //             }
        //         }
        //     }
        // }
        // Add basic fields
        html.push_str(&format!("<li>account: {}</li>", self.account));
        html.push_str(&format!("<li>object: {}</li>", self.object));
        html.push_str(&format!("<li>ssn_id: {}</li>", self.ssn_id));
        html.push_str(&format!("<li>mqtt_host: {}</li>", self.mqtt_host));
        html.push_str(&format!("<li>mqtt_user: {}</li>", self.mqtt_user));
        html.push_str(&format!("<li>mqtt_pass: [HIDDEN]</li>"));
        html.push_str(&format!("<li>wifi_ssid: {}</li>", self.wifi_ssid));
        html.push_str(&format!("<li>wifi_psk: [HIDDEN]</li>"));
        html.push_str(&format!("<li>mqtt_client_id: {}</li>", self.mqtt_client_id));
        html.push_str(&format!("<li>ssn_wifi_ssid: {}</li>", self.ssn_wifi_ssid));
        html.push_str(&format!("<li>ssn_wifi_pass: [HIDDEN]</li>"));

        // Add devices section
        html.push_str(&format!("<li>devices: [{} items]</li>", self.devices.len()));
        for (i, device) in self.devices.iter().enumerate() {
            html.push_str(&format!(
                "<li>  Device {} ({}): {:?}</li>",
                i, device.dev_id, device.device_type
            ));
        }

        html.push_str("</ul>");
        write!(f, "{}", html)
    }
}

impl Settings {
    pub fn new(nvs: EspNvs<NvsDefault>) -> Self {
        Settings { nvs }
    }
    pub fn get_config(&mut self) -> Result<(Config, usize), Error> {
        let mut buf = [0u8; 1024];

        match self.nvs.get_blob("config", &mut buf) {
            Ok(Some(blob)) => {
                log::info!("Config found in NVS, size: {}", blob.len());
                match serde_json::from_slice(blob) {
                    Ok(config) => {
                        log::debug!("Loaded config: {:?}", config);
                        Ok((config, blob.len()))
                    }
                    Err(e) => {
                        log::error!("Config deserialization failed: {}", e);
                        self.set_default_config()
                    }
                }
            }
            _ => {
                log::warn!("No config found, creating default");
                self.set_default_config()
            }
        }
    }

    pub fn set_config_from_json(&mut self, json: &str) -> Result<(Config, usize), Error> {
        // First validate the JSON by deserializing it
        let config: Config = serde_json::from_str(&json).map_err(|e| {
            log::error!("Invalid config JSON: {}", e);
            Error::new(ErrorKind::InvalidData, e)
        })?;

        log::info!("*** config: {}", config);

        // Convert string to bytes for storage
        let json_bytes = json.as_bytes();

        // Store the validated config
        self.nvs.set_blob("config", json_bytes).map_err(|e| {
            log::error!("Failed to store config: {}", e);
            Error::new(ErrorKind::Other, e)
        })?;

        log::info!("Config stored, size: {}", json_bytes.len());
        Ok((config, json_bytes.len()))
    }

    pub fn set_default_config(&mut self) -> Result<(Config, usize), Error> {
        let default_config = Config::default();
        let json = serde_json::to_string(&default_config).map_err(|e| {
            log::error!("Failed to serialize default config: {}", e);
            Error::new(ErrorKind::InvalidData, e)
        })?;

        self.set_config_from_json(&json)
    }
}
