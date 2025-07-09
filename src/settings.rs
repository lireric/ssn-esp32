use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, EspNvsPartition, NvsDefault};
use log::{info, warn};
use serde::{Deserialize, Serialize};
use std::fmt;
use std::io::{Error, ErrorKind};

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
}

pub struct Settings {
    nvs: EspNvs<NvsDefault>,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            account: "3".to_string(),
            object: "2".to_string(), // esp32-test
            mqtt_host: "192.168.3.150".to_string(),
            mqtt_user: "mosquitto".to_string(),
            mqtt_pass: "test".to_string(),
            wifi_ssid: "lir2".to_string(),
            wifi_psk: "springrain".to_string(),
            mqtt_client_id: "esp_client_id_12345".to_string(),
            ssn_id: "123".to_string(),
            ssn_wifi_ssid: "ssn".to_string(),
            ssn_wifi_pass: "ssn123456".to_string(),
        }
    }
}

impl fmt::Display for Config {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut html = String::from("<ul>");

        // Use a macro to generate the HTML for each field
        macro_rules! add_field {
            ($field:ident) => {
                html.push_str(&format!("<li>{}: {}</li>", stringify!($field), self.$field));
            };
        }

        // Add each field to the HTML
        add_field!(account);
        add_field!(object);
        add_field!(mqtt_host);
        add_field!(mqtt_user);
        add_field!(mqtt_pass);
        add_field!(wifi_ssid);
        add_field!(wifi_psk);
        add_field!(mqtt_client_id);

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

    fn set_default_config(&mut self) -> Result<(Config, usize), Error> {
        let default_config = Config::default();
        let json = serde_json::to_vec(&default_config).map_err(|e| {
            log::error!("Failed to serialize default config: {}", e);
            Error::new(ErrorKind::InvalidData, e)
        })?;
        self.nvs.set_blob("config", &json).map_err(|e| {
            log::error!("Failed to store default config: {}", e);
            Error::new(ErrorKind::Other, e)
        })?;
        log::info!("Default config stored, size: {}", json.len());
        Ok((default_config, json.len()))
    }
}
