//! Configuration service: schema, NVS persistence, reset path, and boot overrides.

use anyhow::Context;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, EspNvsPartition, NvsDefault};
use esp_idf_sys::esp_efuse_mac_get_default;
use log;
use serde::{Deserialize, Serialize};
use std::fmt;
use std::io::{Error, ErrorKind};

use crate::devices::model::DeviceConfig;

const NVS_NAMESPACE: &str = "ssn_ns";

/// Test configuration written after a factory reset.
///
/// KNOWN DEBT: these are test credentials and must be removed or feature-gated
/// (`ARCHITECTURE.md` §19 item 6). Kept verbatim here so this refactor does not change the
/// reset-path behavior.
const TEST_CONFIG_JSON: &str = r#"{
  "account": "3",
  "object": "2",
  "ssn_id": "123",
  "mqtt_host": "192.168.1.105",
  "mqtt_user": "mosquitto",
  "mqtt_pass": "test",
  "wifi_ssid": "lir",
  "wifi_psk": "springrain",
  "mqtt_client_id": "esp_client_id_12345",
  "ssn_wifi_ssid": "ssn",
  "ssn_wifi_pass": "ssn123456",
  "devices": [
    {
      "dev_id": "temperature_sensor",
      "device_type": "Bmp180",
      "period": "Middle",
      "is_active": true,
      "is_paused": false,
      "temperature_delta_over": 0.5,
      "pressure_delta_over": 1.0
    },
    {
      "dev_id": "co2_sensor",
      "device_type": "Ens160",
      "period": "Fast",
      "is_active": true,
      "is_paused": false,
      "co2eq_delta_over": 10.0,
      "tvoc_delta_over": 5.0
    }
  ]
}"#;

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

/// Open the default NVS partition and the `ssn_ns` namespace.
pub fn open_settings() -> anyhow::Result<Settings> {
    let nvs_default_partition: EspNvsPartition<NvsDefault> =
        EspDefaultNvsPartition::take().context("default NVS partition not available")?;
    log::info!("Getting settings from NVS. nvs_namespace={}", NVS_NAMESPACE);

    let nvs = match EspNvs::new(nvs_default_partition, NVS_NAMESPACE, true) {
        Ok(nvs) => {
            log::info!("Got namespace {} from default partition", NVS_NAMESPACE);
            nvs
        }
        Err(e) => anyhow::bail!("Could't get namespace {}: {}", NVS_NAMESPACE, e),
    };

    Ok(Settings::new(nvs))
}

/// If the settings-reset button was held at boot, write defaults and then the test config.
pub fn apply_reset(settings: &mut Settings, reset_pressed: bool) {
    if reset_pressed {
        log::warn!("Reset settings button pushed! Fill defaults values...");
        let _ = settings.set_default_config();
        let _ = settings.set_config_from_json(TEST_CONFIG_JSON);
    }
}

/// Load the persisted config and apply the identity overrides that are never persisted.
pub fn load_config(settings: &mut Settings) -> anyhow::Result<Config> {
    let (mut app_config, app_config_size) = settings
        .get_config()
        .map_err(|e| anyhow::anyhow!("Failed to get config: {}", e))?;

    // ********************************************
    // override default settings:
    app_config.account = "3".to_string();
    app_config.object = "2".to_string();
    app_config.ssn_id = get_mac_address();

    app_config.ssn_wifi_ssid = format!("ssn_{}", app_config.ssn_id);
    app_config.ssn_wifi_pass = "ssn123456".to_string();
    app_config.mqtt_client_id = format!("ssn_esp_{}", app_config.ssn_id);
    log::info!(
        "acc={}, obj={}, ssn_id = {}, mqtt_client_id={}",
        app_config.account,
        app_config.object,
        app_config.ssn_id,
        app_config.mqtt_client_id
    );

    log::info!(
        "Retrieved config: {}, size={}",
        app_config.clone(),
        app_config_size
    );

    Ok(app_config)
}

fn get_mac_address() -> String {
    let mut mac_address = [0u8; 6];

    // Read base MAC from eFuse
    unsafe { esp_efuse_mac_get_default(mac_address.as_mut_ptr()) };
    format!(
        "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
        mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4],
        mac_address[5]
    )
}
