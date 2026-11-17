//! Configuration bootstrap: NVS, optional reset, and MAC-derived identity overrides.

use anyhow::{Context, Result};
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, EspNvsPartition, NvsDefault};
use esp_idf_sys::esp_efuse_mac_get_default;

use crate::settings::{Config, Settings};

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

/// Open the default NVS partition and the `ssn_ns` namespace.
pub fn open_settings() -> Result<Settings> {
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
pub fn load_config(settings: &mut Settings) -> Result<Config> {
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
