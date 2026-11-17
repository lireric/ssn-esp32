//! MQTT connection parameters derived from the persisted configuration.

use esp_idf_svc::mqtt::client::MqttClientConfiguration;

use crate::settings::Config;

/// Broker URL, client ID and subscription list.
///
/// The `MqttClientConfiguration` itself is intentionally not stored here: it borrows the client
/// ID, so it is built at the call site where the client is created.
pub struct MqttConfig {
    pub broker_url: String,
    pub client_id: String,
    pub topics: Vec<String>,
}

impl MqttConfig {
    pub fn from_app_config(app_config: &Config) -> Self {
        let broker_url = if !app_config.mqtt_user.is_empty() {
            format!(
                "mqtt://{}:{}@{}",
                app_config.mqtt_user, app_config.mqtt_pass, app_config.mqtt_host
            )
        } else {
            format!("mqtt://{}", app_config.mqtt_host)
        };

        let client_id = app_config.mqtt_client_id.clone();
        let topics = vec![
            format!("/ssn/acc/{}/raw_data", &app_config.account),
            format!(
                "/ssn/acc/{}/obj/{}/commands/+",
                &app_config.account, &app_config.object
            ),
            format!(
                "/ssn/acc/{}/obj/{}/commands/device/+/+/in",
                &app_config.account, &app_config.object
            ),
        ];

        Self {
            broker_url,
            client_id,
            topics,
        }
    }

    /// Build the esp-idf client configuration for this connection.
    pub fn client_configuration<'a>(&'a self) -> MqttClientConfiguration<'a> {
        MqttClientConfiguration {
            client_id: Some(self.client_id.as_str()),
            ..Default::default()
        }
    }
}
