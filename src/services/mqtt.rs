//! MQTT service: connection parameters, client lifecycle, callback dispatch, and subscriptions.

use anyhow::Result;
use embedded_svc::mqtt::client::{EventPayload, QoS};
use esp_idf_svc::mqtt::client::{EspMqttClient, MqttClientConfiguration};
use std::thread::sleep;
use std::time::Duration;

use crate::app::state::{
    get_sensor_values, SharedSensorData, SharedSsnDeviceCommand, SharedSsnMqttCommand,
};
use crate::protocol::commands;
use crate::protocol::topics;
use crate::services::config::{Config, Settings};

const MAX_RECONNECT_ATTEMPTS: u32 = 5;
const RECONNECT_DELAY_MS: u64 = 5000;

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

/// Create the MQTT client, subscribe to the command topics, and publish `status/start`.
///
/// The callback only parses and enqueues; publishing and device work happen on the main task.
pub fn start(
    app_config: &Config,
    settings: Settings,
    shared_data: SharedSensorData,
    device_commands: SharedSsnDeviceCommand,
    publish_commands: SharedSsnMqttCommand,
) -> Result<EspMqttClient<'static>> {
    let mqtt_config = MqttConfig::from_app_config(app_config);
    let client_config = mqtt_config.client_configuration();

    let account = app_config.account.clone();
    let object = app_config.object.clone();
    let account_ref = account.clone();
    let object_ref = object.clone();

    let mut settings = settings;
    let mut reconnect_attempts: u32 = 0;

    log::info!("SSN. Hello world!");

    let mut client = EspMqttClient::new_cb(
        &mqtt_config.broker_url,
        &client_config,
        move |message_event| {
            match message_event.payload() {
                // ********************************* process receive data from topic:
                EventPayload::Received { data, topic, .. } => {
                    log::info!("Received message on {:?}: {:?}", topic, data);
                    if let Some(topic) = topic {
                        if let Err(e) = commands::handle_message(
                            topic,
                            data,
                            &account_ref,
                            &object_ref,
                            &mut settings,
                            &device_commands,
                            &publish_commands,
                        ) {
                            log::warn!("Ignoring MQTT command on {}: {}", topic, e);
                        }
                    }

                    if let Some(vals) = get_sensor_values(&shared_data, "aht21-test") {
                        log::info!("sensor val: {:?}", vals);
                    };
                }
                EventPayload::Subscribed(id) => log::info!("Subscribed to id: {}", id),
                EventPayload::Disconnected => {
                    log::warn!("MQTT disconnected, attempting to reconnect...");
                    if reconnect_attempts < MAX_RECONNECT_ATTEMPTS {
                        sleep(Duration::from_millis(RECONNECT_DELAY_MS));
                        reconnect_attempts += 1;
                        // You'll need to recreate the client here
                        // This might require restructuring to store broker_url and config
                    } else {
                        log::error!("Max reconnection attempts reached");
                    }
                }
                EventPayload::BeforeConnect => {
                    log::info!("MQTT BeforeConnect");
                    sleep(Duration::from_millis(1000));
                }
                EventPayload::Connected(_) => {
                    log::info!("MQTT reconnected successfully");
                    reconnect_attempts = 0;
                }
                EventPayload::Error(e) => {
                    log::warn!("MQTT error: {:?}", e);
                    sleep(Duration::from_millis(1000));
                }
                _ => log::info!("Received from MQTT: {:?}", message_event.payload()),
            }
        },
    )?;

    sleep(Duration::from_millis(6000));
    // Subscribe to topics
    for topic in &mqtt_config.topics {
        log::info!("MQTT subscribe to {}", &topic);
        client.subscribe(topic, QoS::AtLeastOnce)?;
    }

    // publish start message
    client.enqueue(
        &topics::status_topic(&account, &object, "start"),
        QoS::AtLeastOnce,
        true,
        mqtt_config.client_id.as_bytes(),
    )?;

    Ok(client)
}
