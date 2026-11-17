//! MQTT client lifecycle: callback dispatch, subscriptions, and the retained start message.

use anyhow::Result;
use embedded_svc::mqtt::client::{EventPayload, QoS};
use esp_idf_svc::mqtt::client::EspMqttClient;
use std::thread::sleep;
use std::time::Duration;

use crate::mqtt_commands;
use crate::mqtt_config::MqttConfig;
use crate::mqtt_messages;
use crate::settings::{Config, Settings};
use crate::shared_state::{
    get_sensor_values, SharedSensorData, SharedSsnDeviceCommand, SharedSsnMqttCommand,
};

const MAX_RECONNECT_ATTEMPTS: u32 = 5;
const RECONNECT_DELAY_MS: u64 = 5000;

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
                        mqtt_commands::handle_message(
                            topic,
                            data,
                            &account_ref,
                            &object_ref,
                            &mut settings,
                            &device_commands,
                            &publish_commands,
                        );
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
        &mqtt_messages::status_topic(&account, &object, "start"),
        QoS::AtLeastOnce,
        true,
        mqtt_config.client_id.as_bytes(),
    )?;

    Ok(client)
}
