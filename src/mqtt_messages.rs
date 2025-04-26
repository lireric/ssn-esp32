use anyhow::{Context, Result};
use esp_idf_svc::mqtt::client::{Details, EspMqttClient, MqttClientConfiguration, QoS};
use regex::Regex;
use std::{
    borrow::{Borrow, Cow},
    collections::HashMap,
    str,
};
// mod commons;
use crate::commons::ValuesDev;

pub fn status_topic(acc: &str, obj: &str, event: &str) -> String {
    format!("/ssn/acc/{}/obj/{}/status/{}", acc, obj, event)
}

pub fn publish_sensor_topic(acc: &str, obj: &str, dev: &str, channel: &str) -> String {
    format!(
        "/ssn/acc/{}/obj/{}/device/{}/{}/out",
        acc, obj, dev, channel
    )
}

// // Publish array of the collected sensors values
// pub fn publish_sensors_array(
//     mut client: EspMqttClient,
//     acc: &str,
//     obj: &str,
//     values_array: Vec<ValuesDev>,
// ) -> u16 {
//     let mut counter: u16 = 0;
//     for sensor in values_array {
//         client.enqueue(
//             &publish_sensor_topic(&acc, &obj, sensor.dev.as_str(), sensor.channel.as_str(), ),
//             QoS::AtLeastOnce,
//             false,
//             sensor.val.as_bytes(),
//         ).context("Error at MQTT sending");
//         counter += 1;
//     }

//     (counter)
// }
// function ssnmqtt:publishSensorValue(obj, dev, channel, value, ts, action)
//   logger:debug("Set device value (obj=%d): d[%s,%d]=%s", obj, tostring(dev), channel, value)

//   local dev_topic = "/ssn/acc/"..tostring(self.account).."/obj/"..tostring(obj).."/device/"..tostring(dev).."/"..tostring(channel).."/out"

/// String split to vector
fn csplit(input: &str, sep: char) -> Vec<String> {
    input
        .split(sep)
        .filter(|s| !s.is_empty())
        .map(|s| s.to_string())
        .collect()
}

/// Slice array
fn slice<T: Clone>(tbl: &[T], first: usize, last: Option<usize>, step: usize) -> Vec<T> {
    let last = last.unwrap_or(tbl.len());
    tbl.iter()
        .skip(first)
        .take(last - first)
        .step_by(step)
        .cloned()
        .collect()
}

/// Parse topic string into components
/// Returns: (account, root_token, subtokens)
pub fn parse_topic(topic: &str) -> Option<(u32, String, Vec<String>)> {
    log::debug!("parse_topic start");
    let topic_array = csplit(topic, '/');
    let offset = if topic_array.get(0) == Some(&"".to_string())
        && topic_array.get(1) == Some(&"ssn".to_string())
        && topic_array.get(2) == Some(&"acc".to_string())
    {
        4 // if topic like "/ssn/acc..."
    } else if topic_array.get(0) == Some(&"ssn".to_string())
        && topic_array.get(1) == Some(&"acc".to_string())
    {
        3 // if topic like "ssn/acc..."
    } else {
        return None;
    };

    let account = topic_array.get(offset - 1)?.parse::<u32>().ok()?;
    let root_token = topic_array.get(offset)?.to_string();
    let subtokens = slice(&topic_array, offset + 1, None, 1);

    log::debug!(
        "parse_topic. size={}, account [{}] offset={} root_token={}",
        topic_array.len(),
        account,
        offset,
        root_token
    );

    Some((account, root_token, subtokens))
}

/// Parse tokens array after parsing topic
pub fn parse_token_array(
    root_token: &str,
    sub_tokens: &[String],
) -> Option<HashMap<String, String>> {
    log::debug!(
        "parse_token_array. root_token={}, sub_tokens len={}",
        root_token,
        sub_tokens.len()
    );

    let mut res = HashMap::new();
    res.insert("rootToken".to_string(), root_token.to_string());

    match root_token {
        "raw_data" => {
            log::info!("Process raw_data");
            // TODO: Implement raw_data processing
        }
        "obj" => {
            log::info!("Process obj");
            if !sub_tokens.is_empty() {
                let obj = sub_tokens.get(0)?.parse::<u32>().ok()?;
                res.insert("obj".to_string(), obj.to_string());

                if let Some(sub_token) = sub_tokens.get(1) {
                    res.insert("subToken".to_string(), sub_token.clone());

                    match sub_tokens.len() {
                        2 if sub_token == "commands" => {
                            log::info!("Process commands");
                        }
                        3 if sub_token == "commands" => {
                            res.insert("command".to_string(), sub_tokens[2].clone());
                        }
                        5 if sub_token == "device" => {
                            res.insert("device".to_string(), sub_tokens[2].clone());
                            res.insert("channel".to_string(), sub_tokens[3].clone());
                            res.insert("action".to_string(), sub_tokens[4].clone());
                        }
                        _ => {}
                    }
                }
            }
        }
        "bot" => {
            log::info!("telegram bot");
            if let Some(sub_token) = sub_tokens.get(1) {
                res.insert("subToken".to_string(), sub_token.clone());
            }
        }
        _ => {}
    }

    if res.len() > 1 {
        // More than just rootToken
        Some(res)
    } else {
        None
    }
}

/// Fill map of devices: {key: dev_type}
pub fn fill_device_types_map(
    conf_sensors: &HashMap<String, serde_json::Value>,
) -> HashMap<String, String> {
    let mut device_map = HashMap::new();

    for (dev_type, val) in conf_sensors {
        match dev_type.as_str() {
            "bmp180" | "gpio" => {
                if let Some(id) = val["id"].as_str() {
                    device_map.insert(id.to_string(), dev_type.clone());
                }
            }
            "ds18b20" => {
                if let Some(devices) = val["devices"].as_array() {
                    for device in devices {
                        if let Some(id) = device["id"].as_str() {
                            device_map.insert(id.to_string(), dev_type.clone());
                        }
                    }
                }
            }
            "watchdog_tcp" => {
                if let Some(destinations) = val["destinations"].as_array() {
                    for dest in destinations {
                        if let Some(id) = dest["id"].as_str() {
                            device_map.insert(id.to_string(), dev_type.clone());
                        }
                    }
                }
            }
            _ => {}
        }
    }

    log::info!("DeviceTypesMap filled, size: {}", device_map.len());
    device_map
}

pub fn process_message(topic: Option<&str>, data: &[u8]) {}
