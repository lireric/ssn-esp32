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

pub fn publish_command_response(acc: &str, obj: &str, command: &str) -> String {
    format!("/ssn/acc/{}/obj/{}/commands/response/{}", acc, obj, command)
}

// /// String split to vector
// fn csplit(input: &str, sep: char) -> Vec<String> {
//     input
//         .split(sep)
//         .filter(|s| !s.is_empty())
//         .map(|s| s.to_string())
//         .collect()
// }

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
pub fn parse_topic(topic: &str) -> Option<(&str, &str, Vec<&str>)> {
    log::debug!("parse_topic start");
    let topic_array: Vec<&str> = topic.split('/').collect();

    // Determine offset and account based on topic prefix pattern
    let (offset, account) = match &topic_array.as_slice() {
        ["", "ssn", "acc", account, ..] => (4, account),
        ["ssn", "acc", account, ..] => (3, account),
        _ => {
            log::debug!("parse_topic: invalid topic format");
            return None;
        }
    };

    // Ensure we have enough elements for root_token
    if topic_array.len() <= offset {
        log::debug!("parse_topic: topic too short");
        return None;
    }

    let root_token = topic_array[offset];
    let subtokens = slice(&topic_array, offset + 1, None, 1);

    log::debug!(
        "parse_topic result - size: {}, account: [{}], offset: {}, root_token: {}",
        topic_array.len(),
        account,
        offset,
        root_token
    );

    Some((account, root_token, subtokens))
}

/// Parse tokens array after parsing topic
pub fn parse_token_array<'a>(
    root_token: &'a str,
    sub_tokens: &Vec<&'a str>,
) -> Option<HashMap<&'a str, &'a str>> {
    log::debug!(
        "parse_token_array. root_token={}, sub_tokens len={}",
        root_token,
        sub_tokens.len()
    );

    let mut res = HashMap::from([("rootToken", root_token)]);

    match root_token {
        "raw_data" => log::info!("Process raw_data"),
        "obj" => {
            log::info!("Process obj");
            if let (Some(obj), Some(sub_token)) = (sub_tokens.first(), sub_tokens.get(1)) {
                // if let Ok(obj_num) = obj.parse::<u32>() {
                res.insert("obj", obj);
                res.insert("subToken", sub_token);

                match (sub_token, sub_tokens.len()) {
                    (&"commands", 3) => {
                        log::info!("Process obj -> commands 3");
                        // process json command
                        res.insert("command", sub_tokens[2]);
                    }
                    (&"commands", 6) => {
                        log::info!("Process obj -> commands 6");
                        res.extend([
                            ("device", sub_tokens[3]),
                            ("channel", sub_tokens[4]),
                            ("action", sub_tokens[5]),
                        ]);
                    }
                    _ => {
                        log::info!("Process obj -> other");
                    }
                }
                // }
            }
        }
        "bot" => {
            log::info!("telegram bot");
            if let Some(sub_token) = sub_tokens.get(1) {
                res.insert("subToken", sub_token);
            }
        }
        _ => {}
    }

    (res.len() > 1).then_some(res)
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

// pub fn process_message(topic: Option<&str>, data: &[u8]) {}
