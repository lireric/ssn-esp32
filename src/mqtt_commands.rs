//! MQTT command handling: parse an inbound topic/payload and enqueue work for the main task.
//!
//! This runs on the MQTT client task, so it only parses and enqueues; it never performs sensor
//! reads or long NVS writes (except the existing `get` read, which is unchanged).

use crate::mqtt_messages;
use crate::settings::Settings;
use crate::shared_state::{
    push_command, push_mqtt_command, SharedSsnDeviceCommand, SharedSsnMqttCommand, SsnDeviceCommand,
    SsnMqttCommand,
};

/// Handle a message received on `topic`, filtered to `account`.
pub fn handle_message(
    topic: &str,
    data: &[u8],
    account: &str,
    object: &str,
    settings: &mut Settings,
    device_commands: &SharedSsnDeviceCommand,
    publish_commands: &SharedSsnMqttCommand,
) {
    // NOTE: invalid UTF-8 panics here; preserved from the original callback. See
    // `ARCHITECTURE.md` §19 item 10.
    let payload_str = std::str::from_utf8(data).unwrap();
    log::info!("payload = {:?}", payload_str);

    if let Some((account_rcv, root_token, subtokens)) = mqtt_messages::parse_topic(topic) {
        log::info!(
            "account_rcv = {}, root_token = {}, subtokens = {:?}",
            account_rcv,
            root_token,
            subtokens
        );
        // filter only our account
        if account.eq(account_rcv) {
            if let Some(parsed) = mqtt_messages::parse_token_array(root_token, &subtokens) {
                log::info!("parsed subtokens HashMap = {:?}", parsed);
                match *parsed.get("subToken").unwrap() {
                    // process hi level commands:
                    "commands" => {
                        if let Some(command) = parsed.get("command") {
                            log::info!("command = {:?}", command);
                            match *command {
                                "ini" => {
                                    log::info!("command ini ...");
                                }
                                "get" => {
                                    log::info!("command get config ...");
                                    let (cur_config, _app_config_size) = settings
                                        .get_config()
                                        .unwrap_or_else(|e| {
                                            panic!("Failed to get config: {}", e)
                                        });
                                    // Use Display (password-masked HTML), not Debug.
                                    log::info!("config = {}", cur_config);
                                    let _ = push_mqtt_command(
                                        publish_commands,
                                        SsnMqttCommand {
                                            topic: mqtt_messages::publish_command_response(
                                                account, object, command,
                                            )
                                            .to_string(),
                                            message: cur_config.to_string(),
                                        },
                                    );
                                }
                                &_ => {
                                    log::info!("another command ...");
                                }
                            }
                        } else {
                            log::info!("command empty...");
                            // process device command (this case has 6 subtokens):
                            if subtokens.len() == 6 {
                                // process device operations:
                                let device = *parsed.get("device").unwrap();
                                let channel = *parsed.get("channel").unwrap();
                                match channel.parse::<u8>() {
                                    Ok(channel_u8) => match *parsed.get("action").unwrap() {
                                        "in" => {
                                            // process set new device value
                                            match payload_str.parse::<f32>() {
                                                Ok(val) => {
                                                    log::info!("Parsed float: {}", val);
                                                    let _ = push_command(
                                                        device_commands,
                                                        SsnDeviceCommand {
                                                            dev_id: device.to_string(),
                                                            channel: channel_u8,
                                                            value: val,
                                                        },
                                                    );
                                                }
                                                Err(e) => {
                                                    log::info!("Error parsing float: {}", e)
                                                }
                                            }
                                        }
                                        "reset" => {
                                            // TO DO
                                        }
                                        &_ => {}
                                    },
                                    // NOTE: non-numeric channel panics here; preserved from the
                                    // original callback (`ARCHITECTURE.md` §19 item 10).
                                    Err(_) => todo!(),
                                }
                            }
                        }
                    }
                    &_ => {}
                }
            } else {
                log::warn!("Failed to parse tokens for topic: {}", topic);
            }
        }
    }
}
