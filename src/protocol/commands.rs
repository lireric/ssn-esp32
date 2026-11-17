//! MQTT command handling: parse an inbound topic/payload and enqueue work for the main task.
//!
//! This runs on the MQTT client task, so it only parses and enqueues; it never performs sensor
//! reads or long NVS writes (except the existing `get` read, which is unchanged).
//!
//! Parse failures are returned as [`CommandError`] and logged by the caller. They never panic and
//! never reboot the node: the message is dropped and the previous state is kept. See
//! `ARCHITECTURE.md` §17 ("Degraded modes").

use std::fmt;

use crate::app::state::{
    push_command, push_mqtt_command, SharedSsnDeviceCommand, SharedSsnMqttCommand, SsnDeviceCommand,
    SsnMqttCommand,
};
use crate::protocol::topics;
use crate::services::config::Settings;

/// A malformed or unsupported inbound command.
#[derive(Debug)]
pub enum CommandError {
    /// Payload is not valid UTF-8.
    InvalidUtf8,
    /// Topic does not match the `/ssn/acc/{account}/...` shape.
    InvalidTopic(String),
    /// A token that the command shape requires is missing.
    MissingToken(&'static str),
    /// Device channel is not a `u8`.
    InvalidChannel(String),
    /// Command payload is not an `f32`.
    InvalidValue(String),
    /// Config could not be read for the `get` command.
    Config(String),
}

impl fmt::Display for CommandError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CommandError::InvalidUtf8 => write!(f, "payload is not valid UTF-8"),
            CommandError::InvalidTopic(topic) => write!(f, "invalid topic: {}", topic),
            CommandError::MissingToken(token) => write!(f, "missing token: {}", token),
            CommandError::InvalidChannel(channel) => {
                write!(f, "invalid channel: {}", channel)
            }
            CommandError::InvalidValue(value) => write!(f, "invalid value: {:?}", value),
            CommandError::Config(e) => write!(f, "config read failed: {}", e),
        }
    }
}

impl std::error::Error for CommandError {}

/// Handle a message received on `topic`, filtered to `account`.
///
/// Errors describe malformed input or an unavailable config; the caller logs and drops the
/// message.
pub fn handle_message(
    topic: &str,
    data: &[u8],
    account: &str,
    object: &str,
    settings: &mut Settings,
    device_commands: &SharedSsnDeviceCommand,
    publish_commands: &SharedSsnMqttCommand,
) -> Result<(), CommandError> {
    let payload_str = std::str::from_utf8(data).map_err(|_| CommandError::InvalidUtf8)?;
    log::info!("payload = {:?}", payload_str);

    let Some((account_rcv, root_token, subtokens)) = topics::parse_topic(topic) else {
        return Err(CommandError::InvalidTopic(topic.to_string()));
    };

    log::info!(
        "account_rcv = {}, root_token = {}, subtokens = {:?}",
        account_rcv,
        root_token,
        subtokens
    );

    // filter only our account
    if !account.eq(account_rcv) {
        return Ok(());
    }

    let Some(parsed) = topics::parse_token_array(root_token, &subtokens) else {
        log::warn!("Failed to parse tokens for topic: {}", topic);
        return Ok(());
    };

    log::info!("parsed subtokens HashMap = {:?}", parsed);
    let sub_token = *parsed
        .get("subToken")
        .ok_or(CommandError::MissingToken("subToken"))?;

    match sub_token {
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
                            .map_err(|e| CommandError::Config(e.to_string()))?;
                        // Use Display (password-masked HTML), not Debug.
                        log::info!("config = {}", cur_config);
                        let _ = push_mqtt_command(
                            publish_commands,
                            SsnMqttCommand {
                                topic: topics::publish_command_response(account, object, command)
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
                    let device = *parsed
                        .get("device")
                        .ok_or(CommandError::MissingToken("device"))?;
                    let channel = *parsed
                        .get("channel")
                        .ok_or(CommandError::MissingToken("channel"))?;
                    let action = *parsed
                        .get("action")
                        .ok_or(CommandError::MissingToken("action"))?;

                    let channel_u8 = channel
                        .parse::<u8>()
                        .map_err(|_| CommandError::InvalidChannel(channel.to_string()))?;

                    match action {
                        "in" => {
                            // process set new device value
                            let value = payload_str
                                .parse::<f32>()
                                .map_err(|_| CommandError::InvalidValue(payload_str.to_string()))?;
                            log::info!("Parsed float: {}", value);
                            let _ = push_command(
                                device_commands,
                                SsnDeviceCommand {
                                    dev_id: device.to_string(),
                                    channel: channel_u8,
                                    value,
                                },
                            );
                        }
                        "reset" => {
                            // TO DO
                        }
                        &_ => {}
                    }
                }
            }
        }
        &_ => {}
    }

    Ok(())
}
