//! Protocol layer: topics, commands, payload types.

pub mod commands;
pub mod topics;

/// Legacy device/channel/value triple.
///
/// Currently unused (`ARCHITECTURE.md` §19 item 17); kept as a placeholder for a typed payload.
pub struct ValuesDev {
    pub dev: String,
    pub channel: String,
    pub val: String,
}
