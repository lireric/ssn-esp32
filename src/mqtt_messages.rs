use std::{
    borrow::{Borrow, Cow},
    str,
};

pub fn temperature_data_topic(uuid: &str) -> String {
    format!("{}/sensor_data/temperature", uuid)
}

pub fn pressure_data_topic(uuid: &str) -> String {
    format!("{}/sensor_data/pressure", uuid)
}

pub fn hello_topic(uuid: &str) -> String {
    format!("{}/hello", uuid)
}
