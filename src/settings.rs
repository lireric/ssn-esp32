use bincode::{config, Decode, Encode};
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, EspNvsPartition, NvsDefault};
use log::{info, warn};
use serde::{Deserialize, Serialize};
use std::fmt;

#[derive(Serialize, Deserialize, Debug, Encode, Decode, Clone)]

pub struct Config {
    pub mqtt_host: String,
    pub mqtt_user: String,
    pub mqtt_pass: String,
    pub wifi_ssid: String,
    pub wifi_psk: String,
    pub mqtt_client_id: String,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            mqtt_host: "192.168.1.105".to_string(),
            mqtt_user: "mosquitto".to_string(),
            mqtt_pass: "test".to_string(),
            wifi_ssid: "lir".to_string(),
            wifi_psk: "springrain".to_string(),
            mqtt_client_id: "esp_client_id_12345".to_string(),
        }
    }
}

impl fmt::Display for Config {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut html = String::from("<ul>");

        // Use a macro to generate the HTML for each field
        macro_rules! add_field {
            ($field:ident) => {
                html.push_str(&format!("<li>{}: {}</li>", stringify!($field), self.$field));
            };
        }

        // Add each field to the HTML
        add_field!(mqtt_host);
        add_field!(mqtt_user);
        add_field!(mqtt_pass);
        add_field!(wifi_ssid);
        add_field!(wifi_psk);
        add_field!(mqtt_client_id);

        html.push_str("</ul>");

        write!(f, "{}", html)
    }
}

struct Buffer {
    raw_data: Vec<u8>,
}

pub fn get_config(mut nvs: EspNvs<NvsDefault>) -> (Config, usize) {
    // Check if config exists in NVS
    let bincode_config = bincode::config::standard();
    // const MAX_BUF_LEN: usize = 10000;
    // let mut stored_data: [u8; MAX_BUF_LEN] = [0; MAX_BUF_LEN];
    // let mut stored_data: Vec<u8> = vec![];
    // let app_config = match nvs.get_raw("config", &mut stored_data) {
    //     Ok(Some(v)) => {
    //         // Deserialize the stored config
    //         bincode::deserialize(&v).unwrap()
    //     }
    //     Ok(None) | Err(_) => {
    //         // If config does not exist, create a default config and store it
    //         let default_config = Config::default();
    //         let serialized = bincode::serialize(&default_config).unwrap();
    //         nvs.set_blob("config", &serialized).unwrap();
    //         println!("Stored default config in NVS.");
    //         default_config
    //     }
    // };

    // Define a buffer to store the serialized data
    let mut b: Buffer = Buffer { raw_data: (vec![]) };

    // Step 1: Get the blob from NVS
    let blob_result = nvs.get_blob("config", &mut b.raw_data);

    // Step 2: Deserialize the stored data into app_config
    let (app_config, app_config_size) = match blob_result {
        Ok(Some(v)) => {
            // Deserialize the stored config
            // bincode::decode_from_slice::<Config>(&v, bincode_config).unwrap()
            log::info!("Loaded config from NVS.");
            bincode::decode_from_slice(v, bincode_config).unwrap()
        }
        Ok(None) | Err(_) => {
            // If config does not exist, create a default config and store it
            let default_config = Config::default();
            let serialized = bincode::encode_to_vec(&default_config, bincode_config).unwrap();
            nvs.set_blob("config", &serialized).unwrap();
            log::info!("Stored default config in NVS.");
            (default_config, 0) // TO DO: get real size..
        }
    };
    (app_config, app_config_size)
}
