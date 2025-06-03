use anyhow::{bail, Result};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    hal::peripheral,
    sntp::{EspSntp, SyncMode},
    wifi::{
        AccessPointConfiguration, AuthMethod, BlockingWifi, ClientConfiguration, Configuration,
        EspWifi,
    },
};
use esp_idf_sys::esp_restart;
use log::info;
use std::time::Duration;

pub fn connect_wifi(
    wifi: &mut BlockingWifi<EspWifi>,
    sysloop: &EspSystemEventLoop,
) -> anyhow::Result<()> {
    let mut retries = 0;
    const MAX_RETRIES: u8 = 5;
    let ssid = wifi
        .get_configuration()
        .unwrap()
        .as_mixed_conf_mut()
        .0
        .ssid
        .clone();

    loop {
        match wifi.connect() {
            Ok(_) => {
                log::info!("WiFi connected successfully");
                info!("Waiting for DHCP lease...");

                wifi.wait_netif_up()?;

                let ip_info = wifi.wifi().sta_netif().get_ip_info()?;

                info!("Wifi DHCP info: {:?}", ip_info);
                return Ok(());
            }
            Err(e) => {
                retries += 1;
                log::warn!(
                    "WiFi connection failed (attempt {}/{}): {:?}",
                    retries,
                    MAX_RETRIES,
                    e
                );

                if retries >= MAX_RETRIES {
                    log::error!("Rebooting...");

                    // Reboot the ESP32 only on error
                    unsafe {
                        esp_restart();
                    }
                    // return Err(e.into());
                }

                // Wait before retrying
                std::thread::sleep(Duration::from_secs(5));

                // Reconfigure WiFi before retrying
                wifi.stop()?;

                info!("Scanning...");

                let ap_infos = wifi.scan()?;

                let ours = ap_infos.into_iter().find(|a| a.ssid == ssid);

                let channel = if let Some(ours) = ours {
                    info!(
                        "Found configured access point {} on channel {}",
                        ssid, ours.channel
                    );
                    Some(ours.channel)
                } else {
                    info!(
                        "Configured access point {} not found during scanning, will go with unknown channel",
                        ssid
                    );
                    None
                };
                wifi.start()?;
            }
        }
    }
}

pub fn check_wifi_connection(wifi: &EspWifi) -> bool {
    wifi.is_up().unwrap_or(false) && wifi.sta_netif().get_ip_info().is_ok()
}

pub fn wifi<'a>(
    ssid: &'a str,
    pass: &'a str,
    modem: impl peripheral::Peripheral<P = esp_idf_svc::hal::modem::Modem> + 'static,
    sysloop: EspSystemEventLoop,
    ssn_wifi_ssid: &'a str,
    ssn_wifi_pass: &'a str,
) -> anyhow::Result<BlockingWifi<EspWifi<'static>>> {
    let mut auth_method = AuthMethod::WPA2Personal;
    // let mut auth_method = AuthMethod::WPA;
    if ssid.is_empty() {
        bail!("Missing WiFi name")
    }
    if pass.is_empty() {
        auth_method = AuthMethod::None;
        info!("Wifi password is empty");
    }
    let esp_wifi = EspWifi::new(modem, sysloop.clone(), None)?;

    let mut wifi = BlockingWifi::wrap(esp_wifi, sysloop.clone())?;

    wifi.set_configuration(&Configuration::Client(ClientConfiguration::default()))?;

    info!("Starting wifi...");
    wifi.start()?;
    info!("Scanning...");

    let ap_infos = {
        let mut retries = 5;
        let mut last_error = None;

        loop {
            match wifi.scan() {
                Ok(infos) => break Ok(infos),
                Err(e) if retries > 0 => {
                    log::warn!("WiFi scan failed ({} retries left): {:?}", retries, e);
                    retries -= 1;
                    last_error = Some(e);
                    std::thread::sleep(std::time::Duration::from_secs(3));
                }
                Err(e) => {
                    log::error!("WiFi scan failed after retries: {:?}", e);
                    break Err(e);
                }
            }
        }
    }?;
    let ours = ap_infos.into_iter().find(|a| a.ssid == ssid);
    let channel = if let Some(ours) = ours {
        info!(
            "Found configured access point {} on channel {}",
            ssid, ours.channel
        );
        Some(ours.channel)
    } else {
        info!(
            "Configured access point {} not found during scanning, will go with unknown channel",
            ssid
        );
        None
    };

    wifi.set_configuration(&Configuration::Mixed(
        ClientConfiguration {
            ssid: ssid.try_into().expect("err"),
            password: pass.try_into().expect("err"),
            channel,
            auth_method,
            ..Default::default()
        },
        AccessPointConfiguration {
            ssid: ssn_wifi_ssid.try_into().expect("err"), //"ssn".try_into().expect("err"),
            password: ssn_wifi_pass.try_into().expect("err"), //"ssn123456".try_into().expect("err"),
            channel: channel.unwrap_or(1),
            ..Default::default()
        },
    ))?;

    // info!("Connecting wifi...");

    // wifi.connect()?;

    // info!("Waiting for DHCP lease...");

    // wifi.wait_netif_up()?;

    // let ip_info = wifi.wifi().sta_netif().get_ip_info()?;

    // info!("Wifi DHCP info: {:?}", ip_info);

    if let Err(e) = connect_wifi(&mut wifi, &sysloop.clone()) {
        log::error!("Failed to reconnect WiFi: {:?}", e);
        // delay here before retrying
        std::thread::sleep(Duration::from_secs(10));
    }
    Ok(wifi)
}
