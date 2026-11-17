# ssn-esp32 Architecture

> **Status: DRAFT**  
> This document is derived from a static read of the codebase on **2026-11-10**. It describes
> **what the firmware does today**, not a target design. Sections marked *"Gap"*, *"Risk"* or
> *"Open question"* are candidates for follow-up. When the code and this document disagree,
> the code wins — please update this file in the same change that alters the architecture.
>
> **Scope:** the Rust firmware in this repository (ESP32-C3), its runtime structure, protocols,
> configuration, and known limitations. Hardware schematics and the SSN backend are out of scope.

---

## 1. Purpose

`ssn-esp32` is a single-binary firmware for an ESP32-C3 sensor node. The node:

- reads local sensors over I2C, ADC, and GPIO;
- publishes sensor channels to an MQTT broker using the SSN topic hierarchy;
- accepts MQTT commands to drive GPIO outputs and update device state;
- maintains a Wi-Fi **station + access point** (AP) simultaneously;
- synchronizes wall-clock time over SNTP;
- serves a small HTTP page exposing its configuration;
- reports coarse status on a WS2812 RGB LED;
- stores its configuration in NVS flash.

It is a **blocking, single-main-loop** design with no async runtime. MQTT events, GPIO
interrupts, and HTTP requests are handled by ESP-IDF tasks/callbacks and communicate with the
main loop through shared `Arc<Mutex<...>>` queues and an interrupt `Notification`.

### Current non-goals

OTA updates, secure provisioning, TLS, local time-series storage, power management, and a
machine-readable local API are **not implemented**.

---

## 2. System context

```mermaid
flowchart LR
    subgraph Field["Field I/O"]
        S["Sensors<br/>I2C / ADC / GPIO"]
        A["Actuators<br/>GPIO outputs"]
        L["WS2812 status LED"]
    end

    F["ssn-esp32 firmware<br/>(ESP32-C3)"]
    N[("NVS flash")]
    B["MQTT broker"]
    P["SSN backend<br/>(account / object)"]
    H["HTTP client<br/>(browser)"]
    C["Wi-Fi AP client<br/>(provisioning / diagnostics)"]

    S --> F
    F --> A
    F --> L
    F <-->|config blob| N
    F <-->|publish / subscribe| B
    B <--> P
    H -->|"GET / , /settings"| F
    C -.->|"associates to ssn_MAC"| F
```

The firmware identifies itself with the factory base MAC address. The MAC becomes the
`ssn_id`, part of the AP SSID, and part of the MQTT client ID.

---

## 3. Platform and build

| Item | Value |
|---|---|
| MCU | ESP32-C3 (RISC-V `rv32imc`), set in `Cargo.toml` `[package.metadata.esp-idf]` and `.cargo/config.toml` |
| Rust target | `riscv32imc-esp-espidf` |
| Rust toolchain | `nightly` + `rust-src`, `build-std = ["std", "panic_abort"]` |
| ESP-IDF | `v5.3.2` via `esp-idf-sys` 0.36 / `embuild` 0.33 (`build.rs`) |
| Crate | Binary-only `ssn-esp32`, edition 2021, declared `rust-version = "1.77"` |
| Profiles | `release`: `opt-level = "s"`; `dev`: debug symbols + `opt-level = "z"` |
| Features | `default = []`; `experimental` forwards to `esp-idf-svc/experimental` |
| sdkconfig | `CONFIG_ESP_MAIN_TASK_STACK_SIZE=8000`; everything else ESP-IDF defaults |
| Runner | `espflash flash --monitor` |
| License | Apache-2.0 |

Build and flash:

```bash
cargo build --release
# or: scripts/build.sh [debug|release]

espflash flash target/riscv32imc-esp-espidf/release/ssn-esp32 --monitor
# or: scripts/flash.sh [debug|release]
```

The dev container and VS Code tasks wrap the same scripts. Wokwi is mentioned in
`docs/README.md`, but no `wokwi.toml` is present in this snapshot.

---

## 4. High-level structure

```mermaid
flowchart TB
    subgraph Orchestration["Orchestration — main.rs"]
        MAIN["main() / main_logic()<br/>wiring + main loop"]
    end

    subgraph Callbacks["Services and callbacks"]
        MQTT_CB["MQTT callback<br/>mqtt_service.rs → mqtt_commands.rs"]
        GPIO_ISR["GPIO ISR callback<br/>ssndevices.rs → interrupts.rs"]
        HTTP_CB["HTTP handlers<br/>http_server.rs"]
        AL["Alarm store<br/>alarms.rs"]
        Q["Queues + shared data<br/>shared_state.rs"]
        CFGS["NVS config bootstrap<br/>config_service.rs"]
        LEDS["Status LED<br/>led.rs"]
    end

    subgraph Domain["Device domain"]
        SD["SsnDevice / SsnDevices<br/>ssndevices.rs"]
        REG["Hardcoded registration<br/>device_registration.rs"]
        DM["DeviceManager<br/>device_manager.rs"]
        TM["TimeManager<br/>time_manager.rs"]
    end

    subgraph Adapters["Driver adapters"]
        BMP180["bmp180.rs"]
        ENS["ens160.rs"]
        AHT["aht21.rs"]
        ADC["adc_reader.rs"]
        BMP["bmp.rs<br/>(BMP280/BME280)"]
        GPIO["GPIO traits<br/>(ssndevices.rs)"]
    end

    subgraph Platform["Platform services"]
        CFG["settings.rs / NVS"]
        WIFI["ssn_wifi.rs"]
        MQ["esp-idf-svc MQTT"]
        SNTP["esp-idf-svc SNTP"]
        HTTPD["esp-idf-svc HTTP server"]
        LED["WS2812 over RMT"]
    end

    subgraph HAL["ESP-IDF / HAL"]
        HW["GPIO · I2C · ADC · RMT · FreeRTOS · Wi-Fi"]
    end

    MAIN --> DM
    MAIN --> REG
    MAIN --> AL
    MAIN --> Q
    MAIN --> CFGS
    MAIN --> WIFI
    MAIN --> MQ
    MAIN --> HTTPD
    MAIN --> LED
    MAIN --> LEDS
    MAIN --> MQTT_CB
    MQTT_CB --> Q
    HTTP_CB --> CFG
    GPIO_ISR -.-> MAIN
    REG --> SD
    DM --> SD
    TM --> SD
    SD --> BMP180
    SD --> ENS
    SD --> AHT
    SD --> ADC
    SD --> BMP
    SD --> GPIO
    BMP180 --> HW
    ENS --> HW
    AHT --> HW
    ADC --> HW
    BMP --> HW
    GPIO --> HW
    CFG --> HW
    WIFI --> HW
    MQ --> HW
    SNTP --> HW
    HTTPD --> HW
    LED --> HW
```

### Runtime model

| Context | Executes | Interaction with shared state |
|---|---|---|
| **Main task** | `main_logic()` init, then an infinite loop; `DeviceManager::process_all_devices` | Owns the device list; locks each `SsnDevice` while reading; drains command queues |
| **MQTT client task** | `mqtt_service::start` callback → `mqtt_commands::handle_message` | Parses topics, pushes `SsnDeviceCommand` / `SsnMqttCommand`, reads `shared_data`; may read NVS for the `get` command |
| **GPIO interrupt** | `PinDriver::subscribe` callback in `ssndevices.rs` → `interrupts.rs` | Increments atomics, locks the per-pin counter map, notifies the main task |
| **HTTP server task** | `http_server.rs` handler closures | Reads captured `app_config`; no shared mutable state |
| **FreeRTOS timers/drivers** | ESP-IDF internals | N/A |

There is **no `async` executor**, no `tokio` (commented out in `Cargo.toml`), and no worker
thread pool. Delays use `FreeRtos::delay_ms` or `std::thread::sleep`.

---

## 5. Module map

| Module | Responsibility | Status in runtime |
|---|---|---|
| `main.rs` | Composition root: entry point, initialization order, main-loop orchestration | **Active** (~320 lines) |
| `shared_state.rs` | Shared aliases (`SharedSensorData`, command queues), queue helpers, `SharedI2c` adapter | **Active** |
| `config_service.rs` | NVS open, reset path, config load, MAC-derived identity overrides | **Active** |
| `alarms.rs` | `Alarm` store, `add_alarm`, `check_alarms` | **Active** |
| `interrupts.rs` | GPIO interrupt counters and `record_interrupt` / `take_pin_count` | **Active** |
| `led.rs` | WS2812 status LED helper | **Active** |
| `mqtt_service.rs` | MQTT client creation, callback dispatch, subscriptions, retained `status/start` | **Active** |
| `mqtt_commands.rs` | Inbound command/topic parsing and queueing | **Active** |
| `mqtt_config.rs` | Broker URL, client ID, subscription list, client configuration | **Active** |
| `device_registration.rs` | Hardcoded device set (GPIO, ENS160, BMP180) | **Active** |
| `device_manager.rs` | Single device sampling/publishing path for every `SsnDevices` variant | **Active** |
| `time_manager.rs` | Fast/Middle/Slow sampling scheduler | **Active** |
| `http_server.rs` | `/` and `/settings` handlers and HTML templates | **Active** |
| `ssndevices.rs` | `SsnDevice` wrapper, `SsnDevices` enum, `SsnSensorPeriod`, shared-device helpers, GPIO input/output traits and the `PinDriverBasicInput` impl, `DeviceType` / `DeviceConfig` | **Active** |
| `settings.rs` | `Config` schema, NVS load/save/defaults, HTML `Display` | **Active** |
| `mqtt_messages.rs` | Topic builders, topic/token parsers, `fill_device_types_map` | **Active** (topic build/parse); `fill_device_types_map` unused |
| `ssn_wifi.rs` | STA+AP setup, initial connect with scan/retry, connection check | **Active** |
| `utils.rs` | `calculate_percentage_change`, `handle_sensor_value_update` publish policy | **Active**; many commented-out channel helpers |
| `helpers.rs` | Extracted MQTT-command, alarm, GPIO-notification, and LED helpers | **Scaffold** — compiled but not called |
| `commons.rs` | `Colors` enum, `ValuesDev` struct | `Colors` used; `ValuesDev` unused |
| `bmp180.rs` | BMP180 adapter over `bmp180-driver` | Active when sensor init succeeds |
| `ens160.rs` | ENS160 adapter over `ens160` crate | Active when sensor init succeeds |
| `aht21.rs` | AHT21 adapter over `ahtx0` | Instantiation commented out |
| `adc_reader.rs` | ADC1 one-shot adapter + `AdcReaderTrait` | Instantiation commented out |
| `bmp.rs` | BMP280/BME280 adapter over `bmpe280` | Wrapper exists; `SsnDevices::Bmp` arm is `todo!()` |

---

## 6. Boot and initialization sequence

`main()` repeatedly calls `main_logic()`; `main_logic()` returns only on error, after which the
firmware calls `esp_restart()`. Initialization order:

1. `esp_idf_svc::sys::link_patches()` and `EspLogger::initialize_default()`.
2. Create shared runtime state (`shared_state`): device vector, sensor-data map, device-command
   queue, MQTT-publish queue, and the GPIO `Notification`.
3. Take peripherals.
4. Configure the settings-reset button: **GPIO8 input with pull-up**, 100 ms settle.
5. Create the WS2812 driver on **GPIO10 / RMT channel 0** (`led`).
6. Open the default NVS partition, namespace `ssn_ns`, and construct `Settings`
   (`config_service::open_settings`).
7. If GPIO8 is low at boot, write defaults **and then overwrite them with a hardcoded test
   JSON configuration** (see [§15](#15-configuration-and-persistence)).
8. Load `Config` from NVS (`config_service::load_config`); NVS errors propagate and reboot
   instead of panicking.
9. Override identity fields in memory:
   - `account = "3"`, `object = "2"`;
   - `ssn_id = MAC`;
   - `ssn_wifi_ssid = "ssn_<MAC>"`, `ssn_wifi_pass` set to a hardcoded default;
   - `mqtt_client_id = "ssn_esp_<MAC>"`.
10. Set the LED **yellow**, then initialize Wi-Fi in **Mixed** mode (station + AP). On failure:
    red LED, error propagates, firmware reboots.
11. Start SNTP (`EspSntp::new_default()`); log local time using the hardcoded `DEF_TZ = +3`;
    set the LED **green**.
12. Initialize **I2C0** on **SCL=GPIO0, SDA=GPIO1 at 400 kHz** and wrap it in
    `SharedI2c(Arc<Mutex<I2cDriver>>)`.
13. Register devices (`device_registration`):
    - GPIO outputs on GPIO3, GPIO4, GPIO5 (`gpio-out3/4/5`);
    - GPIO input on GPIO6 (`gpio-in6`, any-edge, pull-up);
    - ENS160 (`ens160-test`) if initialization succeeds;
    - BMP180 (`bmp180`) if initialization succeeds.
    - AHT21, ADC, `StoreInt`, and GPIO7 are present in code but commented out.
14. Add **two hardcoded alarms** for 14:45 and 14:46 `UTC+3` on 2025-08-02, each 10 s, driving
    `gpio-out4` and `gpio-out5` (`alarms`).
15. Create the MQTT client and callback (`mqtt_service`); sleep 6 s; subscribe to topics; publish
    a **retained** `status/start` message containing the client ID.
16. Start the HTTP server (default port 80) with `/` and `/settings` (`http_server`).
17. Enter the main loop (`device_manager::DeviceManager` + `time_manager::TimeManager`).

**Failure policy:** peripheral, NVS, and Wi-Fi errors abort startup and reboot; sensor
initialization errors are logged and the node continues without that sensor.

---

## 7. Core data model

### 7.1 Configuration

`settings::Config` is a flat JSON object persisted in NVS:

```jsonc
{
  "account": "3",
  "object": "2",
  "ssn_id": "AA:BB:CC:DD:EE:FF",
  "mqtt_host": "192.168.1.10",
  "mqtt_user": "user",
  "mqtt_pass": "***",
  "wifi_ssid": "uplink-ssid",
  "wifi_psk": "***",
  "mqtt_client_id": "ssn_esp_AA:BB:CC:DD:EE:FF",
  "ssn_wifi_ssid": "ssn_AA:BB:CC:DD:EE:FF",
  "ssn_wifi_pass": "***",
  "devices": [ /* DeviceConfig objects */ ]
}
```

`Config::devices` is parsed and stored but **never used to construct devices**. Device
registration is hardcoded in `device_registration.rs`.

### 7.2 Device configuration (declared, not applied)

`DeviceConfig` (`ssndevices.rs`) anticipates a config-driven model:

| Field | Intended use | Applied today? |
|---|---|---|
| `dev_id`, `device_type` | identity and variant selection | No — runtime IDs differ |
| `period` (`"Fast"`, `"Middle"`, `"Slow"`) | sampling cadence | No — no string→enum mapping |
| `is_active`, `is_paused` | enable/disable | No — never read |
| `temperature_delta_over`, `pressure_delta_over`, `humidity_delta_over` | spike thresholds | No |
| `co2eq_delta_over`, `tvoc_delta_over`, `aqi_delta_over` | spike thresholds | No |
| `delta_over` | generic spike threshold | No |
| `gpio_pin`, `interrupt_type`, `pull_type`, `initial_state` | GPIO setup | No |
| `adc_channel`, `voltage_reference` | ADC setup | No |
| `i2c_address` | sensor address | No |
| `store_keys` | `StoreInt` channels | No |
| `description`, `unit`, `min_value`, `max_value` | metadata | No |

### 7.3 Runtime device model

```rust
SsnDevice {
    dev_id: String,
    period: SsnSensorPeriod,   // Fast | Middle | Slow
    is_active: bool,           // stored, not enforced
    is_paused: bool,           // stored, not enforced
    device: SsnDevices,        // variant carrying state + driver handle
}
```

`SsnDevices` variants and their logical channels:

| Variant | State held | Channels published | Notes |
|---|---|---|---|
| `Bmp180` | temp, pressure, altitude, deltas, timestamps | `0` temperature, `1` pressure | Altitude computed but not published |
| `Ens160` | CO₂eq, TVOC, AQI, deltas, timestamps | `0` CO₂eq, `1` TVOC, `2` AQI | `DataNotReady` becomes `(-1, -1, -1)` |
| `Aht21` | temp, humidity, deltas, timestamps | `0` temperature, `1` humidity | Not instantiated in this snapshot |
| `Adc` | raw, mV, delta, timestamps | `0` voltage, `1` raw | Not instantiated |
| `StoreInt` | `HashMap<String, i32>` + per-key timing | one topic per map key | Not instantiated |
| `GpioInput` | level, counter, pull/interrupt config | `0` level, `1` interrupt counter | Active on GPIO6 |
| `GpioOutput` | level | none | Command-driven only |
| `Bmp` | BMP280/BME280 measurements | — | `todo!()`; not instantiated |

Shared aliases in `shared_state.rs`:

```rust
type SharedSensorData        = Arc<Mutex<HashMap<String, Vec<f32>>>>;
type SharedSsnDevice         = Arc<Mutex<SsnDevice>>;
type SharedSsnDeviceCommand  = Arc<Mutex<Vec<SsnDeviceCommand>>>;
type SharedSsnMqttCommand    = Arc<Mutex<Vec<SsnMqttCommand>>>;
```

---

## 8. Main loop

```mermaid
flowchart TD
    A["Start iteration"] --> B{"Wi-Fi up?"}
    B -- "no" --> B1["Red LED<br/>stop → start → reconnect"]
    B1 --> B2{"Reconnected?"}
    B2 -- "no" --> B3["Sleep 10 s<br/>continue"] --> A
    B2 -- "yes" --> C
    B -- "yes" --> C["Snapshot Utc::now / Instant"]
    C --> D["Pop one MQTT command<br/>and publish"]
    D --> E["check_alarms()<br/>push device commands"]
    E --> F["Blue LED<br/>sleep 100 ms"]
    F --> G["Pop one device command"]
    G --> H["Non-blocking GPIO<br/>notification check"]
    H --> I["For each device:<br/>if period fires → read → maybe publish"]
    I --> J["Green LED<br/>reset period counters<br/>sleep 500 ms"]
    J --> A
```

Key properties:

- The loop itself stays in `main.rs`; the device pass is
  `device_manager::DeviceManager::process_all_devices`.
- **One command per loop.** Both the MQTT-publish queue and the device-command queue are
  drained one item at a time. The queues are `Vec::pop()` (LIFO).
- **Sequential device reads.** Each device is locked, sampled, and published in order.
- **Blocking delays.** 100 ms before command handling, 500 ms between measured devices, and
  500 ms at the end of each iteration. With several active sensors, a "Fast" cycle is on the
  order of seconds, not 100 ms.
- **Blocking work while holding a device lock.** Sensor reads, MQTT enqueue, and the 500 ms
  inter-device delay all happen under the per-device `Mutex`.
- **`is_active` / `is_paused` are ignored**; every registered device is processed.

### Sampling periods

| Period | Intended cadence | Implementation |
|---|---|---|
| `Fast` | every loop | `TimeManager::should_measure` returns `true` unconditionally |
| `Middle` | ~20 s | `MIDDLE_LOOP_PERIOD_MS = 20_000`; counters reset at end of loop |
| `Slow` | ~60 s | `SLOW_LOOP_PERIOD_MS = 60_000`; counters reset at end of loop |

`device_manager::MAX_PERIOD_REFRESH_MQTT = 60` seconds forces a publish even when the value is
unchanged.

---

## 9. Publish policy

All sensor channels go through `utils::handle_sensor_value_update` (called from
`device_manager.rs`). Simplified pseudocode:

```text
if change_pct >= delta_threshold OR since_last_update > 60 s:
    baseline = last_sent_value.unwrap_or(current_value)

    if delta_over is set AND abs(pct(current, baseline)) >= delta_over:
        last_sent_value = current_value      // adopt the jump as the new baseline
        return                               // suppress publication

    publish(current_value as text)
    last_update = now
    last_sent_value = current_value
```

Consequences:

- `delta` is the **minimum positive percentage change** needed to publish before the 60 s
  refresh. Negative changes do not satisfy `change_pct >= delta` unless the refresh timer
  expires.
- `delta_over` acts as a **spike suppressor**: unusually large jumps are not published and
  become the new baseline.
- Payloads are **bare scalar text** produced by `Display` (e.g. `23.41`), not JSON.
- Publishes use **QoS 1 (`AtLeastOnce`)**, **not retained**.
- Errors from `enqueue` are discarded by callers (`let _ = ...`), so a failed publish does not
  propagate to the main loop.

---

## 10. Sensors and driver adapters

| Device | Bus / pins | Adapter | Behavior | Status |
|---|---|---|---|---|
| BMP180 | I2C (driver default address) | `bmp180::BMP180Sensor` | `check_connection`, `initialize`, `read_all(UltraHighResolution)` → `(temp °C, pressure Pa, altitude m)` | Active if present |
| ENS160 | I2C address `0x53` | `ens160::ENS160Sensor` | Reads firmware version, resets, enters operational mode; returns CO₂eq/TVOC/AQI | Active if present |
| AHT21 | I2C | `aht21::AHT21Sensor` | Calibrates, then measures temperature/humidity | Driver ready; instantiation commented out |
| ADC | ADC1 one-shot | `adc_reader::AdcReader` | `DB_11` attenuation; raw → `raw * 2500 / 4095` mV | Driver ready; instantiation commented out |
| BMP280 / BME280 | I2C | `bmp::BMPSensor` | Temperature, pressure, optional humidity | Wrapper ready; runtime arm is `todo!()` |
| GPIO input | GPIO6 | `PinDriverBasicInput` trait | Interrupt-driven edge counting + level read | Active |
| GPIO output | GPIO3/4/5 | `PinDriverTraitOutput` trait | `set_high` / `set_low` on command | Active |
| `StoreInt` | none | in-memory `HashMap` | Publishes each stored key | Not instantiated |

I2C access is serialized through `SharedI2c`, which implements `embedded_hal::i2c::I2c` by
locking an `Arc<Mutex<I2cDriver>>` for every transaction (1 s timeout).

---

## 11. Pin and resource map

| Resource | Use | Configuration | Notes |
|---|---|---|---|
| GPIO0 | I2C0 **SCL** | 400 kHz | Shared bus |
| GPIO1 | I2C0 **SDA** | 400 kHz | Shared bus |
| GPIO3 | Output `gpio-out3` | `PinDriver::input_output` | Command-driven |
| GPIO4 | Output `gpio-out4` | `PinDriver::input_output` | Also used by commented ADC example → conflict if enabled |
| GPIO5 | Output `gpio-out5` | `PinDriver::input_output` | Command-driven |
| GPIO6 | Input `gpio-in6` | Any-edge interrupt, pull-up | Publishes level + counter |
| GPIO7 | Input `gpio-in7` | Any-edge, pull-up | Registration commented out |
| GPIO8 | Settings-reset button | Input, pull-up, active low | Sampled **once at boot** |
| GPIO10 | WS2812 data | RMT channel 0 | Status LED |
| ADC1 | — | `DB_11` attenuation | No channel registered |
| RMT0 | WS2812 | — | LED driver |
| UART0 | Console/logs | ESP-IDF default | `log` → ESP logging |

**Risk:** the interrupt counter map in `interrupts.rs` is a fixed `[AtomicU32; 10]` indexed by pin
number. An interrupt-enabled pin ≥ 10 would panic. The current inputs (6, 7) are safe.

---

## 12. Networking and protocols

### 12.1 Wi-Fi

- **Mixed mode:** station uplink + soft AP simultaneously.
- Station credentials come from `Config`.
- AP SSID is `ssn_<MAC>`; the AP password is a fixed hardcoded default (redacted here); AP channel follows the
  station channel (or 1).
- Initial connect: start → scan (up to 5 retries, 3 s apart) → pin the found channel →
  connect → wait for DHCP.
- Connect retry loop: 5 attempts, 5 s apart, rescan, then `esp_restart()`.
- Runtime health check: `wifi.is_up()` **and** station netif has an IP.
- On loss: red LED, `stop()` / `start()`, `connect_wifi()`, or 10 s sleep and retry.

### 12.2 Time

- SNTP starts after Wi-Fi initialization, using ESP-IDF default servers.
- `DEF_TZ = +3` is hardcoded for logging and for constructing alarm times. It is **not**
  persisted.
- No RTC; until SNTP syncs, the clock is not meaningful.
- Alarms are stored in UTC after conversion, but activation compares **UTC time-of-day**
  only (see §13).

### 12.3 MQTT

Broker URI is built as `mqtt://[user:pass@]host` — plaintext, no TLS.

| Direction | Topic | Payload | QoS / retain |
|---|---|---|---|
| Subscribe | `/ssn/acc/{account}/raw_data` | — | QoS 1; parser has no meaningful branch |
| Subscribe | `/ssn/acc/{account}/obj/{object}/commands/+` | command token | QoS 1 |
| Subscribe | `/ssn/acc/{account}/obj/{object}/commands/device/+/+/in` | numeric value | QoS 1 |
| Publish | `/ssn/acc/{account}/obj/{object}/device/{dev_id}/{channel}/out` | scalar text | QoS 1, not retained |
| Publish | `/ssn/acc/{account}/obj/{object}/status/{event}` | client ID | QoS 1, **retained** for `start` |
| Publish | `/ssn/acc/{account}/obj/{object}/commands/response/{command}` | config text | QoS 1, not retained |

Inbound handling in the callback:

1. Parse `/ssn/acc/{account}/...` or `ssn/acc/{account}/...`.
2. Drop messages whose account ≠ configured account.
3. `root_token == "obj"`:
   - `.../obj/{object}/commands/{command}` (3 tokens after `obj`): handles `get` (publishes
     config) and `ini` (log only); other commands are ignored.
   - `.../obj/{object}/commands/device/{dev_id}/{channel}/{action}` (6 tokens after `obj`):
     `in` parses the payload as `f32` and enqueues a device command; `reset` is a TODO.
4. `root_token == "raw_data"` is accepted but not processed.
5. `root_token == "bot"` is parsed minimally and otherwise ignored.

Command application in the main loop:

| Device | Effect |
|---|---|
| `GpioOutput` | Value ≠ 0 → `set_high`, else `set_low`; **channel is not checked** |
| `GpioInput` | Writes `data`, then immediately overwrites it with the pin level; effectively no lasting effect |
| `StoreInt` | Inserts `channel → value as i32` into the map |
| Others | Ignored |

**Reconnect behavior:** ESP-IDF's client handles transport reconnects internally. The
application callback has manual attempt counting, but it captures a **copy** of the counter and
never recreates the client; the logic is effectively a no-op and sleeps block the MQTT task.

### 12.4 HTTP

| Route | Method | Response |
|---|---|---|
| `/` | GET | Static "Hello from ESP32-C3!" HTML page |
| `/settings` | GET | `Config` formatted as an HTML `<ul>`; passwords masked |

No authentication, no JSON API, no mutation endpoints. The server uses ESP-IDF defaults
(port 80) and is reachable from the station/AP interfaces.

---

## 13. Commands, alarms, and interrupts

### Command flow

```mermaid
sequenceDiagram
    participant B as MQTT broker
    participant C as MQTT callback task
    participant Q as Device command queue
    participant M as Main loop
    participant D as SsnDevice

    B->>C: commands/device/gpio-out4/0/in
    C->>C: parse topic, filter account, parse f32
    C->>Q: push(SsnDeviceCommand)
    M->>Q: pop() — one per iteration (LIFO)
    M->>D: lock and apply value
    D-->>M: updated state
```

Alarm-generated commands follow the same queue and are evaluated at the start of every loop.

### Alarm subsystem

- In-memory `Vec<Alarm>` behind a `Mutex` (`alarms.rs`), created at boot with `add_alarm(...)`.
- Each alarm stores trigger time, active duration, device ID, active/inactive levels, and
  previous state.
- Matching uses **time-of-day only**, with midnight wrap-around support. The date part of
  `trigger_time` is ignored.
- Commands are emitted only on state transitions (inactive→active and active→inactive) and are
  always sent on **channel 0**.
- Current alarms are test data: 2025-08-02 14:45 and 14:46 `UTC+3` (= 11:45/11:46 UTC),
  10 s duration, on `gpio-out4`/`gpio-out5`.
- Alarms are not persistent, not editable over MQTT, and not derived from `Config`.

### GPIO interrupts

- `PinDriverBasicInput::subscribe_boxed` (`ssndevices.rs`) calls `PinDriver::subscribe` during
  device initialization with a closure capturing `Notification`.
- The callback calls `interrupts::record_interrupt(pin)`, which increments
  `INTERRUPT_COUNTER_GPIO` and the per-pin entry in `INTERRUPT_GPIO_HASHMAP`, then calls
  `notify_and_yield(pin)`.
- The main loop calls `notification.wait(0)` (non-blocking) once per iteration;
  `DeviceManager` accumulates the per-pin counter into the device's `counter` and resets it via
  `interrupts::take_pin_count`, then re-enables the interrupt.
- Only one notification value is consumed per loop; counters coalesce missed notifications.

**Risk:** the ISR callback takes a `std::sync::Mutex` (`INTERRUPT_GPIO_HASHMAP.lock()`). Locking
a blocking mutex in interrupt context is not generally safe. If the callback truly runs in ISR
context, this should be replaced with atomics or an ISR-safe primitive. Confirm against the
`esp-idf-hal` callback contract.

---

## 14. Concurrency and shared state

| State | Type | Writer(s) | Reader(s) | Synchronization |
|---|---|---|---|---|
| Registered devices | `Vec<Arc<Mutex<SsnDevice>>>` | Init only | Main loop | Per-device `Mutex` |
| I2C bus | `Arc<Mutex<I2cDriver>>` | Sensor adapters | Sensor adapters | `Mutex` |
| Sensor snapshot | `Arc<Mutex<HashMap<String, Vec<f32>>>>` | Main loop | MQTT callback (debug read) | `Mutex` |
| Device commands | `Arc<Mutex<Vec<SsnDeviceCommand>>>` | MQTT callback, alarms | Main loop | `Mutex`, LIFO |
| MQTT publishes | `Arc<Mutex<Vec<SsnMqttCommand>>>` | MQTT callback | Main loop | `Mutex`, LIFO |
| Alarms | `Mutex<Vec<Alarm>>` | Init, main loop | Main loop | `Mutex` |
| GPIO notification | `Arc<Notification>` | ISR | Main loop | ESP-IDF notification |
| Interrupt counters | `AtomicU32` | ISR | Main loop | Atomics |
| Config | `Config` value / `Settings` | Init, reset path | HTTP handler, MQTT `get` | Ownership/capture |

Lock ordering observed: **device → I2C**. No code path takes I2C then a device lock, so no
obvious cycle. However:

- the main loop holds a device lock across I2C reads, MQTT enqueue, and 500 ms sleeps;
- the MQTT task can perform blocking NVS reads and sleeps;
- the ISR can contend on a mutex.

None of these currently deadlock given the single main task, but they increase latency and make
timing analysis difficult.

---

## 15. Configuration and persistence

| Item | Value |
|---|---|
| Partition | Default NVS partition |
| Namespace | `ssn_ns` |
| Key | `config` |
| Format | JSON blob (`serde_json`) |
| Read buffer | Fixed `[0u8; 1024]` |
| Defaults | `settings::Config::default()` (includes test credentials) |
| Reset trigger | GPIO8 low at boot |

Behavior:

- `get_config()` returns `(Config, size)`. **Any** failure — missing key, read error, blob
  larger than 1 KB, or deserialization error — falls back to writing defaults. A too-large
  config can therefore be silently replaced by defaults.
- `set_config_from_json()` validates by deserializing, then stores the **raw JSON string**.
- The reset path writes defaults and then immediately overwrites them with a **hardcoded test
  configuration** containing Wi-Fi and MQTT credentials. This block is active in the current
  source and should be removed or guarded before deployment.
- Identity fields are overridden at every boot (account, object, `ssn_id`, AP credentials,
  client ID) but are **not persisted**, so NVS can disagree with runtime behavior.
- `Config::Display` renders an HTML list with `mqtt_pass`, `wifi_psk`, and `ssn_wifi_pass`
  masked. It is used by `/settings` and by the MQTT `get` response — meaning the MQTT response
  is HTML, not JSON.
- Debug-level logging of the config (`log::debug!("Loaded config: {:?}", config)`) prints
  secrets in plaintext.

---

## 16. Status LED

| Color | Meaning |
|---|---|
| Yellow | Boot / configuration phase |
| Red | Wi-Fi failure or connection lost |
| Green | Initialized, idle, or end of a measurement cycle |
| Blue | Measurement cycle in progress |
| White / Black | Defined but unused |

Brightness is fixed at 30/255 per channel. Writes are blocking (`write_blocking`). The main
loop alternates blue → green every iteration; red is set on Wi-Fi problems.

---

## 17. Error handling and observability

- `main()` loops on `main_logic()`; any propagated error is logged and followed by
  `esp_restart()`.
- Startup errors (peripherals, NVS, Wi-Fi) are fatal; sensor initialization errors are logged
  and skipped. `config_service` returns NVS/config errors instead of panicking, so they take the
  same reboot path.
- Read-path error handling is inconsistent (`device_manager.rs`):
  - BMP180: `.context(...).unwrap()` → **panic** on read failure;
  - AHT21: `.unwrap()` → **panic**;
  - ADC: `.unwrap()` → **panic**;
  - ENS160: `unwrap_or((-1, -1, -1))` → degrades to invalid values.
- MQTT publish errors are ignored.
- MQTT parser error paths (`mqtt_commands.rs`) include `todo!()` (panic) for a non-numeric
  channel and `unwrap()` on missing parsed tokens; invalid UTF-8 payloads also panic.
- The MQTT `get` command logs the config with `Display` (masked) rather than `Debug`; the
  `settings::get_config` debug log still prints secrets when debug logging is enabled.
- Logging goes through the `log` crate into ESP-IDF logging. The default level is not set in
  this repository.
- No application-level watchdog, metrics endpoint, structured logs, or crash reporting is
  configured. No core-dump/partition configuration is present.

---

## 18. Security considerations

This is a lab/prototype posture, not a production posture.

| Area | Current state |
|---|---|
| MQTT transport | `mqtt://` only; no TLS, no certificate validation |
| MQTT credentials | In the broker URI; stored in NVS and defaults |
| Wi-Fi AP | Always on, fixed default password, SSID derived from MAC |
| HTTP | No authentication; config page exposed on station and AP |
| NVS | Not encrypted by configuration in this repo |
| Logging | Debug logs can print the full config, including secrets |
| Firmware update | No OTA path |
| Boot integrity | No secure boot / flash encryption configuration |
| Input validation | Minimal; malformed MQTT topics can panic |
| Provisioning | Hardcoded test credentials on reset |

**Recommendation (draft):** treat the current build as development-only until TLS, credential
provisioning, AP gating, and input hardening are addressed.

---

## 19. Known gaps and technical debt

| # | Gap | Impact |
|---|---|---|
| 1 | ~~Monolithic `main.rs` (~1,700 lines) owns init, device logic, alarms, MQTT, HTTP~~ **Resolved 2026-11-16**: split into `shared_state`, `config_service`, `alarms`, `interrupts`, `led`, `mqtt_service`, `mqtt_commands`, `mqtt_config`, `device_registration`, `device_manager`, `time_manager`, and `http_server`; `main.rs` is ~320 lines of wiring + loop | — |
| 2 | `helpers` is still compiled but not wired | Dead/duplicated helper paths invite drift |
| 3 | ~~Device processing exists twice (inline main loop and `DeviceManager`)~~ **Resolved 2026-11-16**: `DeviceManager` is the only device-processing path, covering every variant | — |
| 4 | `Config.devices` is ignored; devices are hardcoded | Remote configuration has no effect |
| 5 | `is_active` / `is_paused` are never checked | Cannot disable a device without reflashing |
| 6 | Hardcoded test configuration and credentials in the reset path | Credential leakage and wrong deployment behavior |
| 7 | Alarms are hardcoded daily time-of-day rules with 2025 test dates | No real scheduling; date is ignored; not configurable |
| 8 | MQTT reconnect bookkeeping is ineffective; sleeps block the MQTT task | Recovery depends entirely on ESP-IDF internals |
| 9 | Device mutex held across 500 ms delays and I2C/MQTT operations | Long critical sections; poor timing predictability |
| 10 | Panics/`unwrap()`/`todo!()` in read and parse paths | A sensor fault or malformed topic can reboot the node |
| 11 | LIFO command queues, one command per loop | Command ordering and throughput are unintuitive |
| 12 | Fixed 1 KB config buffer with silent fallback to defaults | Large configs are lost without a clear error |
| 13 | MQTT `get` response is HTML while sensor payloads are bare scalars | Protocol mismatch; no schema versioning |
| 14 | GPIO ISR takes a blocking `Mutex` | Potential ISR-safety issue |
| 15 | No tests; `harness = false` | No host or integration coverage |
| 16 | Unfinished paths: `Bmp => todo!()`, AHT21/ADC/StoreInt/GPIO7 commented out | Feature set depends on source edits |
| 17 | Unused/duplicated code: `helpers` functions, `ValuesDev`, `fill_device_types_map`, `temperature_html`, `SsnChannelInfo`, commented channel helpers in `utils.rs` | Confusing dead code |
| 18 | No TLS, OTA, secure boot, flash encryption, or application watchdog | Not production-ready |
| 19 | Hardcoded timezone `+3` | Wrong local time outside that zone |
| 20 | Fixed 10-entry per-pin interrupt array indexed by pin number | Panic for interrupt pins ≥ 10 |
| 21 | `GPIOInput` command value is overwritten by the pin read; `GpioOutput` ignores the channel | Commands can appear to succeed without effect |

---

## 20. Suggested evolution (non-binding)

This is a draft roadmap, not an agreed plan.

### Phase 0 — Stabilize the baseline

- Remove or feature-gate the hardcoded test configuration and credentials.
- Add host-side unit tests for the pure logic: topic parsing, publish decision, config
  (de)serialization, alarm state transitions.
- Replace panics in sensor/parse paths with typed errors and a documented degraded mode.

### Phase 1 — Extract services from `main.rs`

Status (2026-11-16): the module split is done; the loop still lives in `main.rs`.

- ~~Make `DeviceManager` the single device-processing path, covering every variant.~~
- ~~Make `TimeManager` the single sampling scheduler.~~
- ~~Introduce `MqttService`, `WifiService`, `ConfigService`, `AlarmService`, `LedService`, and
  an `HttpService`.~~ (as modules: `mqtt_service`, `ssn_wifi`, `config_service`, `alarms`,
  `led`, `http_server`)
- ~~Reduce `main.rs` to composition + the loop~~, or replace the loop with a service registry.

### Phase 2 — Configuration-driven devices

- Build devices from `Config.devices` (`DeviceType` + per-type fields).
- Honor `period`, `is_active`, `is_paused`, `gpio_pin`, `interrupt_type`, `pull_type`,
  `adc_channel`, `i2c_address`, and metadata.
- Validate configuration before persisting; surface errors instead of silently resetting.
- Decide whether identity fields belong in NVS or remain MAC-derived.

### Phase 3 — Protocol hardening

- Define a versioned JSON payload schema for sensor values and command responses.
- Replace HTML `Config::Display` usage on MQTT with a proper JSON response.
- Model commands as a typed enum; make the queues FIFO and drain predictably.
- Add command acknowledgements/errors and per-device correlation IDs.
- Move to `mqtts://` with certificate validation; rotate credentials.

### Phase 4 — Reliability and operations

- Remove blocking sleeps from the MQTT callback and from device critical sections.
- Add a task/application watchdog and bounded sensor timeouts.
- Add OTA updates, secure boot, flash encryption, and a provisioning flow.
- Add health/status telemetry and structured logging.
- Consider power management if battery operation is required.

### Phase 5 — Testability

- Introduce traits for sensor reads and MQTT egress to allow host-side simulation.
- Add integration tests for the command → device → publish path.
- Add CI: build for `riscv32imc-esp-espidf`, clippy, formatting, unit tests.

### Possible target layout

```text
src/
  main.rs            # entry point + wiring only
  app/               # loop, services, lifecycle
  devices/           # device model, manager, scheduler
  drivers/           # sensor adapters (I2C/ADC/GPIO)
  protocol/          # topics, commands, payloads
  services/          # config, wifi, mqtt, http, alarms, led
  platform/          # shared I2C, notifications, time
```

---

## 21. Open questions

1. Is the MQTT `get` response expected to be JSON? The current HTML output looks accidental.
2. Are alarms meant to be date-specific, daily time-of-day, or cron-like? The code and comments
   disagree.
3. Should `account` / `object` / `ssn_id` come from NVS, MAC derivation, or backend
   provisioning? Today they are overwritten in code.
4. Which sensor set is the production configuration? AHT21, ADC, `StoreInt`, GPIO7, and
   BMP280/BME280 are partially implemented.
5. Should the AP remain always on, or only during provisioning? What is the password policy?
6. What is the backend contract for payload format, cadence, and command acknowledgement?
7. What are the flash size, partition layout, and OTA requirements?
8. Should commands be FIFO and fully drained each cycle?
9. Is the MQTT client expected to re-create itself on reconnect, or rely on ESP-IDF?
10. Is interrupt latency critical enough to require ISR-safe counters only?

---

## 22. Appendix — source inventory

| File | Lines | Purpose |
|---|---:|---|
| `src/main.rs` | 322 | Entry point, initialization order, main-loop orchestration |
| `src/device_manager.rs` | 523 | Single device processing path (every variant), publish policy call sites |
| `src/ssndevices.rs` | 336 | Device enum/model, periods, GPIO traits + impl, config types |
| `src/utils.rs` | 245 | Publish policy and percentage-change helper |
| `src/device_registration.rs` | 206 | Hardcoded device set (GPIO, ENS160, BMP180) |
| `src/ssn_wifi.rs` | 192 | STA+AP setup, connect/retry, health check |
| `src/mqtt_messages.rs` | 177 | Topic builders and parsers |
| `src/settings.rs` | 149 | Config schema, NVS persistence, display |
| `src/shared_state.rs` | 134 | Shared aliases, queues, sensor-data store, `SharedI2c` |
| `src/mqtt_commands.rs` | 119 | Inbound command parsing and queueing |
| `src/config_service.rs` | 118 | NVS bootstrap, reset path, config load/overrides |
| `src/mqtt_service.rs` | 112 | MQTT client, callback dispatch, subscriptions, start message |
| `src/ens160.rs` | 109 | ENS160 adapter |
| `src/bmp.rs` | 101 | BMP280/BME280 adapter |
| `src/alarms.rs` | 81 | Alarm store and time-of-day matching |
| `src/aht21.rs` | 80 | AHT21 adapter |
| `src/bmp180.rs` | 76 | BMP180 adapter |
| `src/http_server.rs` | 71 | `/` and `/settings` handlers, HTML templates |
| `src/helpers.rs` | 68 | Scaffolded command/alarm/GPIO/LED helpers |
| `src/adc_reader.rs` | 66 | ADC1 adapter |
| `src/time_manager.rs` | 63 | Fast/Middle/Slow scheduler |
| `src/mqtt_config.rs` | 55 | Broker URL, client ID, topics, client configuration |
| `src/interrupts.rs` | 44 | GPIO interrupt counters |
| `src/led.rs` | 19 | WS2812 status LED |
| `src/commons.rs` | 14 | Colors and `ValuesDev` |

### Glossary

| Term | Meaning in this codebase |
|---|---|
| **SSN** | The platform/account/object naming scheme used in MQTT topics |
| **Account / object** | Tenant and object identifiers embedded in every topic |
| **Device (`dev_id`)** | A registered sensor or actuator, e.g. `bmp180`, `gpio-out4` |
| **Channel** | Numeric sub-value of a device, e.g. BMP180 `0` temperature, `1` pressure |
| **Period** | `Fast`, `Middle`, or `Slow` sampling cadence |
| **Delta** | Minimum positive percentage change that triggers an early publish |
| **Delta over** | Spike-suppression threshold; larger relative jumps are not published |
| **Baseline** | `last_sent_value`, used for change calculations and spike suppression |

---

*End of draft. Update this document when module boundaries, protocols, configuration, or
operational assumptions change.*
