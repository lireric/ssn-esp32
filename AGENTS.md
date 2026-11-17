# AGENTS.md

> **Status: DRAFT (2026-11-17)**
> This file is derived from `ARCHITECTURE.md` and describes the repository **as it currently
> behaves**, including known technical debt. It is guidance for coding agents, not a promise
> that the current design is correct. Read `ARCHITECTURE.md` before non-trivial changes, and
> update both files whenever architecture or workflow changes.

---

## 1. What this project is

- Firmware for an **ESP32-C3** sensor node, written in Rust against ESP-IDF via
  `esp-idf-svc` / `esp-idf-hal`.
- Single binary: **`ssn-esp32`**.
- Responsibilities: read local sensors, publish values over MQTT, accept MQTT commands,
  drive GPIO outputs, expose a small HTTP config page, run Wi-Fi **station + AP**, synchronize
  time over SNTP, and show status on a WS2812 LED.
- Runtime model: **blocking single main loop**. No async executor, no Tokio.
- Target: `riscv32imc-esp-espidf`, ESP-IDF **v5.3.2**, Rust nightly with `rust-src`.
- Detailed behavior: `ARCHITECTURE.md`.

---

## 2. Ground rules

1. **Read first.** Consult `ARCHITECTURE.md`, especially §5 (modules), §8 (main loop),
   §9 (publish policy), §12 (protocols), §14 (concurrency), and §15 (configuration).
2. **Preserve behavior unless asked otherwise.** Many current behaviors are deliberate
   workarounds or known debt; changing them silently can break the backend contract.
3. **No drive-by refactors.** Keep diffs focused. Do not reformat unrelated files.
4. **Do not add secrets.** Never copy real credentials into code, tests, logs, commits, or
   documentation. The current reset path contains hardcoded test credentials — do not
   propagate them.
5. **Do not switch runtime models.** Do not introduce Tokio, async tasks, or a second
   executor unless the task explicitly requests that migration.
6. **Do not edit generated or vendored trees.** Never modify `target/` or `.embuild/`.
   `Cargo.lock` is managed by Cargo; do not hand-edit it.
7. **No new panics in reachable runtime paths.** Avoid adding `unwrap()`, `expect()`,
   `panic!()`, or `todo!()`. When touching existing ones, prefer replacing them with
   `anyhow::Result` or a logged degraded path.
8. **Keep the external contract stable.** MQTT topics, channel numbering, payload shape, and
   NVS keys are consumed by other systems. Treat changes as breaking unless the task says so.
9. **State what you verified.** There is no automated test suite; hardware validation is often
   required. Say explicitly what was built and what still needs a device/broker.
10. **Update documentation.** If you change module boundaries, protocols, config, pins,
    timing, or security posture, update `ARCHITECTURE.md` in the same change.

---

## 3. Quick commands

The target is pinned in `.cargo/config.toml`, so commands from the repository root build for
`riscv32imc-esp-espidf`. You need the ESP-IDF environment; `scripts/build.sh` sources
`~/export-esp.sh` when `idf.py` is not on `PATH`.

| Task | Command | Notes |
|---|---|---|
| Fast type check | `cargo check` | Uses the configured embedded target |
| Debug build | `cargo build` | Output: `target/riscv32imc-esp-espidf/debug/ssn-esp32` |
| Release build | `cargo build --release` | Primary build gate; output under `.../release/` |
| Build wrapper | `scripts/build.sh [debug\|release]` | Defaults to release |
| Build + flash + monitor | `cargo run` | Runner is `espflash flash --monitor`; needs hardware |
| Flash an ELF | `espflash flash --monitor <path-to-elf>` | Debug or release artifact |
| Flash wrapper | `scripts/flash.sh [debug\|release]` | Builds, then uses `web-flash` |
| Release flash script | `./flash_riscv32imc.sh` | Uses `espflash` on the release artifact only |
| Tests | — | No tests exist; `harness = false` on the binary |

**Build requirement:** nightly Rust with `rust-src` (`rust-toolchain.toml`) and the ESP-IDF
toolchain. A plain host build is not a supported target.

**Formatting/linting:** no `rustfmt` or `clippy` configuration is checked in. Match surrounding
style. Do not run a repo-wide reformat as part of a focused change.

---

## 4. Where things live

| Path | Use it for |
|---|---|
| `src/main.rs` | Entry point, module wiring, reboot loop (~34 lines) |
| `src/app/mod.rs` | Initialization order and main loop |
| `src/app/state.rs` | Shared aliases, command queues + helpers, sensor-data store |
| `src/app/publish.rs` | Publish policy and percentage-change helper |
| `src/app/helpers.rs` | **Scaffold** — extracted MQTT/alarm/GPIO/LED helpers, not called by the loop |
| `src/devices/model.rs` | `SsnDevice` / `SsnDevices` model, periods, GPIO traits + `PinDriverBasicInput` impl, `DeviceType` / `DeviceConfig` |
| `src/devices/registration.rs` | **Hardcoded** device set (GPIO, ENS160, BMP180) |
| `src/devices/manager.rs` | **Single** device sampling/publishing path for every variant; degraded read modes |
| `src/devices/scheduler.rs` | Fast/Middle/Slow sampling scheduler |
| `src/drivers/bmp180.rs`, `aht21.rs`, `bmp.rs`, `ens160.rs` | I2C sensor adapters |
| `src/drivers/adc.rs` | ADC1 adapter |
| `src/protocol/topics.rs` | MQTT topic builders and topic/token parsers |
| `src/protocol/commands.rs` | Inbound command parser (`CommandError`) and queueing |
| `src/services/config.rs` | `Config` schema, NVS load/save/defaults, HTML rendering, reset path, identity overrides |
| `src/services/wifi.rs` | STA+AP setup, connect/retry, connection check |
| `src/services/mqtt.rs` | Broker/client configuration, MQTT client, callback dispatch, subscriptions, start message |
| `src/services/http.rs` | `/` and `/settings` handlers and HTML templates |
| `src/services/alarms.rs` | Alarm store and `check_alarms` transitions |
| `src/services/led.rs` | `Colors` and WS2812 status LED |
| `src/platform/i2c.rs` | `SharedI2c` adapter |
| `src/platform/interrupts.rs` | GPIO interrupt counters and helpers |
| `src/platform/mod.rs` | `lock_recover` mutex-poisoning recovery |
| `ARCHITECTURE.md` | Source of truth for current architecture and known gaps |
| `docs/README.md` | Build/flash/dev-container template (not architecture-specific) |
| `docs/diagram.mmd` | Older Mermaid overview; prefer `ARCHITECTURE.md` diagrams |

---

## 5. Current behavior you must account for

### Runtime and device processing

- `main()` loops on `app::run()`; errors propagate, then `esp_restart()`.
- The main loop processes **one MQTT publish command** and **one device command** per
  iteration. Both queues use `Vec::pop()` — **LIFO**.
- Device processing lives in `DeviceManager::process_all_devices` (`devices/manager.rs`), not in
  `app/mod.rs`. Only `app/helpers.rs` remains a scaffold.
- `DeviceManager` is the **only** device-processing path and covers every `SsnDevices` variant.
  Do not reintroduce an inline match in `app/mod.rs` or a second implementation.
- Devices are registered **hardcoded** in `devices/registration.rs`:
  - GPIO outputs `gpio-out3/4/5` (GPIO3/4/5);
  - GPIO input `gpio-in6` (GPIO6, any-edge, pull-up);
  - ENS160 `ens160-test` and BMP180 `bmp180` when initialization succeeds.
  - AHT21, ADC, `StoreInt`, and GPIO7 are commented out.
- `Config.devices` is parsed but **never used** to create devices.
- `is_active` and `is_paused` are stored but **never checked**. Every registered device is
  processed according to its period.
- Sensor read and MQTT parse failures are **degraded, not fatal**: see `ARCHITECTURE.md` §17.
  Do not add `unwrap()`/`todo!()` to these paths.

### Sampling and publishing

- `Fast` = every loop; `Middle` ≈ 20 s; `Slow` ≈ 60 s. Counters reset at the end of a loop.
- A channel with `last_sent_value == None` publishes on its first sample after boot.
- A publish is forced after **60 s** regardless of change (`MAX_PERIOD_REFRESH_MQTT`).
- `delta` is the minimum **positive** percentage change that triggers an early publish.
- `delta_over` is a **spike suppressor**: if the change from the last sent value is at or above
  it, the value is not published and becomes the new baseline.
- Sensor payloads are **bare scalar text**, QoS 1, not retained.
- Publish errors are currently discarded by callers.

### Configuration and storage

- NVS namespace `ssn_ns`, key `config`, JSON blob.
- Read buffer is a fixed **1 KB**. Any read/deserialize failure falls back to defaults.
- Identity fields (`account`, `object`, `ssn_id`, AP SSID/password, MQTT client ID) are
  **overridden at boot** and not persisted.
- GPIO8 low at boot triggers the reset path; it writes defaults and then a hardcoded test
  configuration.
- `Config::Display` renders **HTML** with passwords masked. It is used by `/settings` and by
  the MQTT `get` response. `Debug` does **not** mask secrets.

### MQTT

- Subscribe:
  - `/ssn/acc/{account}/raw_data` (accepted, not meaningfully handled)
  - `/ssn/acc/{account}/obj/{object}/commands/+`
  - `/ssn/acc/{account}/obj/{object}/commands/device/+/+/in`
- Publish:
  - sensors: `/ssn/acc/{account}/obj/{object}/device/{dev_id}/{channel}/out`
  - status: `/ssn/acc/{account}/obj/{object}/status/{event}` (retained for `start`)
  - command response: `/ssn/acc/{account}/obj/{object}/commands/response/{command}`
- Messages for other accounts are filtered out. Keep that filter.
- Generic commands use 3 tokens after `obj`; device commands use 6.
- The current `get` response is HTML, not JSON. Treat that as an open question, not an
  invitation to silently change the protocol.

### Alarms and GPIO

- Alarms are in-memory, added at boot, and match **time-of-day only**; the date is ignored.
  Commands are emitted on state transitions and always use channel 0.
- Current alarms are test data (2025 dates).
- GPIO interrupt callback increments atomics and notifies the main task. It currently also
  takes a `std::sync::Mutex` — this is an ISR-safety risk; do not add more locking there.
- The per-pin interrupt table is fixed at `MAX_TRACKED_PIN = 10`; higher pins count only in the
  total (no panic).
- Only one notification is consumed per loop; counters coalesce.

### Security and operations

- MQTT is plaintext `mqtt://`; no TLS.
- AP is always on with a fixed default password.
- HTTP has no authentication.
- No OTA, secure boot, flash encryption, or application watchdog is configured.
- No automated tests, no CI, no formatter/linter configuration.

---

## 6. Change recipes

### A. Add or change a sensor/device type

1. Add an adapter under `src/drivers/` (constructor + `get_data`-style method, typed error).
2. Add fields/variant to `SsnDevices` in `src/devices/model.rs`.
3. Register the device in `devices/registration.rs` (hardcoded; `Config.devices` is still unused).
4. Add the match arm in `DeviceManager::process_all_devices` (`devices/manager.rs`). It is the
   only device-processing path; return/log a typed error instead of adding a panic.
5. Keep logical channel indexes stable. Current mappings are documented in `ARCHITECTURE.md`
   §7.3.
6. If you add `DeviceType`/`DeviceConfig` fields, remember that config-driven creation is not
   implemented; either wire it or state that the fields are declarative only.
7. Update `ARCHITECTURE.md` §5, §7.3, §10, and §11 if modules/pins/buses are involved.

### B. Change publish behavior

- Use `app::publish::handle_sensor_value_update` for sensor values; do not hand-roll
  `client.enqueue` except for status/response messages.
- First-sample publishing is keyed on `last_sent_value == None`; preserve that when adding
  channels.
- Do not change `delta`/`delta_over` semantics without confirming with maintainers; the
  backend may rely on the current cadence and filtering.
- Changing payloads from scalar text (for example, to JSON) is a **breaking protocol change**.
  Update `ARCHITECTURE.md` and the open questions.
- Preserve QoS 1 and the non-retained behavior for sensor topics.
- If you change error handling, decide deliberately whether publish failures should be logged,
  retried, or propagated; today they are ignored.

### C. Add an MQTT command or topic

1. Add the builder/parser in `src/protocol/topics.rs`; do not scatter topic strings.
2. In `protocol::commands::handle_message`, only parse, validate, and enqueue. Do not block the
   callback with sleeps, long NVS work, or sensor reads.
3. Return a `CommandError` variant for new malformed-input cases; never `unwrap()` or panic on
   subscriber input.
4. Extend the command queue / `SsnDeviceCommand` if needed, and apply the command in the main
   loop or `DeviceManager`.
5. Preserve the account filter.
6. Update the topic table in `ARCHITECTURE.md` §12.3 and any open questions.

### D. Change configuration

- Update `Config`, its `Default`, and serde derives together. Keep JSON round-tripping.
- The 1 KB read buffer is a hard limit today. If configuration can grow, update the buffer in
  `src/services/config.rs` and verify the failure path.
- Do not add credentials to defaults, test JSON, examples, or logs.
- If you touch the reset path, remove or feature-gate the hardcoded test configuration rather
  than extending it.
- `Config::Display` is HTML; `Debug` leaks secrets. Use the display form for user-facing
  output and avoid `{:?}` on `Config`.
- Adding `DeviceConfig` fields alone changes nothing at runtime until the mapping and device
  construction are implemented.

### E. Touch GPIO or interrupts

- Check the pin map in `ARCHITECTURE.md` §11 first. GPIO4 is both an output and a pin used by
  the commented ADC example; do not enable both.
- Keep the ISR callback minimal: no mutexes, allocation, logging, or blocking calls. Use
  atomics and `Notification`.
- Per-pin counters only cover pins `< MAX_TRACKED_PIN` (10); out-of-range pins are counted only
  in the total. Increase the constant in `platform/interrupts.rs` if a pin needs a per-pin count.
- Remember `notification.wait(0)` consumes at most one notification per main-loop iteration.
- Keep `PinDriverBasicInput` / `PinDriverTraitOutput` implementations object-safe.

### F. Refactor or extend the loop

- Do it incrementally, one behavior at a time, preserving external behavior.
- Move pure logic first (topic parsing, publish policy, alarm transitions) so it can be tested.
- `DeviceManager::process_all_devices` is the single device-processing path. Add new sampling
  there; do not reintroduce an inline match in `app/mod.rs`.
- `TimeManager` only resets counters in `reset_if_needed()` at the end of an iteration; keep
  that timing if you touch the scheduler.
- Keep `main.rs` as entry/wiring and `app/mod.rs` as lifecycle/composition; put service details
  in their own modules.

### G. Dependencies

- Keep the pinned ESP-IDF stack (`esp-idf-svc = "=0.51.0"`, etc.) unless the task explicitly
  upgrades it.
- Avoid large or async-heavy crates; flash size matters (`release` uses `opt-level = "s"`).
- Do not add wildcard versions (`*`) for new dependencies; the existing `smart-leds = "*"` is
  an exception, not a pattern.
- If you add a dependency, verify it builds for `riscv32imc-esp-espidf`, not just host.

---

## 7. Concurrency rules

- There is no async runtime. Use the existing blocking/queue/notification patterns.
- Observed lock order is **device → I2C**. Do not invert it.
- Do not add blocking sleeps or long I2C transactions while holding a device lock.
- Do not block the MQTT callback task.
- Do not take a blocking `Mutex` in ISR context. Prefer atomics.
- Commands are LIFO and one-per-loop. If you change to FIFO or drain all commands per loop,
  update the behavior deliberately and document it.
- Shared state lives in `Arc<Mutex<...>>` or `lazy_static`; keep critical sections short.

---

## 8. Error handling rules

- Initialization: use `anyhow::Result`, `?`, and `.context(...)`.
- Sensor reads: log the typed error and skip the cycle; never add `unwrap()`/`panic!()`/`todo!()`.
  Keep the previous values so the next period retries.
- MQTT parsing: return a `protocol::commands::CommandError`; the MQTT service logs it and drops
  the message. Never panic on subscriber input.
- Shared locks: use `platform::lock_recover` (or the same recovery pattern) instead of
  `lock().unwrap()` in runtime paths.
- `app::run` errors reboot the device. Startup errors are fatal; sensor initialization errors
  are non-fatal by design.
- `get_config()` currently falls back to defaults on any failure. If you change this, preserve
  a clear diagnostic and avoid silently overwriting valid configuration.
- If you change MQTT publish error handling, be explicit about whether failures are logged,
  retried, or propagated.
- Keep `ARCHITECTURE.md` §17 ("Degraded modes") in sync when you add or change a failure path.

---

## 9. Security rules

- Never commit real credentials. Treat the current hardcoded test configuration as a bug.
- Do not claim the firmware is secure: MQTT is plaintext, HTTP is unauthenticated, the AP is
  always on, and NVS is not encrypted by configuration in this repo.
- Do not add endpoints or logs that expose secrets. `Config::Display` masks passwords;
  `Debug` does not.
- If you add a new network service, consider authentication, exposure on the AP interface, and
  resource limits.
- If you add OTA/provisioning, update the threat model and `ARCHITECTURE.md` §18.

---

## 10. Verification checklist

Run or confirm the following before finishing:

- [ ] `cargo check` passes.
- [ ] `cargo build --release` passes for `riscv32imc-esp-espidf`.
- [ ] If behavior changed, describe hardware validation (sensor, GPIO, Wi-Fi, MQTT broker) or
      explicitly state that it was not tested on hardware.
- [ ] No new secrets in code, logs, tests, or docs.
- [ ] No new panics in runtime paths; existing ones touched were improved or left with a
      documented reason.
- [ ] MQTT topics, channel numbers, payload shape, and NVS keys are unchanged or the change is
      called out as breaking.
- [ ] `ARCHITECTURE.md` is updated for architecture/protocol/config/pin/timing changes.
- [ ] `AGENTS.md` is updated if workflow or ground rules change.
- [ ] No generated files (`target/`, `.embuild/`) were modified.
- [ ] The diff is focused; unrelated files were not reformatted.

---

## 11. Common traps

| Trap | What to know |
|---|---|
| `SsnDevices::Bmp` | The arm logs an error and skips the device; BMP280/BME280 processing is still unimplemented |
| Degraded sensor reads | Read failures are logged and skipped; previous values are retained, so a fault can look like stale data until the next successful read |
| Degraded MQTT parsing | Malformed topics/payloads are dropped with a warning (`CommandError`), not acknowledged |
| Mutex recovery | `platform::lock_recover` recovers poisoned locks; contents after a panic may be inconsistent |
| `get_config` fallback | Any error, including a >1 KB blob, silently writes defaults |
| `Config::Display` | Renders HTML and is used by the MQTT `get` response |
| `Config` debug log | `Debug` prints passwords; `Display` masks them |
| `is_active` / `is_paused` | Stored but ignored; devices cannot be disabled remotely |
| `Config.devices` | Parsed but ignored; registration is hardcoded in `devices/registration.rs` |
| GPIO command semantics | `GpioInput` command value is overwritten by the pin read; `GpioOutput` ignores the channel |
| Interrupt pin range | Per-pin counters only cover pins `< MAX_TRACKED_PIN` (10); higher pins count only in the total |
| Flash scripts | `scripts/flash.sh` uses `web-flash`; `flash_riscv32imc.sh` uses `espflash` |
| Scaffold module | `app/helpers.rs` is compiled but not called by the loop |
| Generated trees | `target/` and `.embuild/` are build artifacts; never edit them |

---

## 12. Decisions requiring confirmation

Do not guess on these; ask or leave the current behavior intact and document the question:

1. Should the MQTT `get` response be JSON? The current HTML output appears accidental.
2. Are alarms date-specific, daily time-of-day, or cron-like?
3. Should `account` / `object` / `ssn_id` come from NVS, MAC derivation, or provisioning?
4. Which sensor set is the production configuration? AHT21, ADC, `StoreInt`, GPIO7, and
   BMP280/BME280 are partially implemented.
5. Should the AP remain always on, or only during provisioning? What is the password policy?
6. What is the backend contract for payload format, cadence, and command acknowledgement?
7. What are the flash size, partition layout, and OTA requirements?
8. Should command queues be FIFO and drained every cycle?
9. Should the MQTT client re-create itself on reconnect, or rely on ESP-IDF?
10. Is interrupt latency critical enough to require ISR-safe counters only?

---

## 13. Documentation maintenance

- `ARCHITECTURE.md` is the detailed source of truth for current behavior, data flow, and
  known gaps.
- Keep this file short and operational; link to architecture sections instead of duplicating
  long explanations.
- If you discover a new constraint, trap, or failed assumption while working, add it to
  `ARCHITECTURE.md` §19/§21 and, if it affects agent workflow, to this file.
- `docs/README.md` covers build/flash/dev containers; `docs/diagram.mmd` is an older overview.
  Do not treat either as authoritative over `ARCHITECTURE.md`.

---

*End of draft. This file reflects the repository as of 2026-11-17 and should be revised when
the architecture or working agreements change.*
