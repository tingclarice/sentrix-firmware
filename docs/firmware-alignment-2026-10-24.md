# Firmware alignment — 2026-10-24

## Approved PRD

1. **Problem:** Legacy MQTT topics, payloads, and plaintext transport do not match the current Go ingest handlers. GPIO2 does not distinguish connection states.
2. **Success criteria:** Correct reconnect handshake; one valid NTP-timestamped batch per minute; CA-verified TLS on 8883; timer-driven GPIO2 patterns for configuration, BLE, Wi-Fi, time sync, MQTT, connected, sensor warning, and fatal error.
3. **Scope:** Firmware, documentation, and reproducible build configuration. Preserve BLE encryption, CT calibration/sampling, IR decoding, and external indicators. API changes, real mesh child transport, and flashing hardware are out of scope.
4. **Constraints:** Match actual Go code over the stale batch example in AGENTS.md; six-digit PIN strings; exact group mesh ID; preserve saved network configuration; never publish invalid current as zero.
5. **Plan:** Inspect Go parsers and ingest behavior; update transport and provisioning; replace publishers; add GPIO2 patterns; compile and check compatibility; commit the firmware checkpoint. Revert that firmware commit to roll back code without touching NVS.
6. **Open questions:** No implementation blocker. Deployment requires the broker host, credentials, matching issuing root CA, factory-seeded MAC/PIN, and assigned mesh ID. Physical board behavior requires bench verification.

Felix approved implementation with “okay proceed”.

## Decisions

- Use `sentrix-api/internal/ingest/messages.go` as the wire contract: handshake `mac_address`, batch reading `mac`.
- Preserve periodic 200 ms CT sample windows each second. Use the latest fresh valid window for the minute batch rather than inventing a continuous sampling claim.
- Retain legacy provisioning `group_id` as an alias for `mesh_id`. A UUID saved by an older client must be replaced by the real mesh ID; it cannot be resolved locally.
- Keep IR diagnosis on Serial; send decoded setpoints only as `target_temperature`. There is no ambient temperature sensor.
- Use existing Ticker support for GPIO2, even if RGB is disabled. Only its callback writes GPIO2; fatal failures also have a visible state.
- Keep PubSubClient 2.8. Its publishes are QoS 0; the LED means broker connected and handshake locally sent. No claim of API acceptance, reliable offline delivery, or stored telemetry.
- Add a pinned PlatformIO build because the previous README referenced a missing compile script. Target is ESP32 Dev Module with Huge APP (3MB No OTA).

## API findings — left unchanged

These are code-review findings, not confirmed production incidents. Paths are relative to the adjacent `sentrix-api` checkout.

| Finding | Evidence and effect |
|---|---|
| Telemetry is not authenticated by the PIN handshake | `internal/ingest/ingest.go`, `processReading`: checks a known/claimed MAC and matching group mesh, but never checks a successful handshake session or broker publisher identity. `processBatch` also accepts `offline_nodes` without such a session. Broker ACLs and credentials therefore determine who can inject readings/offline reports. |
| No device acknowledgment | `processHandshake` logs failures and updates an existing database row, with no reply topic. Firmware cannot distinguish accepted from rejected PIN/group registration. |
| No automatic stale-device transition in this ingest service | `markOffline` runs only when a root reports `offline_nodes`. A standalone root losing power/network cannot report itself. No MQTT LWT/status topic is consumed. `last_seen` can age while status stays ACTIVE/CALIBRATING unless another service handles it. |
| Inconsistent timestamp validation | `parseBatch` rejects nonpositive Unix timestamps but has no age/future limit; `parseHandshake` accepts absent/nonpositive timestamps and stores server receipt time as `last_seen`. Firmware still requires a plausible NTP clock. |
| MQTT subscriptions are not retried after a failed SUBACK | `Connect` logs a subscription failure and continues; another connection event is needed to retry. |
| Missing contract document makes its regression test skip | `internal/ingest/iotformat_test.go` reads `docs/iotformat.md`, which is absent in this checkout, then calls `t.Skipf`. Existing parser tests remain present, but that document-to-parser comparison is not running. |
| Backend TLS policy is configurable rather than enforced | `internal/config/config.go` requires a nonempty `MQTT_BROKER_URL`; `Connect` passes it to Paho without requiring a TLS scheme. Ensure deployment config uses the secure endpoint. Firmware enforces TLS regardless. |

The firmware's untracked AGENTS.md retains an older batch example. It was supplied as project instructions and was not edited or included in the implementation commit. The actual API parser and this README use the correct `mac` field.

## Verification

- Baseline Go ingest/config unit tests: 27 passed across two packages before implementation.
- Full PlatformIO ESP32 build passed with the pinned dependencies. Firmware uses 2,048,289 / 3,145,728 flash bytes (65.1%) and 64,112 / 327,680 static RAM bytes (19.6%). `firmware.bin` and the combined `firmware.factory.bin` are under `.pio/build/esp32dev/`.
- Four GPIO2 compile-time assertions passed, checking the two/three/four-flash boundaries and connected heartbeat. Existing RGB and temperature-conversion assertions also compiled.
- Six ad hoc compatibility smoke tests passed against an unchanged copy of Go's real `messages.go` parser in the ignored `.pio/contract_check/` directory: README handshake; README batch; missing setpoint plus valid zero current; rejection of the legacy batch MAC field; leading-zero PIN; wrong mesh/topic rejection. These exercise the parser and documented examples, not a live device or network.
- Checked that the sketch's handshake, batch, and reading JSON field names exactly match those examples. Verified the embedded ISRG Root X1 DER SHA-256 fingerprint: `96bcec06264976f37460779acf28c5a7cfe8a3c0aae11a8ffcee05c0bddf08c6`.
- `git diff --check` passed. API working-tree status matches the initial user-owned changes; no API files were edited.
- No ESP32 was flashed. Wi-Fi, BLE/NVS migration, NTP, TLS/broker behavior, sensor calibration, and visible LED timing still need a physical board and deployment credentials.

## Bench check

1. Boot with a valid biased CT input and no credentials: slow configuration blink. An unwired/faulty sensor produces the faster warning instead.
2. Connect BLE, scan Wi-Fi, and provision with Go's mesh ID: setup blink persists during blocking operations. Reboot to confirm saved configuration loads.
3. Disconnect the AP: two flashes; restore it while blocking UDP/123: three flashes on a cold boot without a retained valid clock.
4. Supply a wrong broker password or CA: four flashes. Restore valid configuration: one pulse every three seconds after the handshake is sent.
5. Verify Go logs the handshake, then observes one fresh reading per minute for the claimed device. Confirm an IR setpoint appears as `target_temperature`, with no fabricated measured temperature.
6. Disconnect/reconnect Wi-Fi or the broker: confirm another handshake, no burst of queued samples, and the appropriate GPIO2 pattern throughout.
7. Make the CT input invalid: warning blink and no fake zero reading. Test GPIO2 with RGB disabled and set LED polarity for the board variant.
