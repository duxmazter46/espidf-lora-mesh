# espidf-lora-mesh

ESP32 LoRa mesh with TDMA scheduling.

**Credits:** The LoRa PHY driver (`components/phy/phy.c`) is based on code from [nopnop2002/esp-idf-sx127x](https://github.com/nopnop2002/esp-idf-sx127x). Thanks to nopnop2002 for the SX127x ESP-IDF component.

**Status:** Only **single-hop** operation is currently stable and supported.  
Multi-hop routing will be added later.

---

## 1. Configure LoRa PHY (Kconfig)

1. Run `idf.py menuconfig` in this project.
2. Navigate to: `Component config -> LoRa PHY Configuration`.
3. Set the radio parameters as needed:
   - **LoRa Frequency (Hz)** (`CONFIG_LORA_FREQUENCY`) – e.g. `915000000` for 915 MHz.
   - **Spreading Factor (7–12)** (`CONFIG_LORA_SPREADING_FACTOR`) – higher = longer range, lower data rate.
   - **Bandwidth Index** (`CONFIG_LORA_BANDWIDTH`) – `7 = 125kHz`, `8 = 250kHz`, `9 = 500kHz`.
   - **Coding Rate (1–4)** (`CONFIG_LORA_CODING_RATE`) – higher = more robust, lower throughput.
   - **Preamble Length** (`CONFIG_LORA_PREAMBLE_LENGTH`).
   - **Sync Word** (`CONFIG_LORA_SYNC_WORD`).
   - **Enable CRC** (`CONFIG_LORA_ENABLE_CRC`) – normally leave enabled.

Rebuild and flash after changing PHY settings.

---

## 2. Map MAC addresses to node IDs (`node_config.c`)

The `node_config` component assigns each physical board a logical mesh node ID by matching its Wi‑Fi STA MAC address.

1. Open `components/node_config/node_config.c`.
2. In `node_map[]`, each entry has:
   - `mac[6]` – the board’s Wi‑Fi STA MAC address.
   - `node_id` – the logical mesh node ID (0 is typically the root).
3. For each board in your deployment:
   - Find its Wi‑Fi MAC (e.g. from `idf.py monitor` boot logs or device label).
   - Add or edit an entry in `node_map[]` so that MAC → desired `node_id` is unique.
4. Ensure the root board’s MAC is mapped to `node_id = 0`, matching `ROOT_NODE_ID`.

Only nodes present in `node_map[]` will get a valid ID; unknown MACs return `0xFFFF` and are treated as invalid.

---

## 3. Configure reporting intervals (`node_config.c`)

Reporting intervals are defined in the same file:

- `REPORTING_INTERVAL_S` – default interval (seconds) used when a node has no override.
- `node_reporting_interval_s[]` – per‑node overrides indexed by `node_id`:
  - `0` entry is the root (usually `0` = “use default”).
  - Non‑zero entries override the default for that node.

To change intervals:

1. Open `components/node_config/node_config.c`.
2. Adjust `REPORTING_INTERVAL_S` to change the global default (e.g. `60` for 1 minute).
3. Edit `node_reporting_interval_s[]` so each node ID has the desired interval:
   - Example: `[1] = 60` (1 min), `[2] = 600` (10 min), `0` means “fall back to `REPORTING_INTERVAL_S`”.

Rebuild and flash both root and leaf nodes after changing intervals so TDMA schedule and node behavior stay consistent.

---

## 4. Basic `root_cli` usage

The root node exposes a simple text CLI on its UART (`idf.py monitor`).

1. Flash this project to the root board.
2. Connect to the root serial port, e.g. `idf.py -p COMx monitor`.
3. When you see the `root>` prompt, you can type commands:
   - `help` – list all available commands.
   - `start` – start mesh runtime, probe nodes, and build the TDMA schedule.
   - `status` – show which node IDs are ONLINE/OFFLINE.
   - `cycle` – show current TDMA cycle and time to next cycle.
   - `tdma` – print the full TDMA schedule.
   - `ping <node_id>` – ping a specific node.
   - `sync <t_s_or_ms>` / `brdsync [t_s_or_ms]` – set the root clock and broadcast SYNC.
   - `csvdump` – dump buffered data as CSV (`node_id,timestamp,payload`) to the terminal.

Use the CLI on the root board only; non‑root boards just run the node firmware.
