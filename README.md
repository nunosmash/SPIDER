[README.md](https://github.com/user-attachments/files/26697123/README.md)
# SPIDER

**BLE MIDI gateway** — bridge **Bluetooth LE MIDI** controllers to **USB MIDI** and **DIN MIDI**, with up to **three** simultaneous BLE devices, **fixed slot routing**, and **HOST** / **CLIENT** roles for stage and DAWless setups.

Part of the **Ash Sound Works** custom tools for live and studio — by sound artist and producer **Ash (Ahn Sunghoon)** · [GitHub](https://github.com/nunosmash)

[![SPIDER](https://raw.githubusercontent.com/nunosmash/SPIDER/refs/heads/main/SPIDER_img.jpg)](https://youtu.be/Al2et1Eq5mc?si=DPkdCoxhtiU5xMxv)

---

## Why SPIDER?

Running cables across a crowded **pedalboard** or needing **remote** control for other players is awkward. SPIDER was built around **BLE MIDI** on a dedicated **nRF52840** radio (e.g. **nice!nano** class hardware): low power, battery-friendly, and focused on **wireless MIDI** rather than sharing the stack with a desktop OS BLE stack.

---

## Key features

- Up to **3** BLE MIDI controllers connected at once  
- **USB MIDI** via **TinyUSB** — **3 independent virtual MIDI ports** (one per slot)  
- **DIN MIDI** — **Slot 0** routes to DIN when appropriate (see routing below)  
- **7.5 ms** BLE connection interval class; low-latency slot timing (~**5 ms** average response in typical use)  
- **TX power** auto-adjust **0 ~ +8 dBm** from **RSSI**  
- **Battery monitoring** for the unit and connected gear (where supported)  
- **OLED** UI with power-save brightness (USB vs battery profiles)  
- **HOST** and **CLIENT** modes  

---

## Routing architecture

| Role | Behavior (summary) |
|------|---------------------|
| **Slot 0 (master)** | USB ↔ BLE ↔ DIN MIDI |
| **Slots 1–2** | USB ↔ BLE |
| **Port pinning** | Each slot stores **MAC + name** so devices **always** map to the same slot regardless of power-on order |

### HOST mode

- **USB data connected:** BLE → **USB MIDI** (DIN blocked for that path per firmware logic)  
- **No USB data:** Slot 0 → **DIN MIDI** output  
- Slot management: **RESCAN**, **CLEAR SLOT**, auto-reconnect  

### CLIENT mode

- Uses **one slot** to join a host  
- Remembers first trusted host **MAC** (save window ~30 s on first connect); optional **block** for unwanted devices  

---

## OLED / status (summary)

- Top line: **LOW BAT**, **WEAK SIG**, **M** (DIN active), TX power hints  
- Per slot: **EMPTY** / **OFFLINE** / **CONNECTED**  
- MIDI traffic: ▽ in / △ out  
- Battery: **%** or **N/A** if unknown  

### Antenna bars (signal guide)

- **6–4 bars** — best; within ~**10 m** reliable  
- **3 bars** — stable for normal use  
- **≤2 bars** — TX power ramps up; maintain connection  
- **0 bars** — unreliable; move closer  

**Tip:** Keep **3+ bars** within ~**10 m** for dependable live use.

---

## Ableton Live (quick setup)

**Preferences → Link / Tempo / MIDI**

- **Input:** SPIDER MIDI **Port 1, 2, 3** → **Track** ON, **Remote** ON as needed  
- **Output:** enable if you send MIDI back to wireless gear  

| Slot | USB MIDI port |
|------|----------------|
| SLOT 1 | Port 1 |
| SLOT 2 | Port 2 |
| SLOT 3 | Port 3 |

---

## Sync calibration

BLE adds variable delay; align with audio using **Track Delay** like any MIDI path. Many setups start around **−10 ms ~ −20 ms** (often **−18 ms**) and refine by recording MIDI out + external audio and matching to the grid.

---

## Firmware update

1. Connect SPIDER over **USB**.  
2. Hold the **encoder button** for **5+ seconds** to enter firmware update mode (**USB mass storage** / removable drive).  
3. Copy the released firmware file to the mounted drive; device updates and reboots.  

(Exact filename and format follow your published release notes.)

---

## Use cases

- **Band live** — multiple wireless controllers into one DAW reliably  
- **DAWless** — Slot 0 → hardware synths via DIN  
- **Multi-computer** — advanced MIDI routing between several hosts (see full manual)  

---

## Repository contents

Depending on branch or release:

- Firmware (Zephyr / Arduino / etc. — match your repo)  
- Schematics / PCB / case files  
- Build and flash instructions  

Full wiring, menus, and edge cases: **`spider-manual.html`** in the Ash Sound Works bundle.

---

## Hardware Specifications (BOM)

| Category | Component Description | Qty |
| :--- | :--- | :---: |
| **MCU** | nRF52840 (nice!nano v2.0 SuperMini) | 1 |
| **Display** | 0.91" OLED Module (128x64) | 1 |
| **Optocoupler** | 6N138 High-Speed Optocoupler | 1 |
| **Diode** | 1N4001 Rectifier Diode | 1 |
| **Capacitor** | 100μF Electrolytic Capacitor | 1 |
| **Input** | EC11 Rotary Encoder w/ Push Switch | 1 |
| **Connectors** | PJ-320A 3.5mm Stereo Jack | 2 |
| **Power** | SS-12F23 Slide Switch | 1 |
| **Resistors** | 220Ω (x2), 330Ω (x2), 150Ω, 1kΩ, 4.7kΩ, 10Ω, 30Ω | 9 |

## Documentation

- Full manual: `spider-manual.html`  
- Other projects: [github.com/nunosmash](https://github.com/nunosmash)

---

## Disclaimer

- **BLE** performance depends on environment, antennas, and interference — always **test** before a show.  
- Verify **DIN polarity** (Type A/B) when connecting legacy MIDI gear.  
- DIY: observe battery and USB power safety.

---

## License

See the `LICENSE` file in the repository root.
