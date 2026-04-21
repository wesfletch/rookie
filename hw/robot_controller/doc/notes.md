# Robot Controller PCB — Design Notes

## Overview

Base PCB assembly for a ground-based differential drive robot. Primary MCU is the **Raspberry Pi Pico**. Powered by a **4S LiPo battery** (12V–16.8V). Communicates with an external SBC via USB.

---

## Motor Drive

**Configuration:** Differential drive, 2 motors, 1A max per motor.

**Selected driver:** DRV8876 (x2, one per motor)
- Single H-bridge per package
- 6.5V–37V VM — compatible with 4S LiPo direct or regulated 12V
- 3.5A continuous — well-headroomed for 1A load
- Integrated current sense output (IPROPI) — direct ADC connection for stall detection and current monitoring
- 3.3V logic compatible — no level shifting needed
- HTSSOP-16 with exposed thermal pad — manageable with hot air and solder paste

**Considered and rejected:**
- DRV8833 — dual H-bridge, 1.5A/ch, but VM max 10.8V is incompatible with 4S LiPo
- DRV8800PWP — single H-bridge, 8–38V, but no integrated current sensing; otherwise similar to DRV8876

**Assembly note:** The exposed thermal pad requires solder paste and hot air or reflow. At 1A operation thermal stress is low, but the pad should be properly bonded. Include thermal relief vias in the copper pour beneath the pad.

---

## Power Architecture

### Input: 4S LiPo
- Nominal: 14.8V | Fully charged: 16.8V | Low cutoff: ~12V
- Main connector: XT60

### Input Protection (in order from battery)

1. **Blade fuse** — automotive-style, field-replaceable, sized to total worst-case draw
2. **Reverse polarity protection** — P-channel MOSFET (e.g., DMG2305UX or Si2333DS)
   - Source → VBAT+, Drain → protected rail, Gate → GND through resistor
   - Add a 12V zener between Gate and Source to clamp Vgs within safe limits at full 4S charge voltage
3. **TVS diode** — across VBAT after fuse (e.g., P6KE18A, 18V standoff) to clamp inductive transients
4. **Bulk capacitance** — 470µF–1000µF electrolytic, 25V rating, after protection stage; handles motor inrush and prevents buck converter input sag

### Regulated Rails

| Rail | Purpose | Recommended Part | Notes |
|------|---------|-----------------|-------|
| 12V | Motor supply (DRV8876 VM) | LM2596-12 | Fixed 12V variant, 3A, ~$0.60–0.80; larger inductor required |
| 5V | Pico VSYS + SBC power | TPS54531 or equivalent 5A buck | SBC can pull 3–5A; do not undersize this rail |
| 3.3V | Peripheral sensors beyond Pico's 3V3 pin | TLV1117-33 (SOT-223) | Simple, 1A, two-cap BOM; Pico handles its own 3.3V internally |

**Note on 12V motor rail:** Consistent 12V VM simplifies open-loop PWM control — a given duty cycle produces the same motor speed regardless of battery state of charge. The alternative (battery-direct VM with duty cycle compensation in firmware, or encoder-based velocity PID) is viable but adds firmware complexity.

### Battery Monitoring and Protection

| Function | Implementation |
|----------|---------------|
| Pack voltage | Resistor divider → Pico ADC pin |
| Pack current | INA228 or shunt + ADC |
| Undervoltage lockout (UVLO) | TPS3840 supervisory IC — disables buck converters below ~12V (3.0V/cell) to prevent over-discharge |

### Charging

Charging is handled externally. Expose:
- **XT60** main pack connector
- **JST-XH 5-pin** balance connector header (passthrough to pack balance lead)

Do not integrate a charger. Onboard 4S CC/CV + balancing circuitry is complex, costly, and difficult to make safe on a prototype. External LiPo chargers (iCharger, Junsi, etc.) handle this correctly and are already certified.

---

## Sensors

### Onboard

| Sensor | Part | Interface | Notes |
|--------|------|-----------|-------|
| IMU | ICM-42688-P or LSM6DSO | SPI | 6-axis; core motion sensing |
| Magnetometer | LIS3MDL | I2C | Absolute heading reference; prevents heading drift from IMU integration alone. Alternative: ICM-20948 (IMU + mag combo) |
| Temperature | TMP102 | I2C | Motor driver thermal monitoring; supplements DRV8876 nFAULT binary output with actual temperature data |

### Connector-Based

| Sensor Type | Connector | Interface | Notes |
|-------------|-----------|-----------|-------|
| Encoders | JST (x2–4) | GPIO (quadrature) | Pico PIO state machines handle quadrature decoding efficiently |
| Lidar / GPS / telemetry | UART headers (x2–3) | UART | Most external ranging and positioning sensors speak UART |
| I2C expansion | Qwiic/Stemma QT (JST-SH 4-pin) | I2C | Plug-and-play sensor ecosystem |
| Ultrasonic proximity | 4-pin header | GPIO (trigger + echo) | HC-SR04 compatible |
| Cliff / IR sensors | GPIO header | Analog or digital GPIO | Important for ledge/stair detection |
| Bump switches | GPIO header | Digital GPIO | Mechanical contact; reliable and cheap |
| Servo / PWM outputs | PWM header | PWM GPIO | Future actuator expansion; low cost to include |

### Skipped (rationale)

- **ToF sensors (VL53L1X etc.)** — placement-sensitive; better as external modules positioned freely on the chassis
- **Camera** — connect to SBC, not Pico
- **GPS onboard** — antenna placement is a separate problem; UART header is sufficient

---

## Other Connections and Features

| Feature | Notes |
|---------|-------|
| USB | Pico native USB exposed with ESD protection (e.g., PRTR5V0U2X); primary SBC communication link |
| SWD debug header | 3-pin (SWDIO / SWDCLK / GND); required for firmware development |
| E-stop input | Hardware kill line tied to DRV8876 nSLEEP pins; must be independent of Pico and SBC firmware |
| Status LEDs | At minimum: power-good per rail, user-controllable RGB for system state indication |
| CAN transceiver | Optional; worth considering if daisy-chaining boards or using CAN-bus actuators later |

---

## Assembly Considerations

- All hand-assembled; prefer packages with larger pad geometry where functionally equivalent options exist
- SOT-223 (TLV1117-33) and HTSSOP with exposed pad (DRV8876, LM2596) are manageable with hot air and solder paste
- 0.65mm pitch is the finest pitch on the board; workable but requires flux and care
- Thermal pads on motor drivers and buck converters should be properly bonded; include vias to inner/back copper for heat spreading
- Use WEBENCH (TI) to generate and verify passive component values for buck converter designs

---

## Hand Assembly Process

### Process Overview

1. **Visual inspection** — verify boards arrived without obvious defects; check dimensions and layer stackup
2. **Apply solder paste** — via stencil (preferred) or syringe, to all SMD pads
3. **Place components** — tweezers or vacuum pen; place passives and small ICs first, large ICs and connectors last
4. **Reflow** — hot air station or reflow oven
5. **Hand solder** — through-hole connectors, JST headers, fuse holder, any SMD components missed in reflow
6. **Inspect** — under magnification; check for solder bridges, cold joints, and tombstoned passives
7. **Clean** — isopropyl alcohol (99%) and brush to remove flux residue
8. **Bring-up** — see section below; bench supply before battery

### Tools

#### Must Have

| Tool | Notes |
|------|-------|
| Temperature-controlled soldering iron | Hakko FX-888D (benchtop), TS101, or Pinecil (USB-C portable). Fine chisel or conical tip for general work; larger chisel for connectors. |
| Hot air station | Required for DRV8876 and LM2596 thermal pads. A basic 858D-style unit (~$30–50) is adequate. |
| Flux | No-clean flux pen for general work; tacky paste flux (jar) for hot air and drag soldering. Use liberally. |
| Fine tweezers | Straight and angled pair minimum. Anti-magnetic, ESD-safe. |
| Multimeter | Continuity, voltage, resistance. Check every rail before applying power. |
| Solder — 63/37 tin-lead, 0.5–0.8mm | Leaded solder has lower melting point, better wetting, and a clean eutectic point. Meaningfully easier than lead-free for hand assembly. Lead-free only necessary for regulatory compliance on a product. |
| Isopropyl alcohol (99%) + brush | Flux cleanup. Lower concentrations leave water residue. |
| Solder wick | Bridge removal and excess solder. Pre-flux before use. |
| Bench power supply with current limiting | Essential for safe bring-up. 30V/5A covers all needs on this board. |

#### Makes Life Significantly Easier

| Tool | Notes |
|------|-------|
| Stereo microscope or USB microscope | Transformative for 0.65mm pitch inspection and rework. 10x–45x stereo scope is most comfortable for extended work; USB microscope is cheaper. High-value investment if you plan to build more than one board. |
| Solder paste + PCB stencil | Order stencil from your fab (JLCPCB and PCBWay both offer them cheaply). Uniform paste application causes components to self-align during reflow. Dramatically more consistent than hand-applying solder per pad. |
| PCB holder / vise | Keeps board stable and at a comfortable working angle. |
| Vacuum pen | Rubber bulb or motorized pickup for placing small components without disturbing neighbors. Helpful for 0402 passives and fine-pitch ICs. |
| Fume extractor | Flux smoke is irritating and mildly harmful. A desktop unit with activated carbon filter is adequate. |

#### Nice to Have

| Tool | Notes |
|------|-------|
| Reflow oven or hotplate | More repeatable than hot air for full-board reflow. A converted toaster oven with thermocouple controller is a common DIY solution. |
| PCB preheater | Reduces thermal shock on larger boards; helps solder flow evenly. Less critical at this board size. |

### Package Selection Guidelines

| Package | Pitch | Hand Assembly Notes |
|---------|-------|---------------------|
| Through-hole | — | Easiest. Reserve for connectors, fuse holders, and switches. |
| SOT-223 | — | Large pads, excellent hand-solderable. Preferred for LDOs. |
| SOIC / SOP | 1.27mm | Very manageable. First choice for ICs where available. |
| SOT-23 (3–5 pin) | — | Small but manageable. Standard for discrete semiconductors. |
| TSSOP / HTSSOP | 0.65mm | Workable with good iron, flux, and magnification. DRV8876 and LM2596 live here. |
| 0603 passives | — | Recommended default. Comfortable minimum for hand work. |
| 0402 passives | — | Doable with practice and magnification. Avoid where 0603 fits. |
| QFN / DFN | 0.5mm | Exposed pad only; requires hot air and paste. Challenging but achievable. Avoid if an equivalent TSSOP/SOIC exists. |
| 0201 passives | — | Avoid for hand assembly. |
| BGA | — | Not hand-solderable. Do not use. |

**Rules for this board:**
- Default to **0603 passives**. BOM cost difference vs. 0402 is negligible; assembly experience is meaningfully better.
- **Connectors through-hole** wherever possible — JST headers, USB, SWD, fuse holder. Through-hole connectors handle mating/unmating mechanical stress far better than SMD.
- Check for **SOIC alternatives** before committing to TSSOP. A wider-pitch package often exists for buck controllers and smaller ICs.

---

## Board Bring-Up Sequence

Work through this in order. Do not skip stages. A fault caught at stage 2 is a correctable solder bridge; the same fault caught at stage 6 may be a damaged IC.

### Stage 1 — Pre-Power Visual Inspection

- Inspect all joints under magnification for bridges, cold joints, and tombstoned passives
- Verify electrolytic capacitor polarity (bulk input cap, rail decoupling caps)
- Verify DRV8876 and LM2596 orientation — pin 1 markers
- Verify P-channel MOSFET orientation (reverse polarity protection)
- Verify TVS diode orientation
- Check for any obvious solder bridges on HTSSOP pads

### Stage 2 — Pre-Power Continuity Checks

With multimeter in continuity/resistance mode, before any power is applied:

- VBAT to GND — should read open (not short)
- 12V rail to GND — should read open
- 5V rail to GND — should read open
- 3.3V rail to GND — should read open
- Verify TVS diode conducts only in correct direction

Any short here means a solder bridge or incorrect component. Find it before proceeding.

### Stage 3 — First Power, Rails Only (No ICs Populated)

If possible, bring up rails before populating ICs — easier to catch wiring errors without components in the way. If populating the full board at once, keep current limit low.

- Set bench supply to 16V (simulating full 4S charge), current limit 200mA
- Connect to board input (bypassing XT60 if needed via bench clip leads)
- Verify P-FET conducts — VBAT should appear at bulk capacitor
- Observe current draw — idle quiescent should be very low; any significant draw indicates a short

### Stage 4 — Regulated Rail Verification

With no load connected beyond the buck converters and LDO:

| Rail | Expected | Test Point | Accept if... |
|------|----------|------------|-------------|
| 12V motor | 12.0V ± 0.5V | 12V test point | Stable, no oscillation |
| 5V logic | 5.0V ± 0.25V | 5V test point | Stable under no load |
| 3.3V peripheral | 3.3V ± 0.15V | 3.3V test point | Stable |

Check ripple if you have an oscilloscope — excessive ripple on the 12V or 5V rail indicates a buck converter layout or component value issue. Verify UVLO circuit does not falsely trigger at 16V input.

### Stage 5 — Pico Bring-Up

- Connect Pico VSYS to 5V rail, GND to GND
- Verify Pico's onboard 3V3 pin reads 3.3V
- Verify Pico enumerates over USB as a USB device
- Load a minimal test firmware (blink LED) to confirm basic MCU function
- Verify I2C bus with a scan — IMU, magnetometer, and TMP102 should respond at their expected addresses
- Verify SPI bus responds to IMU

### Stage 6 — Motor Driver Bring-Up

Do this before connecting motors or the SBC.

- Verify 12V present on DRV8876 VM pins
- Verify nFAULT pin is high (no fault asserted) on both drivers
- Drive nSLEEP high from Pico — drivers must be explicitly enabled; they default to sleep state
- Apply a low PWM duty cycle (~20%) to one driver
- Connect a small test motor or resistive load — **not your actual robot motors yet**
- Verify motor/load responds and rotates in both directions
- Verify IPROPI current sense output scales with load on Pico ADC
- Repeat for second driver
- Test nFAULT response: briefly stall the test motor and verify fault is asserted and Pico detects it
- Test e-stop: assert kill line and verify both nSLEEP pins go low and motors stop

### Stage 7 — Sensor and Peripheral Bring-Up

Add peripherals one at a time, verifying each before adding the next:

- Encoder connectors — rotate encoder by hand, verify quadrature signals on oscilloscope or logic analyzer
- UART headers — loopback test (connect TX to RX) to verify each UART port
- Remaining I2C/SPI peripherals
- Status LEDs and buzzer

### Stage 8 — SBC Connection

- Connect SBC to 5V rail — monitor current draw as SBC boots
- Verify 5V rail holds under SBC load (no significant sag)
- Verify USB communication between SBC and Pico enumerates correctly
- Run a basic command from SBC to drive a motor through Pico — end-to-end communication test

### Stage 9 — Battery Connection

Only after all previous stages pass on bench supply.

- Have a LiPo-safe bag or fireproof surface nearby for first battery connection
- Ensure battery is not fully charged for first connection — a partially discharged pack limits energy in the event of a fault
- Connect battery, observe current draw at rest
- Verify UVLO threshold — discharge a known-low pack and verify buck converters shut down at the configured threshold
- Run full system under load, monitor DRV8876 temperature via TMP102 over a sustained run

---

## Nice-to-Have Features

These are not hard requirements. None are necessary for a functional first prototype, but each adds meaningful value and most are low-effort to include during initial layout. Features marked **(free)** add negligible board space or BOM cost.

### Debugging and Development

| Feature | Notes |
|---------|-------|
| Test points on critical nets **(free)** | All major rails (VBAT, 12V, 5V, 3.3V), motor driver I/O, I2C/SPI/UART lines, encoder signals. Invaluable during bring-up. |
| BOOTSEL + Reset buttons | BOOTSEL must be held at power-on to enter USB bootloader; a dedicated button eliminates unplugging the board on every firmware flash. |
| Pico UART debug header **(free)** | Break out GP0/GP1 (UART0) to a 3-pin header (TX/RX/GND). Provides a serial console independent of USB. |

### Firmware Robustness

| Feature | Notes |
|---------|-------|
| External watchdog IC | Resets the Pico if firmware hangs; harder guarantee than the internal watchdog. Consider TPS3431 or MAX6369. Important if a firmware lockup could leave motors running. |

### Data and Logging

| Feature | Notes |
|---------|-------|
| SD card slot | SPI-connected microSD for logging sensor data, encoder telemetry, and fault events. Useful for PID tuning and field diagnostics. Pico handles SPI SD cards well. |
| RTC (DS3231, I2C) | Timestamped log entries. Without it, logs start at t=0 every boot. |

### User Interaction

| Feature | Notes |
|---------|-------|
| User buttons (x2–3) | Physical GPIO buttons for triggering calibration, changing modes, or acknowledging faults without a connected computer. |
| Passive piezo buzzer | PWM-driven. Audible feedback for startup, low battery, and fault conditions. Minimal BOM impact. |

### Wireless

| Feature | Notes |
|---------|-------|
| Pico W instead of Pico | Same footprint, drop-in replacement. Adds onboard WiFi and Bluetooth for telemetry, wireless e-stop, and OTA configuration. No additional board complexity. |

### Thermal and Power

| Feature | Notes |
|---------|-------|
| Fan control header **(free)** | 2-pin PWM header near motor drivers. Unlikely to be needed at 1A, but costs nothing to include. |
| Per-rail power-good LEDs | Individual indicators per buck output. Makes rail fault isolation trivial during bring-up. |

### Mechanical

| Feature | Notes |
|---------|-------|
| Mounting holes | Four corners, M3 hardware, with copper keepout zones. Easy to forget during layout. |
| Connector egress planning | Decide which board edges cables exit from relative to the chassis before layout begins. Difficult to fix after mounting. |

---

## Iterative Design Strategy

Designing and fabbing the full board in one pass is a common mistake on a first PCB. The design has natural fault lines that map cleanly to independent subsystems — build and validate them in sequence rather than all at once.

### Board 1: Power Distribution

**Contents:** Input protection (P-FET, fuse, TVS, bulk capacitor), 12V buck, 5V buck, 3.3V LDO, battery voltage divider, UVLO circuit, output connectors for each rail.

**Why first:** Every other subsystem depends on correct power. Validating this in isolation means a wiring mistake or component error can't damage downstream ICs. Fully testable with a bench supply and multimeter — no firmware required.

**Pass criteria:** All rails at correct voltage under no load and under load; UVLO trips at configured threshold; no rail sags under expected current draw; reverse polarity protection blocks reversed input.

**Board size:** Likely fits within 50x50mm — the cheapest fab tier at JLCPCB/PCBWay.

**What you learn:** Buck converter layout, thermal pad soldering technique, passive component sizing, bench supply bring-up procedure.

---

### Board 2: Motor Driver

**Contents:** DRV8876 x2, encoder connectors, motor output connectors, e-stop circuit, current sense outputs. Power via header from Board 1. Control signals via header to an off-the-shelf Pico — no onboard MCU.

**Why second:** Motor drive is the second-highest-risk subsystem. Validating the DRV8876 layout and control interface before integration saves debugging time and protects the rest of the board from motor-related faults during development.

**Pass criteria:** Both motors spin in both directions; encoder signals are clean; IPROPI current sense scales correctly with load; e-stop kills both drivers; nFAULT asserts on stall.

**What you learn:** HTSSOP thermal pad technique, PWM motor control, reading IPROPI on a Pico ADC, fault handling.

---

### Board 3: Full Integration

Carries forward validated schematics from Boards 1 and 2. Adds the Pico natively, onboard sensors, SBC connector, USB, and all peripheral headers. By this point both high-risk subsystems are proven and the integration board is primarily a layout and routing challenge.

**What you learn:** Full board layout, managing multiple power domains, ground plane strategy, routing density across a complex design.

---

### What Doesn't Need a Dedicated Board

**Sensors** — all onboard sensors (IMU, magnetometer, temperature) have commercially available breakout boards (Adafruit, SparkFun, Stemma QT ecosystem). Validate sensor firmware against breakouts first. The schematic for connecting them to the Pico is identical whether the chip is on a breakout or soldered directly — you are substituting a header for a footprint.

**Pico** — already a validated module. Use a bare Pico on a breadboard or cheap carrier during Board 1 and 2 testing.

**USB/SBC communication** — test with a direct Pico-to-SBC USB cable before the integration board exists.

---

### Practical Notes

**Schematic work carries forward.** Blocks drawn for Boards 1 and 2 are copied directly into the integration board. The work is not thrown away — it is validated incrementally.

**Standardize inter-board connector pinouts before starting Board 1 layout.** Board 1's output connectors must match what Board 2 and eventually Board 3 expect. Define this once and don't change it.

**Budget for two spins of each board.** First-time layouts almost always have at least one mistake. At JLCPCB prices for small boards (~$5 for 5 copies), a second spin is cheap insurance.

**Use spare copies for destructive testing.** Fab minimums give you 5 copies of each board. Use extras to practice assembly, test rework techniques, and intentionally stress-test protection circuits before those skills are needed on the integration board.

### Summary

| Board | Contents | Validates | Prerequisite |
|-------|----------|-----------|-------------|
| 1 — Power | Input protection, all regulators, UVLO | Rails, protection, battery monitoring | None |
| 2 — Motor drive | DRV8876 x2, encoders, e-stop | Motor control, current sense, fault handling | Board 1 for power |
| 3 — Integration | Everything | Full system | Boards 1 and 2 proven |
| — (no board) | Commercial breakout boards | Sensor firmware | Pico only |

---

## Open Questions — Lock Down Before Design

The following decisions have direct impact on schematic and layout. Changing them after layout begins is likely to require a respin.

| Decision | Impact |
|----------|--------|
| **PCB form factor / dimensions** | Drives everything in layout. Should match chassis mounting points. |
| **SBC selection** | Determines 5V rail current budget. A Pi 4 peaks ~3A; a Pi 5 peaks ~5A. Wrong sizing causes brownouts. |
| **Pico vs Pico W** | Same footprint, but decide before schematic is finalized. Retrofitting wireless later means a board respin. |
| **Motor output connectors** | Screw terminals, JST, or solder pads? Affects footprint and current rating. |
| **Encoder JST series** | XH (2.5mm), PH (2.0mm), or SH (1.0mm) — must match your encoder cable assemblies. |
| **Layer count** | 2-layer is cheaper and simpler; 4-layer gives a dedicated ground plane and power plane, meaningfully improving noise isolation between motor and logic domains. Recommend 4-layer if budget allows. |
| **E-stop interface** | What triggers it — external button, SBC GPIO signal, or both? Defines the input circuit. |
| **Main power switch** | Onboard switch or external? If external (e.g., panel-mount on chassis), the PCB just needs a switched input. |
| **CAN transceiver** | In or out of first prototype? Easier to include now than to add in a respin. |
| **Number of PWM/servo outputs** | Determines how many PWM-capable GPIO pins are allocated and how large the header is. |
| **Ground strategy** | Single shared ground plane with careful motor current routing (simpler, usually sufficient) vs. split ground with single-point join (more complex, higher isolation). Decide before placing components. |
