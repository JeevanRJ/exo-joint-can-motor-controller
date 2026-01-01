# FSR‑Based Motor Control over CAN (Arduino + MCP2515 + RMDX)

Arduino sketch for reading **two force-sensitive resistors (FSRs)**, computing a **PID control signal** from the difference between the sensors, and (optionally) commanding an **RMD‑X series motor** over **CAN bus** using an **MCP2515** CAN controller.

This repo is intended as a compact starting point for closed-loop control where the **error signal = (FSR1 − FSR2)**.

---

## Author

**Jeevan Jayasuriya**  
NeuroErgonomics Lab  
University of Wisconsin–Madison

---

## What this code does

1. Initializes a CAN interface using **ACAN2515** (MCP2515 + SPI).
2. Enables an RMD‑X motor (example uses **motor ID = 1**).
3. Reads two FSR analog channels (**A0** and **A1**).
4. Maps raw ADC values to a 0–100 “pressure” scale (with clamping).
5. Computes a PID output based on sensor difference:
   - `input = -(FSR1 - FSR2)`
6. Prints FSR readings, input, and output to Serial for debugging (Serial Plotter friendly).

> ⚠️ Note: In the provided sketch, the PID output is **not yet sent to the motor** (motor command lines are commented). This repo includes the motor driver files (`RMDX.*`) so you can add the command that applies `output` as current/torque/speed/position depending on your motor mode.

---

## Files required for this project

### Required repo files
These files must be present together for compilation:

- `motor_control_for_exo_joints.ino`  
  Main Arduino sketch (setup + loop).

- `RMDX.h`  
  RMD‑X motor driver header.

- `RMDX.cpp`  
  RMD‑X motor driver implementation.

> If you rename the sketch folder, keep the `.ino` filename matching the folder name (Arduino IDE convention).

### Required external libraries (Arduino Library Manager)

Install these libraries in Arduino IDE (**Sketch → Include Library → Manage Libraries…**):

- **ACAN2515** (for MCP2515 CAN controller)
- **PID_v1** (Arduino PID library)

Also required:
- **SPI** (built-in with Arduino core)

---

## Hardware requirements

- Arduino-compatible board with SPI (e.g., Uno/Nano/Mega; ESP32 also possible with SPI pin changes)
- MCP2515 CAN module (8 MHz crystal assumed by default)
- RMD‑X motor + CAN wiring
- Two FSR sensors (with appropriate resistor dividers)

### Default pin configuration in the sketch
- MCP2515 CS: **D10**
- MCP2515 INT: **D3**
- FSR1: **A0**
- FSR2: **A1**

Update these at the top of the sketch if your wiring differs.

---

## CAN configuration

The sketch is configured for:
- MCP2515 crystal: **8 MHz**
- CAN bitrate: **500 kbps**

```cpp
static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL;
static const uint32_t CAN_BAUDRATE = 500UL * 1000UL;
```

> If your MCP2515 module uses a **16 MHz** crystal, update `QUARTZ_FREQUENCY` accordingly.

---

## How to run

1. Install required libraries (**ACAN2515**, **PID_v1**).
2. Open the `.ino` in Arduino IDE (ensure `RMDX.h` and `RMDX.cpp` are in the same sketch folder).
3. Select the correct board and port.
4. Upload.
5. Open Serial Monitor/Plotter at **38400 baud**.

You should see output like:
- FSR1 / FSR2 mapped values
- PID input (difference)
- PID output (scaled)

---

## Calibrating the FSR mapping

The sketch maps raw readings like this:

```cpp
fsr_1_Reading = map(fsr_1_Reading, 850, 950, 0, 100);
fsr_2_Reading = map(fsr_2_Reading, 850, 950, 0, 100);
```

Adjust `850` and `950` for your specific FSR + resistor divider so the mapped range covers your expected force range.

---

## PID tuning

PID is created as:

```cpp
PID motorPID(&input, &output, &setpoint, 0.1, 0.0, 0.01, DIRECT);
```

Where:
- `setpoint` defaults to 0 (balanced force)
- `output` is limited to `[-100, 100]`

Tune `Kp, Ki, Kd` and the output limits based on your motor control mode and safety constraints.

---

## Applying PID output to the motor (TODO)

Right now, output is printed but not applied to the motor (motor commands are commented).

To close the loop, add a command in the `if (output > 0) { ... } else { ... }` block that sends the desired control quantity (e.g., current/torque/speed/position) through the `RMDX` API.

Because different RMD‑X setups use different control modes, this repo intentionally leaves the final command integration as a small “plug-in” step.

---

## Troubleshooting

- **No CAN communication**
  - Verify MCP2515 wiring (CS, INT, SPI pins)
  - Confirm correct crystal frequency (8 MHz vs 16 MHz)
  - Confirm bitrate matches motor network (500 kbps vs 1 Mbps)

- **FSR values always 0 or 100**
  - Your `map()` calibration range likely needs adjustment
  - Check the FSR resistor divider wiring and resistor value

- **Motor enables but doesn’t move**
  - Output is not being sent to motor yet (see TODO section)

---

## License

MIT License (see `LICENSE`).


