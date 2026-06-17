# 🚀 Pointy Rocket V1

**Pointy Rocket** is an actively stabilised model-scale rocket developed by the Engineering Society of Castleknock College. It uses **Thrust Vector Control (TVC)** — the same technique used in orbital-class rockets — to actively stabilise the vehicle in flight.

The project spans hardware design, embedded firmware, and a high-fidelity physics simulation, all living in this repository.

-----

## An SVCC Project with support from RS Components

<img width="100" height="100" alt="RS Logo" src="https://github.com/BasilAmin/pointy_rocket/blob/main/Media/svcclogo.png" />&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<img width="100" height="100" alt="RS Logo" src="https://github.com/BasilAmin/pointy_rocket/blob/main/Media/RSlogo.png" />

-----

## Authors

- James Harcourt
- Basil Amin
- Ashkan Samali
- Feargal Browne
- Ben Liu
- Ryan Doran
  
-----

## Project Overview

|Component    |Details                                                           |
|-------------|------------------------------------------------------------------|
|Stabilisation|2-axis Thrust Vector Control (TVC) via servo-gimballed motor mount|
|Motors       |4× Estes D12 (clustered)                                          |
|Recovery     |Parachute deployment                                              |

-----

## Repository Structure

```
pointy_rocket/
├── firmware/               # Embedded firmware (Teensy 4.0 / PlatformIO)
│   ├── src/main.cpp
│   ├── include/
│   └── platformio.ini
├── hardware/
│   └── pcb/                # KiCad PCB schematic & layout
│       └── Pointy R1 Flight controller/
│           ├── *.kicad_sch
│           ├── *.kicad_pcb
│           ├── symbols.kicad_sym
│           └── footprints.pretty/
├── simulation/             # Python flight simulation
│   ├── rocket_sim.py       # Full 6-DOF 3D simulation (RK4 + Quaternions)
│   ├── motor_sweep.py      # Motor configuration parameter sweep
│   ├── pygame_simulation.py
│   ├── requirements.txt
│   └── simulation_results.png
└── sim_specs.txt           # Simulation variable reference
```

-----

## Firmware

The flight computer runs on a **Teensy 4.1** (NXP i.MX RT1062, Cortex-M7 @ 600 MHz), programmed via PlatformIO with the Arduino framework.

### Hardware

|Component      |Details                                        |
|---------------|-----------------------------------------------|
|Microcontroller|Teensy 4.1                                     |
|IMU            |MPU6050 (6-axis accelerometer + gyroscope, I2C)|
|Actuation      |2× Servo (dual-axis TVC gimbal)                |
|Data logging   |SD card (SPI)                                  |

### Dependencies

|Library                  |Version |
|-------------------------|--------|
|`arduino-libraries/Servo`|`^1.3.0`|
|`electroniccats/MPU6050` |`^1.4.4`|
|`arduino-libraries/SD`   |`^1.3.0`|

### Build & Flash

**Prerequisites:** [PlatformIO](https://platformio.org/) (CLI or VS Code extension)

```bash
# Clone the repo
git clone https://github.com/BasilAmin/pointy_rocket.git
cd pointy_rocket/firmware

# Build
pio run

# Flash to Teensy (press the button on the board if needed)
pio run --target upload

# Open serial monitor
pio device monitor
```

-----

## Hardware / PCB

The custom flight controller PCB is designed in **KiCad** and lives in `hardware/pcb/Pointy R1 Flight controller/`. Custom footprints are included for:

- Teensy 4.1 (form-factor reference)
- MPU6050 breakout (GY-521 module)
- MPL3115A2 barometric altimeter
- AMASS XT60 power connector
- N-MOSFET (D2PAK) motor ignition circuit
- Tactile and slide switches
- Passive components (diode, SOT-23 transistor)

To open the design, install [KiCad 7+](https://www.kicad.org/) and open `Pointy R1 Flight controller.kicad_pro`.

-----

## Simulation

The simulation suite models the full flight from ignition through parachute landing.

### Setup

```bash
cd simulation
pip install -r requirements.txt
```

Requirements: `numpy`, `matplotlib`

### 6-DOF Flight Simulation

`rocket_sim.py` implements a full **6 Degrees of Freedom** rigid-body simulation using:

- **RK4 numerical integration** for accurate state propagation
- **Quaternion-based attitude representation** (no gimbal lock)
- **PID controller** for dual-axis TVC stabilisation
- **Dynamic mass model** — propellant mass decreases with cumulative impulse
- **High-fidelity D12 thrust curve** (4× scaled cluster, from verified RASP data)
- **Aerodynamic drag** with wind shear model
- **Automatic parachute deployment** on descent

**Controller parameters:**

|Gain            |Value|
|----------------|-----|
|Kp              |0.8  |
|Kd              |1.5  |
|Ki              |0  |
|Max gimbal angle|±8°  |

```bash
python rocket_sim.py
```

Outputs a 4-panel plot (`simulation_results.png`) showing:

- 3D flight trajectory
- Altitude vs. time
- Dual-axis TVC gimbal angles
- Dynamic mass and thrust profile

### Motor Configuration Sweep

`motor_sweep.py` sweeps across 1–4 motor configurations to compare predicted performance across different cluster sizes.

```bash
python motor_sweep.py
```

-----

## License

MIT License — © 2026 Basil Amin. See [LICENSE](firmware/LICENSE) for details.
