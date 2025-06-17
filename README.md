# F1TENTH Braking System

**Disc Brake System for Controlled Drifting in F1TENTH**

This package is part of the broader [F1TENTH Project](https://github.com/kkwxnn/F1TENTH_PROJECT) and provides a disc brake control system to enable precise braking and controlled drifting in F1TENTH vehicles.

---

## Table of Contents

* [System Overview](#system-overview)
* [Break Node System Overview](#break-node-system-overview)
* [Installation](#installation)
* [Usage](#usage)
* [Hardware](#hardware)
* [Additional Sensors](#additional-sensors)
* [Our Team](#our-team)

---

## System Overview

<p align="center">
  <img src="./images/system_overview.png" alt="F1TENTH Braking System" width="800"/>
</p>

The F1TENTH Braking System integrates both software and hardware components to enable disc brake actuation and controlled drifting on your F1TENTH vehicle. The system uses ROS2 for modular communication between nodes, an ESP32 microcontroller for real-time brake control, and a dedicated coil-based disc brake mechanism for reliable stopping power.

### Key Features

* **Joystick-based control** for speed, steering, and braking
* **ROS2 nodes** for modular command processing
* **ESP32 microcontroller** with micro-ROS for low-latency actuation
* **Custom mechanical assemblies** for brake actuation

---

## Break Node System Overview

<p align="center">
  <img src="./images/break_node_overview.png" alt="Break Node System Overview" width="1000"/>
</p>

The **Break Node System** manages the actuation logic for the disc brake using ROS 2 topics and the ESP32 microcontroller:

| Component              | Description                                                    |
| ---------------------- | -------------------------------------------------------------- |
| Joy Controller Node    | Publishes `/break_mode` (enable brake) and `/break_pwm` (brake level) |
| Dice Break System Node | Checks enable flag, passes duty to ESP32 or zeroes if disabled |
| ESP32 microcontroller  | Drives brake coil using micro-ROS commands                     |

This structure ensures **safety**, **flexibility**, and **real-time control** for autonomous driving.

---

## Installation

> [!NOTE]
> Ensure the [F1TENTH Project](https://github.com/kkwxnn/F1TENTH_PROJECT) environment is set up before proceeding.

1. **Clone F1TENTH Project:**

```bash
git clone https://github.com/kkwxnn/F1TENTH_PROJECT.git
```

2. **Set Up Environment:**
   Follow the [README guide](https://github.com/kkwxnn/F1TENTH_PROJECT/blob/humble/README.md)

3. **Navigate to src Directory:**

```bash
cd ~/F1TENTH_PROJECT/f1tenth_ws/src
```

4. **Clone Brake Package:**

```bash
git clone https://github.com/peeradonmoke2002/f1tenth_breaking_system.git
```

5. **Build the Workspace:**

```bash
cd ~/f1tenth_ws/
rosdep update
rosdep install --from-paths src
colcon build --symlink-install
```

6. **Source the Workspace:**

```bash
source ~/f1tenth_ws/install/setup.bash
```

7. **Launch Braking System:**

```bash
ros2 launch break_controller joystick.launch.py
```

> [!WARNING] 
>Ensure ESP32 and joystick are connected; also ensure `esp32_micro_ros` container is running.

8. **Verify Status:**

```bash
ros2 node list
ros2 topic list
```

Expected output:

**Nodes:**

```
/break_controller
/joy_control
/joy_node
```

**Topics:**

```
/cmd_vel
/joy
/joy/set_feedback
/break_mode
/break_pwm
```

> [!WARNING]
> If `/break_mode` and `/break_pwm` are missing, press the reset button on the ESP32.
><p align="center">
>    <img src="./images/esp32_resetbutton.png" alt="ESP32 Reset Button" width="400" />
></p>

---

## Usage

After launching the system, you can control braking and drifting via joystick input.

<p align="center">
  <img src="./images/xbox_button.png" alt="Xbox Joystick" width="400" />
</p>

### Joystick Mapping

| Button | Action                                     |
| ------ | ------------------------------------------ |
| B      | Change direction (forward/backward)        |
| 3      | Steer left/right (horizontal axis)         |
| 5      | Enable brake (hold)                        |
| 6      | Activate brake (hold to control 0–100%)    |
| 10     | Enable speed and steering control                       |
| 11     | Activate speed (hold to control 0–2.5 m/s) |

> [!CAUTION]
> Don't hold press the break too long, as it may cause the brake to overheat and damage the coil and mosfet. The system is designed for short bursts of braking to allow for controlled drifting.



---

## Hardware

* **Braking Schematic:** [Braking Schematic](/.doc/Schematic_break_2025-06-09.pdf)

### Known Issues (MK I)

* **1.1** Mount misalignment between the coil’s actuator surface and the brake disc

<p align="center">
    <img src="./images/problem_1.JPG" alt="coil" width="200" />
</p>

* **1.2** The current coil is physically too small to generate a strong magnetic force due to tight mechanical space in the chassis. More space would improve performance.

### Coil Specifications

| Property         | Value              |
| ---------------- | ------------------ |
| Core Type        | Steel              |
| Outer Diameter   | 12 mm              |
| Inner Diameter   | 8 mm               |
| Wire Used        | 0.5 mm magnet wire |
| Power            | 12V DC             |
| Resistance       | \~1.5 Ω            |
| Current          | \~8 A              |
| Application Load | \~2 kg             |

### 🧮 Magnetic Force Estimation

| Symbol | Value          | Description                   |
| ------ | -------------- | ----------------------------- |
| N      | 270 turns      | Number of coil windings       |
| I      | 8 A            | Coil current                  |
| A      | 1.13 × 10⁻⁴ m² | Core area (radius = 6 mm)     |
| g      | 0.0003 m       | Air gap (0.3 mm)              |
| μᵣ     | 1000           | Relative permeability (steel) |
| μ₀     | 4π × 10⁻⁷ H/m  | Vacuum permeability constant  |

$$
F = \frac{N^2 \cdot \mu \cdot A \cdot I^2}{2 \cdot g^2}
$$

* **Estimated force:** 60 N at full load


---

## Additional Sensors

To enhance steering control, the system integrates an **AMT103 rotary encoder**, which provides high-resolution angular feedback to improve vehicle direction control.

<p align="center">
  <img src="./images/AMT103.png" alt="AMT103 Encoder" width="400"/>
</p>

### Encoder Usage

1. **Check Topic:**
First, ensure the encoder topic is available base from the system flow:
```bash
[AMT103 Encoder] → [ESP32 + micro-ROS] → [/enc_steer_raw] → [encoder2angle.py] → [/enc_steer]
```
To verify the encoder topic, run:
```bash
ros2 topic list  # Ensure /enc_steer_raw is visible
```

2. **Run Processing Script:**

```bash
ros2 run break_controller encoder2angle.py  # Publishes /enc_steer
```

---

## Our Team

| Student ID  | Name                   |
| ----------- | ---------------------- |
| 67340700402 | พงษ์พัฒน์ วงศ์กำแหงหาญ |
| 67340700403 | พีรดนย์ เรืองแก้ว      |
