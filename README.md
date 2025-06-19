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

The F1TENTH Braking System integrates both software and hardware components to enable disc brake actuation and controlled drifting on your F1TENTH vehicle. The system uses ROS 2 for modular communication between nodes, an ESP32 microcontroller for real-time brake control, and a dedicated coil-based disc brake mechanism for reliable stopping power.

### Key Features

* **Joystick-based control** for speed, steering, and braking
* **ROS 2 nodes** for modular command processing
* **ESP32 microcontroller** with micro-ROS for low-latency actuation
* **Custom mechanical assemblies** for brake actuation

---

## Break Node System Overview

<p align="center">
  <img src="./images/break_node_overview.png" alt="Break Node System Overview" width="1000"/>
</p>

The **Break Node System** manages the actuation logic for the disc brake using ROS 2 topics and the ESP32 microcontroller:

| Component              | Description                                                           |
| ---------------------- | --------------------------------------------------------------------- |
| Joy Controller Node    | Publishes `/break_mode` (enable brake) and `/break_pwm` (brake level) |
| Dice Break System Node | Checks enable flag, passes duty to ESP32 or zeroes if disabled        |
| ESP32 microcontroller  | Drives brake coil using micro-ROS commands                            |

This structure ensures **safety**, **flexibility**, and **real-time control** for autonomous driving.

---

## Installation

> [!NOTE]
> Ensure the [F1TENTH Project](https://github.com/kkwxnn/F1TENTH_PROJECT) environment is set up before proceeding. This setup should be installed on a **Raspberry Pi**, which operates the F1TENTH Project.

### 1. Clone F1TENTH Project

> [!IMPORTANT]
> Clone the F1TENTH repository directly on your **Raspberry Pi** before continuing.

```bash
git clone https://github.com/kkwxnn/F1TENTH_PROJECT.git
```

### 2. Set Up Environment

Follow the [F1TENTH README](https://github.com/kkwxnn/F1TENTH_PROJECT/blob/humble/README.md) to configure your environment. Then **replace the [`docker-compose.yml`](./docker-compose.yml)** with the one from this repository to ensure brake system compatibility.

### 3. Navigate to `src` Directory

```bash
cd ~/F1TENTH_PROJECT/f1tenth_ws/src
```

### 4. Clone Brake Package

```bash
git clone https://github.com/peeradonmoke2002/f1tenth_breaking_system.git
```

### 5. Access ROS 2 Desktop via VNC

1. **Find your Raspberry Pi IP (wlan0 interface):**

```bash
ifconfig
```

2. **Open VNC in a browser:**

Navigate to:

```
http://<Your-IP-Address>:6080/
```

Replace `<Your-IP-Address>` with the one found in the previous step.

### 6. Build the Workspace

```bash
cd ~/f1tenth_ws/
rosdep update
rosdep install --from-paths src
colcon build --symlink-install
```

### 7. Source the Workspace

```bash
source ~/f1tenth_ws/install/setup.bash
```

---

## Usage

> \[!IMPORTANT]
> This section assumes you are running on the **VNC ROS 2 Desktop** on your Raspberry Pi. If not, please return to the [Installation](#installation) section.

### 1. Start Robot Command Node

```bash
ros2 run robot_bridge RobotCommand.py
```

### 2. Launch Braking System

```bash
ros2 launch break_controller joystick.launch.py
```

> \[!WARNING]
> Make sure both the **ESP32** and **joystick** are connected. Also verify that the `micro-ros-esp32` Docker container is running.

2. **Verify Status:**

```bash
ros2 node list
ros2 topic list
```

Expected Nodes:

```
/break_controller
/joy_control
/joy_node
/robot_commnad_node
```

Expected Topics:

```
/cmd_vel
/cmd_steer
/mcu_cmd
/joy
/joy/set_feedback
/break_mode
/break_pwm
```

> \[!WARNING]
> If `/break_mode` or `/break_pwm` are missing, press the reset button on your ESP32 board.

<p align="center">
  <img src="./images/esp32_resetbutton.png" alt="ESP32 Reset Button" width="400" />
</p>

### 4. Joystick Control Reference

<p align="center">
  <img src="./images/xbox_button.png" alt="Xbox Joystick" width="400" />
</p>

| Button | Action                                      |
| ------ | ------------------------------------------- |
| B      | Toggle driving direction (forward/backward) |
| 3      | Steer left/right (horizontal axis)          |
| 5      | Enable brake (hold)                         |
| 6      | Apply brake force (hold, variable strength) |
| 10     | Enable speed and steering control           |
| 11     | Apply throttle (0–2.5 m/s)                  |

> \[!CAUTION]
> Do not hold the brake for long periods. Continuous actuation may cause overheating and damage to the coil or MOSFET. Use short bursts for optimal drifting.

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

#### 1. Check Topic

Ensure the encoder topic is published as shown below:

```
[AMT103 Encoder] → [ESP32 + micro-ROS] → [/enc_steer_raw] → [encoder2angle.py] → [/enc_steer]
```

To verify:

```bash
ros2 topic list  # Ensure /enc_steer_raw is visible
```

#### 2. Run Processing Script

```bash
ros2 run break_controller encoder2angle.py  # Publishes /enc_steer
```

---

## Our Team

| Student ID  | Name                   |
| ----------- | ---------------------- |
| 67340700402 | พงษ์พัฒน์ วงศ์กำแหงหาญ |
| 67340700403 | พีรดนย์ เรืองแก้ว      |
