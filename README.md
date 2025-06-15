# F1TENTH Braking System

**Disc Brake System for Controlled Drifting in F1TENTH**

## Overview

This package is part of the broader [F1TENTH Project](https://github.com/kkwxnn/F1TENTH_PROJECT) and provides a disc brake control system to enable precise braking and controlled drifting in F1TENTH vehicles.

---

## Table of Contents
- [Overview](#overview)
- [Installation](#installation)
- [Our Team](#our-team)

## Installation
Our Team
> [!Note]
> This package depends on the main [F1TENTH Project](https://github.com/kkwxnn/F1TENTH_PROJECT). Please ensure you’ve set up that environment before continuing.

### 1. Clone the F1TENTH Project Repository

```bash
git clone https://github.com/kkwxnn/F1TENTH_PROJECT.git
```

### 2. Set Up the Environment

Follow the instructions provided in the [F1TENTH Project README](https://github.com/kkwxnn/F1TENTH_PROJECT/blob/humble/README.md) to set up dependencies.

### 3. Navigate to the `src` Directory

```bash
cd ~/F1TENTH_PROJECT/f1tenth_ws/src
```

### 4. Clone the Braking System Package

```bash
git clone https://github.com/peeradonmoke2002/f1tenth_breaking_system.git
```

### 5. Build the Workspace

```bash
cd ~/f1tenth_ws/
rosdep update
rosdep install --from-paths src
colcon build --symlink-install
```

### 6. Source the Workspace

```bash
source ~/f1tenth_ws/install/setup.bash
```

---
## Hardware 

Braking Schematic: [Braking Schematic](/.doc/Schematic_break_2025-06-09.pdf)

### problem / Goal
 problem 1.1 Mount Misalignment between the coil’s actuator surface and the brake target disc
 problem 1.2  The current coil is physically too small to generate strong magnetic force because of tight mechanical space in the chassis. If more space were available

### coil spec 

| Property | Value |
|----------|-------|
| Core Type | Steel  |
| Outer Diameter | 12 mm |
| Inner Diameter | 8 mm |
| Wire Used | 0.5 mm magnet wire |
| Power | 12V DC |
| Resistance | ~1.5 ohms  |
| Current | ~8 amp |
|Application Load |  ~2 KG |

### 🧮 Magnetic Force Estimation


| Symbol | Value                        | Description                     |
|--------|------------------------------|---------------------------------|
| N      | 270 turns                    | Number of coil windings         |
| I      | 8 A                          | Coil current                    |
| A      | 1.13 × 10⁻⁴ m²              | Core area (radius = 6 mm)       |
| g      | 0.0003 m                     | Air gap (0.3 mm)                |
| μᵣ     | 1000                         | Relative permeability (steel)   |
| μ₀     | 4π × 10⁻⁷ H/m               | Vacuum permeability constant    |

$$
F = \frac{N^2 \cdot \mu \cdot A \cdot I^2}{2 \cdot g^2}
$$

- Estimated force = 60 N woth full load


---
## Our Team
- **67340700402** พงษ์พัฒน์ วงศ์กำแหงหาญ
- **67340700403** พีรดนย์ เรืองแก้ว

