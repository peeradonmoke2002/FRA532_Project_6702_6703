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

## Our Team
- **67340700402** พงษ์พัฒน์ วงศ์กำแหงหาญ
- **67340700403** พีรดนย์ เรืองแก้ว

