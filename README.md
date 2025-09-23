# Crazy Controller

MAP (Model-based Adaptive Pursuit) controller for F1TENTH with safety features and dynamic parameter tuning.

## Quick Start

```bash
# Build
colcon build --packages-select crazy_controller
source install/setup.bash

# Real hardware
ros2 launch crazy_controller controller_launch.py mod:=real

# Simulation (using simulator odometry)
ros2 launch crazy_controller controller_launch.py mod:=sim

# Simulation with MCL (using particle filter localization)
ros2 launch crazy_controller controller_launch.py mod:=sim_pf
```

## Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mod` | `real` | Launch mode: `real`, `sim`, or `sim_pf` |
| `controller_mode` | `MAP` | Control algorithm (MAP only) |
| `l1_params_path` | Auto-selected | L1 parameter file by mode |
| `lookup_table_path` | Auto-selected | Steering lookup table by mode |

## Topics

### Real Mode (`mod:=real`)
- **Input**: `/pf/pose/odom`, `/car_state/frenet/odom` - MCL pose and Frenet state
- **Input**: `/global_waypoints`, `/local_waypoints` - Track and path data
- **Output**: `/drive` - Ackermann drive commands
- **Timing**: Real time

### Simulation Mode (`mod:=sim`)
- **Input**: `/ego_racecar/odom`, `/car_state/frenet/odom` - Direct simulator odometry and Frenet state
- **Input**: `/global_waypoints`, `/local_waypoints` - Track and path data
- **Output**: `/drive` - Ackermann drive commands
- **Timing**: Simulation time

### Simulation + MCL Mode (`mod:=sim_pf`)
- **Input**: `/pf/pose/odom`, `/car_state/frenet/odom` - MCL localization in simulation and Frenet state
- **Input**: `/global_waypoints`, `/local_waypoints` - Track and path data
- **Output**: `/drive` - Ackermann drive commands
- **Timing**: Simulation time

## Key Configuration

Edit `config/l1_params.yaml` (real) or `config/l1_params_sim.yaml` (sim/sim_pf):

```yaml
crazy_controller:
  ros__parameters:
    # Core L1 Parameters
    t_clip_min: 1.5               # Minimum time horizon
    m_l1: 0.3                     # L1 gain multiplier
    q_l1: 0.5                     # L1 damping coefficient
    
    # Speed and Steering
    speed_lookahead: 0.25         # Speed-based lookahead
    acc_scaler_for_steer: 1.2     # Steering scaling factors
    
    # Vehicle parameters (auto-set by mod)
    # Real: Pacejka tire model, wheelbase=0.325
    # Sim/Sim_pf: Linear tire model, wheelbase=0.324
```

## Control Algorithm

MAP controller with multi-stage pipeline:
- **Message synchronization**: Waits for all required topics with timeout
- **L1 adaptive guidance**: Dynamic lookahead based on speed and curvature
- **Speed planning**: Curvature-aware velocity with acceleration scaling
- **Steering control**: Pacejka/linear tire model lookup with calibration

## Safety Features

- Signal handling with graceful shutdown
- Emergency stop on termination
- Waypoint timeout protection (5 seconds)
- Input validation and error recovery
- Thread-safe atomic operations

## Dynamic Tuning

```bash
# Real-time parameter updates
ros2 param set /controller_manager m_l1 0.4
ros2 param set /controller_manager speed_lookahead 0.3
ros2 param set /controller_manager acc_scaler_for_steer 1.1
```

## Prerequisites

- Lattice planner running
- Monte Carlo Localization (real hardware)
- Steering lookup table files loaded