# rr_bosch_lc_base

ROS2 lifecycle base package for Bosch sensor nodes. Part of the `rr_bosch_lc_nodes` repository.

## Overview

`rr_bosch_lc_base` provides the abstract parent lifecycle node from which all Bosch sensor nodes in this repository derive. It is not a sensor-specific implementation — it is the scaffolding that owns the lifecycle, manages transport initialisation, and instantiates child nodes within a shared component container boundary.

The package is intentionally scoped to Bosch sensors rather than a specific sensor type (e.g. IMU), reflecting that Bosch produces a range of sensors — inertial, barometric, environmental — that may be integrated into the mazebot platform over time.

## Architecture

### Repository Structure

```
rr_bosch_lc_nodes/
└── rr_bosch_lc_base/       ← this package (abstract parent)
└── rr_bosch_imu_node/      ← BNO055 IMU lifecycle node (planned)
```

### Parent / Child Pattern

The parent node (`rr_bosch_lc_base`) owns the ROS2 lifecycle and is responsible for:

- **`on_configure`** — validates that all required parameters are defined; fails fast if any are missing
- **`on_activate`** — constructs the transport driver; instantiates child node(s); injects transport into each child via setter and `shared_ptr`
- **`on_deactivate` / `on_cleanup` / `on_shutdown`** — mirrors lifecycle transitions downward to children in order

Child nodes are concrete sensor implementations (e.g. `RrBoschImuNode` for the BNO055). They receive the transport driver via setter injection and own their own timer and publisher. They are full `nav2_util::LifecycleNode` participants.

The parent and all children are loaded into the same ROS2 component container (`component_container_mt`), which means they share a process boundary. This is a deliberate design constraint: `shared_ptr` cannot be safely passed across component container boundaries in ROS2, so transport injection is valid only within the same container.

### Why a Separate Base Package

The `lc` in the package name signals that this is a ROS2 lifecycle node. The separation from the driver layer (`rr-bno055`) is intentional and meaningful:

- `rr-bno055` — pure C++ driver, no ROS2 dependency, wraps Bosch SensorAPI
- `rr_bosch_lc_base` — ROS2 lifecycle scaffolding, sensor-agnostic
- `rr_bosch_imu_node` — sensor-specific composable node, publishes `sensor_msgs/Imu`

This mirrors the naming and separation conventions established elsewhere in the mazebot stack.

## Dependencies

| Package | Purpose |
|---|---|
| `rclcpp` | Core ROS2 C++ client library |
| `rclcpp_lifecycle` | Lifecycle node base |
| `rclcpp_components` | Composable node registration |
| `nav2_util` | `nav2_util::LifecycleNode` for lifecycle manager bonding |

## Integration

The node is designed to run as a composable node within a dedicated `sensor_container`, separate from the `driver_container` that manages GPIO and motor control. This separation provides fault isolation — a sensor node fault does not affect motor control — and a clean organisational boundary consistent with ROS2 best practices.

The lifecycle manager bonds to the node via `nav2_util::LifecycleNode`. The node name and namespace are declared in the mazebot bringup launch file (`rr_mousebot_bringup`).

## Defaults and Parameters

Sensor-specific parameters are defined in child node packages with sensible defaults. Parameters may be overridden via the launch file. See the relevant child node package for parameter documentation.

## Naming Conventions

This package follows ROS2 style guide conventions:

- Underscores as word separators
- `lc` suffix signals lifecycle node
- `base` suffix signals abstract/foundational class, consistent with `rr_common_base` elsewhere in the stack
- Plural repository name (`rr_bosch_lc_nodes`) signals multiple nodes may reside here

## License

MIT
