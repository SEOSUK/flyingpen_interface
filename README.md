# flyingpen_interface

`flyingpen_interface` is a ROS2 package for the **Flying Pen** project, designed to be used together with:

- **crazyswarm2** (branch: `flyingpen_dev`)  
  https://github.com/SEOSUK/crazyswarm2  *(checkout `flyingpen_dev`)*
- **crazyflie-firmware** (branch: `flyingpen`)  
  https://github.com/SEOSUK/crazyflie-firmware  *(checkout `flyingpen`)*

---

## Purpose

This package provides **high-level utilities** that work on top of Crazyflie firmware features:

- **High-level commander**: generates/streams commands to be injected into firmware
- **Data logging**: collects multiple topics/signals and saves them to CSV for analysis
- **RViz visualization**: visual debugging/monitoring tools for Flying Pen experiments

In short: **firmware-side features are enabled by the firmware branch, and this package provides the ROS2-side commander + logging + visualization.**

---

## Branch Compatibility

You should use the following branches together:

| Repository | Branch |
|---|---|
| `SEOSUK/crazyswarm2` | `flyingpen_dev` |
| `SEOSUK/crazyflie-firmware` | `flyingpen` |

---

## What’s Inside

- **Commander node(s)**
  - Publishes high-level command signals required by the firmware-side controller logic
- **Logging node(s)**
  - Aggregates multiple measurements/diagnostics into a unified topic
  - Saves to CSV (`MMDDHHMM.csv`) for MATLAB/Python post-processing
- **RViz visual node(s)**
  - Displays pose/trajectory/markers/force vectors (as applicable)
  - Helps debugging during flight/contact experiments

---

## Notes

- This package assumes the firmware branch provides the required logging/parameters/topics.
- The exact topic names and message layout may be branch-specific; keep the branches matched as above.

