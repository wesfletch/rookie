# rookie

A differential-drive robot. The SBC runs ROS; an RPi Pico handles motor control and low-level I/O.

## Dev environment

Requires Docker.

```bash
# First-time setup (installs git hooks)
./rookie setup

# Build the dev container
./rookie build

# Run the dev container
./rookie run
```

All subsequent build steps run inside the container. The `src/` directory is mounted at `~/workspace/`.

## Building

See the README in each subdirectory:

- [`src/pico_interface`](src/pico_interface/README.md) — shared interface/messaging library (build first)
- [`src/rookie_pico`](src/rookie_pico/README.md) — Pico firmware
- [`src/ros`](src/ros/README.md) — ROS workspace
