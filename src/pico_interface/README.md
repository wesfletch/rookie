# pico_interface

Shared framing library used by both the Pico firmware and the ROS side. Must be built and installed before building `rookie_pico`.

## Build

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build .
sudo cmake --install .
```

Proto generation runs automatically at configure time. To regenerate manually:

```bash
bash scripts/generate_protos.sh
```

## Adding messages

Define the message in a `.proto` file under `protos/`, then run `generate_protos.sh` to regenerate. No other files need updating.

Messages come in two kinds:

- **Top-level messages** are sent as standalone frames. They require a unique non-zero msgid:
  ```proto
  message MyMessage {
      option (nanopb_msgopt).msgid = 12;
      ...
  }
  ```
- **Submessage types** (e.g. `Velocity`, `Motors`) have no msgid and cannot be sent as standalone frames — they can only be embedded as fields inside a top-level message.

The Python package emits a `logging.WARNING` at import time for any message class without a msgid, as a reminder to add one if the omission was unintentional.

## Python package

`generate_protos.sh` also generates the Python bindings under `py/pico-interface/`. If `uv` is available it will build the package automatically; otherwise build it manually:

```bash
cd py/pico-interface
pip install .
```

## console.py

A curses-based serial console for viewing and sending pico_interface messages. Runs on the **host** since it needs direct access to the serial port.

**Prerequisites:** Python 3.12+, `uv` (recommended), the `pico_interface` Python package installed.

**With uv** (handles dependencies automatically):

```bash
uv run scripts/console.py
```

**Without uv:**

```bash
pip install pyserial pico-interface
python3 scripts/console.py
```

Connects to `/dev/ttyACM0` at 115200 baud by default.

| Key | Action |
|-----|--------|
| `q` | Quit |
| `space` | Pause/resume |
| `f` | Set filter |
| `s` | Set highlight |
| `w` | Write a message |
| `c` | Clear screen |
| `d` | Detach serial (keeps process alive) |
| `↑` / `↓` | Scroll (while paused) |
