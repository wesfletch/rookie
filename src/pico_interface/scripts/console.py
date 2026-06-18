#!/usr/bin/env python3

# Dev tool that opens a minicom-style console to view pico_interface messages.
# Supports some basic wireshark-esque commands, like filtering and highlighting.

# /// script
# requires-python = ">=3.10.12"
# dependencies = [
#   "pyserial",
#   "pico-interface",
# ]
#
# [tool.uv.sources]
# pico-interface = { path = "../py/pico-interface", editable = true }
# ///

import argparse
from collections import deque
import curses
import curses.ascii
from dataclasses import dataclass
import queue
import serial
import sys
import threading
import time
from typing import IO, Union

from google.protobuf import json_format
import pico_interface as pico_if

_COLOR_TIMESTAMP = 1
_COLOR_MSG_TYPE = 2
_COLOR_BODY = 3
_COLOR_HEADER = 4
_COLOR_PAYLOAD = 5


@dataclass
class DisplayLine:
    timestamp: float
    msg_id: int
    msg_type: str  # class name, or str(msg_id) if unknown
    header_hex: str
    payload_hex: str
    body: str  # decoded JSON, or error string

    def __str__(self) -> str:
        return f"[{self.timestamp:.4f}]: <{self.msg_type}>: {self.body}"


DisplayItem = DisplayLine


def line_to_message(line: str) -> pico_if.ProtoMessage | None:
    """Convert a string to a message for sending."""
    msg_class: type | None = pico_if.MSG_NAMES.get(line.strip(), None)
    if msg_class is None:
        return None
    return msg_class()


def read_frame(ser: serial.Serial, timestamp: float) -> DisplayLine:
    """Read one frame and return a DisplayLine with both decoded and raw data."""
    byte: bytes = ser.read(1)
    while True:
        if byte[0] == pico_if.SYNC_0:
            next_byte: bytes = ser.read(1)
            if next_byte[0] == pico_if.SYNC_1:
                break
            byte = next_byte
        else:
            byte = ser.read(1)

    header: bytes = ser.read(pico_if.FRAME_HEADER_SIZE - 2)
    payload_len: int = header[0] | (header[1] << 8)
    msg_id: int = header[2]
    payload: bytes = ser.read(payload_len) if payload_len > 0 else b""

    header_bytes = bytes([pico_if.SYNC_0, pico_if.SYNC_1]) + header
    header_hex = " ".join(f"{b:02X}" for b in header_bytes)
    payload_hex = " ".join(f"{b:02X}" for b in payload)

    buf: bytes = header_bytes + payload
    msg_type = str(msg_id)
    body = "UNKNOWN MESSAGE ID"

    try:
        msg: pico_if.ProtoMessage | None = pico_if.decode_message(buf)
        if msg is not None:
            msg_type = type(msg).__name__
            body = json_format.MessageToJson(
                msg,
                always_print_fields_with_no_presence=True,
                preserving_proto_field_name=True,
                indent=None,
            )
    except pico_if.FrameError as e:
        msg_type = "????"
        body = f"FRAME ERROR: {e}"

    return DisplayLine(
        timestamp=timestamp,
        msg_id=msg_id,
        msg_type=msg_type,
        header_hex=header_hex,
        payload_hex=payload_hex,
        body=body,
    )


def serial_loop(
    ser: serial.Serial,
    in_queue: queue.Queue[DisplayItem],
    out_queue: queue.Queue[bytes],
    shutdown_event: threading.Event,
    connected_event: threading.Event,
) -> None:

    start: float = time.time()
    connected_event.set()

    while not shutdown_event.is_set():
        stamp = round(time.time() - start, 4)
        try:
            item: DisplayItem = read_frame(ser=ser, timestamp=stamp)
            in_queue.put(item)

            while not out_queue.empty():
                try:
                    line: bytes = out_queue.get_nowait()
                    ser.write(line)
                    if line[4] == pico_if.MSG_IDS[pico_if.Reset]:
                        shutdown_event.set()
                        break
                except queue.Empty:
                    break

        except (serial.SerialException, OSError):
            connected_event.clear()
            try:
                ser.close()
            except Exception:
                pass
            # Drain stale outbound messages — they can't be delivered until reconnect.
            while not out_queue.empty():
                try:
                    out_queue.get_nowait()
                except queue.Empty:
                    break
            while not shutdown_event.is_set():
                try:
                    ser.open()
                    connected_event.set()
                    break
                except (serial.SerialException, OSError):
                    time.sleep(1.0)

    ser.close()


def read_input(
    window: curses.window,
    x: int,
    y: int,
) -> str | None:
    """
    Read user input.
    Rolling our own lets us still allow users to hit ESC to cancel,
        while the default curses input reader doesn't for some reason.
    """
    curses.noecho()

    cursor_x: int = x
    cursor_y: int = y

    input: str = ""
    while True:
        ch = window.getch()
        if ch == curses.ascii.ESC:  # QUIT
            return None
        elif ch == ord("\n"):  # TERMINATE
            break
        elif ch in (curses.KEY_BACKSPACE, 127, 8):
            if len(input) > 0:
                input = input[:-1]
                cursor_x -= 1
                window.move(cursor_y, x)
                window.clrtoeol()
                window.addstr(y, x, input)
                window.refresh()
        else:
            input += chr(ch)
            window.addstr(y, x, input)
            window.refresh()

    return input


def input_modal(window: curses.window, prompt: str) -> str:
    """
    Spawn a modal that takes user input.
    """
    curses.curs_set(1)

    max_y, max_x = window.getmaxyx()

    modal_width: int = max(40, int(max_y * 0.25))
    modal_height: int = 3

    modal_y: int = (max_y - modal_height) // 2
    modal_x: int = (max_x - modal_width) // 2

    modal: curses.window = curses.newwin(modal_height, modal_width, modal_y, modal_x)
    modal.border()
    modal.addstr(1, 2, prompt)
    modal.refresh()

    input: str | None = read_input(modal, x=len(prompt) + 2, y=1)
    if input is None:
        return ""

    return input


def output_modal(parent: curses.window, message: str) -> None:
    """
    Spawn a modal that shows `message` and can be dismissed by the user.
    """
    curses.curs_set(0)

    max_y, max_x = parent.getmaxyx()

    modal_width: int = min(len(message) + 2, int(max_x * 0.5))
    modal_height: int = 2

    line_len: int = modal_width - 2
    lines: int = 1
    if len(message) > line_len:
        chars: int = len(message) - line_len
        while chars > 0:
            chars = max(chars - line_len, 0)
            lines += 1
    modal_height += lines

    modal_y: int = (max_y - modal_height) // 2
    modal_x: int = (max_x - modal_width) // 2

    modal: curses.window = curses.newwin(modal_height, modal_width, modal_y, modal_x)
    modal.border()
    modal.addstr(1, 1, message)
    modal.refresh()

    parent.nodelay(False)
    _ = parent.getch()
    parent.nodelay(True)


_HELP_LINES: list[tuple[str, str]] = [
    ("h", "this help"),
    ("q", "quit"),
    ("f", "filter messages"),
    ("s", "highlight messages"),
    ("SPACE", "pause / resume"),
    ("r", "toggle raw / decoded view"),
    ("w", "compose + send a message  (^W to send, ESC cancel)"),
    ("c", "clear"),
    ("d", "detach"),
    ("↑ / ↓", "scroll  (while paused)"),
    ("L", "toggle log to file"),
]


def help_modal(parent: curses.window) -> None:
    curses.curs_set(0)

    max_y, max_x = parent.getmaxyx()

    key_col_w = max(len(k) for k, _ in _HELP_LINES) + 2
    desc_col_w = max(len(d) for _, d in _HELP_LINES)
    inner_w = key_col_w + desc_col_w + 2  # 2 for the ·separator + space

    modal_w = inner_w + 4  # 2 border + 2 padding
    modal_h = len(_HELP_LINES) + 4  # border top/bot + header row + blank separator

    modal_y = (max_y - modal_h) // 2
    modal_x = (max_x - modal_w) // 2

    modal = curses.newwin(modal_h, modal_w, modal_y, modal_x)
    modal.border()

    title = " keys "
    modal.addstr(0, (modal_w - len(title)) // 2, title)

    for idx, (key, desc) in enumerate(_HELP_LINES):
        row = idx + 2
        modal.addstr(row, 2, key.rjust(key_col_w - 1))
        modal.addstr(row, key_col_w + 1, "·")
        modal.addstr(row, key_col_w + 3, desc)

    dismiss = "press any key"
    modal.addstr(modal_h - 1, (modal_w - len(dismiss)) // 2, dismiss)
    modal.refresh()

    parent.nodelay(False)
    _ = parent.getch()
    parent.nodelay(True)


@dataclass
class ConsoleConfig:
    log_filter: str = ""
    highlight: str = ""
    paused: bool = False
    refresh_rate: int = 60
    raw: bool = False
    log_path: str = ""


def _render_decoded(
    window: curses.window,
    row: int,
    col: int,
    item: DisplayLine,
    colors_available: bool,
    dimmed: bool = False,
) -> None:
    dim = curses.A_DIM if dimmed else 0
    if colors_available:
        ts_attr = curses.color_pair(_COLOR_TIMESTAMP) | dim
        type_attr = curses.color_pair(_COLOR_MSG_TYPE) | dim
        body_attr = curses.color_pair(_COLOR_BODY) | dim
    else:
        ts_attr = curses.A_NORMAL | dim
        type_attr = curses.A_BOLD | dim
        body_attr = curses.A_NORMAL | dim
    ts_str = f"[{item.timestamp:.4f}]"
    type_str = f"<{item.msg_type}>"
    try:
        window.addstr(row, col, ts_str, ts_attr)
        c = col + len(ts_str)
        window.addstr(row, c, ": ")
        c += 2
        window.addstr(row, c, type_str, type_attr)
        c += len(type_str)
        window.addstr(row, c, ": ")
        c += 2
        window.addstr(row, c, item.body, body_attr)
    except curses.error:
        pass


def _render_raw(
    window: curses.window,
    row: int,
    col: int,
    item: DisplayLine,
    colors_available: bool,
    dimmed: bool = False,
) -> None:
    dim = curses.A_DIM if dimmed else 0
    if colors_available:
        ts_attr = curses.color_pair(_COLOR_TIMESTAMP) | dim
        type_attr = curses.color_pair(_COLOR_MSG_TYPE) | dim
        header_attr = curses.color_pair(_COLOR_HEADER) | dim
        payload_attr = curses.color_pair(_COLOR_PAYLOAD) | dim
    else:
        ts_attr = curses.A_NORMAL | dim
        type_attr = curses.A_BOLD | dim
        header_attr = curses.A_NORMAL | dim
        payload_attr = curses.A_BOLD | dim
    ts_str = f"[{item.timestamp:.4f}]"
    type_str = f"<{item.msg_type} ({item.msg_id})>"
    try:
        window.addstr(row, col, ts_str, ts_attr)
        c = col + len(ts_str)
        window.addstr(row, c, ": ")
        c += 2
        window.addstr(row, c, type_str, type_attr)
        c += len(type_str)
        window.addstr(row, c, ": ")
        c += 2
        window.addstr(row, c, item.header_hex, header_attr)
        c += len(item.header_hex)
        if item.payload_hex:
            window.addstr(row, c, " | ")
            c += 3
            window.addstr(row, c, item.payload_hex, payload_attr)
    except curses.error:
        pass


def _render_item(
    window: curses.window,
    row: int,
    col: int,
    item: DisplayItem,
    colors_available: bool,
    raw: bool = False,
    dimmed: bool = False,
) -> None:
    if raw:
        _render_raw(window, row, col, item, colors_available, dimmed)
    else:
        _render_decoded(window, row, col, item, colors_available, dimmed)


def _draw_status(
    window: curses.window,
    msg_count: int,
    filtered_count: int,
    config: ConsoleConfig,
    connected: bool,
) -> None:
    _, max_x = window.getmaxyx()
    if config.log_filter:
        parts: list[str] = [f"msgs: {filtered_count}/{msg_count}"]
    else:
        parts = [f"msgs: {msg_count}"]
    if not connected:
        parts.append("DISCONNECTED")
    if config.paused:
        parts.append("PAUSED")
    if config.raw:
        parts.append("RAW")
    if config.log_path:
        parts.append(f"log: {config.log_path}")
    if config.log_filter:
        parts.append(f"filter: {config.log_filter!r}")
    if config.highlight:
        parts.append(f"hl: {config.highlight!r}")
    text = "  " + "  │  ".join(parts) + "  "
    text = text[: max_x - 1].ljust(max_x - 1)
    try:
        window.addstr(0, 0, text, curses.A_REVERSE)
    except curses.error:
        pass
    window.refresh()


from google.protobuf.descriptor import FieldDescriptor as _FD

_SCALAR_DEFAULTS = {
    _FD.TYPE_DOUBLE: 0.0,
    _FD.TYPE_FLOAT: 0.0,
    _FD.TYPE_INT64: 0,
    _FD.TYPE_UINT64: 0,
    _FD.TYPE_INT32: 0,
    _FD.TYPE_FIXED64: 0,
    _FD.TYPE_FIXED32: 0,
    _FD.TYPE_BOOL: False,
    _FD.TYPE_STRING: "",
    _FD.TYPE_BYTES: b"",
    _FD.TYPE_UINT32: 0,
    _FD.TYPE_ENUM: 0,
    _FD.TYPE_SFIXED32: 0,
    _FD.TYPE_SFIXED64: 0,
    _FD.TYPE_SINT32: 0,
    _FD.TYPE_SINT64: 0,
}


def _fill_defaults(msg):
    """Recursively populate every field with its default value to produce a useful edit template."""
    for field in msg.DESCRIPTOR.fields:
        if field.is_repeated:
            continue
        if field.message_type is not None:
            sub = getattr(msg, field.name)
            _fill_defaults(sub)
            getattr(msg, field.name).MergeFrom(sub)
        elif field.has_presence and field.type in _SCALAR_DEFAULTS:
            setattr(msg, field.name, _SCALAR_DEFAULTS[field.type])
    return msg


def edit_modal(parent: curses.window, msg_class) -> bytes | None:
    """
    Multi-line JSON editor for composing a protobuf message.
    Returns encoded frame bytes on :w<Enter>, None on ESC.
    """
    inst = _fill_defaults(msg_class())
    json_str = json_format.MessageToJson(
        inst,
        always_print_fields_with_no_presence=True,
        preserving_proto_field_name=True,
        indent=2,
    )
    lines: list[str] = json_str.splitlines()

    max_y, max_x = parent.getmaxyx()
    modal_h = max_y - 4
    modal_w = max_x - 8
    modal_y = 2
    modal_x = 4

    modal = curses.newwin(modal_h, modal_w, modal_y, modal_x)
    modal.keypad(True)
    curses.curs_set(1)

    title = f"[ {msg_class.__name__} ]"
    hint = "  ESC: cancel | ^W: send  "
    text_h = modal_h - 3  # rows between border+title and footer row
    text_w = modal_w - 4  # usable cols (2 border + 2 margin)

    cur_y = 0
    cur_x = 0
    scroll = 0

    while True:
        modal.erase()
        modal.border()
        try:
            modal.addstr(0, (modal_w - len(title)) // 2, title)
        except curses.error:
            pass

        for row in range(text_h):
            li = scroll + row
            if li >= len(lines):
                break
            try:
                modal.addstr(row + 1, 2, lines[li][:text_w])
            except curses.error:
                pass

        try:
            modal.addstr(modal_h - 2, 2, hint[:text_w])
        except curses.error:
            pass
        sy = 1 + (cur_y - scroll)
        sx = 2 + cur_x
        if 1 <= sy < modal_h - 2 and 2 <= sx < modal_w - 1:
            try:
                modal.move(sy, sx)
            except curses.error:
                pass

        modal.refresh()
        ch = modal.getch()

        if ch == 27:
            return None
        elif ch == 23:  # Ctrl+W: send
            text = "\n".join(lines)
            try:
                msg = json_format.Parse(text, msg_class())
                return pico_if.encode_frame(msg)
            except Exception as e:
                output_modal(modal, f"PARSE ERROR: {e}")
        elif ch == curses.KEY_UP:
            if cur_y > 0:
                cur_y -= 1
                cur_x = min(cur_x, len(lines[cur_y]))
                if cur_y < scroll:
                    scroll = cur_y
        elif ch == curses.KEY_DOWN:
            if cur_y < len(lines) - 1:
                cur_y += 1
                cur_x = min(cur_x, len(lines[cur_y]))
                if cur_y >= scroll + text_h:
                    scroll = cur_y - text_h + 1
        elif ch == curses.KEY_LEFT:
            if cur_x > 0:
                cur_x -= 1
            elif cur_y > 0:
                cur_y -= 1
                cur_x = len(lines[cur_y])
                if cur_y < scroll:
                    scroll = cur_y
        elif ch == curses.KEY_RIGHT:
            if cur_x < len(lines[cur_y]):
                cur_x += 1
            elif cur_y < len(lines) - 1:
                cur_y += 1
                cur_x = 0
                if cur_y >= scroll + text_h:
                    scroll = cur_y - text_h + 1
        elif ch == curses.KEY_HOME:
            cur_x = 0
        elif ch == curses.KEY_END:
            cur_x = len(lines[cur_y])
        elif ch in (curses.KEY_BACKSPACE, 127, 8):
            if cur_x > 0:
                line = lines[cur_y]
                lines[cur_y] = line[: cur_x - 1] + line[cur_x:]
                cur_x -= 1
            elif cur_y > 0:
                prev_len = len(lines[cur_y - 1])
                lines[cur_y - 1] += lines[cur_y]
                lines.pop(cur_y)
                cur_y -= 1
                cur_x = prev_len
                if cur_y < scroll:
                    scroll = cur_y
        elif ch == curses.KEY_DC:
            line = lines[cur_y]
            if cur_x < len(line):
                lines[cur_y] = line[:cur_x] + line[cur_x + 1 :]
            elif cur_y < len(lines) - 1:
                lines[cur_y] += lines[cur_y + 1]
                lines.pop(cur_y + 1)
        elif ch in (curses.KEY_ENTER, ord("\n"), ord("\r")):
            line = lines[cur_y]
            lines[cur_y] = line[:cur_x]
            lines.insert(cur_y + 1, line[cur_x:])
            cur_y += 1
            cur_x = 0
            if cur_y >= scroll + text_h:
                scroll = cur_y - text_h + 1
        elif 32 <= ch < 127:
            line = lines[cur_y]
            lines[cur_y] = line[:cur_x] + chr(ch) + line[cur_x:]
            cur_x += 1


def gui_loop(
    window: curses.window,
    in_queue: queue.Queue[DisplayItem],
    out_queue: queue.Queue,
    config: ConsoleConfig,
    shutdown_event: threading.Event,
    connected_event: threading.Event,
) -> None:

    window.clear()
    window.nodelay(True)
    window.border()
    curses.curs_set(0)

    colors_available: bool = curses.has_colors()
    if colors_available:
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(_COLOR_TIMESTAMP, curses.COLOR_BLUE, -1)
        curses.init_pair(_COLOR_MSG_TYPE, curses.COLOR_MAGENTA, -1)
        curses.init_pair(_COLOR_BODY, curses.COLOR_GREEN, -1)
        curses.init_pair(_COLOR_HEADER, curses.COLOR_CYAN, -1)
        curses.init_pair(_COLOR_PAYLOAD, curses.COLOR_YELLOW, -1)

    attr: int = curses.A_NORMAL

    start_row: int = 1
    start_col: int = 1

    current_col: int = 0

    total_rows, total_cols = window.getmaxyx()
    total_rows -= 2  # border
    total_cols -= 2  # border
    data_rows: int = total_rows - 1  # 1 row reserved for status bar

    data_window: curses.window = curses.newwin(
        data_rows, total_cols, start_row, start_col
    )
    data_window.scrollok(True)
    data_window.idlok(True)

    status_window: curses.window = curses.newwin(
        1, total_cols, start_row + data_rows, start_col
    )

    current_row = 0
    msg_count: int = 0
    filtered_count: int = 0

    log_file: IO[str] | None = None
    if config.log_path:
        try:
            log_file = open(config.log_path, "a")
        except OSError as e:
            output_modal(window, f"LOG OPEN FAILED: {e}")
            config.log_path = ""

    scrollback: deque[DisplayItem] = deque()
    scroll_offset: int = 0

    cursor: int = data_rows

    def redraw():
        data_window.erase()
        visible_rows = data_window.getmaxyx()[0]

        start: int = max(0, (len(scrollback) - visible_rows - scroll_offset))
        end = start + visible_rows
        items = list(scrollback)[start:end]
        for row, item in enumerate(items):
            if row >= visible_rows:
                break

            item_str = str(item)
            if config.log_filter != "" and config.log_filter not in item_str:
                continue
            matched = config.highlight != "" and config.highlight in item_str
            dim = config.highlight != "" and not matched
            try:
                data_window.addstr(row, 0, "▶ " if matched else "  ")
            except curses.error:
                pass
            _render_item(data_window, row, 2, item, colors_available, config.raw, dim)

            if cursor == row:
                data_window.chgat(row, 0, curses.A_REVERSE)

        data_window.refresh()

    sleep_dur: float = 1 / config.refresh_rate
    while True:
        time.sleep(sleep_dur)

        # Handle resizes
        new_rows, new_cols = window.getmaxyx()
        if curses.is_term_resized(new_rows, new_cols):
            curses.resizeterm(new_rows, new_cols)
            window.clear()
            window.border()
            window.refresh()

            total_rows = new_rows - 2
            total_cols = new_cols - 2
            data_rows = total_rows - 1
            data_window.resize(data_rows, total_cols)
            data_window.mvwin(1, 1)
            data_window.refresh()
            status_window.resize(1, total_cols)
            status_window.mvwin(1 + data_rows, 1)

        # Write new lines from the serial inbound queue to the screen
        while not in_queue.empty():
            try:
                item: DisplayItem = in_queue.get_nowait()
            except queue.Empty:
                break

            item_str = str(item)
            msg_count += 1

            if log_file is not None:
                log_file.write(item_str + "\n")
                log_file.flush()

            if config.paused:
                redraw()
                continue
            if config.log_filter != "" and config.log_filter not in item_str:
                continue

            filtered_count += 1
            highlighted = config.highlight != "" and config.highlight in item_str
            scrollback.append(item)

            # Advance by one row. Scroll the data window instead of overflowing —
            # do this BEFORE writing to avoid a blank line at the bottom.
            current_row += 1
            if current_row > data_window.getmaxyx()[0] - 1:
                data_window.scroll(1)
                current_row -= 1

            try:
                dim = config.highlight != "" and not highlighted
                data_window.addstr(current_row, 0, "▶ " if highlighted else "  ")
                _render_item(
                    data_window, current_row, 2, item, colors_available, config.raw, dim
                )
                data_window.refresh()
            except curses.error:
                pass

        _draw_status(
            status_window, msg_count, filtered_count, config, connected_event.is_set()
        )

        # Input handling
        ch = window.getch()
        if ch == ord("q"):  # 'QUIT'
            break
        elif ch == ord("L"):  # 'LOG'
            if log_file is not None:
                log_file.close()
                log_file = None
                config.log_path = ""
            else:
                path = input_modal(data_window, "log to: ")
                if path:
                    try:
                        log_file = open(path, "a")
                        config.log_path = path
                    except OSError as e:
                        output_modal(data_window, f"LOG OPEN FAILED: {e}")
        elif ch == ord("h"):  # 'HELP'
            help_modal(window)
        elif ch == ord("f"):  # 'FILTER'
            data_window.clear()
            config.log_filter = input_modal(data_window, "filter: ")
            filtered_count = (
                sum(1 for item in scrollback if config.log_filter in str(item))
                if config.log_filter
                else 0
            )
        elif ch == ord("s"):  # 'SELECT'
            config.highlight = input_modal(data_window, "highlight: ")
        elif ch == ord(" "):  # 'PAUSE'
            config.paused = not config.paused
        elif ch == ord("w"):  # 'WRITE'
            msg_name: str = input_modal(data_window, "message: ")
            if not msg_name:
                continue
            msg_class = pico_if.MSG_NAMES.get(msg_name.strip())
            if msg_class is None:
                output_modal(data_window, f"Unknown message: {msg_name.strip()}")
                continue
            packed = edit_modal(window, msg_class)
            if packed is not None:
                out_queue.put(packed)
        elif ch == ord(":"):  # 'MORE OPTIONS'
            command = input_modal(window, ":")
        elif ch == curses.KEY_UP:
            if not config.paused:
                continue
            cursor = max(0, cursor - 1)
            if cursor == 0:
                if scroll_offset < len(scrollback):
                    scroll_offset += 1
        elif ch == curses.KEY_DOWN:
            if not config.paused:
                continue
            cursor += 1
            if cursor >= data_rows:
                if scroll_offset > 0:
                    scroll_offset -= 1
                cursor -= 1
        elif ch == ord("r"):  # 'RAW toggle'
            config.raw = not config.raw
            redraw()
        elif ch == ord("c"):  # 'CLEAR'
            scrollback.clear()
            data_window.clear()
            data_window.refresh()
            cursor = total_rows
            current_row = 0
        elif ch == ord("d"):  # 'DETACH'
            shutdown_event.set()
            output_modal(window, "DETACHED")

    if log_file is not None:
        log_file.close()


def plain_loop(
    in_queue: queue.Queue[DisplayItem],
    shutdown_event: threading.Event,
    raw: bool = False,
    log_file: IO[str] | None = None,
) -> None:
    while not shutdown_event.is_set():
        try:
            item: DisplayItem = in_queue.get(timeout=0.1)
            if raw:
                sep = " | " if item.payload_hex else ""
                line = f"[{item.timestamp:.4f}]: <{item.msg_type} ({item.msg_id})>: {item.header_hex}{sep}{item.payload_hex}"
            else:
                line = str(item)
            print(line, flush=True)
            if log_file is not None:
                log_file.write(line + "\n")
                log_file.flush()
        except queue.Empty:
            continue


def main() -> None:

    parser: argparse.ArgumentParser = argparse.ArgumentParser(
        description="pico_interface serial console"
    )
    parser.add_argument(
        "--plain",
        "-p",
        action="store_true",
        help="print messages to stdout instead of opening the curses UI",
    )
    parser.add_argument(
        "--raw",
        "-r",
        action="store_true",
        help="display raw hex bytes instead of decoded messages",
    )
    parser.add_argument(
        "--timeout",
        "-t",
        type=float,
        metavar="SECONDS",
        help="stop capturing after SECONDS and exit",
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="serial port to open (default: /dev/ttyACM0)",
    )
    parser.add_argument(
        "--log",
        "-l",
        metavar="FILE",
        help="log all messages to FILE",
    )
    args: argparse.Namespace = parser.parse_args()

    try:
        ser: serial.Serial = serial.Serial(
            port=args.port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )
    except Exception as e:
        print(f"Failed to open {args.port}: {e}", file=sys.stderr)
        return

    log_file: IO[str] | None = None
    if args.log:
        try:
            log_file = open(args.log, "a")
        except OSError as e:
            print(f"Failed to open log file {args.log}: {e}", file=sys.stderr)
            return

    in_queue: queue.Queue[DisplayItem] = queue.Queue()
    out_queue: queue.Queue[bytes] = queue.Queue()

    ser_shutdown_event: threading.Event = threading.Event()
    connected_event: threading.Event = threading.Event()
    ser_thread: threading.Thread = threading.Thread(
        target=serial_loop,
        args=(ser, in_queue, out_queue, ser_shutdown_event, connected_event),
        daemon=True,
    )
    ser_thread.start()

    if args.timeout is not None:
        timer: threading.Timer = threading.Timer(args.timeout, ser_shutdown_event.set)
        timer.daemon = True
        timer.start()

    try:
        if args.plain:
            plain_loop(in_queue, ser_shutdown_event, raw=args.raw, log_file=log_file)
        else:
            config: ConsoleConfig = ConsoleConfig(raw=args.raw, log_path=args.log or "")
            curses.wrapper(
                gui_loop,
                in_queue,
                out_queue,
                config,
                ser_shutdown_event,
                connected_event,
            )
    except KeyboardInterrupt:
        ser_shutdown_event.set()
    finally:
        if log_file is not None:
            log_file.close()


if __name__ == "__main__":
    main()
