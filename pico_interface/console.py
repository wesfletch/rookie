#!/usr/bin/env python3

# Dev tool that opens a minicom-style console to view pico_interface messages.
# Supports some basic wireshark-esque commands, like filtering and highlighting.

# /// script
# requires-python = ">=3.10.12"
# dependencies = [
#   "curses",
#   "serial",
# ]
# ///

from collections import deque
import curses
import curses.ascii
from dataclasses import dataclass
import queue
import serial
import threading
import time

import pico_interface as pico_if


def line_to_message(line: str) -> pico_if.Message | None:

    msg_id: str = line.decode()[:4]

    # Get the message object
    msg_type: type | None = pico_if.MSG_IDS.get(msg_id, None)
    if msg_type is None:
        return None

    # Instantiate the message
    unpacked: pico_if.Message = msg_type.unpack(line)
    return unpacked

def get_line(
    ser: serial.Serial,
    timestamp: float
) -> str:

    line: str = ser.readline()

    msg_id: str = line.decode()[:4]

    # Get the message object
    msg_type: type | None = pico_if.MSG_IDS.get(msg_id, None)
    if msg_type is None:
        return f"[{timestamp:.4f}]: <????>: '{line.decode().rstrip()}'"

    # Instantiate the message
    try:
        unpacked: pico_if.Message = msg_type.unpack(line)
    except Exception as e:
        return f"[{timestamp:.4f}]: <????>: MALFORMED LINE <{line}>"

    prefix: str = ""
    if unpacked._msg_type:
        if unpacked._msg_type == pico_if.Message.MESSAGE_TYPE.COMMAND:
            prefix = "[CMD] "
        elif unpacked._msg_type == pico_if.Message.MESSAGE_TYPE.STATUS:
            prefix = "[STS] "

    return f"[{timestamp:.4f}]: <{msg_id}>: {prefix}{str(unpacked)}"


def serial_loop(
    ser: serial.Serial,
    in_queue: queue.Queue[str],
    out_queue: queue.Queue[bytes],
    shutdown_event: threading.Event,
) -> None:

    start: float = time.time()
    stamp: float = 0

    while not shutdown_event.is_set():
        stamp = round(time.time() - start, 4)

        # Read inputs
        line: str = get_line(ser=ser, timestamp=stamp)
        in_queue.put(line)

        # Send outputs
        while not out_queue.empty():
            try:
                line: bytes = out_queue.get_nowait()
                ser.write(line)

                # SPECIAL CASE: if we're sending a reset command, we need to kill our serial console PRONTO
                # to ensure we can actually access it when we come back
                if b'$RST' in line:
                    shutdown_event.set()
                    break
            except queue.Empty:
                break

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

    # Get the user input
    input: str = ""
    while True:
        ch = window.getch()
        if ch == curses.ascii.ESC: # QUIT
            return None
        elif ch == ord("\n"): # TERMINATE
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

def input_modal(
    window: curses.window, 
    prompt: str
) -> str:
    """
    Spawn a modal that takes user input.
    """
    curses.curs_set(1)

    # get size of parent window
    max_y, max_x = window.getmaxyx()

    # define modal size
    modal_width: int = max(40, int(max_y * 0.25))
    modal_height: int = 3

    # center the modal
    modal_y: int = (max_y - modal_height) // 2
    modal_x: int = (max_x - modal_width) // 2

    # create the modal window
    modal: curses.window = curses.newwin(modal_height, modal_width, modal_y, modal_x)
    modal.border()
    modal.addstr(1, 2, prompt)
    modal.refresh()

    input: str | None = read_input(modal, x=len(prompt) + 2, y=1)
    if input is None:
        return ""

    return input


def output_modal(
    parent: curses.window, 
    message: str
) -> None:
    """
    Spawn a modal that shows `message` and can be dismissed by the user.
    """
    curses.curs_set(0)

    # get size of parent window
    max_y, max_x = parent.getmaxyx()

    # define modal size
    modal_width: int = min(len(message) + 2, int(max_x * 0.5))
    modal_height: int = 2

    # Figure out how many lines we need in the modal to fit our text (give or take)
    line_len: int = modal_width - 2
    lines: int = 1
    if len(message) > line_len:
        chars: int = len(message) - line_len
        while chars > 0:
            chars = max(chars - line_len, 0)
            lines += 1
    modal_height += lines

    # center the modal in the parent
    modal_y: int = (max_y - modal_height) // 2
    modal_x: int = (max_x - modal_width) // 2

    # create the modal window
    modal: curses.window = curses.newwin(modal_height, modal_width, modal_y, modal_x)
    modal.border()
    modal.addstr(1, 1, message)
    modal.refresh()

    # Wait for ANY input to act as acknowledgement, we don't care what it is...
    parent.nodelay(False)
    _ = parent.getch()
    # ... then reset and continue
    parent.nodelay(True)


@dataclass
class ConsoleConfig:
    log_filter: str = ""
    highlight: str = ""
    paused: bool = False
    refresh_rate: int = 60


def gui_loop(
    window: curses.window,
    in_queue: queue.Queue,
    out_queue: queue.Queue,
    config: ConsoleConfig,
    shutdown_event: threading.Event, 
) -> None:

    window.clear()
    window.nodelay(True)
    window.border()
    curses.curs_set(0)

    # Attribute of the line we're writing
    attr: int = curses.A_NORMAL

    start_row: int = 1
    start_col: int = 1 

    current_col: int = 0

    total_rows, total_cols = window.getmaxyx()
    total_rows -= 2 # border
    total_cols -= 2 # border

    data_window: curses.window = curses.newwin(
        total_rows, total_cols,
        start_row, start_col 
    )
    data_window.scrollok(True)
    data_window.idlok(True)

    current_row = 0

    scrollback: deque[str] = deque()
    scroll_offset: int = 0

    cursor: int = total_rows

    def redraw():
        data_window.erase()
        visible_rows = data_window.getmaxyx()[0]

        start: int = max(0, (len(scrollback) - visible_rows - scroll_offset))
        end = start + visible_rows
        lines = list(scrollback)[start:end]
        for row, line in enumerate(lines):
            if row >= visible_rows:
                break

            if config.log_filter != "" and config.log_filter not in line:
                continue
            if config.highlight != "" and config.highlight in line:
                attr = curses.A_REVERSE
            else:
                attr = curses.A_NORMAL

            data_window.addstr(row, 0, line, attr)
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
            data_window.resize(total_rows, total_cols)
            data_window.mvwin(1,1)
            data_window.refresh()

        # Write new lines from the serial inbound queue to the screen 
        while not in_queue.empty():
            try:
                line: str = in_queue.get_nowait()
            except queue.Empty:
                break

            if config.paused:
                redraw()
                continue
            if config.log_filter != "" and config.log_filter not in line:
                continue
            if config.highlight != "" and config.highlight in line:
                attr = curses.A_REVERSE
            else:
                attr = curses.A_NORMAL

            scrollback.append(line)

            # Advance by one row
            current_row += 1
            # If we're about to run off the screen, scroll the data window instead.
            # Do this BEFORE we write the string to the screen to avoid that annoying
            # blank line at the bottom.
            if current_row > data_window.getmaxyx()[0] - 1:
                data_window.scroll(1)
                current_row -= 1

            try:
                # Actually write the string to the window
                data_window.addstr(current_row, current_col, line)
                # Apply the attribute to the entire line
                data_window.chgat(current_row, current_col, data_window.getmaxyx()[1], attr)
                # refresh every time we add in the hopes of looking less jumpy
                data_window.refresh()
            except curses.error:
                pass

        # Input handling
        ch = window.getch()
        if ch == ord('q'): # 'QUIT'
            break
        elif ch == ord('f'): # 'FILTER'
            data_window.clear()
            config.log_filter = input_modal(data_window, "filter: ")
        elif ch == ord('s'): # 'SELECT'
            config.highlight = input_modal(data_window, "highlight: ")
        elif ch == ord(' '): # 'PAUSE
            config.paused = not config.paused
        elif ch == ord('w'): # 'WRITE'
            output: str = input_modal(data_window, "write: ")
            if not output:
                continue
            message: pico_if.Message | None = line_to_message(output.encode())
            if not message:
                output_modal(data_window, message="MALFORMED MESSAGE")
                continue
            packed: bytes = message.pack(message._msg_type)
            out_queue.put(packed)
        elif ch == ord(':'): # 'MORE OPTIONS'
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
            if cursor >= total_rows:
                if scroll_offset > 0:
                    scroll_offset -= 1
                cursor -= 1
        elif ch == ord('c'): # 'CLEAR'
            scrollback.clear()
            data_window.clear()
            data_window.refresh()
            cursor = total_rows
            current_row = 0
        elif ch == ord('d'): # 'DETACH'
            shutdown_event.set()
            output_modal(window, "DETACHED")


def main() -> None:

    try:
        ser: serial.Serial = serial.Serial(
            port="/dev/ttyACM0",
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
    except Exception as e:
        return None

    in_queue: queue.Queue[str] = queue.Queue()
    out_queue: queue.Queue[str] = queue.Queue()

    ser_shutdown_event: threading.Event = threading.Event()
    ser_thread: threading.Thread = threading.Thread(
        target=serial_loop,
        args=(ser, in_queue, out_queue, ser_shutdown_event),
        daemon=True
    )
    ser_thread.start()

    config: ConsoleConfig = ConsoleConfig()

    # Returns only after KeyboardInterrupt or user quits (or it crashes)
    curses.wrapper(gui_loop, in_queue, out_queue, config, ser_shutdown_event)

if __name__ == "__main__":
    main()