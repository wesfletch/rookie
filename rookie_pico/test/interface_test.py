#!/usr/bin/env python3
"""
This was intended to replicate the issue I was having with repeated commands.
However, I can't seem to get it to happen here. Hmmmmm...
"""

import struct
import sys
import time

import serial

port: str = "/dev/ttyACM0"
baudrate: int = 115200

ser: serial.Serial = serial.Serial(port=port, baudrate=baudrate, timeout=0.001)

STATUS_CMD: bytes = b"$SYS.C 0 STATUS\n"

import pico_interface as interface


def main() -> None:
    last_cmd: bytes = b""
    time_last_cmd: float = 0.0
    expecting_ack: bool = False
    while True:
        # READ
        data: str = ser.readline().rstrip()
        if not data:
            continue

        print(data)
        msg_id: bytes = b"".join(struct.unpack("6c", data[:6]))
        print(msg_id)
        msg_type: interface.Message | None = interface.MSG_IDS.get(
            msg_id.decode("utf-8")[:4], None
        )

        # # .decode("utf-8").strip()

        # message_type: pico_interface.Message = pico_interface.MSG_IDS.get(data)

        # print(data)

        # if expecting_ack:
        #     decoded_last_cmd = last_cmd.decode("utf-8").strip()
        #     expected_response = f"$ACK {decoded_last_cmd}"
        #     if expected_response not in data:
        #         print(
        #             f"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA NO ACK '{decoded_last_cmd}`"
        #         )
        #     else:
        #         print("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB GOT ACK")
        #         expecting_ack = False

        # # WRITE
        # if time.time() > (time_last_cmd + 0.5):
        #     ser.write(STATUS_CMD)

        #     last_cmd = STATUS_CMD
        #     time_last_cmd = time.time()
        #     expecting_ack = True


if __name__ == "__main__":
    main()
