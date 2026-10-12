"""
OpenMV multimode launcher for 2026 E puzzle device.

STM32 commands:
    $MODE,1#  Requirement 1 fixed-piece successful algorithm
    $MODE,2#  Requirement 2(1) unknown white-piece successful solver
    $MODE,3#  Requirement 2(2) 2-4-piece successful solver

Files required beside main.py:
    req1_success_mode.py
    req2_white_success_mode.py
    geometry_solver.py
    req2_2_success_mode.py
    geometry_solver_req2_2.py
"""

import time
import gc
import sys

UART_ID = 3
UART_BAUD = 115200
UART_RX_BUFFER_MAX = 256

uart_rx_buffer = ""


def init_uart():
    try:
        from pyb import UART
        port = UART(UART_ID, UART_BAUD, timeout_char=100)
        print("UART%d @ %d OK (P4=TX, P5=RX)" % (UART_ID, UART_BAUD))
        return port
    except Exception as first_error:
        try:
            from machine import UART
            port = UART(UART_ID, UART_BAUD)
            print("machine.UART%d @ %d OK" % (UART_ID, UART_BAUD))
            return port
        except Exception as second_error:
            print("UART init failed:", first_error, "/", second_error)
            return None


def send_frame(uart_obj, payload):
    if uart_obj is None:
        return False

    frame = "$" + payload + "#\n"

    try:
        uart_obj.write(frame)
        print("UART TX:", frame.strip())
        return True
    except Exception as error:
        print("UART TX failed:", error)
        return False


def poll_frames(uart_obj):
    global uart_rx_buffer

    if uart_obj is None:
        return []

    try:
        count = uart_obj.any()
    except Exception:
        count = 0

    if not count:
        return []

    try:
        raw = uart_obj.read(count)
    except Exception:
        return []

    if raw is None:
        return []

    try:
        if isinstance(raw, (bytes, bytearray)):
            try:
                chunk = raw.decode()
            except Exception:
                chunk = "".join(chr(value) for value in raw)
        else:
            chunk = str(raw)
    except Exception:
        chunk = ""

    if not chunk:
        return []

    uart_rx_buffer += chunk

    if len(uart_rx_buffer) > UART_RX_BUFFER_MAX:
        start = uart_rx_buffer.rfind("$")
        uart_rx_buffer = uart_rx_buffer[start:] if start >= 0 else ""

    messages = []

    while True:
        start = uart_rx_buffer.find("$")

        if start < 0:
            uart_rx_buffer = ""
            break

        if start > 0:
            uart_rx_buffer = uart_rx_buffer[start:]

        end = uart_rx_buffer.find("#", 1)

        if end < 0:
            break

        payload = uart_rx_buffer[1:end].strip()
        uart_rx_buffer = uart_rx_buffer[end + 1:]

        if payload:
            messages.append(payload)

    return messages


def parse_mode(message):
    if not message.startswith("MODE,"):
        return 0

    try:
        mode = int(message.split(",", 1)[1])
    except Exception:
        return 0

    return mode if mode in (1, 2, 3) else 0


def unload_module(name):
    try:
        if name in sys.modules:
            del sys.modules[name]
    except Exception:
        pass


def run_selected_mode(mode, uart_obj):
    gc.collect()

    if mode == 1:
        send_frame(uart_obj, "MODE_ACK,1")
        send_frame(uart_obj, "STATUS,REQ1_LOADING")

        module = __import__("req1_success_mode")
        next_mode = module.run(uart_obj)

        del module
        unload_module("req1_success_mode")
        gc.collect()
        return next_mode

    if mode == 2:
        send_frame(uart_obj, "MODE_ACK,2")
        send_frame(uart_obj, "STATUS,REQ2_WHITE_LOADING")

        module = __import__("req2_white_success_mode")
        next_mode = module.run(uart_obj)

        del module
        unload_module("req2_white_success_mode")
        unload_module("geometry_solver")
        gc.collect()
        return next_mode

    send_frame(uart_obj, "MODE_ACK,3")
    send_frame(uart_obj, "STATUS,REQ2_CARD_LOADING")

    module = __import__("req2_2_success_mode")
    next_mode = module.run(uart_obj)

    del module
    unload_module("req2_2_success_mode")
    unload_module("geometry_solver_req2_2")
    gc.collect()
    return next_mode


uart = init_uart()

if uart is None:
    while True:
        time.sleep_ms(500)

send_frame(uart, "READY")
print(">>> Waiting for STM32 mode selection")

pending_mode = 0

while True:
    if pending_mode == 0:
        for message in poll_frames(uart):
            print("MCU RX:", message)
            mode = parse_mode(message)

            if mode:
                pending_mode = mode
                break

        if pending_mode == 0:
            time.sleep_ms(10)
            continue

    selected_mode = pending_mode
    pending_mode = 0

    try:
        next_mode = run_selected_mode(selected_mode, uart)
    except Exception as error:
        print("Mode runtime failed:", error)
        send_frame(uart, "ERR,OPENMV_RUNTIME")
        next_mode = 0

    if next_mode in (1, 2, 3):
        pending_mode = next_mode
    else:
        send_frame(uart, "READY")
        print(">>> Waiting for next STM32 mode selection")
