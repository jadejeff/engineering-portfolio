import argparse
import csv
import re
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Missing dependency: pyserial")
    print("Install it with: python -m pip install pyserial")
    input("Press Enter to close...")
    raise SystemExit(1)


BAUD_RATE = 115200
SAMPLE_DIR = Path(__file__).resolve().parent / "samples"


def choose_port(requested_port: str | None) -> str:
    if requested_port:
        return requested_port

    ports = list(list_ports.comports())
    if not ports:
        print("No serial port found. Connect the U5 serial adapter and retry.")
        input("Press Enter to close...")
        raise SystemExit(1)

    if len(ports) == 1:
        print(f"Using {ports[0].device}: {ports[0].description}")
        return ports[0].device

    print("Available serial ports:")
    for index, port in enumerate(ports, start=1):
        print(f"  {index}. {port.device} - {port.description}")

    while True:
        selection = input("Select port number: ").strip()
        if selection.isdigit() and 1 <= int(selection) <= len(ports):
            return ports[int(selection) - 1].device
        print("Invalid selection.")


def next_sample_path() -> Path:
    SAMPLE_DIR.mkdir(parents=True, exist_ok=True)
    pattern = re.compile(r"sample_(\d{4})\.csv$", re.IGNORECASE)
    numbers = []

    for path in SAMPLE_DIR.glob("sample_*.csv"):
        match = pattern.match(path.name)
        if match:
            numbers.append(int(match.group(1)))

    next_number = max(numbers, default=0) + 1
    return SAMPLE_DIR / f"sample_{next_number:04d}.csv"


def capture(port_name: str, baud_rate: int) -> None:
    output_path = next_sample_path()
    sample_count = 0
    first_device_ms = None

    print(f"Opening {port_name} at {baud_rate} baud")
    print(f"Saving to: {output_path}")
    print("Sampling started. Press Ctrl+C or close this window to stop.")

    with serial.Serial(port_name, baud_rate, timeout=1) as port:
        port.reset_input_buffer()

        with output_path.open("w", newline="", encoding="utf-8-sig") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(["time_s", "current_a", "power_w"])
            csv_file.flush()

            try:
                while True:
                    raw_line = port.readline()
                    if not raw_line:
                        continue

                    line = raw_line.decode("ascii", errors="ignore").strip()
                    parts = line.split(",")
                    if len(parts) != 4 or parts[0] != "DATA":
                        continue

                    try:
                        device_ms = int(parts[1])
                        current_a = float(parts[2])
                        power_w = float(parts[3])
                    except ValueError:
                        print(f"Ignored malformed line: {line}")
                        continue

                    if first_device_ms is None:
                        first_device_ms = device_ms

                    elapsed_ms = (device_ms - first_device_ms) & 0xFFFFFFFF
                    time_s = elapsed_ms / 1000.0
                    sample_count += 1

                    writer.writerow([
                        f"{time_s:.3f}",
                        f"{current_a:.4f}",
                        f"{power_w:.2f}",
                    ])
                    csv_file.flush()
                    print(
                        f"#{sample_count:06d} "
                        f"t={time_s:.3f}s I={current_a:.4f}A P={power_w:.2f}W"
                    )
            except KeyboardInterrupt:
                pass

    print(f"Stopped. Saved {sample_count} samples to:")
    print(output_path)


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture STM32U5 current and power to CSV")
    parser.add_argument("--port", help="Serial port, for example COM7")
    parser.add_argument("--baud", type=int, default=BAUD_RATE)
    args = parser.parse_args()

    try:
        capture(choose_port(args.port), args.baud)
    except serial.SerialException as exc:
        print(f"Serial error: {exc}")
        raise SystemExit(1) from exc


if __name__ == "__main__":
    main()
