STM32U5 USART1 CSV sampler

1. Install Python 3.
2. Install the dependency:
   python -m pip install -r requirements.txt
3. Connect USART1 TX (PA9) to the USB-to-serial adapter RX.
4. Connect U5 GND to the adapter GND.
5. Run:
   python serial_logger.py

The script selects a serial port, starts immediately, and stores each run in:
samples/sample_0001.csv
samples/sample_0002.csv
...

Press Ctrl+C or close the console window to stop. Every received row is flushed
to disk immediately, so completed samples remain saved after the window closes.
