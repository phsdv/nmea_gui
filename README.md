# nmea_gui
Real-time GUI for monitoring a GNSS receiver on a serial/COM port
(e.g. COM4 on Windows, /dev/ttyUSB0 on Linux/macOS).

Reads NMEA 0183 sentences (GGA, RMC, GSA, GSV, GLL, VTG) from the port in
a background thread and displays live-updating:
  - Position / fix quality / HDOP / satellite count / speed
  - A per-satellite signal-strength (SNR) table
  - A live track plot and altitude trace
  - A scrolling raw NMEA console

Requirements:
  PyQt6 pyserial matplotlib

Usage:
    uv run nmea_gui.py
    (then enter your COM port in the gui, e.g. COM4, and click Connect)

    python3 nmea_gui.py --port COM4 --baud 9600   (auto-connects on launch, 9600 is now the default)

This software id provided as is. There is no plan to improve it nor maintain it.
