#!/usr/bin/env python3
"""
Configure the beacon's u-blox NEO-M8P for the HC-12 link, over its USB port.

UART1 (wired to the HC-12) sends only NMEA GGA, 9600 baud 8N1, at 5 Hz, which
is what the robot's beacon receiver (config/beacon_gps.yaml) expects. All NMEA
messages at 5 Hz would overflow 9600 baud. USB output is left at its defaults.
The configuration is saved to flash and read back.

  python3 configure_beacon_gps.py /dev/ttyACM6        # configure + verify
  python3 configure_beacon_gps.py /dev/ttyACM6 --check  # verify only
"""
import argparse
import struct
import sys
import time

import serial

# NMEA message ids (class 0xF0). GGA is the only one sent on UART1.
NMEA_IDS = {'GGA': 0x00, 'GLL': 0x01, 'GSA': 0x02, 'GSV': 0x03, 'RMC': 0x04, 'VTG': 0x05,
            'GRS': 0x06, 'GST': 0x07, 'ZDA': 0x08, 'GBS': 0x09, 'DTM': 0x0A, 'GNS': 0x0D,
            'VLW': 0x0F}
USB_DEFAULT_ON = ('GGA', 'GLL', 'GSA', 'GSV', 'RMC', 'VTG')
RATE_MS = 200  # 5 Hz
BAUD = 9600


def frame(cls, mid, payload=b''):
    body = bytes([cls, mid]) + struct.pack('<H', len(payload)) + payload
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return b'\xb5\x62' + body + bytes([a, b])


def read_ubx(ser, want, timeout):
    end = time.time() + timeout
    buf = b''
    while time.time() < end:
        buf += ser.read(ser.in_waiting or 1)
        while True:
            i = buf.find(b'\xb5\x62')
            if i < 0 or len(buf) < i + 6:
                break
            cls, mid = buf[i + 2], buf[i + 3]
            n = struct.unpack('<H', buf[i + 4:i + 6])[0]
            if len(buf) < i + 8 + n:
                break
            payload, buf = buf[i + 6:i + 6 + n], buf[i + 8 + n:]
            if (cls, mid) == want:
                return payload
    return None


def poll(ser, cls, mid, payload=b'', tries=4):
    for _ in range(tries):  # replies can get lost in the NMEA stream
        ser.reset_input_buffer()
        ser.write(frame(cls, mid, payload))
        reply = read_ubx(ser, (cls, mid), 1.0)
        if reply is not None:
            return reply
    return None


def send_acked(ser, cls, mid, payload):
    ser.reset_input_buffer()
    ser.write(frame(cls, mid, payload))
    end = time.time() + 2.0
    while time.time() < end:
        ack = read_ubx(ser, (0x05, 0x01), end - time.time())
        if ack is not None and ack[:2] == bytes([cls, mid]):
            return True
    return False


def configure(ser):
    ok = True
    for name, mid in NMEA_IDS.items():
        uart1 = 1 if name == 'GGA' else 0
        usb = 1 if name in USB_DEFAULT_ON else 0
        # Rates per port: DDC, UART1, UART2, USB, SPI, reserved
        ok &= send_acked(ser, 0x06, 0x01, bytes([0xF0, mid, 0, uart1, 0, usb, 0, 0]))
    ok &= send_acked(ser, 0x06, 0x08, struct.pack('<HHH', RATE_MS, 1, 1))
    # Save all sections to battery-backed RAM and flash.
    ok &= send_acked(ser, 0x06, 0x09, struct.pack('<IIIB', 0, 0x0000FFFF, 0, 0x17))
    return ok


def check(ser):
    good = True
    version = poll(ser, 0x0A, 0x04)
    if version:
        extensions = [version[i:i + 30].rstrip(b'\0').decode()
                      for i in range(40, len(version), 30)]
        print('Module:', ', '.join(e for e in extensions if e.startswith(('MOD=', 'FWVER='))))
    for name, mid in NMEA_IDS.items():
        rates = poll(ser, 0x06, 0x01, bytes([0xF0, mid]))
        want = 1 if name == 'GGA' else 0
        if rates is None or rates[3] != want:
            print(f'  {name}: UART1 rate {rates[3] if rates else "?"}, want {want}')
            good = False
    port = poll(ser, 0x06, 0x00, b'\x01')
    baud, out_proto = struct.unpack('<IxxH', port[8:16]) if port else (None, None)
    if baud != BAUD or not out_proto or not out_proto & 0x0002:
        print(f'  UART1: baud {baud}, out protocols 0x{out_proto or 0:04X}; want {BAUD}, NMEA')
        good = False
    rate = poll(ser, 0x06, 0x08)
    if rate is None or struct.unpack('<H', rate[:2])[0] != RATE_MS:
        print(f'  measurement rate {struct.unpack("<H", rate[:2])[0] if rate else "?"} ms, '
              f'want {RATE_MS}')
        good = False
    print('UART1: GGA only, 9600 baud, 5 Hz' if good else 'Configuration does not match')
    return good


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('port', help='NEO-M8P USB serial port, e.g. /dev/ttyACM6')
    parser.add_argument('--check', action='store_true', help='only read back and verify')
    args = parser.parse_args()
    with serial.Serial(args.port, 9600, timeout=0.2) as ser:
        if not args.check and not configure(ser):
            print('The module did not acknowledge every command')
            return 1
        return 0 if check(ser) else 1


if __name__ == '__main__':
    sys.exit(main())
