"""Listen for a bridge startup banner; opening serial can reset the Arduino."""
import argparse
import json
import time

import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--motor-power-off', action='store_true')
    parser.add_argument('--wheels-raised', action='store_true')
    args = parser.parse_args()
    if not (args.motor_power_off and args.wheels_raised):
        parser.error('Switch motor battery power off and raise wheels before using both confirmation flags.')
    # Do not transmit commands to unidentified firmware. Limit input size and
    # escape control characters before displaying unknown serial output.
    with serial.Serial(args.port, 115200, timeout=.2, exclusive=True) as device:
        deadline = time.monotonic() + 5
        data = bytearray()
        while time.monotonic() < deadline and len(data) < 4096:
            data.extend(device.read(min(512, 4096 - len(data))))
    output = data.decode('utf-8', errors='replace')
    print('Startup output:', json.dumps(output))
    lines = {line.strip() for line in output.splitlines()}
    if 'READY WN1' in lines:
        print('W.A.R.M bridge banner observed. This does not validate wiring or motion.')
    else:
        print('W.A.R.M bridge NOT identified. No output does not prove the board is faulty.')
    print('No serial commands were transmitted.')


if __name__ == '__main__':
    main()
