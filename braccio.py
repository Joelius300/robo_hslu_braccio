from io import RawIOBase
from typing import Optional

import serial
from serial import Serial


def _validate_angle(value: Optional[int]):
    if value is None:
        return

    if type(value) is not int or value < 0 or value >= 360:
        raise ValueError(f"Invalid angle: {value}")


def _validate_grip(value: Optional[int]):
    if value is None:
        return

    if type(value) is not int or value < 10 or value > 73:
        raise ValueError(f"Invalid grip strength: {value}")


class Braccio:
    key_mappings = {
        'base': 'b',
        'shoulder': 's',
        'elbow': 'e',
        'wrist_tilt': 'v',
        'wrist_rotate': 'w',
        'grip': 'g'
    }

    """
    Class to communicate with an Arduino controlled Braccio using the Postfix protocol by J.W. (see arduino folder).
    """

    def __init__(self, port: str | Serial):
        if isinstance(port, str):
            self._port = serial.Serial(port, baudrate=9600, parity='N', bytesize=8, stopbits=1, timeout=None)
        else:
            self._port = port

    def send(self, *,
             base: Optional[int] = None,
             shoulder: Optional[int] = None,
             elbow: Optional[int] = None,
             wrist_tilt: Optional[int] = None,
             wrist_rotate: Optional[int] = None,
             grip: Optional[int] = None) -> None:
        """
        Sets the specified angles of the Braccio joints over the Arduino.
        """

        _validate_angle(base)
        _validate_angle(shoulder)
        _validate_angle(elbow)
        _validate_angle(wrist_tilt)
        _validate_angle(wrist_rotate)
        _validate_grip(grip)

        self._send(base=base, shoulder=shoulder,
                   elbow=elbow, wrist_tilt=wrist_tilt,
                   wrist_rotate=wrist_rotate, grip=grip)

    def _send(self, **kwargs) -> int:
        payload = self._build_payload(**kwargs)
        print("Sending:", payload)
        payload += '\r\n'

        return self._port.write(payload.encode('ASCII'))

    def _build_payload(self, **kwargs):
        payload = ''
        for key, value in kwargs.items():
            if value is not None:
                payload += f'{value} '
                key = self.key_mappings[key]
                payload += f'{key} '

        return payload.rstrip()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._port.close()


if __name__ == '__main__':
    port = serial.serial_for_url("loop://", timeout=.1)  # testing
    # port = "COM4"  # windows
    # port = "/dev/ttyACM0"  # linux
    with Braccio(port) as braccio:
        braccio.send(base=1, shoulder=4, wrist_tilt=6, grip=10)
        braccio.send(base=100, shoulder=100, elbow=359, wrist_tilt=0, wrist_rotate=50)

        if isinstance(port, RawIOBase):
            print("Dumping received data on port:")
            print(port.readall().decode('ASCII'))
