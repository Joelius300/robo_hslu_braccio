import time

import serial

from braccio import Braccio

if __name__ == '__main__':
    port = "/dev/ttyACM2"
    # port = serial.serial_for_url("loop://", timeout=.1)  # testing
    with Braccio(port) as b:
        time.sleep(2)

        # sent = b.send(base=90, shoulder=90, elbow=90, wrist_tilt=90, wrist_rotate=90, grip=40)  # default position
        # sent = b.send(base=120, shoulder=120, elbow=120, wrist_tilt=120, wrist_rotate=120, grip=40)
        sent = b.send(base=60, shoulder=60, elbow=60, wrist_tilt=60, wrist_rotate=60, grip=40)
        print("Sent no. bytes:", sent)
