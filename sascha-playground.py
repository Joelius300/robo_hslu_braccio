import time
import numpy
import serial

from braccio import Braccio

if __name__ == '__main__':

    # port = "COM7"  # windows
    port = serial.serial_for_url("COM7", timeout=.1)  # testing
    with Braccio(port) as braccio:
        time.sleep(5)
        # 120 b 75 s 20 e 20 v 90 w 0 g
        braccio.send(base=0, shoulder=0, elbow=0, wrist_tilt=5, wrist_rotate=0)

        print(braccio.current_angles)
        print(braccio.get_end_point())

        time.sleep(30)
        braccio.reset_position()


