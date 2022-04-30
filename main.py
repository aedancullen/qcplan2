import subprocess
import signal
import time

import maestrocar

gamepadproc = subprocess.Popen("nc -l 9867 > gamepadpipe", shell=True, stdin=subprocess.PIPE)
import gamepadpipe

try:
    while True:
        if gamepadpipe.get_updated():
            gpdata = gamepadpipe.get_data()
            if gpdata is None:
                break
            accelerator = min(-gpdata["left_stick_y"], 0.25)
            steering = gpdata["right_stick_x"]
            maestrocar.set_control(accelerator, steering)
        time.sleep(0.001)
except KeyboardInterrupt:
    pass

maestrocar.set_control(-1, 0)

gamepadproc.send_signal(signal.SIGTERM)
gamepadproc.wait()
