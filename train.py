import subprocess
import signal
import time

import maestrocar
maestrocar.set_controls(0, 0)

slamproc = subprocess.Popen("../ORB_SLAM3/Examples/Monocular/mono_camera", stdin=subprocess.PIPE)
import slampipe
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
            maestrocar.set_controls(accelerator, steering)
        time.sleep(0.001)
except KeyboardInterrupt:
    pass

maestrocar.set_controls(0, 0)

slamproc.send_signal(signal.SIGINT)
slamproc.wait()
gamepadproc.send_signal(signal.SIGTERM)
gamepadproc.wait()
