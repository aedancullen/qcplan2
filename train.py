import subprocess
import signal
import time

import maestrocar

slamproc = subprocess.Popen("../ORB_SLAM3/Examples/Monocular/mono_camera", stdin=subprocess.PIPE)
import slampipe
gamepadproc = subprocess.Popen("nc -l 9867 > gamepadpipe", shell=True, stdin=subprocess.PIPE)
import gamepadpipe

try:
    while True:
        gpdata = gamepadpipe.get_data()
        if gpdata is None:
            break
        accelerator = 0#-gpdata["left_stick_y"]
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
