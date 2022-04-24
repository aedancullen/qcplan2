import subprocess
import signal
import time

import maestrocar

slamproc = subprocess.Popen("../ORB_SLAM3/Examples/Monocular/mono_camera")
import slampipe
gamepadproc = subprocess.Popen("nc -l 9867 > gamepadpipe", shell=True)
import gamepadpipe

print(slampipe.get_data())
print(gamepadpipe.get_data())
time.sleep(30)
print(slampipe.get_data())
print(gamepadpipe.get_data())

slamproc.send_signal(signal.SIGINT)
slamproc.wait()
gamepadproc.send_signal(signal.SIGTERM)
gamepadproc.wait()
