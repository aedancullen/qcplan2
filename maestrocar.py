import maestro
import threading
import time

INACTIVITY_TIMEOUT = 1
ACCELERATOR_CHANNEL = 5
STEERING_CHANNEL = 4

controller = maestro.Controller()
last_update = 0

def scale_qus(value):
    value = max(-1, value)
    value = min(1, value)
    return int((500*value + 1500) * 4)

def set_control(accelerator, steering):
    global last_update

    controller.setTarget(ACCELERATOR_CHANNEL, scale_qus(accelerator))
    controller.setTarget(STEERING_CHANNEL, scale_qus(steering))
    last_update = time.time()

def inactivity_thread():
    global last_update

    while True:
        if last_update != -1 and time.time() - last_update > INACTIVITY_TIMEOUT:
            print("maestrocar: inactive")
            set_control(0, 0)
            last_update = -1
        time.sleep(0.1)

inactivity_daemon = threading.Thread(target=inactivity_thread)
inactivity_daemon.setDaemon(True)
inactivity_daemon.start()
