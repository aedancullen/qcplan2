import threading

pipe = open("gamepadpipe", 'r')
data = {
    "left_stick_x": 0,
    "left_stick_y": 0,
    "right_stick_x": 0,
    "right_stick_y": 0,
    "x": 0,
    "y": 0,
    "a": 0,
    "b": 0,
}

def scale_js(value):
    return value / 32768

def get_data():
    return data

def pipe_thread():
    global data

    while True:
        line = pipe.readline()
        if line == "":
            break
        if "value" in line:
            value = float(line.split("value")[-1])
            if "ABS_X" in line:
                data["left_stick_x"] = scale_js(value) if value != 128 else 0
            elif "ABS_Y" in line:
                data["left_stick_y"] = scale_js(value) if value != -129 else 0
            elif "ABS_RX" in line:
                data["right_stick_x"] = scale_js(value) if value != 128 else 0
            elif "ABS_RY" in line:
                data["right_stick_y"] = scale_js(value) if value != -129 else 0
            elif "BTN_NORTH" in line:
                data["x"] = value
            elif "BTN_WEST" in line:
                data["y"] = value
            elif "BTN_SOUTH" in line:
                data["a"] = value
            elif "BTN_EAST" in line:
                data["b"] = value
    data = None

pipe_daemon = threading.Thread(target=pipe_thread)
pipe_daemon.setDaemon(True)
pipe_daemon.start()
