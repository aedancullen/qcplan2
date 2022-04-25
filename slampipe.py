import threading

pipe = open("slampipe", 'r')
data = {
    "translation": [0, 0, 0],
    "quaternion": [0, 0, 0, 0],
    "tracking_state": 0,
}
updated = False

def get_updated():
    global data
    global updated

    return updated

def get_data():
    global data
    global updated

    updated = False
    return data

def pipe_thread():
    global data
    global updated

    while True:
        line = pipe.readline()
        if line == "":
            break
        if '*' in line:
            trans = pipe.readline().split(' ')
            data["translation"] = [float(x) for x in trans]
            quat = pipe.readline().split(' ')
            data["quaternion"] = [float(x) for x in quat]
            state = pipe.readline()
            data["tracking_state"] = int(state)
            timestamp = pipe.readline()
            updated = True
    data = None
    updated = True

pipe_daemon = threading.Thread(target=pipe_thread)
pipe_daemon.setDaemon(True)
pipe_daemon.start()
