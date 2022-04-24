import threading

pipe = open("slampipe", 'r')
data = {
    "translation": [0, 0, 0],
    "quaternion": [0, 0, 0, 0],
    "tracking_state": 0,
}

def get_data():
    return data

def pipe_thread():
    global data

    while True:
        line = pipe.readline()
        if line == "":
            break
        if '*' in line:
            trans = pipe.readline().split(' ')
            data["translation"] = [int(x) for x in trans]
            quat = pipe.readline().split(' ')
            data["quaternion"] = [int(x) for x in quat]
            state = pipe.readline()
            data["tracking_state"] = int(state)
            timestamp = pipe.readline()
    data = None

pipe_daemon = threading.Thread(target=pipe_thread)
pipe_daemon.setDaemon(True)
pipe_daemon.start()
