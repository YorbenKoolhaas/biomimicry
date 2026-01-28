from flask import Flask
from flask_socketio import SocketIO, emit
import threading
from flask import render_template
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import arm.comms as comms

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

current_pos = {"x": 0, "y": 0, "z": 0}

@app.route('/')
def home():
    return render_template('index.html')

@socketio.on('send_coords')
def handle_send_coords(data):
    global current_pos

    x = float(data['x'])
    y = float(data['y'])
    z = float(data['z'])

    current_pos["x"] = x
    current_pos["y"] = y
    current_pos["z"] = z

    print(f"Received coordinates: x={x}, y={y}, z={z}")
    
    succes = comms.move_arm(x, y, z)
    if succes:
        emit('coords_updated', {"status": "ok", "position": current_pos})
    else:
        emit('error_msg', {"status": "error", "message": "Invalid coordinates"})

@socketio.on('increase_coords')
def handle_increase_coords(data):
    global current_pos

    axis = str(data['axis']).lower()
    delta = float(data['amount'])

    current_pos[axis] += delta

    succes = comms.move_arm(current_pos["x"], current_pos["y"], current_pos["z"])

    if succes:
        emit('coords_updated', {"status": "ok", "position": current_pos})
    else:
        emit('error_msg', {"status": "error", "message": "Invalid coordinates"})

@socketio.on('received_data')
def handle_received_data(data):
    print(data['data'])
    if data == "arm in position":
        emit('arm_in_position', {"status": "ok"})

@socketio.on('arm_in_position')
def handle_arm_in_position(data):
    pass

@socketio.on('move_scissors')
def handle_move_scissors(data):
    amount = float(data['amount'])

    succes = comms.move_scissors(amount)

    if not succes:
        emit('error_msg', {"status": "error", "message": "Failed to move scissors"})
    else:
        emit('scissors_moved', {"status": "ok"})

@socketio.on('error_msg')
def handle_error(data):
    print(data['message'])


if __name__ == "__main__":
    comms = comms.Comms()
    threads = []
    threads.append(threading.Thread(target=socketio.run, args=(app, ), kwargs={"port": 5000, "debug": False}))
    threads.append(threading.Thread(target=comms.receive_data))

    for thread in threads:
        thread.start()


