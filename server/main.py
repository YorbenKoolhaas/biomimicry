from flask_socketio import SocketIO, emit
from flask import Flask, render_template
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import arm.comms as comms

current_pos = {"x": 0, "y": 0, "z": 0}

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

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
    
    succes = comms.move_arm(x, y, z)
    if not succes:
        emit('error_msg', {"status": "error", "message": "Invalid coordinates"})
    else:
        emit('coords_updated', {"status": "ok", "position": current_pos})

@socketio.on('increase_coords')
def handle_increase_coords(data):
    global current_pos

    axis = str(data['axis']).lower()
    delta = float(data['amount'])

    current_pos[axis] += delta

    succes = comms.move_arm(current_pos["x"], current_pos["y"], current_pos["z"])

    if not succes:
        emit('error_msg', {"status": "error", "message": "Invalid coordinates"})
    else:
        emit('coords_updated', {"status": "ok", "position": current_pos})

if __name__ == "__main__":
    comms = comms.Comms()
    socketio.run(app, port=5000) 



