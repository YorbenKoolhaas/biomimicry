from flask import Flask, render_template, request, jsonify
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import arm.comms as comms

current_pos = {"x": 0, "y": 0, "z": 0}

app = Flask(__name__)

@app.route('/')
def home():
    return render_template('index.html')

@app.route('/send_coords', methods=['POST'])
def send_coords():
    data = request.get_json()
    x = float(data['x'])
    y = float(data['y'])
    z = float(data['z'])

    current_pos["x"] = x
    current_pos["y"] = y
    current_pos["z"] = z
    
    succes = comms.move_arm(x, y, z)
    if not succes:
        return jsonify({"status": "error", "message": "Invalid coordinates"}), 400
    else:
        return jsonify({"status": "ok"})

@app.route('/increase_coords', methods=['POST'])
def increase_coords():
    global current_pos

    data = request.get_json()
    axis = f"{data['axis']}".lower()
    delta = float(data['amount'])

    current_pos[axis] += delta

    succes = comms.move_arm(current_pos["x"], current_pos["y"], current_pos["z"])
    if not succes:
        return jsonify({"status": "error", "message": "Invalid coordinates"}), 400
    else:
        return jsonify({"status": "ok"})

if __name__ == "__main__":
    app.run(port=5000)



