from flask import Flask, render_template, request, jsonify
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import arm.comms as comms

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
    
    succes = comms.move_arm(x, y, z)
    if not succes:
        return jsonify({"status": "error", "message": "Invalid coordinates"}), 400
    else:
        return jsonify({"status": "ok"})

if __name__ == "__main__":
    app.run(port=5000)



