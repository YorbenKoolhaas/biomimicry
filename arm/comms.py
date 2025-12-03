from serial import Serial
from arm.inv_kinemetics import calculate_angles
import os
from flask_socketio import emit


class Comms:
    def __init__(self):
        if os.path.exists("/dev/ttyUSB0"):
            usb_path = "/dev/ttyUSB0"
        elif os.path.exists("/dev/ttyUSB1"):
            usb_path = "/dev/ttyUSB1"
        else:
            raise FileNotFoundError("Could not find the USB device for the robotic arm.")
        
        self.SER = Serial(usb_path, 9600, timeout=1)
        self.SER.reset_input_buffer()
        
    def move_arm(self, x: float | int, y: float | int, z: float | int) -> bool:
        """Takes in coordinates and sends them to the arm via serial communication.

        Returns True if the command was sent successfully."""
        
        angles = calculate_angles(x, y, z)
        if not angles["valid"]:
            return False
        
        command = f"MOVE {angles["theta1"]:.0f} {angles["theta2"]:.0f} {angles["theta3"]:.0f} {angles["delta"]:.0f}\n"
        self.SER.write(command.encode('utf-8'))

        return True
    
    def receive_data(self) -> None:
        """Receives data from the arm via serial communication.
        
        Please run this method in a separate thread to avoid blocking the main program."""

        while True:
            if self.SER.in_waiting > 0:
                line = self.SER.readline().decode('utf-8').rstrip()
                emit("received_data", {"data": line})

    