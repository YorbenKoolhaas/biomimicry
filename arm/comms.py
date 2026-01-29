from serial import Serial
from arm.inv_kinemetics import calculate_angles
import os


class Comms:
    def __init__(self):
        try:
            self.SER_ARM = Serial("/dev/arduino_arm", 9600, timeout=1)
            self.SER_ARM.reset_input_buffer()

            # self.SER_END = Serial("/dev/arduino_gripnsnip", 9600, timeout=1)
            # self.SER_END.reset_input_buffer()
        except Exception as e:
            print(f"Error initializing serial communication: {e}")
            os._exit(1)
        
    def move_arm(self, x: float | int, y: float | int, z: float | int) -> bool:
        """Takes in coordinates and sends them to the arm via serial communication.

        Returns True if the command was sent successfully."""
        
        angles = calculate_angles(x, y, z)
        if not angles["valid"]:
            return False
        
        # Send command to arm
        # angles are multiplied by 100 to convert to integer representation for serial communication
        command = f"MOVE {int(angles["theta1"]*100)} {int(angles["theta2"]*100)} {int(angles["theta3"]*100)} {int(angles["delta"]*100)}\n"
        print(command)
        self.SER_ARM.write(command.encode('utf-8'))

        while True:
            if self.SER_ARM.in_waiting > 0:
                line = self.SER_ARM.readline().decode('utf-8').rstrip()
                print(f"Arm data: {line}")

        return True
    
    def move_scissors(self, amount: float | int) -> True:
        """Moves the scissors of the robotic arm by the specified amount.

        Positive values extend the scissors, negative values retract them.
        
        Closes the scissors when destination is reached."""

        command = f"SCISSORS {int(amount*100)}\n"
        # self.SER_END.write(command.encode('utf-8'))

        return True
    
    def receive_data(self) -> None:
        """Receives data from the arm via serial communication.
        
        Please run this method in a separate thread to avoid blocking the main program."""

        while True:
            if self.SER_ARM.in_waiting > 0:
                line = self.SER_ARM.readline().decode('utf-8').rstrip()
                print(f"Arm data: {line}")
                # emit("received_data", {"data": line})
            
            # if self.SER_END.in_waiting > 0:
            #     line = self.SER_END.readline().decode('utf-8').rstrip()
                # emit("received_data", {"data": line})

    