from serial import Serial
from arm.inv_kinemetics import calculate_angles

def move_arm(x: float | int, y: float | int, z: float | int) -> bool:
    """Takes in angles and sends them to the arm via serial communication.
    
    ser: Serial object representing the serial connection to the arm.
    angles: A tuple of four integers representing the angles for the arm's joints.

    Returns True if the command was sent successfully."""
    
    angles = calculate_angles(x, y, z)

    command = f"{angles[0]},{angles[1]},{angles[2]},{angles[3]}\n"
    SER.write(command.encode('utf-8'))
    return True

if __name__ == "__main__":
    HOME_POSITION = (90, 90, 90, 0)

    global SER
    SER = Serial('/dev/ttyUSB0', 9600, timeout=1)
    SER.reset_input_buffer()

    move_arm()

    while True:
        pass