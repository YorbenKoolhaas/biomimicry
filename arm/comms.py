from serial import Serial

HOME_POSITION = (90, 90, 90, 0)

def move_arm(ser: Serial, angles: tuple[int, int, int, int]) -> bool:
    """Takes in angles and sends them to the arm via serial communication.
    
    ser: Serial object representing the serial connection to the arm.
    angles: A tuple of four integers representing the angles for the arm's joints.

    Returns True if the command was sent successfully."""
    
    command = f"{angles[0]},{angles[1]},{angles[2]},{angles[3]}\n"
    ser.write(command.encode('utf-8'))
    return True

if __name__ == "__main__":
    ser = Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.reset_input_buffer()

    while True:
        