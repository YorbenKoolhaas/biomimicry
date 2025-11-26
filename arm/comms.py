from serial import Serial
from arm.inv_kinemetics import calculate_angles

SER = Serial('/dev/ttyUSB1', 9600, timeout=1)
SER.reset_input_buffer()

def move_arm(x: float | int, y: float | int, z: float | int) -> bool:
    """Takes in angles and sends them to the arm via serial communication.
    
    ser: Serial object representing the serial connection to the arm.
    angles: A tuple of four integers representing the angles for the arm's joints.

    Returns True if the command was sent successfully."""
    
    angles = calculate_angles(x, y, z)
    if not angles["valid"]:
        return False
    
    command = f"MOVE {angles["theta1"]:.0f} {angles["theta2"]:.0f} {angles["theta3"]:.0f} {angles["delta"]:.0f}\n"
    SER.write(command.encode('utf-8'))

    while True:
        if SER.in_waiting > 0:
            line = SER.readline().decode('utf-8').rstrip()
            print(line)
            # break
    return True