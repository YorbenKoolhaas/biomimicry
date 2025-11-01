import numpy as np
import matplotlib.pyplot as plt

PI = np.pi
PSI = -90  # degrees

def calculate_angles(x, y, z, j1, j2, j3):
    if x == 0:
        x = 0.0001  # Prevent division by zero
    delta = np.degrees(np.atan(z/x)) # Angle base needs to move

    if np.sign(x) == -1:
        delta += 180

    r = np.sqrt(np.square(x) + np.square(z)) - j3
    z = r*np.sin(delta)
    x = r*np.cos(delta)

    x2 = np.sqrt(np.square(z) + np.square(x)) - x
    if z != 0:
        x += x2
    
    theta2 = -np.acos((np.square(x) + np.square(y) - np.square(j1) - np.square(j2)) / (2 * j1 * j2))
    theta1 = np.atan(y / x) + np.atan((j2 * np.sin(theta2)) / (j1 + j2 * np.cos(theta2)))

    if (theta2 < 0 and theta1 > 0):
        if theta1 < 90:
            theta1 += (180 - (theta1 * 2)) # Mirror across y
            theta2 *= -1
        elif theta1 > 90:
            theta1 = theta1 - (2 * (theta1 - 90)) # Mirror across y
            theta2 *= -1
    elif (theta1 < 0 and theta2 < 0):
        theta1 *= -1
        theta2 *= -1

    theta3 = PSI - np.degrees(theta2) - (90 - np.degrees(theta1))
    theta3 = 180 - theta3

    return np.degrees(theta1), np.degrees(theta2), theta3, delta

def calculate_vector(angleX, angleY, angleZ, vector):
    rotX = np.array([[1, 0, 0],
                     [0, np.cos(angleX), -np.sin(angleX)],
                     [0, np.sin(angleX), np.cos(angleX)]])
    rotY = np.array([[np.cos(angleY), 0, np.sin(angleY)],
                     [0, 1, 0],
                     [-np.sin(angleY), 0, np.cos(angleY)]])
    rotZ = np.array([[np.cos(angleZ), -np.sin(angleZ), 0],
                     [np.sin(angleZ), np.cos(angleZ), 0],
                     [0, 0, 1]])
    rotation_matrix = rotZ @ rotY @ rotX
    rotated_vector = rotation_matrix @ vector

    return rotated_vector


def plot_arm(theta1, theta2, theta3, delta, j1, j2, j3):
    theta1 = np.radians(theta1)
    theta2 = np.radians(-theta2)
    theta3 = np.radians(theta3)
    delta = np.radians(delta)

    base_vector = np.array([j1, 0, 0])
    base_vector = calculate_vector(0, delta, theta1, base_vector)
    x1, y1, z1 = base_vector

    arm_vector = base_vector * j2/np.linalg.norm(base_vector)
    arm_vector = calculate_vector(0, delta, theta2, arm_vector)
    x2, y2, z2 = arm_vector

    end_vector = arm_vector * j3/np.linalg.norm(arm_vector)
    end_vector = calculate_vector(0, delta, theta3, end_vector)
    x3, y3, z3 = end_vector

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.quiver(0, 0, 0, x1, z1, y1, color='r', arrow_length_ratio=0.1)
    ax.quiver(x1, z1, y1, x2, z2, y2, color='g', arrow_length_ratio=0.1)
    ax.quiver(x1 + x2, z1 + z2, y1 + y2, x3, z3, y3, color='b', arrow_length_ratio=0.2)

    ax.set_xlim([-400, 400])
    ax.set_ylim([-400, 400])
    ax.set_zlim([-400, 400])

    ax.invert_yaxis()

    ax.set_xlabel('X')
    ax.set_ylabel('z')
    ax.set_zlabel('y')

    plt.title('Robotic Arm Configuration')
    plt.show()

if __name__ == "__main__":
    j1 = 200  # length of first arm segment in mm
    j2 = 200  # length of second arm segment in mm
    j3 = 50   # length of end effector in mm

    x = -400
    y = 0
    z = 0

    theta1, theta2, theta3, delta = calculate_angles(x, y, z, j1, j2, j3)
    print(f"Theta1: {theta1:.2f} degrees")
    print(f"Theta2: {theta2:.2f} degrees")
    print(f"Theta3: {theta3:.2f} degrees")
    print(f"Delta: {delta:.2f} degrees")

    plot_arm(theta1, theta2, theta3, delta, j1, j2, j3)