import numpy as np
import matplotlib.pyplot as plt

def calculate_angles(
    x: float | int, y: float | int, z: float | int,
    L1: int=180, L2: int=180, L3: int=70, psi: int=0,
    limits: dict[str, tuple[int | float]]= {"theta1": (34.6, 135), "theta2": (-163, 0)}
) -> dict[str, int | float]:
    """
    Elbow-up only IK for a 3DOF arm + base.
    x, y, z : target coordinates in mm
    L1, L2, L3 : lengths of arm segments in mm
    psi : desired end-effector orientation in degrees (0 = horizontal)
    
    limits example:
        limits = {
            "theta1": (-90, 90),
            "theta2": (-120, 120),
            "theta3": (-180, 180),
            "delta":  (-180, 180)
        }

    Returns a dictionary with joint angles in degrees and a validity flag.
    """

    # Base rotation
    delta = np.degrees(-np.arctan2(z, x))

    # Subtract tool length
    r = np.sqrt(x*x + z*z) - L3
    px = r

    # 2-link Inverse Kinematics
    D = (px*px + y*y - L1*L1 - L2*L2) / (2 * L1 * L2)
    D = np.clip(D, -1.0, 1.0)

    theta2 = -np.arccos(D) # elbow up

    theta1 = np.arctan2(y, px) - np.arctan2(
        L2 * np.sin(theta2),
        L1 + L2 * np.cos(theta2)
    )

    # Wrist orientation
    theta3 = np.radians(psi) - theta1 - theta2

    theta1_d = np.degrees(theta1)
    theta2_d = np.degrees(theta2)
    theta3_d = np.degrees(theta3)

    sol = {
        "theta1": theta1_d,
        "theta2": theta2_d,
        "theta3": theta3_d,
        "delta":  delta,
        "valid": True
    }

    # Joint limit enforcement
    # if limits:
    #     for j in ["theta1", "theta2", "theta3", "delta"]:
    #         if j in limits:
    #             lo, hi = limits[j]
    #             if not (lo <= sol[j] <= hi):
    #                 sol["valid"] = False

    sol["valid"] = True

    print(f"IK Solution: {sol}")
    return sol


def draw_robot(solution, L1=180, L2=180, L3=70):
    theta1 = np.radians(solution["theta1"])
    theta2 = np.radians(solution["theta2"])
    theta3 = np.radians(solution["theta3"])
    delta = np.radians(solution["delta"])

    # Rotation around vertical Y axis
    R = np.array([
        [ np.cos(delta), 0, np.sin(delta)],
        [ 0,             1, 0            ],
        [-np.sin(delta), 0, np.cos(delta)]
    ])

    # Arm plane in 2D (X-Y)
    p0 = np.array([0, 0, 0])
    p1 = np.array([L1*np.cos(theta1), L1*np.sin(theta1), 0])
    p2 = p1 + np.array([L2*np.cos(theta1+theta2), L2*np.sin(theta1+theta2), 0])
    p3 = p2 + np.array([L3*np.cos(theta1+theta2+theta3), L3*np.sin(theta1+theta2+theta3), 0])

    # Rotate into 3D
    p1 = R @ p1
    p2 = R @ p2
    p3 = R @ p3

    # Plot points
    xs = [p0[0], p1[0], p2[0], p3[0]]
    ys = [p0[1], p1[1], p2[1], p3[1]]
    zs = [p0[2], p1[2], p2[2], p3[2]]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(xs, ys, zs, marker="o", linewidth=3)

    ax.set_xlabel("X (forward)")
    ax.set_ylabel("Y (up)")
    ax.set_zlabel("Z (side)")

    ax.set_box_aspect([1,1,1])
    plt.title("Robot Configuration")
    plt.show()


if __name__ == "__main__":
    x = 80
    y = 150
    z = 0

    solution = calculate_angles(x, y, z, L1=180, L2=180, L3=70, psi=0)
    print(solution)
    draw_robot(solution)