import sys
## TODO: Confirm values with stakeholders

## Weight of 1 strawberry according to "https://strawberryplants.org/strawberry-serving/"
# small: 7 grams, 1 inch diameter
# medium: 12 grams, 1.25 inch diameter
# large: 18 grams, 1.375 inch diameter
# extra large: 27 grams, 1.525 inch diameter

# Taking the heaviest strawberry for worst case scenario
STRAWBERRY_WEIGHT_GRAMS = 27

## Breadth of walking paths between the plant beds according to different forums and chatgpt "https://chatgpt.com/share/68fcdd5e-d79c-8012-a4a2-6e4dd4052f4e"
# Hand picking: 50-70 cm
# Using small carts: 70-90 cm
# Strawberry specific racks: 60-80 cm

# Taking half the expected breadth for strawberry specific racks as the length of the arm and taking height of the racks into account
ARM_LENGTH_CM = 50

## Taking available servo in the makerspace as the servo for the arm. This is called the small servo.
# Servo: "https://www.kiwi-electronics.com/nl/sg90-servo-360-degree-continuous-rotation-1-3kg-cm-9g-10866?search=servo"
SERVO_WEIGHT_GRAMS = 9
SERVO_TORQUE_KGCM = 1.3

# Previously mentioned servo probably not strong enough. This is called the stronger servo.
# Taking a stronger servo as alternative: "https://www.kiwi-electronics.com/nl/mg996r-360-digital-metal-gear-servo-11kg-cm-55g-3707?search=servo"
SERVO2_WEIGHT_GRAMS = 55
SERVO2_TORQUE_KGCM = 11

# Assuming the wrist servo is 5 cm before the end of the arm
WRIST_SERVO_DISTANCE_CM = 5

## Taking basic camera which is expected suffice for the task
# Camera: "https://www.kiwi-electronics.com/nl/raspberry-pi-camera-3-11239?search=camera"
CAMERA_WEIGHT_GRAMS = 4

## Estimating the weight of the plastic parts of the arm
# Taking PLA as the material for 3D printing with a density of 1.24 g/cm3
# Estimating the volume of plastic used to be around 88 cm3 with an infill of 25% (original volume ~353,5 cm3). Estimations made by chatgpt: "https://chatgpt.com/share/68fcdd5e-d79c-8012-a4a2-6e4dd4052f4e"
# Taking a bit of margin this result in about 110 grams of PLA used
PLASTIC_WEIGHT_GRAMS = 110

## Estimating weight of scissors
# Taking small scissors with an estimated weight of about 50 grams. Source: "https://www.amazon.nl/-/en/Westcott-Childrens-Scissor-Assorted-Colour/dp/B077S6WLGZ?th=1"
SCISSORS_WEIGHT_GRAMS = 50

## Estimating weight of soft gripper
# Rough estimation of about 20 grams since silicone is light and the gripper is small
SOFT_GRIPPER_WEIGHT_GRAMS = 20

## Calculations

def calculate_torque(weights: list[tuple[float, float]]) -> float:
    "weights: list of tuples (weight in grams, distance in cm from base)"
    return sum((weight/1000) * distance for weight, distance in weights)

# Creating list of weights at position in cm from the rotation point
weights = [
    (SERVO_WEIGHT_GRAMS, ARM_LENGTH_CM / 2),  # Assuming servo is mounted at the middle of the arm
    (SERVO_WEIGHT_GRAMS, ARM_LENGTH_CM - WRIST_SERVO_DISTANCE_CM),
    (PLASTIC_WEIGHT_GRAMS, ARM_LENGTH_CM / 2),  # Assuming plastic parts are evenly distributed along the arm

    # parts at the end of the arm
    (STRAWBERRY_WEIGHT_GRAMS, ARM_LENGTH_CM),
    (CAMERA_WEIGHT_GRAMS, ARM_LENGTH_CM),
    (SCISSORS_WEIGHT_GRAMS, ARM_LENGTH_CM),
    (SOFT_GRIPPER_WEIGHT_GRAMS, ARM_LENGTH_CM),
]

# Calculating total torque in kg*cm
total_torque_kgcm = calculate_torque(weights)
print(f"Total required torque: {total_torque_kgcm:.2f} kg*cm")

# Checking if the selected servo can provide enough torque
if total_torque_kgcm <= SERVO_TORQUE_KGCM:
    print("The small servo can provide enough torque.", "\n")
    print(f"Torque required: {total_torque_kgcm:.2f} kg*cm \n Torque provided by small servo: {SERVO_TORQUE_KGCM} kg*cm")
    base_servo = "small"
else:
    print("The small servo cannot provide enough torque. Trying stronger servo at base...")
    if total_torque_kgcm <= SERVO2_TORQUE_KGCM:
        print("The stronger servo can provide enough torque.", "\n")
        print(f"Torque required: {total_torque_kgcm:.2f} kg*cm \n Torque provided by stronger servo: {SERVO2_TORQUE_KGCM} kg*cm")
        base_servo = "stronger"
    else:
        print("Even the stronger servo cannot provide enough torque. A different solution is needed.")
        sys.exit()

print("Testing torque of halfway servo...")
weights2 = [
    (SERVO_WEIGHT_GRAMS, (ARM_LENGTH_CM / 2) - WRIST_SERVO_DISTANCE_CM),
    (PLASTIC_WEIGHT_GRAMS / 2, ARM_LENGTH_CM / 2),  # Assuming plastic parts are evenly distributed along the arm

    # parts at the end of the arm
    (STRAWBERRY_WEIGHT_GRAMS, ARM_LENGTH_CM / 2),
    (CAMERA_WEIGHT_GRAMS, ARM_LENGTH_CM / 2),
    (SCISSORS_WEIGHT_GRAMS, ARM_LENGTH_CM / 2),
    (SOFT_GRIPPER_WEIGHT_GRAMS, ARM_LENGTH_CM / 2),
]

# Calculating total torque in kg*cm for halfway servo
halfway_torque_kgcm = calculate_torque(weights2)

# Checking if the halfway servo can provide enough torque
if halfway_torque_kgcm <= SERVO_TORQUE_KGCM:
    print("The small servo at halfway can provide enough torque.", "\n")
    print(f"Torque at halfway servo: {halfway_torque_kgcm:.2f} kg*cm \n Torque provided by small servo: {SERVO_TORQUE_KGCM} kg*cm")
    halfway_servo = "small"
else:
    print("The small servo at halfway cannot provide enough torque. Trying stronger servo at halfway...")
    if halfway_torque_kgcm <= SERVO2_TORQUE_KGCM:
        print("The stronger servo at halfway can provide enough torque.", "\n")

        print("Testing new configuration for base servo...")
        weights[0] = (SERVO2_WEIGHT_GRAMS, ARM_LENGTH_CM / 2)  # Updating halfway servo to stronger one
        total_torque_kgcm = calculate_torque(weights)
        if total_torque_kgcm <= SERVO2_TORQUE_KGCM:
            print("With the stronger servo at the base and halfway, the configuration can provide enough torque.", "\n")
            print(f"Total required torque: {total_torque_kgcm:.2f} kg*cm \n Torque provided by stronger servo: {SERVO2_TORQUE_KGCM} kg*cm")
            halfway_servo = "stronger"
            base_servo = "stronger"
        else:
            print("Even with the stronger servo at the base, the configuration cannot provide enough torque. A different solution is needed.")
            sys.exit()
    else:
        print("Even the stronger servo at halfway cannot provide enough torque. A different solution is needed.")
        sys.exit()

print("Testing torque of wrist servo...")
weights3 = [
    (PLASTIC_WEIGHT_GRAMS * (WRIST_SERVO_DISTANCE_CM/ARM_LENGTH_CM), ARM_LENGTH_CM / 2),  # Assuming plastic parts are evenly distributed along the arm

    # parts at the end of the arm
    (STRAWBERRY_WEIGHT_GRAMS, WRIST_SERVO_DISTANCE_CM),
    (CAMERA_WEIGHT_GRAMS, WRIST_SERVO_DISTANCE_CM),
    (SCISSORS_WEIGHT_GRAMS, WRIST_SERVO_DISTANCE_CM),
    (SOFT_GRIPPER_WEIGHT_GRAMS, WRIST_SERVO_DISTANCE_CM),
]

# Calculating total torque in kg*cm for wrist servo
wrist_torque_kgcm = calculate_torque(weights3)

# Checking if the wrist servo can provide enough torque
if wrist_torque_kgcm <= SERVO_TORQUE_KGCM:
    print("The small servo at the wrist can provide enough torque.", "\n")
    print(f"Torque at wrist servo: {wrist_torque_kgcm:.2f} kg*cm \n Torque provided by small servo: {SERVO_TORQUE_KGCM} kg*cm")
    wrist_servo = "small"
else:
    print("The small servo at the wrist cannot provide enough torque. Trying stronger servo at wrist...")
    if wrist_torque_kgcm <= SERVO2_TORQUE_KGCM:
        print("The stronger servo at the wrist can provide enough torque.", "\n")

        print("Testing new configuration for base and halfway servos...")
        weights[0] = (SERVO2_WEIGHT_GRAMS, ARM_LENGTH_CM / 2)  # Updating base servo to stronger one
        weights[1] = (SERVO2_WEIGHT_GRAMS, ARM_LENGTH_CM - WRIST_SERVO_DISTANCE_CM)  # Updating halfway servo to stronger one, because small can't handle the stronger servo at wrist
        total_torque_kgcm = calculate_torque(weights)
        if total_torque_kgcm <= SERVO2_TORQUE_KGCM:
            print("With the stronger servos at the base, halfway, and wrist, the configuration can provide enough torque.", "\n")
            print(f"Total required torque: {total_torque_kgcm:.2f} kg*cm \n Torque provided by stronger servo: {SERVO2_TORQUE_KGCM} kg*cm", "\n")
            wrist_servo = "stronger"

        else:
            print("Even with the stronger servos at the base and halfway, the configuration cannot provide enough torque. A different solution is needed.")
            sys.exit()
    else:
        print("Even the stronger servo at the wrist cannot provide enough torque. A different solution is needed.")
        sys.exit()

print(f"Final configuration \n Base servo: {base_servo} \n Halfway servo: {halfway_servo} \n Wrist servo: {wrist_servo}")