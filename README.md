# Biomimicry: Snip 'n' Grip
This git repository is used for the Snip 'n' Grip project for the minor Biomimicry. In this repository you can find all the control code for the arm and the code for the AI + stereovision.

---

<h2 style="color:red"><b>***IMPORTANT***</b></h2>
Make sure the raspberry pi has identified the two arduinos. To check this, make sure the pi is powered on and the arduinos are connected. The LED of the arduinos should be on. Follow the next steps to ensure the 2 arduinos are identified

### 1. Connect to the raspberry pi
Open a terminal on your laptop and make sure your laptop is connected to 'PiNetwork'.
Then execute the following command:
`ssh group7@<ip-of-raspi>`
The password is "googlyeyes"

### 2. Verify that the arduinos are recognised
To verify that the raspberry pi recognizes the arduinos, execute the following command:
`ls /dev/`
Check if `/dev/arduino_arm` and `/dev/arduino_end` are in the output of the command. If so skip to the user instructions.

### 3. Adding a symlink to the arduino
If one or both of them are not in there, check for `/dev/ttyACM0` and `/dev/ttyACM1`
Now we need the unique identifier. To get this use the following command with either ACM0 or ACM1:
`udevadm info -a -n /dev/ttyACM0 | grep serial`
The output should have something like this:
`ATTRS{serial}=="85430333238351A0B1C1"`
Now create a rule file:
`sudo nano /etc/udev/rules.d/99-arduino.rules`
And add the following rule with the previous output depending on if the arduino_arm or arduino_end are missing:
`SUBSYSTEM=="tty", ATTRS{serial}=="85430333238351A0B1C1", SYMLINK+="arduino_arm"`

Now that symlinks are created you can proceed to the user instructions

---

## User Instructions
To use the robotic arm with the webinterface, simply run `server/main.py`. This will run on [`https://127.0.0.1:5000`](https://127.0.0.1:5000)

![Picture of the user interface](/server/pictures/ui.png "User Interface")

There are several fields here. The top field "Set Coordinates" is where you can put in coordinates for the arm to move to.
In the section "Increment Movement" you can click on the buttons to increase or decrease the current coordinates by 10mm.

Lastly there is the "Move Scissors" field. Here you can specify how far you want to extend the scissors. <p style="color:red"><b>When the scissors reach the desired position, they close so watch out</b></p>