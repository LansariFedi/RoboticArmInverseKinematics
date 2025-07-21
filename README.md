# Arduino Servo Controller

This project is designed to control servo motors using an ESP32 microcontroller. It calculates the joint angles based on user-provided height (Z) and length (L) inputs via serial communication. The output angles are limited to a range between 0 and 180 degrees.

## Project Structure

```
arduino-servo-controller
├── arduino-servo-controller.ino      # Main Arduino sketch
├── libraries
│   └── ESP32Servo
│       └── src
│           └── ESP32Servo.h          # ESP32Servo library header
└── README.md                         # Project documentation
```

## Setup Instructions

1. **Install the ESP32 Board**: Make sure you have the ESP32 board package installed in your Arduino IDE. You can do this through the Board Manager.

2. **Install the ESP32Servo Library**: Ensure that the ESP32Servo library is included in your project. You can find it in the `libraries` folder of this project.

3. **Connect the Servo Motor**: Connect your servo motor to the appropriate GPIO pins on the ESP32.

4. **Upload the Sketch**: Open the `arduino-servo-controller.ino` file in the Arduino IDE, select the correct board and port, and upload the sketch.

## Usage

1. Open the Serial Monitor in the Arduino IDE.
2. Set the baud rate to 115200.
3. Enter the Coordinates (X, Y, Z) values separated by a space (e.g 14 25 45).
4. The calculated joint angles will be displayed in the Serial Monitor, limited to the range of 0 to 180 degrees.

## Notes

- Ensure that the input values for height and length are within reasonable limits to avoid unreachable positions.
- The angles are calculated based on the lengths of the servo arms defined in the sketch. Adjust these values as necessary for your specific setup.
