# Smart Door Closure with Adjustable Actuator

### Description
Automated Smart Door Closure System with Adjustable Linear Actuator Control is a practical solution for automating the closing of doors or trap-doors using a linear actuator while preventing damage or over-closure. This project addresses the common problem where a linear actuator has a longer range of motion than required, causing doors or trap-doors to close too much and potentially collide with the frame.

In this project, we've employed an Arduino Nano to orchestrate the control of a linear actuator and integrated several hardware components for precise operation. Notably, we've bypassed an external RF receiver, which was originally connected directly to the linear actuator, in order to gain finer control over the actuator's movements and achieve a carefully managed door closure process.


### Features
- Limit Switch Integration: This project incorporates a limit switch to detect when the door or trap-door is fully closed. The limit switch acts as a safeguard, ensuring that the closing action stops once the door reaches its closed position.
- Accelerometer Position Detection: An accelerometer is employed to accurately determine the position of the trap door when fully opened. This measurement is crucial for coordinating the actuator's movements and achieving precise door control.
- Customizable Code: The codebase included in this project can be easily customized to suit various door or trap-door configurations. You can adapt it to different linear actuator sizes and door types.
- Reliable Automation: With the combination of the limit switch and accelerometer, you can achieve reliable and automated door or trap-door closing, eliminating the risk of damage to the frame.

### Hardware Components
This project utilizes a range of hardware components to achieve precise and automated door closure. Here's an overview of the key components employed:

- Adafruit_ADXL345 Accelerometer: The Adafruit_ADXL345 accelerometer is used to detect and monitor the position of the trap door when it's fully opened. This sensor provides crucial data for controlling the linear actuator and ensuring accurate door closure.
- Relay Module: A relay module is integrated into the system to control the linear actuator's operation. The relay acts as a switch, allowing you to activate and deactivate the actuator as needed.
- Simple End-Switch: A basic end-switch is included to detect when the door or trap-door reaches its fully closed position. The end-switch serves as a safety feature, ensuring that the closure process halts when the door is closed to prevent any damage or over-closure.
- RF Receiver Bypass: We've bypassed the original RF receiver that was directly connected to the linear actuator. This modification enables more precise control over the actuator, allowing us to carefully stop its movements during the door closure process.

These hardware components, in combination with the Arduino Nano and the software code, create a reliable and efficient smart door closure system that enhances control and safety while maintaining ease of operation.
