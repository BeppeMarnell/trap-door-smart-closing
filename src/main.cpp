#include <Arduino.h>
#include <RCSwitch.h>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

/* Enum for the RF signals */
enum RFSignal
{
    RF_UP = 1,
    RF_MIDDLE = 2,
    RF_DOWN = 3,
    RF_NONE = 0
};

/* Enum for the trap door state */
enum TrapDoorState
{
    TRAP_DOOR_CLOSED = 0,
    TRAP_DOOR_MOVING = 1,
    TRAP_DOOR_FULLY_OPENED = 2,
    TRAP_DOOR_LIFT_OFF = 3,
    TRAP_DOOR_UNKNOWN = 4
};

/* Create a new instance of the RCSwitch class for receiving messages from the remote */
RCSwitch ReceiverModule = RCSwitch();
/* Assign a unique ID to this sensor */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
boolean isAccelThere = false;
/* Create a bool that deactivates the state machine and the actuator 
   To reset the freeFalling you need to restart the program */
boolean freeFalling = false;
int freeFallingCounter = 0;

/* Constants */
const int ACTUATOR_RELAY_PIN = 4;
const int END_SWITCH_PIN = 5;
const int END_SWITCH_OPEN_VALUE = 1;
const int END_SWITCH_CLOSED_VALUE = 0;
const int SERIAL_BAUD_RATE = 9600;

/* Trap Door state */
TrapDoorState trapDoorCurrentState = TRAP_DOOR_UNKNOWN;
/* Trap Door angles */
const int TRAP_DOOR_CLOSED_ANGLE = 270;
const int TRAP_DOOR_OPENED_ANGLE = 180;

/* Receiver values */
const unsigned long RECEIVER_UP_VALUE = 2678888;
const unsigned long RECEIVER_MIDDLE_VALUE = 2678884;
const unsigned long RECEIVER_DOWN_VALUE = 2678882;
const int RECEIVER_PROTOCOL = 1;
const int RECEIVER_BIT_LENGTH = 24;

/* Get angle for the y axis of the accelerometer */
double getAngleAroundYAxis(double x, double y, double z);
/* RF function method */
RFSignal receiveRFSignals();
/* Check if the door is fully opened */
boolean isDoorFullyOpened(double angle);
/* Trap door state machine */
void trapDoorStateMachine(double tilt_angle, int received_signal);

/* Setup for the arudino */
void setup()
{
    /* Setup the serial connection with 9600 baud rate */
    Serial.begin(SERIAL_BAUD_RATE);

    /* This needs to be digital pin 2 on the arduino */
    ReceiverModule.enableReceive(0); // Receiver on interrupt 0 => that is pin #2

    /* Make the actuator relay pin as output */
    pinMode(ACTUATOR_RELAY_PIN, OUTPUT);

    /* Turn off the relay */
    digitalWrite(ACTUATOR_RELAY_PIN, LOW);

    /* Make the end switch pin as input */
    pinMode(END_SWITCH_PIN, INPUT_PULLUP);

    /* Initialise the sensor */
    if (!accel.begin())
    {
        /* There was a problem detecting the ADXL345 ... check your connections */
        Serial.println("No ADXL345 detected ... Check your wiring!");
        while (1)
            ;
    }else{
        isAccelThere = true;
    }

    /* Set the range to whatever is appropriate for your project */
    accel.setRange(ADXL345_RANGE_2_G);

    /* After the setup and before the main loop,
    we need to set the state of the trap door */
    if (digitalRead(END_SWITCH_PIN) == END_SWITCH_CLOSED_VALUE)
    {
        trapDoorCurrentState = TRAP_DOOR_CLOSED;
    }
    else if (digitalRead(END_SWITCH_PIN) == END_SWITCH_OPEN_VALUE)
    {
        trapDoorCurrentState = TRAP_DOOR_MOVING;
        digitalWrite(ACTUATOR_RELAY_PIN, HIGH);
    }

    /* Get a new sensor event */
    if(isAccelThere){
        sensors_event_t event;
        accel.getEvent(&event);

        /* Get the angle */
        float tilt_angle = getAngleAroundYAxis(event.acceleration.x, event.acceleration.y, event.acceleration.z);

        /* Check if the trap door is fully opened at the start */
        if (isDoorFullyOpened(tilt_angle))
        {
            trapDoorCurrentState = TRAP_DOOR_FULLY_OPENED;
        }
    }
}

/* Loop for the arduino */
void loop()
{
    /* Receive the rf signals */
    int received_signal = receiveRFSignals();

    /* If we receive a middle signal, we increase the free falling counter */
    if (received_signal == RF_MIDDLE)
    {
        freeFallingCounter++;
        if (freeFallingCounter >= 30)
        {
            freeFalling = true;
        }
    }

    /* Get a new sensor event */
    sensors_event_t event;
    accel.getEvent(&event);

    /* Get the angle */
    float tilt_angle = getAngleAroundYAxis(event.acceleration.x, event.acceleration.y, event.acceleration.z);

    if (freeFalling){
        /* Activate the actuator, always on */
        digitalWrite(ACTUATOR_RELAY_PIN, HIGH);
    }else{
        /* Run the trap door state machine if not on free falling*/
        trapDoorStateMachine(tilt_angle, received_signal);
    }

    /* Reset the receiver. Always keep this here */
    ReceiverModule.resetAvailable();

    /* Wait for 100ms */
    delay(200);
}

/**
 * @brief Receive the RF signals from the remote.
 *
 * This function checks if the receiver module is available and if a signal has been received. If a signal has been received, it checks the value, bit length, and protocol of the signal and returns the corresponding RFSignal enum value. If no signal has been received, it returns RF_NONE.
 *
 * @return The RFSignal enum value corresponding to the received signal, or RF_NONE if no signal has been received.
 */
RFSignal receiveRFSignals()
{
    /* Return value */
    RFSignal returnValue = RF_NONE;

    if (ReceiverModule.available())
    {
        /* Detect the received value */
        if (ReceiverModule.getReceivedValue() == RECEIVER_UP_VALUE && ReceiverModule.getReceivedBitlength() == RECEIVER_BIT_LENGTH && ReceiverModule.getReceivedProtocol() == RECEIVER_PROTOCOL)
        {
            returnValue = RF_UP;
        }
        else if (ReceiverModule.getReceivedValue() == RECEIVER_MIDDLE_VALUE && ReceiverModule.getReceivedBitlength() == RECEIVER_BIT_LENGTH && ReceiverModule.getReceivedProtocol() == RECEIVER_PROTOCOL)
        {
            returnValue = RF_MIDDLE;
        }
        else if (ReceiverModule.getReceivedValue() == RECEIVER_DOWN_VALUE && ReceiverModule.getReceivedBitlength() == RECEIVER_BIT_LENGTH && ReceiverModule.getReceivedProtocol() == RECEIVER_PROTOCOL)
        {
            returnValue = RF_DOWN;
        }
    }

    return returnValue;
}

/**
 * Calculates the angle around the y-axis in radians.
 *
 * @param x The x-coordinate of the point.
 * @param y The y-coordinate of the point (not used in the calculation).
 * @param z The z-coordinate of the point.
 *
 * @return The angle around the y-axis in degrees, in the range of 0 to 360.
 */
double getAngleAroundYAxis(double x, double y, double z)
{
    double projection = x * cos(M_PI / 2) + z * sin(M_PI / 2);
    double angle = atan2(projection, x) * 180 / M_PI;
    if (angle < 0)
    {
        angle += 360;
    }
    return angle;
}

/**
 * @brief Checks if the trap door is fully opened.
 *
 * This function checks if the trap door is fully opened by checking if the angle is greater than or equal to the TRAP_DOOR_OPENED_ANGLE.
 *
 * @param angle The angle of the trap door.
 *
 * @return True if the trap door is fully opened, false otherwise.
 */
boolean isDoorFullyOpened(double angle)
{
    /* If the accelerometer is not there, always return false */
    if (!isAccelThere){
        return false;
    }

    /* Constant for the offset */
    double const offset = 1;

    /* Check for the angle and print error */
    if (angle < 160 || angle > 290){
        Serial.println('SENSOR READING ERROR');
        return false;
    }

    if ((angle - offset) <= TRAP_DOOR_OPENED_ANGLE)
    {
        return true;
    }
    return false;
}

void trapDoorStateMachine(double tilt_angle, int received_signal)
{
    /// WHEN THE DOOR IS CLOSED ---------------------------------------------

    /* If the end switch is hit, stop the actuator, only if the state is moving */
    if (digitalRead(END_SWITCH_PIN) == END_SWITCH_CLOSED_VALUE && trapDoorCurrentState == TRAP_DOOR_MOVING)
    {
        /* If the end switch is hit, stop the actuator */
        digitalWrite(ACTUATOR_RELAY_PIN, LOW);
        /* Set the trap door state to closed */
        trapDoorCurrentState = TRAP_DOOR_CLOSED;
    }

    /* If the end switch is hit and we receive an up signal, turn on the actuator*/
    if (digitalRead(END_SWITCH_PIN) == END_SWITCH_CLOSED_VALUE && received_signal == RF_UP)
    {
        /* If the door is closed, the end switch is on, and the up signal is received, turn on the actuator */
        digitalWrite(ACTUATOR_RELAY_PIN, HIGH);
        /* Set the trap door state to moving */
        trapDoorCurrentState = TRAP_DOOR_LIFT_OFF;
    }

    /* Here we prevent if while lifting off we receive a down or middle signals */
    if (trapDoorCurrentState == TRAP_DOOR_LIFT_OFF && (received_signal == RF_DOWN || received_signal == RF_MIDDLE))
    {
        /* If the door is closed, the end switch is on, and the up signal is received, turn on the actuator */
        digitalWrite(ACTUATOR_RELAY_PIN, LOW);
        /* Set the trap door state to moving */
        trapDoorCurrentState = TRAP_DOOR_LIFT_OFF;
    }

    /* If the end switch is fully released, then we know the door is moving */
    if (digitalRead(END_SWITCH_PIN) == END_SWITCH_OPEN_VALUE && trapDoorCurrentState == TRAP_DOOR_LIFT_OFF)
    {
        /* Set the trap door state to moving */
        trapDoorCurrentState = TRAP_DOOR_MOVING;
    }

    /* If for some reason someone lifts the door manually and then starts the actuator - optional */
    if (digitalRead(END_SWITCH_PIN) == END_SWITCH_OPEN_VALUE && trapDoorCurrentState == TRAP_DOOR_CLOSED)
    {
        /* Set the trap door state to moving */
        trapDoorCurrentState = TRAP_DOOR_MOVING;
    }

    /// WHEN THE DOOR IS MOVING ---------------------------------------------

    /* if the signal is down or up and the door is moving, we turn on the relay */
    if ((received_signal == RF_DOWN || received_signal == RF_UP) && trapDoorCurrentState == TRAP_DOOR_MOVING)
    {
        digitalWrite(ACTUATOR_RELAY_PIN, HIGH);
        /* Set the trap door state to moving */
        trapDoorCurrentState = TRAP_DOOR_MOVING;
    }

    /// WHEN THE DOOR IS OPENED ---------------------------------------------

    /* If we trap door is opening and it reaches its angle, we shut off the relay */
    if (isDoorFullyOpened(tilt_angle) && trapDoorCurrentState == TRAP_DOOR_MOVING)
    {
        digitalWrite(ACTUATOR_RELAY_PIN, LOW);
        /* Set the trap door state to fully opened */
        trapDoorCurrentState = TRAP_DOOR_FULLY_OPENED;
    }

    /* If the trap door is fully opened, and we receive a down signal, we turn on the relay */
    if (received_signal == RF_DOWN && trapDoorCurrentState == TRAP_DOOR_FULLY_OPENED)
    {
        digitalWrite(ACTUATOR_RELAY_PIN, HIGH);
        /* Set the trap door state to moving */
        trapDoorCurrentState = TRAP_DOOR_MOVING;
    }
}