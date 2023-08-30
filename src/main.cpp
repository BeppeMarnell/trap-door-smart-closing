#include <Arduino.h>
#include <RCSwitch.h>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

/* Create a new instance of the RCSwitch class for receiving messages from the remote */
RCSwitch ReceiverModule = RCSwitch();
/* Assign a unique ID to this sensor */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

/* Constants */
const int ACTUATOR_RELAY_PIN = 4;
const int SERIAL_BAUD_RATE = 9600;

/* Receiver values */
const unsigned long RECEIVER_UP_VALUE = 2678888;
const unsigned long RECEIVER_MIDDLE_VALUE = 2678884;
const unsigned long RECEIVER_DOWN_VALUE = 2678882;
const int RECEIVER_PROTOCOL = 1;
const int RECEIVER_BIT_LENGTH = 24;

/* Enum for the RF signals */
enum RFSignal {
    RF_UP = 1,
    RF_MIDDLE = 2,
    RF_DOWN = 3,
    RF_NONE = 0
};


dataRate_t dataRate = ADXL345_DATARATE_100_HZ;

/* Get angle for the y axis of the accelerometer */
double get_angle_around_y_axis(double x, double y, double z);
/* RF function method */
RFSignal receiveRFSignals();

/* Setup for the arudino */
void setup()
{
    /* Setup the serial connection with 9600 baud rate */
    Serial.begin(SERIAL_BAUD_RATE);

    /* This needs to be digital pin 2 on the arduino */
    ReceiverModule.enableReceive(0); // Receiver on interrupt 0 => that is pin #2

    /* Make the actuator relay pin as output */
    pinMode(ACTUATOR_RELAY_PIN, OUTPUT);

    /* Initialise the sensor */
    if (!accel.begin())
    {
        /* There was a problem detecting the ADXL345 ... check your connections */
        Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
        while (1)
            ;
    }

    /* Set the range to whatever is appropriate for your project */
    accel.setRange(ADXL345_RANGE_2_G);
}

/* Loop for the arduino */
void loop()
{
    /* Get a new sensor event */
    sensors_event_t event;
    accel.getEvent(&event);

    /* Print the angle */
    Serial.print("Angle : ");
    Serial.println(get_angle_around_y_axis(event.acceleration.x, event.acceleration.y, event.acceleration.z) * 180 / M_PI);

    /* Receive the rf signals */
    int received_signal = receiveRFSignals();

    /* Print the received signal */
    // Serial.println(received_signal);

    /* If the signal is up, then turn off the relay */
    if (received_signal == RF_UP)
    {
        digitalWrite(ACTUATOR_RELAY_PIN, LOW);
    }else if (received_signal == RF_DOWN)
    {
        digitalWrite(ACTUATOR_RELAY_PIN, HIGH);
    }

    /* Wait for 100ms */
    delay(100);
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

        /* Reset the receiver */
        ReceiverModule.resetAvailable();
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
 * @return The angle around the y-axis in radians.
 */
double get_angle_around_y_axis(double x, double y, double z)
{
    double projection = x * cos(M_PI / 2) + z * sin(M_PI / 2);
    double angle = atan2(projection, x);
    return angle;
}