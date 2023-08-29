#include <Arduino.h>
#include <RCSwitch.h>

// Create a new instance of the RCSwitch class for receiving messages from the remote
RCSwitch ReceiverModule = RCSwitch();

// Constants
const int ACTUATOR_RELAY_PIN = 4;
const int SERIAL_BAUD_RATE = 9600;

// Receiver values
const unsigned long RECEIVER_UP_VALUE = 2678888;
const unsigned long RECEIVER_MIDDLE_VALUE = 2678884;
const unsigned long RECEIVER_DOWN_VALUE = 2678882;
const int RECEIVER_PROTOCOL = 1;
const int RECEIVER_BIT_LENGTH = 24;

// Enum for the RF signals
enum RFSignal {
    RF_UP = 1,
    RF_MIDDLE = 2,
    RF_DOWN = 3,
    RF_NONE = 0
};

// Function declarations
RFSignal receiveRFSignals();

// Setup for the arudino
void setup()
{
    // Setup the serial connection with 9600 baud rate
    Serial.begin(SERIAL_BAUD_RATE);

    // This needs to be digital pin 2 on the arduino
    ReceiverModule.enableReceive(0); // Receiver on interrupt 0 => that is pin #2

    // Make the actuator relay pin as output
    pinMode(ACTUATOR_RELAY_PIN, OUTPUT);
}

// Loop for the arduino
void loop()
{
    // Receive the rf signals
    int received_signal = receiveRFSignals();

    // Print the received signal
    // Serial.println(received_signal);

    // if the signal is up, then turn off the relay
    if (received_signal == RF_UP)
    {
        digitalWrite(ACTUATOR_RELAY_PIN, LOW);
    }else if (received_signal == RF_DOWN)
    {
        digitalWrite(ACTUATOR_RELAY_PIN, HIGH);
    }

    // Wait for 100ms
    delay(100);
}

/**
 * @brief Receive the rf signals from the remote
 * @return int
 */
RFSignal receiveRFSignals()
{
    // Return value
    RFSignal returnValue = RF_NONE;

    if (ReceiverModule.available())
    {
        // Detect the received value
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

        // Reset the receiver
        ReceiverModule.resetAvailable();
    }

    return returnValue;
}

// IMU reader

// #include <Arduino.h>
// #include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_ADXL345_U.h>
// /* Assign a unique ID to this sensor at the same time */
// /* Uncomment following line for default Wire bus      */
// Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// dataRate_t dataRate = ADXL345_DATARATE_100_HZ;

// /* NeoTrellis M4, etc.                    */
// /* Uncomment following line for Wire1 bus */
// // Adafruit_ADXL343 accel = Adafruit_ADXL343(12345, &Wire1);

// void displaySensorDetails(void)
// {
//     sensor_t sensor;
//     accel.getSensor(&sensor);
//     Serial.println("------------------------------------");
//     Serial.print("Sensor:       ");
//     Serial.println(sensor.name);
//     Serial.print("Driver Ver:   ");
//     Serial.println(sensor.version);
//     Serial.print("Unique ID:    ");
//     Serial.println(sensor.sensor_id);
//     Serial.print("Max Value:    ");
//     Serial.print(sensor.max_value);
//     Serial.println(" m/s^2");
//     Serial.print("Min Value:    ");
//     Serial.print(sensor.min_value);
//     Serial.println(" m/s^2");
//     Serial.print("Resolution:   ");
//     Serial.print(sensor.resolution);
//     Serial.println(" m/s^2");
//     Serial.println("------------------------------------");
//     Serial.println("");
//     delay(500);
// }

// void setup(void)
// {
//     Serial.begin(9600);
//     while (!Serial)
//         ;
//     Serial.println("Accelerometer Test");
//     Serial.println("");

//     /* Initialise the sensor */
//     if (!accel.begin())
//     {
//         /* There was a problem detecting the ADXL345 ... check your connections */
//         Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
//         while (1)
//             ;
//     }

//     /* Set the range to whatever is appropriate for your project */
//     // accel.setRange(ADXL345_RANGE_16_G);
//     // accel.setRange(ADXL343_RANGE_8_G);
//     accel.setRange(ADXL345_RANGE_4_G);
//     // accel.setRange(ADXL343_RANGE_2_G);

//     /* Display some basic information on this sensor */
//     displaySensorDetails();
//     Serial.println("");
// }

// void loop(void)
// {
//     /* Get a new sensor event */
//     sensors_event_t event;
//     accel.getEvent(&event);

//     /* Display the results (acceleration is measured in m/s^2) */
//     Serial.print("X: ");
//     Serial.print(event.acceleration.x);
//     Serial.print("  ");
//     Serial.print("Y: ");
//     Serial.print(event.acceleration.y);
//     Serial.print("  ");
//     Serial.print("Z: ");
//     Serial.print(event.acceleration.z);
//     Serial.print("  ");
//     Serial.println("m/s^2 ");
//     delay(500);
// }