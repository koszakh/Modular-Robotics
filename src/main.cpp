#include <Arduino.h>
#include <MotorController.h>
#include <SPI.h>
#include "esp32-hal-ledc.h"
#include <MqttClient.h>
#include <PubSubClient.h>
#include "math.h"

#define PIN_ENABLE_R 13
#define PIN_FORWARD_R 12
#define PIN_BACK_R 14
#define PIN_FORWARD_L 27
#define PIN_BACK_L 26
#define PIN_ENABLE_L 25

//std::string SERV_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
//std::string CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
//std::string valueFromJoystick;
std::string receivedData;
std::string valuex;
std::string valuey;

short x, y, x1, x2;
short frequency = 500;
short xCoord;
short yCoord;
short correctValue;
short finishValue = -1;
short errorValue;
short hallSensorLeft = 0;
short hallSensorRight = 0;

uint8_t resolution = 10;
uint8_t splitindex;
uint8_t channel_R = 0;
uint8_t channel_L = 1;
uint8_t topic_id = 5;
uint8_t hallLeftPin = 33;
uint8_t hallRightPin = 32;


float reduceSpeed = 5;
float reduceSpeedSide = 5;

char outputData[10] = "Hi";

// const char* ssid = "SPEECH_405";
// const char* password = "multimodal";
// const char* mqtt_server = "192.168.0.33";

const char* ssid = "iGarage";
const char* password = "igarage18";
const char* mqtt_server = "10.1.30.46";

mqttClient mqtt(ssid, password, mqtt_server);

MotorController DriveCar;

void driveMotor(short);
void parseBLEData(std::string valueFromJoystick);

void callback(char* topic, byte* message, unsigned int length)
{
    if (strcmp(topic, "platforms/5")==0) {

    
        receivedData = "";

        for (int i = 0; i < length; i++)
        {
            receivedData += (char)message[i];

            correctValue = atoi(receivedData.c_str());
           
        }
        Serial.println(correctValue);
    }
    else if ((strcmp(topic, "on_finish/5")==0)) {
        receivedData = "";

        for (int i = 0; i < length; i++)
        {
        receivedData += (char)message[i];
        finishValue = atoi(receivedData.c_str()); 
        }
        Serial.println(finishValue);
         
    }
    
}

    //xErrorValue = atoi(receivedData.c_str());
    //Serial.println(xErrorValue);
    //xErrorValue = atoi(receivedData.c_str());
    //Serial.println(xErrorValue);


// void callbackFinish(char* finish, byte* message, unsigned int length)
// {
//     receivedData = "";

//     for (int i = 0; i < length; i++)
//     {
//         receivedData += (char)message[i];
       
//     }
//     finishValue = atoi(receivedData.c_str());
//     //Serial.println(finishValue);
// }


void setup()
{
    Serial.begin(115200);
    mqtt.setupWifi();
    mqtt.setCallback(*callback);
    //mqtt.setCallbackFinish(*callbackFinish);
    DriveCar.setup(PIN_ENABLE_R,PIN_FORWARD_R,PIN_BACK_R,PIN_FORWARD_L,PIN_BACK_L,PIN_ENABLE_L, channel_R, channel_L);
    DriveCar.setupMotorDriver(channel_R, channel_L, frequency, resolution);
    pinMode(hallLeftPin, INPUT);
    pinMode(hallRightPin, INPUT);

    //Esp32.initialize(parseBLEData,SERV_UUID,CHAR_UUID);
}

/*void parseBLEData(std::string valueFromJoystick)
{
    splitindex = valueFromJoystick.find("/");
    valuex = valueFromJoystick.substr(0,splitindex);
    valuey = valueFromJoystick.substr(splitindex+1);
    xCoord = atoi(valuex.c_str());
    yCoord = atoi(valuey.c_str());
    driveMotor(xCoord, yCoord);
}*/
void driveMotor(short correctValue)
{
    //x = map(x1, 0, 4095, -1000, 1000);
    //y = map(x2, 0, 4095, -1000, 1000);
    DriveCar.controlByCamera(correctValue, reduceSpeed, reduceSpeedSide);
    //Serial.println(x);
    //Serial.println(y);
}

void loop()
{
     correctValue = 0;
     mqtt.initClientLoop();
     mqtt.subscribe(topic_id);

     if (finishValue == 0)
     {
        
        if (correctValue > 5) {
            DriveCar.rotateLeft(correctValue);
        }
        else if (correctValue < -5) {
            DriveCar.rotateRight(correctValue);
        }
        else if (abs(correctValue) <= 5) {
            DriveCar.moveForward(correctValue);
        }
     }
     else if (finishValue == 1)
        {
            DriveCar.stop(correctValue);
        }
     //Serial.println(correctValue);
    
    // correctValue = 0;
    // hallSensorLeft = digitalRead(hallLeftPin);
    // hallSensorRight = digitalRead(hallRightPin);
    // //Serial.println(hallSensorLeft);
    // Serial.println(hallSensorRight);
    // //delay(300);
}
