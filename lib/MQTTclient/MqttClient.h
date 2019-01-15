#ifndef MqttClient 
#define MqttClient


#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <stdio.h>

class mqttClient
{
    public:

        const char* SSID;
        const char* PASSWORD;
        char* MQTT_SERVER;
        String receivedData;
        bool MESSAGE_IS_REC = false;

        mqttClient(const char* ssid, const char* password, const char* mqtt_server);
        void setupWifi();
        void initClientLoop();
        void subscribe(int platform_id);
        void pubMsg(const char* msg, int platform_id);
        void pubFeedback(const char* msg, int platform_id);
        void convertValue(short xValue);
        void setCallback(void (*func)(char* topic, byte* message, unsigned int length));
        void setCallbackFinish(void (*func)(char* finish, byte* message, unsigned int length));
       

};

#endif
