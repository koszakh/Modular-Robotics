#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <stdio.h>
#include <MqttClient.h>


WiFiClient espClient;
PubSubClient* client;


mqttClient::mqttClient(const char* ssid, const char* password, const char* mqtt_server)
{
    SSID = ssid;
    PASSWORD = password;
    client = new PubSubClient(espClient);
    client->setServer(mqtt_server, 1883);
}


void mqttClient::setupWifi()
{
    WiFi.begin(SSID, PASSWORD);
    delay(500);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
    }
}

void mqttClient::initClientLoop()
{
    client->loop();
}

void mqttClient::subscribe(int platform_id)
{
    char topic[64];
    char theme[64];
    char client_name[64];
    sprintf(theme, "Platform %d is connected!", platform_id);
    sprintf(topic, "platforms/%d", platform_id);
    sprintf(client_name, "platform_%d", platform_id);
    //Loop until we're reconnected
    while (!client->connected())
    {
        // Attempt to connect
        if (client->connect(client_name))
        {
            client->subscribe(topic);

            // Subscribe to topic
            client->publish("connected", theme);
        }
        else
        {
            // Wait some seconds before retrying
            delay(100);
        }
    }
}


void mqttClient::pubFeedback(const char* msg, int platform_id)
{
    char theme[64];
    char topic[64];
    sprintf(topic, "esp32/feedback/%d", platform_id);
    sprintf(theme, "Message <<%s>> is received!",msg);
    client->publish(topic, theme);
}

void mqttClient::setCallback(void (*callback)(char* topic, byte* message, unsigned int length))
{
    client->setCallback(callback);
}

void mqttClient:: convertValue(short xValue)
{

}
