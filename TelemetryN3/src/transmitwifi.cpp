#include <WiFi.h>
#include "transmitwifi.h"

void mqttCallback(char *topic,byte *message, unsigned int length){
    debug("Message arrived on topic:");
    debug(topic);
    debug("Message");
    String messageTemp;


    for (int i=0;i<length;i++){
        debug((char)message[i]);
    }
    debugln();

}
void create_Accesspoint()
{
debugln();
debug("creating access point ");
debugln("ssid: ");
debugln(ssid);
debugln("password: ");
debugln(password);
WiFi.softAP(ssid, password);
IPAddress IP = WiFi.softAPIP();
debugln("Access point successully created ");
debugln("IP address: ");
debugln(IP);
client.setBufferSize(MQTT_BUFFER_SIZE);
client.setServer(mqtt_server, MQQT_PORT);
client.setCallback(mqttCallback);

}

void setup_wifi(){
    debugln();
    debug("Connecting to:");
    debugln(ssid);
    Serial.begin(115200);
    WiFi.begin(ssid,password);
    while(WiFi.status() != WL_CONNECTED);

    {
        delay(500);
        debug(".");
    }
    randomSeed(micros());
    debugln("");
    debugln("WiFi connected");
    debugln("IP address: ");
    debugln(WiFi.localIP());

    // client.setBufferSize(MQTT_BUFFER_SIZE);
    // client.setServer(mqtt_server, MQQT_PORT);
    // client.setCallback(mqttCallback);

}
    
void reconnect(){
    while (!client.connected())
    {
        Serial.begin(115200);
        debugln("Attempting MQTT connection...");
        String clientId = "FCClient-";
        // clientId += String(random(0xffff), HEX);
        if(client.connect(clientId.c_str())){
            debugln("Connected");
            // client.publish((const char*)"controls/ejection",message);
            // Subscribe
             client.subscribe("controls/ejection"); 
             client.publish("control/ejection","123");
             client.subscribe("esp32/message")  ;     
    }
    else
    {
        debug("failed,rc=");
        debug(client.state());
        debugln("try again in 5 seconds");
        delay(5000);
    }
}
}

void sendTelemetryWiFi(Data sv){

    char mqttMessage[300];
    sprintf(mqttMessage, "{\"timestamp\":%lld,\"altitude\":%.3f,\"temperature\":%.3f,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,\"longitude\":%.8f,\"latitude\":%.8f}", sv.timeStamp, sv.altitude,sv.temperature,sv.ax,sv.ay,sv.az,sv.gx,sv.gy,sv.gz,sv.longitude, sv.latitude);
    client.subscribe("esp32/message");
    client.publish("esp32/message", mqttMessage);
    debugln(mqttMessage);
    delay(5000);

}


void handleWiFi(Data sv){
    if(!client.connected()){
        reconnect();
    }
    client.loop();
    sendTelemetryWiFi(sv);
}

