#include <Arduino.h>
#include "readsensors.h"
#include "transmitwifi.h"
#include "defs.h"
#include "functions.h"


int access_point = 0;

TaskHandle_t WiFiTelemetryTaskHandle;

TaskHandle_t GetDataTaskHandle;


static uint16_t wifi_queue_length = 100;

static QueueHandle_t wifi_telemetry_queue;
static QueueHandle_t sdwrite_queue;
static QueueHandle_t gps_queue;


// struct Data readData()
// {
//   struct Data ld = {0};
//   struct SensorReadings readings = {0};
//   struct FilteredValues filtered_values = {0};

//   readings = get_readings();

  
// }


void GetDataTask(void *parameter)
{

  struct Data ld = {0};

  static int droppedWiFiPackets = 0;
  // static int droppedSDPackets = 0;

  for (;;)
  {

    // ld = readData();

    if (xQueueSend(wifi_telemetry_queue, (void *)&ld, 0) != pdTRUE)
    {
      droppedWiFiPackets++;
    }

    debugf("Dropped WiFi Packets : %d\n", droppedWiFiPackets);
    // debugf("Dropped SD Packets : %d\n", droppedSDPackets);

    // yield to WiFi Telemetry task
    vTaskDelay(74 / portTICK_PERIOD_MS);
  }
}

void WiFiTelemetryTask(void *parameter)
{

  struct Data sv = {0};
  struct GPSReadings gpsReadings = {0};
  float latitude = 0;
  float longitude = 0;

  for (;;)
  {

    xQueueReceive(wifi_telemetry_queue, (void *)&sv, 10);
    sv = get_readings();
    sv.latitude = latitude;
    sv.longitude = longitude;


    handleWiFi(sv);

    // yield to Get Data task
    vTaskDelay(35 / portTICK_PERIOD_MS);
  }
}

void setup()
{

  Serial.begin(BAUD_RATE);

  debugln("Starting program");

  // set up ejection pin
  pinMode(EJECTION_PIN, OUTPUT);
  digitalWrite(EJECTION_PIN, LOW);

  // set up buzzer pin
  pinMode(buzzer_pin, OUTPUT);

  if (access_point == 1)
  {
    create_Accesspoint();
  }
  else
  {

  setup_wifi();
  client.setServer(mqtt_server, MQQT_PORT);
  client.setCallback(mqttCallback);
  }

  init_sensors();

  // initSDCard();

  // get the base_altitude
  // BASE_ALTITUDE = get_base_altitude();

  wifi_telemetry_queue = xQueueCreate(wifi_queue_length, sizeof(Data));
//   sdwrite_queue = xQueueCreate(sd_queue_length, sizeof(Data));
//   gps_queue = xQueueCreate(gps_queue_length, sizeof(GPSReadings));

  // initialize core tasks
  xTaskCreatePinnedToCore(GetDataTask, "GetDataTask", 3000, NULL, 1, &GetDataTaskHandle, 1);
  xTaskCreatePinnedToCore(WiFiTelemetryTask, "WiFiTelemetryTask", 4000, NULL, 2, &WiFiTelemetryTaskHandle, 1);
  //xTaskCreatePinnedToCore(readGPSTask, "ReadGPSTask", 3000, NULL, 1, &GPSTaskHandle, 1);
  // xTaskCreatePinnedToCore(SDWriteTask, "SDWriteTask", 4000, NULL, 1, &SDWriteTaskHandle, 1);

  // vTaskDelete(NULL);
}
void loop()
{
}