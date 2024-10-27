#include <Arduino.h>
#include <WiFi.h>
#include "ThingSpeak.h"

#define CHANNEL_ID 2703374
#define CHANNEL_API_KEY "Q50JB8MBAJA86KWT"

typedef int32_t s32;
typedef int16_t s16;

WiFiClient client;

#define WIFI_TIMEOUT_MS 20000
#define WIFI_NETWORK "GanJu"
#define WIFI_PASSWORD "hsx123456"

struct dataMember{
  float temp = 0.0;
  float humi = 0.0;
  float windSpeed = 0.0;
  float lightIntensity = 0.0;
  float fst = 0.0;
  float fst2 = 0.0;
  float fst3 = 0.0;
};

struct dataMember original_data;//接受的数据
struct dataMember average_data;//求和后的数据
struct dataMember sum_data;//求平均后要上报的数据

//平均值上传
int sum_count = 0;
int count_times = 60;//60s
int count_wind = 60;
int sum_count_wind = 0;

enum State{
  WaitingForHeader1,//等待包头
  ReadingData,//读取数据
  WaitingForChecksum,//核对校验和
  WaitingForTail//等待包尾
};

//初始化串口相关状态参数
State currentState = WaitingForHeader1;
bool dataPacketReceived_flag = false;//未接受到完整数据
byte checkSum = 0;
int dataLength = 16;
String buffer = "";

void connectToWiFi();
void readMyPack(byte receivedByte);
void checkWiFiStatus();
/**************************************************************************/


void setup() {
  Serial.begin(9600);
  Serial.println("hello world");
  Serial2.begin(9600);
  connectToWiFi();
  ThingSpeak.begin(client);
}

void loop() {
  if(WiFi.status() != WL_CONNECTED){
    connectToWiFi();
  }
  // 从串口读取数据
  while (Serial2.available() > 0&& WiFi.status() == WL_CONNECTED) {
    byte receivedByte = Serial2.read();
    Serial2.println(receivedByte);
    readMyPack(receivedByte);

    //读取数据求和，大于一定次数求平均上传ThingSpeak
    if (dataPacketReceived_flag) {

      sum_data.fst = sum_data.fst + original_data.fst;
      sum_data.fst2 = sum_data.fst2 + original_data.fst2;
      sum_data.fst3 = sum_data.fst3 + original_data.fst3;
      sum_data.temp = sum_data.temp + original_data.temp;
      sum_data.humi = sum_data.humi + original_data.humi;
      sum_data.lightIntensity = sum_data.lightIntensity + original_data.lightIntensity;
      sum_data.windSpeed = sum_data.windSpeed + original_data.windSpeed;
      sum_count++;

      /* if(sum_count_wind >=count_wind)
      {
          data_windSpeed = sum_windSpeed/count_wind;
          AliyunIoTSDK::send("wind", data_windSpeed);
          sum_count_wind = 0;
          sum_windSpeed =0;
      } */
     
      if(sum_count >=count_times)
      {

        average_data.temp = sum_data.temp/count_times;
        average_data.humi = sum_data.humi/count_times;
        average_data.windSpeed = sum_data.windSpeed/count_times;
        average_data.lightIntensity = sum_data.lightIntensity/count_times;
        average_data.fst = sum_data.fst/count_times;
        average_data.fst2 = sum_data.fst2/count_times;
        average_data.fst3 = sum_data.fst3/count_times;

        // 发送数据到ThingSpeak
        ThingSpeak.setField(1, average_data.temp);
        ThingSpeak.setField(2, average_data.humi);
        ThingSpeak.setField(3, average_data.lightIntensity);
        ThingSpeak.setField(4, average_data.windSpeed);
        ThingSpeak.setField(5, average_data.fst);
        ThingSpeak.setField(6, average_data.fst2);
        ThingSpeak.setField(7, average_data.fst3);

        Serial.printf("data send!");

        //将sum_data结构体初始化为0
        memset(&sum_data, 0, sizeof(sum_data));

        sum_count = 0;

        /* Serial.print("Data sent: ");
        Serial.print("Temp: ");
        Serial.print(data_temp);
        Serial.print(", Humi: ");
        Serial.print(data_humi);
        Serial.print(", WindSpeed: ");
        Serial.print(data_windSpeed);
        Serial.print(", LightIntensity: ");
        Serial.print(data_lightIntensity);
        Serial.print(", fst: ");
        Serial.println(data_fst);
        Serial.print(", fst2: ");
        Serial.println(data_fst2);
        Serial.print(", fst3: ");
        Serial.println(data_fst3); */
      }

      // 重置数据包接收标志
      dataPacketReceived_flag = false;
    }
  }
}

void connectToWiFi(){
  Serial.print("Connecting to wifi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_NETWORK,WIFI_PASSWORD);

  unsigned long startAttemptTime = millis();

  while(WiFi.status() != WL_CONNECTED &&millis() - startAttemptTime< WIFI_TIMEOUT_MS){
    Serial.printf(".");
    delay(1000);
  }

  if(WiFi.status() != WL_CONNECTED){
    Serial.printf("Failed to connect WiFi, checking reason....");
    checkWiFiStatus();
    Serial.printf("restarting esp32.....");
    ESP.restart();
  }else{
    Serial.printf("Connected!");
    Serial.println(WiFi.localIP());
  }
}

void readMyPack(byte receivedByte)
{
  if(dataPacketReceived_flag){
    return;
  }

  switch(currentState)
  {
    case WaitingForHeader1:
      if(receivedByte == 0xFF){
        currentState = ReadingData;//改变状态
        checkSum = receivedByte;
        buffer = "";//清空buffer，为接下来读取数据做准备
        Serial.println("Header Found!");
      }
      break;
    
    case ReadingData:
      buffer += (char)receivedByte;
      checkSum += receivedByte;

      if(buffer.length() >= dataLength)
      {
        currentState = WaitingForTail;
      }
      break;

    case WaitingForTail:
      if(receivedByte == 0xB3){
        dataPacketReceived_flag = true;
        Serial.println("Tail Found! Data Received!");

        //接收串口数据
        original_data.windSpeed = ((float)((s16)(buffer[0]<<8|buffer[1])))/100;
        original_data.temp = ((float)((s16)(buffer[2]<<8|buffer[3])))/100;
        original_data.humi = ((float)((s16)(buffer[4]<<8|buffer[5])))/100;
        original_data.lightIntensity =buffer[6]<<24|buffer[7]<<16|buffer[8]<<8|buffer[9];
        original_data.fst = ((float)((s16)(buffer[10]<<8|buffer[11])))/100;
        original_data.fst2 = ((float)((s16)(buffer[12]<<8|buffer[13])))/100;
        original_data.fst3 = ((float)((s16)(buffer[14]<<8|buffer[15])))/100;
        }
      else
      {
        Serial.println("Can not find the tail!");
      }
      currentState = WaitingForHeader1;
      break;
  }
}

void checkWiFiStatus() {
  // 根据WiFi.status() 打印相应的错误信息
  switch (WiFi.status()) {
    case WL_NO_SSID_AVAIL:
      Serial.println("No SSID Available");
      break;
    case WL_CONNECT_FAILED:
      Serial.println("Connection Failed, wrong password?");
      break;
    case WL_IDLE_STATUS:
      Serial.println("Idle Status, waiting for connection");
      break;
    case WL_DISCONNECTED:
      Serial.println("Disconnected from WiFi");
      break;
    default:
      Serial.println("Unknown Error");
      break;
  }
}