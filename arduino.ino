#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h> // メッセージキューを使用するときは忘れない！
#include "DHT.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_mac.h>
#define DHTPIN 15     // DHT11を2番ピンに接続
#define DHTTYPE DHT11 // DHT11を使用
#define HOUSE_DELAY_MS 1500 // 温湿度センサーのディレイ秒数
#define LEDPIN 26
#define BUTTONPIN 22
int trigPin = 2;  // Trigger
int echoPin = 4;  // Echo
// DHTセンサーの初期化
DHT dht(DHTPIN, DHTTYPE);

// 通信の識別番号（クラス全員共通）
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// センサーデータを格納する構造体を定義
// メンバーはfloat型のtemperatureとfloat型のhumidity
typedef struct {
  float temperature; // 温度
  float humidity;    // 湿度
} SensorData;

typedef float SonicSensorData;

// QueuesHandle_t型のグローバル変数を定義
QueueHandle_t sensorDataQueue;
QueueHandle_t sonicSensorDataQueue;

BLECharacteristic *pCharacteristic;  // BLEの状態を機能を管理するポインタ
bool deviceConnected = false;        // BLEの接続状態を管理する変数を定義
int counter = 0;                     // スマホに送るデータの定義、とりあえずint型

// Global
bool isDanger = true;

// 接続状態を管理
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println(">>> スマホが接続されました");  // ログの明確化
  }
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println(">>> スマホが切断されました");  // 同上
    pServer->getAdvertising()->start();        // 切れたらまた電波を出す
  }
};

// 読み込んだデータに応じて処理
class MyCharacteristicsCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    // 値を取得
    String value = pCharacteristic->getValue().c_str();
    value.trim();  // ★追加: 前後の空白や改行を削除（判定ミス防止）

    if (value == "OFF" || value == "off") {
      digitalWrite(LEDPIN, LOW);
      Serial.println("【制御】LEDを消灯しました");
    } else {
      Serial.print("【警告】不明なコマンド: ");
      Serial.println(value);
    }
  }
};

// タスク関数プロトタイプ
void TaskSensor(void *pvParameters);
void TaskComm(void *pvParameters);
void TaskLEDAlert(void *pvParameters);
void TaskSonicSensor(void *pvParameters);
void TaskBLE(void *pvParameters);
void TaskStopAlertButton(void *pvParameters);

void setup() {
  Serial.begin(115200);
  pinMode(echoPin, INPUT);               // Set echo pin as input
  pinMode(trigPin, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(BUTTONPIN, INPUT);
  dht.begin();
  while (!Serial) {}
  // 混線防止（MACアドレスの末尾4桁を利用）
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char name[20];
  sprintf(name, "yuha_ESP32_%02X%02X", mac[4], mac[5]);

  // BLEの準備（デバイス名、サーバー起動、UUID）
  BLEDevice::init(name);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // BLEで利用する機能を指定
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  // キャラクタリスティックの起動
  pCharacteristic->setCallbacks(new MyCharacteristicsCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("Ready");  // 初期値
  pService->start();

  // BLE広告の開始
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);  // サービスUUIDを広告に含めて探しやすくする
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

// 10個の「SensorData」構造体を格納できるキューを作成
  sensorDataQueue = xQueueCreate(10, sizeof(SensorData));
  sonicSensorDataQueue = xQueueCreate(10, sizeof(SonicSensorData));

  if (sensorDataQueue == NULL || sonicSensorDataQueue == NULL) {
    Serial.println("Queue creation failed!");
    for (;;) {} // キューの生成に失敗したら処理を止める
  }

  xTaskCreate(TaskSensor, "SensorTask", 2048, NULL, 1, NULL);
  xTaskCreate(TaskSonicSensor, "SonicSensorTask", 2048, NULL, 1, NULL);
  xTaskCreate(TaskLEDAlert, "LEDAlertTask", 2048, NULL, 1, NULL);
  xTaskCreate(TaskStopAlertButton, "StopAlertButtonTask", 1024, NULL, 1, NULL);
  xTaskCreate(TaskComm, "CommTask", 2048, NULL, 2, NULL);
  xTaskCreate(TaskBLE, "BLETask", 4096, NULL, 2, NULL);
}

void loop() {}

void TaskSensor(void *pvParameters) {
  (void)pvParameters;

  // 構造体の変数dataを定義
  SensorData data;

  for (;;) {
    float h = dht.readHumidity();    // float型で湿度を取得
    float t = dht.readTemperature(); // float型で温度（摂氏）を取得
    data.temperature = t; 
    data.humidity = h;    

    Serial.print("TaskSensor: Sending Temp=");
    Serial.print(data.temperature);
    Serial.print(", Hum=");
    Serial.println(data.humidity);

    // 構造体へのポインタをキューに送信
    xQueueSend(sensorDataQueue, &data, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(HOUSE_DELAY_MS));
  }
}

// 超音波センサータスク: 2000msごとにデータをキューに送信
void TaskSonicSensor(void *pvParameters) {
  (void)pvParameters;
  SonicSensorData sensorValue = 0;
  SonicSensorData duration = 0;

  for (;;) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    sensorValue = (duration / 2) / 29.1;  // Divide by 29.1 or multiply by 0.0343

    Serial.print(sensorValue);
    Serial.println("cm");

    // キューが満杯の場合、100ms待機してから再試行
    if (xQueueSend(sonicSensorDataQueue, &sensorValue, pdMS_TO_TICKS(100)) == pdPASS) {
      Serial.print("TaskSensor: 送信したデータ: ");
      Serial.println(sensorValue);
    };
    vTaskDelay(pdMS_TO_TICKS(2000));  // 2000msごとにデータを送信
  }
}

void TaskLEDAlert(void *pvParameters) {
  (void)pvParameters;
  for(;;) {
    if (isDanger) {
      digitalWrite(LEDPIN, HIGH);
    } else {
      digitalWrite(LEDPIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
}

void TaskStopAlertButton(void *pvParameters) {
  (void) pvParameters;
  for(;;) {
    if (digitalRead(BUTTONPIN) == HIGH) {
      isDanger = false;
    }else {
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void TaskBLE(void *pvParameters) {
  (void)pvParameters;
  float receivedValue;
  bool isPrevDanger = isDanger;

  for (;;) {
    if (deviceConnected) {
      if (isDanger != isPrevDanger) {
        Serial.print("isDanger:");
        Serial.println(isDanger);
        Serial.print("-- isPrevDanger:");
        Serial.println(isPrevDanger);
        isPrevDanger = isDanger;
        if (isDanger) {
          pCharacteristic->setValue("警告");
          pCharacteristic->notify();
          Serial.println("警告をしました");
        } else {
          pCharacteristic->setValue("解除");
          pCharacteristic->notify();   
          Serial.println("解除をしました");  
        }
      }  
    }
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

void TaskComm(void *pvParameters) {
  (void)pvParameters;
  SensorData receivedData;
  SonicSensorData receivedSonicData;


  for (;;) {
    if ((xQueueReceive(sensorDataQueue, &receivedData, portMAX_DELAY) == pdPASS) || (xQueueReceive(sonicSensorDataQueue, &receivedSonicData, portMAX_DELAY) == pdPASS)) {
      Serial.print("TaskComm: Received Temp=");
      Serial.print(receivedData.temperature);
      Serial.print(", Hum=");
      Serial.println(receivedData.humidity);
      Serial.print("cm =");
      Serial.println(receivedSonicData);
      

      if(receivedData.temperature > 28 && receivedSonicData <= 50 && !(isDanger)) {
        isDanger = true;
      }
    } else {
      // データが届かなかった場合のタイムアウト処理
      Serial.println("TaskComm: データを待っています...");
    }
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}