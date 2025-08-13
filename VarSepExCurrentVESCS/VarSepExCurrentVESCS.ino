#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h" // FreeRTOSのミューテックス機能を利用するために必要なヘッダファイル
#include <Arduino.h>
#include <driver/twai.h>
#include <esp32-hal-adc.h> // ESP32-C3 でも analogSetPinAttenuation が使えます

// --- ADC 設定 ---
constexpr gpio_num_t HALL_PIN = GPIO_NUM_3; // スロットルセンサー入力
constexpr gpio_num_t FIELD_CURRENT_PIN = GPIO_NUM_4; // 界磁電流センサー入力
constexpr int ADC_RESOLUTION = 4095;        // 12bit

// --- CAN (TWAI) 設定 ---
constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_1;
constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_0;
constexpr long CAN_BAUD = 500000; // 500 kbps

// --- VESC CAN プロトコル ---
constexpr uint32_t CAN_PACKET_SET_DUTY = 0x0; // デューティコマンド識別子
constexpr uint8_t ARMATURE_VESC_ID = 0x7;     // VESC ID: VESC Tool/App Settings/General/VESC ID
constexpr uint32_t CAN_PACKET_SET_CURRENT = 0x1; // 界磁電流コマンド識別子
constexpr uint8_t FIELD_VESC_ID = 0x8;        // VESC ID: VESC Tool/App Settings/General/VESC ID

// constexpr int SAMPLES_COUNT = 20; // 20msの間に20サンプル（1ms毎）
constexpr int SAMPLES_COUNT = 20; // スロットル入力の平均を取るためのサンプル数。20サンプル（1ms毎に取得）の移動平均を計算します。

// --- 界磁電流 ---
constexpr float FIELD_CURRENT_A_MAX = 1.0f; // 界磁電流[A]
static int field_current_samples[SAMPLES_COUNT];
static int field_current_sample_index = 0;
constexpr float FIELD_CURRENT_MIN = 0.29f;
constexpr float FIELD_CURRENT_MAX = 0.92f;

static SemaphoreHandle_t fieldCurrentMutex = NULL; // Mutex for field current data
// --- スロットル平均化設定 ---
// 値を大きくすると入力が滑らかになりますが、制御の応答は少し遅くなる傾向があります。
// static int throttle_samples[SAMPLES_COUNT];
static int throttle_samples[SAMPLES_COUNT]; // スロットル入力の過去の値を格納する配列（リングバッファとして使用）。この配列の値を合計して平均値を求めます。
static int sample_index = 0; // throttleReadTask で使用されるサンプルインデックス
static float HALL_MIN = 0.29f; // スロットル最小値 (29%)
static float HALL_MAX = 0.92f; // スロットル最大値 (92%)

static SemaphoreHandle_t throttleMutex = NULL; // Mutex for throttle data

// TWAI の一般設定・タイミング設定・フィルタ設定
constexpr twai_general_config_t g_config =
    TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
constexpr twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
constexpr twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

// VESCコントロールタスク: 平均化されたスロットル値を読み取り、VESC制御用のCANメッセージを定期的に送信するタスク
// 20ms周期で実行され、スロットル値に基づいて計算されたデューティ比をVESCに指令します。
void vescControlTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms間隔

  // タスクの初期化: 初回の起床時刻を現在の時刻に設定
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    int32_t field_current_sum = 0;
    if (xSemaphoreTake(fieldCurrentMutex, portMAX_DELAY) == pdTRUE) {
      for (int i = 0; i < SAMPLES_COUNT; i++) {
        field_current_sum += field_current_samples[i];
      }
       xSemaphoreGive(fieldCurrentMutex);
    } else {
      Serial.println("Error: vescControlTask failed to take fieldCurrentMutex");
    }

    int32_t throttle_sum = 0; 
    if (xSemaphoreTake(throttleMutex, portMAX_DELAY) == pdTRUE) {
      for (int i = 0; i < SAMPLES_COUNT; i++) {
        throttle_sum += throttle_samples[i];
      }
       xSemaphoreGive(throttleMutex);
    } else {
      Serial.println("Error: vescControlTask failed to take throttleMutex");
    }

    float field_current_percent = (field_current_sum / (float)(SAMPLES_COUNT * ADC_RESOLUTION));
    float field_current_clamped = (field_current_percent - FIELD_CURRENT_MIN) / (FIELD_CURRENT_MAX - FIELD_CURRENT_MIN);
    float field_current_a = constrain(field_current_clamped, 0.0f, 1.0f)*FIELD_CURRENT_A_MAX;
    int32_t scaled_field_current_a = (int32_t)(field_current_a * 1000.0f);

    float throttle_percent = (throttle_sum / (float)(SAMPLES_COUNT * ADC_RESOLUTION));
    float throttle_clamped = (throttle_percent - HALL_MIN) / (HALL_MAX - HALL_MIN);
    float throttle_duty = constrain(throttle_clamped, 0.0f, 1.0f);
    int32_t scaled_throttle_duty = (int32_t)(throttle_duty * 100000.0f);

    twai_message_t field_msg;
    memset(&field_msg, 0, sizeof(field_msg));

    twai_message_t armature_msg;
    memset(&armature_msg, 0, sizeof(armature_msg));

    armature_msg.identifier = (CAN_PACKET_SET_DUTY << 8) | ARMATURE_VESC_ID;
    field_msg.identifier = (CAN_PACKET_SET_CURRENT << 8) | FIELD_VESC_ID;

    armature_msg.flags = TWAI_MSG_FLAG_EXTD;
    field_msg.flags = TWAI_MSG_FLAG_EXTD;

    uint8_t buf[4] = {(uint8_t)(scaled_throttle_duty >> 24), (uint8_t)(scaled_throttle_duty >> 16),
                      (uint8_t)(scaled_throttle_duty >> 8), (uint8_t)(scaled_throttle_duty)};
    memcpy(armature_msg.data, buf, sizeof(buf));
    armature_msg.data_length_code = sizeof(scaled_throttle_duty);

    uint8_t field_buf[4] = {(uint8_t)(scaled_field_current_a >> 24), (uint8_t)(scaled_field_current_a >> 16),
                            (uint8_t)(scaled_field_current_a >> 8), (uint8_t)(scaled_field_current_a)};
    memcpy(field_msg.data, field_buf, sizeof(field_buf));
    field_msg.data_length_code = sizeof(scaled_field_current_a);

    Serial.print("PWM/%:");
    Serial.println(scaled_throttle_duty / 1000);
    Serial.print("Field Current/mA:");
    Serial.println(scaled_field_current_a);

    if (twai_transmit(&field_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
      if (twai_transmit(&armature_msg, pdMS_TO_TICKS(10)) != ESP_OK) {
        Serial.println("TWAI transmit failed (armature)");
      }
    } else {
      Serial.println("TWAI transmit failed (field)"); // 界磁電流メッセージ送信失敗
      // PWM=0 の armature_msg を送出
      int32_t zero_pwm = 0;
      uint8_t zero_buf[4] = {(uint8_t)(zero_pwm >> 24), (uint8_t)(zero_pwm >> 16),
                             (uint8_t)(zero_pwm >> 8), (uint8_t)(zero_pwm)};
      memcpy(armature_msg.data, zero_buf, sizeof(zero_buf));
      armature_msg.data_length_code = sizeof(zero_pwm);
      if (twai_transmit(&armature_msg, pdMS_TO_TICKS(10)) != ESP_OK) {
        Serial.println("TWAI transmit failed (armature zero)"); // CANメッセージの送信に失敗
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void fieldCurrentReadTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1ms間隔

  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    if (xSemaphoreTake(fieldCurrentMutex, portMAX_DELAY) == pdTRUE) {
      field_current_samples[field_current_sample_index] = analogRead(FIELD_CURRENT_PIN);
      field_current_sample_index = (field_current_sample_index + 1) % SAMPLES_COUNT;
      xSemaphoreGive(fieldCurrentMutex);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void throttleReadTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1ms間隔

  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    if (xSemaphoreTake(throttleMutex, portMAX_DELAY) == pdTRUE) { // pdTRUE は取得成功を示す
      throttle_samples[sample_index] = analogRead(HALL_PIN);
      sample_index = (sample_index + 1) % SAMPLES_COUNT;
      xSemaphoreGive(throttleMutex);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup() {
  Serial.begin(115200);

  fieldCurrentMutex = xSemaphoreCreateMutex();
  if (fieldCurrentMutex == NULL) {
      Serial.println("Error: Failed to create fieldCurrentMutex");
      while(1); // 無限ループでプログラムを停止（エラー発生時は安全のため、システムを停止させる）
  }
  analogSetPinAttenuation(FIELD_CURRENT_PIN, ADC_11db);
  // スロットルサンプル配列の初期化
  for (int i = 0; i < SAMPLES_COUNT; i++) {
    field_current_samples[i] = 0;
  }

  throttleMutex = xSemaphoreCreateMutex();
  if (throttleMutex == NULL) {
      Serial.println("Error: Failed to create throttleMutex");
      while(1); // 無限ループでプログラムを停止（エラー発生時は安全のため、システムを停止させる）
  }
  analogSetPinAttenuation(HALL_PIN, ADC_11db);
  for (int i = 0; i < SAMPLES_COUNT; i++) {
    throttle_samples[i] = 0;
  }

  // TWAI ドライバのインストール
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("Failed to install TWAI driver");
    while (1)
      ;
  }
  // TWAI ドライバ開始
  if (twai_start() != ESP_OK) {
    Serial.println("Failed to start TWAI driver");
    while (1)
      ;
  }

  Serial.println("TWAI (CAN) initialized");

  xTaskCreate(fieldCurrentReadTask,    // タスクとして実行する関数
    "FieldCurrentRead",      // タスク名 (デバッグ用)
    2048,                // スタックサイズ (バイト単位)
    NULL,                // タスクに渡すパラメータ (今回はなし)
    2,                   // タスクの優先度 (数値が大きいほど高優先度)
    NULL);               // タスクハンドル (今回は使用せず)

  xTaskCreate(throttleReadTask,    // タスクとして実行する関数
              "ThrottleRead",      // タスク名 (デバッグ用)
              2048,                // スタックサイズ (バイト単位)
              NULL,                // タスクに渡すパラメータ (今回はなし)
              2,                   // タスクの優先度 (数値が大きいほど高優先度)
              NULL);               // タスクハンドル (今回は使用せず)

  xTaskCreate(vescControlTask,     // タスクとして実行する関数
              "VESCControl",       // タスク名
              2048,                // スタックサイズ
              NULL,                // パラメータ
              1,                   // 優先度
              NULL);               // タスクハンドル
}

void loop() {
  // メインループ (Arduino の loop() 関数) は、FreeRTOSタスクが動作している間は実質的に使用されません。
  // FreeRTOSスケジューラがタスクの実行管理を行うため、loop() 内の処理は通常実行機会がほとんどありません。
  // vTaskDelay(portMAX_DELAY) を呼び出すことで、loopタスク（メインタスク）を無期限スリープ状態にし、
  // CPUリソースを他のアプリケーションタスク（throttleReadTask や vescControlTask）に完全に譲渡します。
  // これがFreeRTOS環境での一般的な loop() の実装方法です。
  vTaskDelay(portMAX_DELAY);
}
