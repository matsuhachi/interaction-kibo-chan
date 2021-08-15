#include <ArduinoOSCWiFi.h>
#include <M5StickC.h>
#include <driver/i2s.h>
#include <esp_wifi.h>

#include "IIC_servo.h"
#define PIN_CLK 0
#define PIN_DATA 34
#define READ_LEN (2 * 256)
uint8_t BUFFER[READ_LEN] = {0};
uint16_t oldy[160];
int16_t* adcBuffer = NULL;

//自分の環境に合わせて書き換えてください
const char* SSID = "iPhone";
const char* PASS = "popnmusic";
const int PORT = 8000;  //適当でOK
const char* pc_addr = "172.20.10.2";
const int pc_port = 10000;  //送信先のポート 適当でOK

void i2s_init() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
        .sample_rate = 44100,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,  // is fixed at 12bit, stereo, MSB
        .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 2,
        .dma_buf_len = 128,
    };
    i2s_pin_config_t pin_config;
    pin_config.bck_io_num = I2S_PIN_NO_CHANGE;
    pin_config.ws_io_num = PIN_CLK;
    pin_config.data_out_num = I2S_PIN_NO_CHANGE;
    pin_config.data_in_num = PIN_DATA;

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    i2s_set_clk(I2S_NUM_0, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}
int mic_counter = 0;
bool now_mic = false;

void mic_record(void) {
    size_t bytesread;
    now_mic = true;

    //i2s_init();
    M5.Lcd.setCursor(0, 1);

    i2s_read(I2S_NUM_0, (char*)BUFFER, READ_LEN, &bytesread, portMAX_DELAY);
    adcBuffer = (int16_t*)BUFFER;
    int32_t offset_sum = 0;
    for (int n = 0; n < 160; n++) {
        offset_sum += (int16_t)adcBuffer[n];
    }
    int offset_val = -(offset_sum / 160);
    // Auto Gain
    int max_val = 2000;
    for (int n = 0; n < 160; n++) {
        int16_t val = (int16_t)adcBuffer[n] + offset_val;
        if (max_val < abs(val)) {
            max_val = abs(val);
        }
    }
    int y;
    static int pre_y = 0;
    for (int n = 0; n < 160; n++) {
        y = adcBuffer[n] + offset_val;
        y = map(y, -max_val, max_val, 0, 2000);

        oldy[n] = y;
    }

    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println(mic_counter);
    M5.Lcd.println(y);
    Serial.println(y);
    OscWiFi.send(pc_addr, pc_port, "/volume", y);

    if (pre_y == y) {
        M5.Lcd.println("not change");
        i2s_init();
    } else {
        M5.Lcd.println("    change");
    }
    pre_y = y;
    mic_counter++;
    now_mic = false;
}

void mic_task(void* arg) {
    while (1) {
        mic_record();
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

void print_wifi_state() {
    M5.Lcd.fillScreen(BLACK);  // clear LCD
    M5.Lcd.setTextColor(YELLOW);
    M5.Lcd.setCursor(3, 3);
    M5.Lcd.println("");
    M5.Lcd.println("WiFi connected.");
    M5.Lcd.print("IP address: ");
    M5.Lcd.println(WiFi.localIP());
    M5.Lcd.print("Port: ");
    M5.Lcd.println(PORT);
}

int counter = 0;
void control_task(const int index, const int angle) {
    Servo_pulse_set(index, angle);
    i2s_init();
}

void initWiFi() {
    WiFi.begin(SSID, PASS);
    esp_wifi_set_ps(WIFI_PS_NONE);
    while (WiFi.status() != WL_CONNECTED) {
        M5.Lcd.println("Now, WiFi Connecting..");
        delay(500);
    }
    M5.Lcd.println("WiFi Connected.");

    print_wifi_state();

    OscWiFi.subscribe(PORT, "/roll",
                      [](const OscMessage& m) {
                          const int servoIndex = 1;
                          const int servoAngle = m.arg<int>(0);
                          control_task(servoIndex, servoAngle);
                      });

    OscWiFi.subscribe(PORT, "/pitch",
                      [](const OscMessage& m) {
                          const int servoIndex = 2;
                          const int servoAngle = m.arg<int>(0);
                          control_task(servoIndex, servoAngle);
                      });

    OscWiFi.subscribe(PORT, "/yaw",
                      [](const OscMessage& m) {
                          const int servoIndex = 3;
                          const int servoAngle = m.arg<int>(0);
                          control_task(servoIndex, servoAngle);
                      });

    OscWiFi.subscribe(PORT, "/vertical",
                      [](const OscMessage& m) {
                          const int servoIndex = 4;
                          const int servoAngle = m.arg<int>(0);
                          control_task(servoIndex, servoAngle);
                      });

    OscWiFi.subscribe(PORT, "/eye",
                      [](const OscMessage& m) {
                          const int servoIndex = 5;
                          const int servoAngle = m.arg<int>(0);
                          control_task(servoIndex, servoAngle);
                      });

    OscWiFi.subscribe(PORT, "/mouth",
                      [](const OscMessage& m) {
                          const int servoIndex = 6;
                          const int servoAngle = m.arg<int>(0);
                          control_task(servoIndex, servoAngle);
                      });
}

void setup() {
    M5.begin();
    IIC_Servo_Init();
    initWiFi();
    M5.Lcd.setRotation(3);
    M5.Lcd.fillScreen(WHITE);
    M5.Lcd.setTextColor(BLACK, WHITE);
    M5.Lcd.println("mic test");
    i2s_init();
    xTaskCreate(mic_task, "mic_record_task", 2048, NULL, 1, NULL);
    delay(1000);
}

void loop() {
    if (M5.BtnA.wasPressed()) {
        esp_restart();
        M5.update();
    }

    OscWiFi.update();
    //i2s_read(I2S_NUM_0, (char*)BUFFER, READ_LEN, &bytesread, portMAX_DELAY);
}
