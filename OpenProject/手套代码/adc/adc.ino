#include <Adafruit_ADXL345_U.h>

// 定义模拟输入引脚和ADXL345对象
const int dwq_pin[5] = {A0, A1, A2, A3, A4};
Adafruit_ADXL345_Unified myAdxl345 = Adafruit_ADXL345_Unified(12345);

// 定义全局变量
static u32 knob_value;
static float ax_t[5], ay_t[5], az_t[5];
static int pos[5];
static float pos_x, pos_y, pos_z;

void setup() {
  Serial.begin(9600);

  // 初始化ADXL345
  if (!myAdxl345.begin()) {
    Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
    while (1);
  }
}

void adc_read2buf() {
  for (int i = 0; i < 5; i++) {
    knob_value = 0;
    for (int j = 0; j < 10; j++) {
      knob_value += analogRead(dwq_pin[i]);
      delayMicroseconds(100);
    }
    knob_value /= 10.0;
    pos[i] = knob_value;
  }

  for (int i = 0; i < 5; i++) {
    sensors_event_t event;
    myAdxl345.getEvent(&event);
    ax_t[i] = event.acceleration.x;
    ay_t[i] = event.acceleration.y;
    az_t[i] = event.acceleration.z;
  }

  int FILTER_N = 5, filter_temp;
  for (int j = 0; j < FILTER_N - 1; j++) {
    for (int i = 0; i < FILTER_N - 1 - j; i++) {
      if (ax_t[i] > ax_t[i + 1]) {
        filter_temp = ax_t[i];
        ax_t[i] = ax_t[i + 1];
        ax_t[i + 1] = filter_temp;
      }

      if (ay_t[i] > ay_t[i + 1]) {
        filter_temp = ay_t[i];
        ay_t[i] = ay_t[i + 1];
        ay_t[i + 1] = filter_temp;
      }
      if (az_t[i] > az_t[i + 1]) {
        filter_temp = az_t[i];
        az_t[i] = az_t[i + 1];
        az_t[i + 1] = filter_temp;
      }
    }
  }

  pos_x = ax_t[2];
  pos_y = ay_t[2];
  pos_z = az_t[2];

  Serial.print("ax: ");
  Serial.print(pos_x);
  Serial.print("  ay: ");
  Serial.print(pos_y);
  Serial.print("  az: ");
  Serial.print(pos_z);


  delay(100);
}

void loop() {
  adc_read2buf();
}
