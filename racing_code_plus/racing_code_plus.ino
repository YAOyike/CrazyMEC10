#include "Adafruit_NeoPixel.h"  //彩色灯珠驱动
#include "comm.h"               //传感器数据读取
#include "motor.h"              //电机控制

#define PIN            4
#define NUMPIXELS      2
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int para1[3] = {0}; 
// para1[0] = 50;  //直线冲刺赛
// para1[1] = 70;
// para1[2] = 300;


void setup() 
{
    shift_reg_init();   //传感器初始化
    motor_init();       //电机初始化
    pixels.begin();     //彩色灯珠初始化
  // para1[0] = 50;  //直线冲刺赛
  // para1[1] = 70;
  // para1[2] = 300;
    para1[0] = 100;  //拉力赛
    para1[1] = 150;
    para1[2] = 300;
}
void loop()                                                                             
{
  int motor_right = -255;
  int motor_left = -255; 
  char r0 = 0,r1 = 0,g0 = 0,g1 = 0,b0 = 0,b1 = 0;
  reload_shift_reg(); //刷新传感器数据

  if(sensor.ir_left_1){
    motor_left= motor_left + para1[0];
    r0 = r0 - 20;
  }
  if(sensor.ir_left_2){
    motor_left= motor_left + para1[1];
    r0 = r0 - 30;
  }
  if(sensor.ir_left_3){
    motor_left= motor_left + para1[2];
    r0 = r0 - 40;
  }
  if(sensor.ir_right_1){
    motor_right = motor_right + para1[0];
    r1 = r1 - 50;
  }
  if(sensor.ir_right_2){
    motor_right = motor_right + para1[1];
    r1 = r1 - 30;
  }
  if(sensor.ir_right_3){
    motor_right = motor_right + para1[2];
    r1 = r1 - 50;  
  }
  if(sensor.ir_mid){
    motor_left = -255;
    motor_right = -255;
    r0 = r0 + 50;
    r1 = r1 + 50;
  }
  if((sensor.ir_left_2 || sensor.ir_left_3) and (sensor.ir_right_2 || sensor.ir_right_3)){
    motor_left = 255;
    motor_right = -200;
  }

  pixels.setPixelColor(0, pixels.Color(r0,g0,b0)); //设定第一个灯珠颜色RGB(0~255)
  pixels.setPixelColor(1, pixels.Color(r1,g1,b1)); //设定第二个灯珠颜色
  pixels.show();  //显示设定好的颜色
  motor_set_PWM(motor_left,motor_right); //设定电机速度(左,右)(0~255)
  delay(10);
}
