//库文件导入
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

#include <SimpleKalmanFilter.h>

//定义
#define DWQ_NUM     5               //宏定义电位器数量
#define ADC_MIN_ADDR   0     //握紧时采集电位器值起始地址
#define ADC_MAX_ADDR   30    //张手时采集电位器值起始地址
#define JXB_PWM_RANGE  1000.0 //量程
#define Straightening   2    //手指伸直时为2
#define bend            1    //手指弯曲时为1


byte dwq_pin[DWQ_NUM] = {A6, A7, A0, A2, A3};               //定义电位器引脚数组
int  ADC_MAX[DWQ_NUM] = {0, 0, 0, 0, 0};  //默认握紧时采集电位器值
int  ADC_MIN[DWQ_NUM] = {0, 0, 0, 0, 0};  //默认张手时采集电位器值
int  ADC_MID[DWQ_NUM] = {0, 0, 0, 0, 0};            //默认电位器中值
int  pos[DWQ_NUM] = {0}, pos_x=0, pos_y=0;          //变量pos用来存储转化后的电位器数据
int dwq_output[DWQ_NUM] = {0,0,0,0,0};              //电位器输出数据
float pos_p[DWQ_NUM] = {1,1,1,1,1};                 //放大倍数
float acc[3]={0,0,0};
float angle[3]={0,0,0};
float acc_estimated[3]={0,0,0};
float angle_estimated[3]={0,0,0};



//实例化
SoftwareSerial BT(A4, A5);
MPU6050 mpu6050(Wire);
SimpleKalmanFilter KalmanFilter(1, 1, 0.01);
//初始化
void setup() {
  Serial.begin(9600);  
  BT.begin(9600);  
  Serial.println("bluetooth is ready!");
  
  eepromRead(ADC_MIN_ADDR, sizeof(ADC_MIN), (u8 *)ADC_MIN);//读取存储电位器最小值
  eepromRead(ADC_MAX_ADDR, sizeof(ADC_MAX), (u8 *)ADC_MAX);//读取存储电位器最大值 
   for(int i=0;i<DWQ_NUM;i++) {
      ADC_MID[i] = (ADC_MIN[i] + ADC_MAX[i])/2;
      pos_p[i] = JXB_PWM_RANGE/abs(ADC_MIN[i] - ADC_MAX[i]);//放大倍数 
   } 

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}
//主循环
void loop() {
  BT.println("循环开始");
  read_DWQ();


  
}
