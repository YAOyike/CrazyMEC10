void read_DWQ() {
    static u32 knob_value;
    static int16_t ax_t[5],ay_t[5],az_t[5];
    for(int i=0;i<5;i++) {
        knob_value = 0;
        for(int j=0;j<10;j++) {
            knob_value+=analogRead(dwq_pin[i]);
            delayMicroseconds(100);
         }
         knob_value = knob_value/10.0;
         pos[i] = knob_value;
    } //读取电位计的模拟值，取连续10个值，最后求平均
}

void judgestate(){
  if(pos[0]<ADC_MID[0] && pos[1]<ADC_MID[1] && pos[2]<ADC_MID[2] && pos[3]<ADC_MID[3] && pos[4]<ADC_MID[4]){
    //进入张手模式，yyk负责
    
    
    }
  else if(pos[0]<ADC_MID[0] && pos[1]<ADC_MID[1] && pos[2]<ADC_MID[2] && pos[3]<ADC_MID[3] && pos[4]<ADC_MID[4]){
    //握拳模式
    Serial.println("握拳模式");
    dwq_output[0] = 1;
    dwq_output[1] = 1;
    dwq_output[2] = 1;
    dwq_output[3] = 1;
    dwq_output[4] = 1;
    }
  else if(pos[0]>ADC_MID[0] && pos[1]<ADC_MID[1] && pos[2]<ADC_MID[2] && pos[3]<ADC_MID[3] && pos[4]<ADC_MID[4]){
    //抬起拇指
    Serial.println("抬起拇指");
    dwq_output[0] = 2;
    dwq_output[1] = 1;
    dwq_output[2] = 1;
    dwq_output[3] = 1;
    dwq_output[4] = 1;
    }
  else if(pos[0]<ADC_MID[0] && pos[1]>ADC_MID[1] && pos[2]<ADC_MID[2] && pos[3]<ADC_MID[3] && pos[4]<ADC_MID[4]){
    //抬起食指
    Serial.println("抬起食指");
    dwq_output[0] = 1;
    dwq_output[1] = 2;
    dwq_output[2] = 1;
    dwq_output[3] = 1;
    dwq_output[4] = 1;
    }
  else if(pos[0]<ADC_MID[0] && pos[1]<ADC_MID[1] && pos[2]>ADC_MID[2] && pos[3]<ADC_MID[3] && pos[4]<ADC_MID[4]){
    //抬起中指
    Serial.println("抬起中指");
    dwq_output[0] = 1;
    dwq_output[1] = 1;
    dwq_output[2] = 2;
    dwq_output[3] = 1;
    dwq_output[4] = 1;
    }
   else{
    Serial.println("其他情况");
    }
}
