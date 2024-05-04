void loop_key_j() {
    static u8 flag=0;
    if((digitalRead(PIN_KEY_J)==LOW) && (flag==0)) {//第一次校准，握手值：在舒适的方式下最大程度的握紧手指，此时按下校准按钮
        delay(10);    
        if((digitalRead(PIN_KEY_J)==LOW) && (flag==0)){
            while(digitalRead(PIN_KEY_J)==LOW);
            adc_read2buf();
            for(int i=0;i<DWQ_NUM;i++) {
                ADC_MIN[i] = pos[i];
            }
            beep_on();delay(100);beep_off();
            eepromWrite(ADC_MIN_ADDR, sizeof(ADC_MIN), (u8 *)ADC_MIN);
            flag = 1; 

            sprintf(cmd_return,"ad0:%d ad1:%d ad2:%d ad3:%d ad4:%d ad5:%d ad6:%d ",
            pos_x, pos_y, pos[0], pos[1], pos[2], pos[3], pos[4]);
            Serial.println(cmd_return);  
        }
    } else if((digitalRead(PIN_KEY_J)==LOW) && (flag==1)) {//第二次校准，伸手值：在舒适的方式下最大程度的伸开手指，此时按下校准按钮
        delay(10);    
        if((digitalRead(PIN_KEY_J)==LOW ) && (flag==1)){
            while(digitalRead(PIN_KEY_J)==LOW);
            adc_read2buf();
            for(int i=0;i<DWQ_NUM;i++) {
                ADC_MAX[i] = pos[i];
            }
            for(int i=0;i<DWQ_NUM;i++) {
                ADC_MID[i] = (ADC_MIN[i] + ADC_MAX[i])/2;
                pos_p[i] = 1.0*JXB_PWM_RANGE/(abs(ADC_MIN[i] - ADC_MAX[i]));//放大倍数
            }
            beep_on();delay(500);beep_off();
            eepromWrite(ADC_MAX_ADDR,  sizeof(ADC_MAX), (u8 *)ADC_MAX);
            flag = 0;  

            sprintf(cmd_return,"ad0:%d ad1:%d ad2:%d ad3:%d ad4:%d ad5:%d ad6:%d ",
            pos_x, pos_y, pos[0], pos[1], pos[2], pos[3], pos[4]);
            Serial.println(cmd_return);          
        }    
    }
    
    if(digitalRead(PIN_KEY_M)==LOW) {//模式切换按钮
        delay(10);    
        if(digitalRead(PIN_KEY_M)==LOW){
            robot_mode++;
        }

        if(robot_mode==2)robot_mode=0;//最多2种模式
        for(int i=0;i<robot_mode+1;i++) {
            beep_on();delay(300);beep_off();delay(100);
        }
    }    
}
