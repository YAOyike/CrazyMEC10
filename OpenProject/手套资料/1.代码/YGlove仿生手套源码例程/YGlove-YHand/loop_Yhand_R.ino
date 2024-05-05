/*以固定格式发送数据,定时100毫秒发一次*/

void loop_Yhand_R(){    
    static u32 systick_ms_bak = 0, knob_value, curMode = 0;
    if(millis() - systick_ms_bak > 100) {
        systick_ms_bak = millis();
      
        //获取5个手指的数值
        adc_read2buf();

        //pwm控制
        for(int i=0;i<5;i++) {
            pwm_value[i] = (int)(1500+(pos[i]-ADC_MID[i])*pos_p[i]);
            
            if(abs(pos_bak[i] - pos[i]) >= 5) {
                pos_bak[i] = pos[i];
                
                if(pwm_value[i]>2500)pwm_value[i]=2500;
                else if(pwm_value[i]<500)pwm_value[i]=500;
                
                myservo[i+1].writeMicroseconds(pwm_value[i]); 
            }
        } 

        //手指数据进行更改
        if((pwm_value[0]>1800) &&(pwm_value[1]<1200) &&(pwm_value[2]<1200) &&(pwm_value[3]>1800) &&(pwm_value[4]>1800)) {//闭合手控制
             if ((pos_y > 150) || (pos_y < -150) || (pos_x > 150) || (pos_x < -150))
        {
             if (pos_x > 150) /* 右斜 */
                {
                    // 右转
                    sprintf(cmd_return,"{#000P0500T2000!#001P2300T0500!#002P0600T0500!#003P0600T0500!#004P2300T0500!#005P2300T0500!}");
                    Serial.println(cmd_return); 
                }
              else if (pos_x < -150) /* 左斜 */
                {
                    // 左转
                    sprintf(cmd_return,"{#000P2400T2000!#001P2300T0500!#002P0600T0500!#003P0600T0500!#004P2300T0500!#005P2300T0500!}");
                    Serial.println(cmd_return); 
                }
        }else
            {
                sprintf(cmd_return,"$DST!");
                 Serial.println(cmd_return); 
                }
        } else  if((pwm_value[0]<1200) &&(pwm_value[1]>1800) &&(pwm_value[2]>1800) &&(pwm_value[3]<1200) &&(pwm_value[4]<1200) 
        ) {//张开手控制
                         if ((pos_y > 150) || (pos_y < -150) || (pos_x > 150) || (pos_x < -150))
        {
             if (pos_x > 150) /* 右斜 */
                {
                    // 右转
                     sprintf(cmd_return,"{#000P0500T2000!#001P0600T0500!#002P2300T0500!#003P2300T0500!#004P0600T0500!#005P0600T0500!}");
                    Serial.println(cmd_return);
                }
              else if (pos_x < -150) /* 左斜 */
                {
                    // 左转
                    sprintf(cmd_return,"{#000P2400T2000!#001P0600T0500!#002P2300T0500!#003P2300T0500!#004P0600T0500!#005P0600T0500!}");
                    Serial.println(cmd_return); 
                } 
        } else
            {
                sprintf(cmd_return,"$DST!");
                 Serial.println(cmd_return); 
                }
            
        } else {//手指控制
            sprintf(cmd_return,"{#001P%04dT0100!#002P%04dT0100!#003P%04dT0100!#004P%04dT0100!#005P%04dT0100!}", pwm_value[0],pwm_value[1],pwm_value[2],pwm_value[3],pwm_value[4]);
            Serial.println(cmd_return);  
        }

        //陀螺仪数据控制板载PWM
        myservo[0].writeMicroseconds(pos_x); 
        
    
    }
    return;
}                                                       
