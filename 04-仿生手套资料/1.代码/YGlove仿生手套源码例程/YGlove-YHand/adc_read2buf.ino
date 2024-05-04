void adc_read2buf() {
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
    }

    for(int i=0;i<5;i++) {
        myAdxl345.getAcceleration(&ax_t[i], &ay_t[i], &az_t[i]);
    }

    int FILTER_N = 5, filter_temp;
    for(int j = 0; j < FILTER_N - 1; j++) {
        for(int i = 0; i < FILTER_N - 1 - j; i++) {
          if(ax_t[i] > ax_t[i + 1]) {
            filter_temp = ax_t[i];
            ax_t[i] = ax_t[i + 1];
            ax_t[i + 1] = filter_temp;
          }

          if(ay_t[i] > ay_t[i + 1]) {
            filter_temp = ay_t[i];
            ay_t[i] = ay_t[i + 1];
            ay_t[i + 1] = filter_temp;
          }
        }
    }
    
    //display tab-separated accel x/y/z values
    //Serial.print("myAdxl345:\t");
    //Serial.print(ax); Serial.print("\t");
    //Serial.print(ay); Serial.print("\t");
    //Serial.println(az);
    
    pos_x = ay_t[2];
    pos_y = ax_t[2];

#if 0
      sprintf(cmd_return,"ax:%04d ay:%04d ad2:%04d ad3:%04d ad4:%04d ad5:%04d ad6:%04d ",
                          pos_x, pos_y, pos[0], pos[1], pos[2], pos[3], pos[4]);
      Serial.println(cmd_return);  
      delay(100);
#endif
}
