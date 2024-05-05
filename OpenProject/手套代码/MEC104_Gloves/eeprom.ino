/*
    读写eeprom数据
    addr 地址
    len 数据长度
    mydat 数据buf
*/
void eepromRead(u32 addr, u8 len, u8 *mydat){
    for(u8 i = 0; i < len; i++) {//求取旋钮平均值，存储到EEPROM
        mydat[i] = EEPROM.read(addr+i);
    }        
}

/*
    读写eeprom数据
    addr 地址
    len 数据长度
    mydat 数据buf
*/
void eepromWrite(u32 addr, u8 len, u8 *mydat){
    for(u8 i = 0; i < len; i++) {//求取旋钮平均值，存储到EEPROM
        EEPROM.write(addr+i, mydat[i]);
    }        
}//死亡不掉落
