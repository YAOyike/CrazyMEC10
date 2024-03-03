/****************************************************************************
文件名称：DreArm-Yi_Arduino_Code.ino
修改时间：2023-09-13
项目版本：V1.0.20211106
作    者：上海俊士科技有限公司（Drealize Co.,Ltd）
****************************************************************************/

#include <Wire.h>                         //Call the library file Wire library, this is the I2C communication library, is the Arduino official library file
#include <SoftwareSerial.h>               //Call the library file SoftwareSerial, this is the soft serial port library, is the Arduino official library file
#include "Adafruit_MotorShield.h"         //Call the library file Adafruit_MotorShield, this is the servo extension board library
#include "Adafruit_MS_PWMServoDriver.h"   //Call the library file Adafruit_MS_PWMServoDriver, this is the servo extension board PWM driver library
#include "PS2X_lib.h"                     //Call the library PS2X_lib, which is the wireless handle library
#include "BTJoystick.h"                   //Call the library file BTJoystick, this is the Bluetooth app joystick library

#define PS2_SEL 10  //Define PS2 controller pin 10
#define PS2_CMD 11  //Define PS2 controller pin 11
#define PS2_DAT 12  //Define PS2 controller pin 12
#define PS2_CLK 13  //Define PS2 controller pin 13

#define pressures   true    //Enable the handle function
#define rumble      true    //Enable the handle function
SoftwareSerial  softSerial( 2, 3 );       //Bluetooth communication module soft serial port pin: Bluetooth Rx → arduino3#, Bluetooth Tx → arduino2#
/***********Currently, the library does not support hot-swappable handle receivers, otherwise the control board must be restarted***********/
PS2X ps2x;                     //Define the PS2 controller
BTJoystick ps2x1 (softSerial);  //Define Bluetooth controller

int error = 0;      //Define variable error
byte type = 0;      //Define variable form
byte vibrate = 0;   //Defined variable vibration

Adafruit_MotorShield AFMS = Adafruit_MotorShield();  //Defines the actuator extension board master name

const int SERVOS = 4;       //Define the variable SERVOS, representing the number of servos, with a pre-assigned value of 4
const int ACC = 10;         //Defines the variable ACC, which is used to eliminate potentiometer errors
int PIN[SERVOS];            //Define the variable array PIN[SERVOS], which represents the PWM pin number of the steering gear
int value[SERVOS];          //Defines the variable array value[SERVOS] to store the analog value of the wireless controller
int Bvalue[SERVOS];         //Defines the variable array Bvalue[SERVOS] to store the analog value of the Bluetooth app handle
int idle[SERVOS];           //Define variable free
int currentAngle[SERVOS];   //Define the variable array currentAngle[SERVOS] to store the current steering gear Angle values
int MIN[SERVOS];            //The variable array MIN[SERVOS] is defined to store the minimum Angle value of the current steering gear
int MAX[SERVOS];            //Defines the variable array MAX[SERVOS] to store the maximum Angle value of the current steering gear
int INITANGLE[SERVOS];      //Defines the variable array INITANGLE[SERVOS] to store the initial Angle value of the current steering gear
int previousAngle[SERVOS];  //Define variable array previousAngle[SERVOS] to store the front Angle value of the current steering gear
int actionIndex = 0;  //Run step pointer
int Totalact = 0;  //Defines the total number of recording points
int Led1 = A0;  //Define the RGB indicator R pin to connect to the analog A0 port
int Led2 = A1;  //Define the RGB indicator G pin to the analog A1 port
int Led3 = A2;  //Define the RGB indicator B pin to connect to the analog A2 port
int Led4 = A3;  //Define the GND pin of the RGB indicator to connect to the analog A3 port
int Pressbuf[10];    //Define the key press buf
int Releasebuf[10];  //Define the key to release buf
int BTenable;  //Removes the Bug that Bluetooth is not connected
int BTbuf;     //Removes the Bluetooth unconnected Bug that causes abnormal action
int Mode;  //Define variable modes, 0: manual mode 1: record 2: auto play
unsigned long ms;     //Define millisecond reading
unsigned long t[10];  //Define t for generating the timer
int autoplaying = false;  //Define the automatic run state
int autotrig =false;  //Define an autoplay rising edge
int autotrigbuf =false;  //Defines autoplay rising along buf
int autoindex = 0;  //Define the autoplay pointer
int indexbuf = 0;  //Define an autoplay pointer cache
int const maxAutoActions = 20;  //Define a maximum automatic data record value of 30, with an initial value of 20 points due to limited Uno memory
int actdelay = 300;  //Automatic play action interval 300 ms, can be modified interval
int Actionmem[maxAutoActions][4] ={{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};  //Define a cache for recording coordinate points
String inputString = "";         //Define PC serial port data
bool stringComplete = false;  //Defines the serial port receiving flag
int stringlength = 0;  //Define string length
String PCcmd[20];  //Defines PC instruction characters
String PCmove[4];  //Define the PC mode move instruction
int PCact[20];  //Defines PC mode button actions
int Serialindex=0;  //Define serial port pointer
String RcvBuf[4];  //Define the received Angle value String
int Rcvint[4];  //Defines the receive Angle value Int

// Declaration function: Set the frequency of the steering gear
void setServoPulse( uint8_t n, double pulse )  //n:Steering gear number; pulse: pulse time
{
    double pulselength;
    pulselength = 1000000;      // A millionth of a second
    pulselength /= 50;          //50 Hz
    pulselength /= 4096;        //(12-bit resolution)
    pulse *= 1000;              //Multiply by 1000
    pulse /= pulselength;       //Divided by the pulselength variable
    AFMS.setPWM( n, pulse );    //Set the PWM value
}

// Declare function: Convert PWM value to Angle
void writeServo( uint8_t n, uint8_t angle )  //n:Steering gear number; angle: The Angle that the steering gear needs to turn to
{
    double pulse;
    angle = (int)angle*0.9;  //According to the different batches of steering gear, there is a coefficient relationship of 0.9 between the actual Angle of steering gear and the theoretical input value. If the actual rotation Angle of the steering gear is the same as the theoretical input value, this statement can be deleted
    pulse = 0.5 + angle / 90.0;  //The relation formula of pulse time and Angle value
    setServoPulse( n, pulse );  //Let the specified steering gear turn to the specified Angle through the corresponding pulse time
}

void setup()
{
    Serial.begin( 57600 );  //Serial port communication baud 57600
    softSerial.begin(9600); //Soft serial port communication Baud rate 9600 is enabled. This setting must be consistent with the baud rate of the serial port on the Bluetooth module
    softSerial.listen();    //Enable soft serial port data monitoring
    delay( 2000 );          //Delay 2 seconds for PS2 controller initialization
    Serial.print( "Search Controller.." );  //Serial port print "Search Handle"
    do
    {
        error = ps2x.config_gamepad( PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble );  //This function configs the initial communication capability of the wireless controller and returns 0 if ok
        if( error == 0 )
        {
            Serial.println( "\nConfigured successful " );  //successfu lconnection
            break;  //Interrupt, jump out of loop function do... while
        }
        else  //if not

        {
            Serial.print( "." );  //Wait for connection, at this time every 0.1 seconds the serial port output '.' for prompt function 
            delay( 100 );
        }
    }while( 1 );  //while loop until the handle connection is successful
    type = ps2x.readType();   //This function determines the type of connection controller，0/1：Wireless DualShock Controller found; 2: Unknown Controller; 
    switch( type )  //According to the connection handle read mode, the serial port output model
    {
        case 0:
            Serial.println( "Wireless DualShock Controller found " );  //wireless controller
            break;
        case 1:
            Serial.println( "Wireless DualShock Controller found " );  //wireless controller
            break;
        case 2:
            Serial.println( "Unknown Controller type found " );  // Unknown controller
            break;
    }
    ps2x.read_gamepad( true, 200 );  //Vibrate 0.2 seconds at startup to indicate that the handle is connected successfully
    delay( 500 );
    pinMode(Led1,OUTPUT);  //Define LED lamp pin as output
    pinMode(Led2,OUTPUT);  //Define LED lamp pin as output
    pinMode(Led3,OUTPUT);  //Define LED lamp pin as output
    pinMode(Led4,OUTPUT);  //Define LED lamp pin as output
    digitalWrite(Led1,LOW);  //LED light initialization pin R
    digitalWrite(Led2,LOW);  //LED light initialization pin G
    digitalWrite(Led3,LOW);  //LED light initialization pin B
    digitalWrite(Led4,LOW);  //LED light initialization pin GND
    int Mode = 0;             //Define schema code
    int BTenable = false;     //Define Bluetooth enablement
    int BTbuf = false;        //Bluetooth relay
    init_Pins();              //The four axes are initialized as setpoints, and the function being called is on line 182
    
    AFMS.begin( 50 ); //Open the function Settings for the manipulator library
    for ( int i = 0; i < SERVOS; i ++ )  //For loop initialization assignment set Angle (start 4 servos back to origin)
    {
        value[i] = 0;
        idle[i] = 0;
        previousAngle[i] = INITANGLE[i];
        currentAngle[i] = INITANGLE[i];
        writeServo( PIN[i], INITANGLE[i] );
    }
}

void loop()
{  
        ps2x.read_gamepad( false, vibrate );    //shake
        ps2x1.readCommand();                    //Bluetooth handle read enabled
        vibrate = ps2x.Analog( PSAB_CROSS );    //Press the 'A' handle vibration function
           
        Control();            //Steering gear control function, the function is located on line 220        
        ReadRockerValue();    //Read the PS handle joystick value, the function is located in line 243
        ReadBtRockerValue();  //Read the Bluetooth handle joystick value, the function is located in line 252
        //ShowRockerValue();    //Debugging function, display the joystick value, the function is 274 lines
        BTonline();           //To verify whether the Bluetooth controller is online, the function is located on line 263
        ModeChange();         //Change mode. The function is located on line 286
        ShowAngle();          //The serial port reads the current four-axis Angle value (debugging), and the function is located on line 316
        MemoryAction();       //The action is recorded. The function is on line 340
        Totalactcnt();        //Currently record the number of actions, 373 lines where the function resides
        Autoplay();           //Autoplay function, 380 lines where the function is located
        Autodelete();         //Delete the record point, line 416 where the function is located
        PCmode();             //PC control mode, line 439 where the function is located
    delay( 10 );
}

void init_Pins(){  //Function 1: actuator initialization parameter function
    /*00:Claw head steering gear parameters*/
    PIN[0] = 0;  //The claw head steering gear corresponds to the steering gear plate insert number
    MIN[0] = 0;  //Minimum Angle limit of claw head steering gear
    MAX[0] = 180;  //Max. Angle limit of claw head steering gear
    INITANGLE[0] = 78;  //Claw head steering gear origin Angle
    /*01:Forearm steering gear parameters*/
    PIN[1] = 1;  //The lower arm steering gear corresponds to the steering gear plate tag
    MIN[1] = 45;  //Minimum Angle limit of forearm steering gear
    MAX[1] = 105;  //Maximum Angle limit of forearm steering gear
    INITANGLE[1] = 90;  //Lower arm steering gear origin Angle
    /*02:Big arm steering gear parameters*/
    PIN[2] = 2;  //The big arm steering gear corresponds to the steering gear plate tag
    MIN[2] = 40;  //Minimum Angle limit of big arm steering gear
    MAX[2] = 135;  //Maximum Angle limit of big arm steering gear
    INITANGLE[2] = 90;  //Big arm steering gear origin Angle
    /*03:Parameters of turntable steering gear*/
    PIN[3] = 3;  //Turntable steering gear corresponding to the steering plate tag
    MIN[3] = 0;  //Minimum Angle limit of turntable steering gear   
    MAX[3] = 180;  //Maximum Angle limit of turntable steering gear 
    INITANGLE[3] =90;  //Turntable steering gear origin Angle
}

void Home()  //Function 2: Press the Left key to return to the origin function 
{
  if (ps2x.Button(PSB_PAD_LEFT)|| ps2x1.Button(BTB_PAD_LEFT)||(PCact[12]==1))  //Press the Left button of the wireless controller, the Left button of the upper computer of the mobile app, and enter "LEFT" through the serial port to return to the origin
  {
    for ( int i = 0; i < SERVOS; i ++ )  //The For loop initialization assignment sets the Angle
    {
      //If the current Angle of the current i# steering gear is less than the initial Angle, the current Angle is increased by 1 degree every 0.005 seconds until the current Angle is equal to the initial Angle, and the cycle is terminated
      while ( currentAngle[i] < INITANGLE[i] )  //If the current Angle is less than the initial Angle
      {
        currentAngle[i] += 1;  //The current Angle increases by 1 degree
        writeServo( PIN[i], currentAngle[i] );  //And turn the current i# steering gear to the current Angle
        delay(5);  //The current Angle increases by 1 degree every 0.005 seconds. This parameter can control the rotation speed of the steering gear
      }
     
     //If the current Angle of the current i# steering gear is greater than the initial Angle, the current Angle decreases by 1 degree every 0.005 seconds until the current Angle is equal to the initial Angle, and the cycle is terminated
      while ( currentAngle[i] > INITANGLE[i] )  //If the current Angle is greater than the initial Angle
      {
        currentAngle[i] -= 1;  //The current Angle increases by 1 degree
        writeServo( PIN[i], currentAngle[i] );  //And turn the current i# steering gear to the current Angle
        delay(5);  //The current Angle increases by 1 degree every 0.005 seconds. This parameter can control the rotation speed of the steering gear
      }
    }
    PCact[12]=0;  //Because PCact[12] is assigned a value of 1 after entering "LEFT" in the serial port, which causes the steering gear to turn to the initial Angle, it is necessary to assign a value of 0 to PCact[12] after the steering gear is turned
  }
}

void Control()  //Function 3: steering gear control function
{
  if(Mode<2)  //Manual control in non-auto play mode
  {
    Home();  //back to the origin

    for ( int i = 0; i < SERVOS; i ++ )  //4 rotation functions For steering gear 0-3
    {   
      if ( (value[i] > 150)||((BTenable==true)&&(Bvalue[i] > 200))||(PCact[i]==1))  //If the analog value of the wireless joystick is >150, or the analog value of the App virtual joystick is >200, or the value of the virtual joystick of the computer upper computer is 1, the Angle increases
      {
        if ( currentAngle[i] < MAX[i] )  //If the current Angle does not reach the set maximum Angle value
        currentAngle[i] += 1;  //The current Angle is incremented by 1 degree
        writeServo( PIN[i], currentAngle[i] );  //Write the output pin to turn the current i# steering gear to the current Angle
      }
        else if ((value[i] < 100)||((BTenable==true)&&(Bvalue[i] < 60))||(PCact[i]==2))  //If the analog value of the wireless joystick is <100, or the Bluetooth function is available and the analog value of the App virtual joystick is <60, or the value of the virtual joystick of the computer upper computer is 2, the Angle decreases
      {
        if ( currentAngle[i] > MIN[i] )  //If the current Angle does not reach the set minimum Angle value
        currentAngle[i] -= 1;  //The current Angle decreases by 1 degree
        writeServo( PIN[i], currentAngle[i] );  //Write the output pin to turn the current i# steering gear to the current Angle
      }        
     }
   } 
}

void ReadRockerValue()  //Function 4: Read the wireless joystick value
{
value[0] = ps2x.Analog( PSS_LX );  //Read the left joystick transverse analog value
value[1] = ps2x.Analog( PSS_RY );  //Read the right joystick longitudinal analog value
value[2] = ps2x.Analog( PSS_LY );  //Read the left rocker longitudinal analog value
value[3] = ps2x.Analog( PSS_RX );  //Read the right joystick transverse analog value
value[3] = 255-value[3];
}

void ReadBtRockerValue()  //函数5：Read the Bluetooth joystick value
{
Bvalue[0] = ps2x1.Analog( PSS_LXX );  //Read the left joystick transverse analog value
Bvalue[1] = ps2x1.Analog( PSS_RYY ); // Read the longitudinal analog value of the right joystick
Bvalue[2] = ps2x1.Analog( PSS_LYY ); // Read the longitudinal analog value of the left rocker
Bvalue[3] = ps2x1.Analog( PSS_RXX ); // Read the right joystick transverse analog value
Bvalue[1] = 255-Bvalue[1];  //Reverse processing
Bvalue[2] = 255-Bvalue[2]; // Reverse processing
Bvalue[3] = 255-Bvalue[3]; // Reverse processing

}

void BTonline()  //Function 6: Check that the Bluetooth controller is online
{
  if(BTbuf==false)  //If the Bluetooth is not connected to the Bug cause abnormal action
  {
    BTenable = true;  //Bluetooth function available
    for ( int i = 0; i < 4; i ++ )  //The for loop checks that the Bluetooth controller is initialized successfully
    {
      if(Bvalue[i]<100||Bvalue[i]>150)  //if the value of the current virtual joystick of the Bluetooth App is <100 or >150
      {BTenable=false;}  //Bluetooth is unavailable
      else  //if not
      {BTbuf = true;BTenable=true;}  //The Bluetooth not connected Bug causes the action to be normal and allows the Bluetooth function to be available
    }
  }
}

void ModeChange()  //Function 7: Press UP mode to change
{
  if (ps2x.Button(PSB_PAD_UP)||ps2x1.Button(BTB_PAD_UP)||(PCact[10]==1))  //If you press the UP key of the wireless controller, or press the UP key of the Bluetooth App, or the PC PC [10] status is 1
  {
    if(Pressbuf[1] == false)  //If you press buf to 0
    {
      if(Mode<2) //If the mode is less than 2, it is manual control mode or action recording mode
      {Mode = Mode + 1;}  //The manual control mode changes to action recording mode, or the action recording mode changes to automatic playback mode
      else  //If the mode is auto play mode
      {Mode = 0;}  //The auto play mode is changed to manual control mode
      PCact[10]=0;  //PCact[10]赋值0
      Serial.print("pattern");  //Mode is printed on the serial port monitor
      Serial.print(Mode);  //Mode is printed on the serial port monitor
      if(Mode==0){Serial.println("Manual control mode");}  //If manual control mode is used, Manual Control Mode is printed on the serial port monitor
      if(Mode==1){Serial.println("Motion recording mode");}  //If action recording mode is used, Action Recording Mode is displayed on the serial port monitor
      if(Mode==2){Serial.println("Autoplay mode");}  //If it is in auto play mode, the message "Auto Play Mode" is printed on the serial port monitor
      Pressbuf[1] = true;  //Press buf to assign a value of 1
      Releasebuf[1] = false;  //Releasebuf is assigned a value of 0
     }  
  }
  if (!ps2x.Button(PSB_PAD_UP)&&!ps2x1.Button(BTB_PAD_UP))  //If you do not press the UP key of the wireless controller, and do not press the UP key of the Bluetooth App
  {
    if(Releasebuf[1] == false)  //if Releasebuf value 0
    {
      Pressbuf[1]=false;  //Press buf to assign a value of 0
      Releasebuf[1]=true;  //ReleasebufAssign 1
    }
  }
  //Follow the RBG indicator light to the color display pin Settings,001 blue light 110 yellow 100 red 010 green
  if(Mode==0){digitalWrite(Led1,LOW);digitalWrite(Led2,LOW);digitalWrite(Led3,HIGH);}  //If the mode is manual control, the blue light is on
  if((Mode==1)&&((!(ps2x.Button(PSB_CIRCLE)&&!ps2x1.Button(BTB_B))&&!(PCact[14]==1))&&(actionIndex<=maxAutoActions))){digitalWrite(Led1,HIGH);digitalWrite(Led2,HIGH);digitalWrite(Led3,LOW);}  //If the mode is action recording mode, and the wireless controller B key is not pressed, or the Bluetooth App B key is not pressed, or the PC upper computer does not set the PCact[14] value to 1, and the number of motion steps actionIndex< the maximum number of steps that can be automatically run, the yellow light is on
  if((Mode==1)&&(ps2x.Button(PSB_CIRCLE)||ps2x1.Button(BTB_B)||(PCact[14]==1))&&(actionIndex<=maxAutoActions)){digitalWrite(Led1,HIGH);digitalWrite(Led2,LOW);digitalWrite(Led3,LOW);}  ////If the mode is action recording mode, and the wireless controller B key is pressed, or the Bluetooth App B key is pressed, or the PC upper computer sets the PCact[14] value to 1, and the number of movement steps actionIndex< the maximum number of steps that can be automatically run, the red light is on
  if(Mode==2){digitalWrite(Led1,LOW);digitalWrite(Led2,HIGH);digitalWrite(Led3,LOW);}  //If the mode is Auto play, the green light is on     
}

void ShowAngle()  //Function 8: Press Down to read the current coordinate value
{
  if (ps2x.Button(PSB_PAD_DOWN)||ps2x1.Button(BTB_PAD_DOWN)||(PCact[11]==1))  //If you press the DOWN key of the wireless controller, or press the DOWN key of the Bluetooth App, or the PC PC [11] status is 1
  {
      if(Pressbuf[4] == false)  //If you press buf to 0
      {
        Serial.print( "a:" );  //a: is printed on the serial port monitor
        Serial.print( currentAngle[0] );  //Serial port monitor printed when the front claw head steering gear Angle value
        Serial.print( ",b:" );  //"b:" is printed on the serial port monitor
        Serial.print( currentAngle[1] );  //The current Angle value of the lower arm steering gear is printed in the serial port monitor
        Serial.print( ",c:" );  //"c:" is printed on the serial port monitor
        Serial.print( currentAngle[2] );  //The current Angle value of the big arm steering gear is printed in the serial port monitor
        Serial.print( ",d:" );  //d: is printed on the serial port monitor
        Serial.print( currentAngle[3] );  //The current turntable steering gear Angle value is printed in the serial port monitor
        Serial.println();  //Line feed in the serial port monitor
        PCact[11]=0;  //PCact[11]valuation 0
        Pressbuf[4] = true;  //Press buf to assign a value of 1
        Releasebuf[4] = false;  //Releasebuf valuation 0
      }
  }
  if(!(ps2x.Button(PSB_PAD_DOWN)||ps2x1.Button(BTB_PAD_DOWN)||(PCact[11]==1)))  //If you do not press the DOWN key of the wireless controller or the DOWN key of the Bluetooth App, the PC PC [11] status is not 1
  {
    Pressbuf[4] = false;  //Press buf to assign a value of 0
    Releasebuf[4] = true;  //Releasebuf valuation 1
  }
}

void MemoryAction()  //Function 9: Automatic recording mode, press ○ or Bluetooth B recording point
{
   if(Mode==1)  //If it is in recording mode
   {
     if (ps2x.Button(PSB_CIRCLE)||ps2x1.Button(BTB_B)||(PCact[14]==1))  //If you press the B key of the wireless controller, or press the B key of the Blueto
     {
       if(Pressbuf[2] == false)  //If you press buf to 0
       {
         if(actionIndex<=maxAutoActions)  //If actionIndex<= Maximum number of steps that can be automatically run
         {
           Actionmem[actionIndex][0]=currentAngle[0];  //Record the current Angle value of the current number of steps of the claw head
           Actionmem[actionIndex][1]=currentAngle[1];  //Record the current Angle value of the current number of steps of the forearm
           Actionmem[actionIndex][2]=currentAngle[2];  //Record the current Angle value of the current number of steps of the big arm
           Actionmem[actionIndex][3]=currentAngle[3];  //Records the current Angle value of the current number of steps of the turntable
           Serial.print("动作记录");  //Action Log is printed on the serial port monitor
           Serial.print(actionIndex);
           Serial.print(":");  //":" is printed on the serial port monitor
           Serial.print(Actionmem[actionIndex][0]);  //The current Angle value of the current number of steps of the claw head is printed in the serial port monitor
           Serial.print(",");  //"," is printed in the serial port monitor
           Serial.print(Actionmem[actionIndex][1]);  //The current Angle value of the current steps of the forearm is printed in the serial port monitor
           Serial.print(",");  //"," is printed in the serial port monitor
           Serial.print(Actionmem[actionIndex][2]);  //The current Angle value of the current number of steps of the arm is printed in the serial port monitor
           Serial.print(",");  //"," is printed in the serial port monitor
           Serial.println(Actionmem[actionIndex][3]);  //The current Angle value of the current number of steps of the turntable is printed in the serial monitor
           if(actionIndex<=maxAutoActions)  //If actionIndex<= Maximum number of steps that can be automatically run
           {
             actionIndex++;  //The number of movement steps actionIndex is self-additive1
           }
         }
         PCact[14]=0;  //PCact[14] assigns a value of 0
         Pressbuf[2] = true;  //Press buf to assign a value of 1
         Releasebuf[2] = false;  //Releasebuf value 0
       }
    }
    if((!ps2x.Button(PSB_CIRCLE))&&(!ps2x1.Button(BTB_B))&&(!(PCact[14]==1)))  //If the B key of the wireless controller is not pressed, or the B key of the Bluetooth App is not pressed, or the PCact[14] status of the upper computer is not 1  
    {
      Pressbuf[2] = false;  //Press buf to assign a value of 0
      Releasebuf[2] = true;  //Releasebuf value 1
    }
  }    
}

void Totalactcnt()  //Function 10: Records the total number of steps currently recorded
{
  for ( int i = 0; i < maxAutoActions; i ++ )  //The for loop checks whether the current number of steps is less than the maximum number of steps that can be automatically run
  {
    if( Actionmem[i][0]!=0)  //If the current step of the current record actuator is not 0
    {
      Totalact=i;  //The total number of steps is the current number
    }
  }
}

void Autoplay()  //Function 11: Auto play mode, press △ or Bluetooth A to start auto play, press X or Bluetooth C to end auto play
{
  ms = millis();  //The return time from arduino power on to the present, in ms
  if(Mode!=2)  //If not in autoplay mode
  {
    autoplaying=false;  //Autoplay disable
  }
  if(Mode==2&&Totalact!=0)  //If the automatic play mode is used, the total number of steps is not 0
  {
    if ((ps2x.Button(PSB_TRIANGLE)||ps2x1.ButtonPressed(BTB_A)||(PCact[13]==1))&&autoplaying==false)  //If you press the Y key on the wireless controller, or press the A key of the Bluetooth App, or the PC PC [13] status is not 1, and the automatic playback is prohibited
    {
      autoplaying=true;  //At this point, automatic playback will be available
      Serial.println("自动运行开始");  //The message "Auto Start" is displayed on the serial port monitor
      PCact[13]=0;  //PCact[13] value 0
    }
    if ((ps2x.Button(PSB_CROSS)||ps2x1.ButtonPressed(BTB_C)||(PCact[15]==1))&&autoplaying==true)  //If the wireless controller A key is not pressed, or the Bluetooth App C key is not pressed, or the PC PC [15] status is not 1, and the automatic playback is available  
    {
      autoplaying=false;  //In this case, auto play is disabled
      Serial.println("自动运行停止");  //The message "Auto Stop" is printed on the serial port monitor

      PCact[15]=0;  //PCact[15] value 0
    }
  }
  if(!autoplaying)  //If autoplay is disabled
  {
    autotrigbuf=true;  //Auto play rising edge buf value 1
    autotrig=false;  //Autoplay rising edge is assigned a value of 0
  }
  if(autoplaying&&autotrigbuf)  //If autoplay is available, and autoplay rises along the buf of 1
  {
    autotrig=true;  //Autoplay rising edge is assigned a value of 1
    autotrigbuf=false;  //Autoplay rising edge is assigned a value of 0
  }
  if(autotrig)  //Autoplay rising edge is assigned a value of 1
  {
    autoindex=0;  //Autoplay rising edge is assigned a value of 0
    autotrig=false;  //Autoplay rising edge is assigned a value of 0
  }
  if(autoplaying)  //If autoplay is available
  {
    writeServo( PIN[0], Actionmem[autoindex][0] );  //Write the current Angle value of the current number of steps to the claw head steering gear and turn the steering gear to this Angle
    writeServo( PIN[1], Actionmem[autoindex][1] );  //Write the current Angle value of the current number of steps to the forearm steering gear and make the steering gear turn to this Angle
    writeServo( PIN[2], Actionmem[autoindex][2] );  //Write the current Angle value of the current number of steps to the big arm steering gear and make the steering gear turn to this Angle
    writeServo( PIN[3], Actionmem[autoindex][3] );  //Write the current Angle value of the current number of steps to the turntable steering gear and make the steering gear turn to this Angle
    if(autoindex!=indexbuf)  //If the current number of steps is not equal to the auto-play pointer cache
    {
      t[0]=ms;  //Record the total time since the Arduino power on to the present, in ms
      indexbuf=autoindex;  //Assigns the current number of steps to the autoplay pointer cache
    }  
    t[1]=ms-t[0];  //Then, the total time from the current power on Arduino to the present, minus the difference of the previous t]0] time, that is, the difference of the time from the previous record time to the current record time, is assigned to t[1]
    if(t[1]>actdelay)  //If this t[1] time >actdelay(autoplay action interval 300 ms) time
    {
      if(autoindex<Totalact+1)  //If this t[1] time >actdelay(autoplay action interval 300 ms) time
      {
        autoindex=autoindex+1;  //The current number of steps adds itself1
      }
    }
    if(autoindex==Totalact+1)  //If the current number of steps = total number of steps+1
    {
      autoindex=0;  //The current step is set to 0
    }
  }
}

void Autodelete()  //Function 12: Press □ or Bluetooth D to delete the recording point
{
    if(Mode==1)  //If it is in recording mode
    {
      if ((ps2x.Button(PSB_SQUARE)||ps2x1.Button(BTB_D)||(PCact[16]==1))&&Totalact!=0)  //If you press the X key on the wireless controller or the D key of the Bluetooth App, or the PC PCact[16] status is not 1, and the total number of steps is not 0
      {
         if(Pressbuf[3] == false)  //f you press buf to 0
         {
           for ( int i = 0; i < maxAutoActions; i ++ )  //The for loop checks whether the current number of steps is less than the maximum number of steps that can be automatically run
           {
             Actionmem[i][0]=0;  //The current number of steps of the claw head steering gear is reset at the current Angle
             Actionmem[i][1]=0;  //Current step number of forearm steering gear current Angle clear zero
             Actionmem[i][2]=0;  //Current step number of big arm steering gear current Angle clear zero
             Actionmem[i][3]=0;  //Current step number of turntable steering gear current Angle clear zero
           } 
           Totalact=0;  //Total steps count to 0
           actionIndex=0;  //The current step is set to 0
           Serial.println("清空已存点位");  ////The message "Empty saved Bits" is displayed on the serial port monitor
           PCact[16]=0;  //PCact[16] 0
           Pressbuf[3] = true;  //pt buf value 1
           Releasebuf[3] = false;  //Releasebuf value 0
         }
      }
      if(!ps2x.Button(PSB_SQUARE)&&!ps2x1.Button(BTB_D))  //If you do not press the wireless controller X key, or do not press the Bluetooth App D key
      {
        Pressbuf[3] = false;  //Press buf to assign a value of 0
        Releasebuf[3] = true;  //Releasebuf value 1
      }
    }
}

void PCmode()  //Function 13:PC control mode communication
{         
  if (stringComplete)  //If the serial port receiving flag is1
  {
    if(PCcmd[0]=="M")  //If PCcmd[0] gets the window data character M
    {
      for ( int i = 0; i < 4; i ++ )  //for circulation
      {
        PCmove[i]=PCcmd[i+1];  //MSubsequent input values are assigned to each steering gear one by one
      }     
    } 
    for ( int i = 0; i < 4; i ++ )  //for circulation
    {
      if(PCmove[i]=="0")  //If the input value of the current steering gear is character 0
      {
        PCact[i]=0;  // adjourn
      }
      if(PCmove[i]=="1")  //If the input value of the current steering gear is character 1
      {
        PCact[i]=1;  //Execution forward
      }
      if(PCmove[i]=="2")  //If the input value of the current steering gear is character 2
      {
        PCact[i]=2;  //Execution inversion
      }
     }
    if((PCcmd[0]=="a")&&(stringlength==16))  //If the first character is a, and the string length is 16 characters
    {
      RcvBuf[0]+=PCcmd[1];  //Claw head: Angle value RcvBuf (String type) is first added by 1 to the next bit (the second bit), then the second character of the string is assigned to it
      RcvBuf[0]+=PCcmd[2];  //Claw head: Angle value RcvBuf (String type) is first incremented by 1 to the next bit (3rd bit), and then the 3rd character of the string is assigned to it
      RcvBuf[0]+=PCcmd[3];  //Claw head: The Angle value RcvBuf (String type) is first incremented by 1 to the next bit (the fourth bit), and then the fourth character of the string is assigned to it
      Rcvint[0]=RcvBuf[0].toInt();  //Convert the Angle value of the claw head steering gear character type to an integer type
      //In the same way, convert the Angle value (character type) entered by the forearm, the forearm and the turntable into an int integer, and determine 1 Angle value for each steering machine
      RcvBuf[1]+=PCcmd[5];
      RcvBuf[1]+=PCcmd[6];
      RcvBuf[1]+=PCcmd[7];
      Rcvint[1]=RcvBuf[1].toInt();
      RcvBuf[2]+=PCcmd[9];
      RcvBuf[2]+=PCcmd[10];
      RcvBuf[2]+=PCcmd[11];
      Rcvint[2]=RcvBuf[2].toInt();
      RcvBuf[3]+=PCcmd[13];
      RcvBuf[3]+=PCcmd[14];
      RcvBuf[3]+=PCcmd[15];
      Rcvint[3]=RcvBuf[3].toInt();
      for( int i = 0; i < 4; i ++ )  //for the cycle, the input Angle value of the four servos is compared with the limit value so that the input Angle value does not exceed the limit value
      {
        if(Rcvint[i]<MIN[i])  //If the Angle value entered by the current steering gear is < the minimum limit value of the steering gear
        {
          Rcvint[i]=MIN[i];  //The current input Angle value of the steering gear is changed to the minimum limit value of the steering gear
        }
        if(Rcvint[i]>MAX[i])  //If the Angle value entered by the current steering gear is > the maximum limit value of the steering gear
        {
          Rcvint[i]=MAX[i];  //he current input Angle value of the steering gear is changed to the maximum limit value of the steering gear
        }
      }
      writeServo( PIN[0], Rcvint[0] );  //The input Angle value is written to the claw head steering gear and the claw head steering gear is turned to this Angle
      writeServo( PIN[1], Rcvint[1] );  //Write the input Angle value to the claw head steering gear and turn the forearm steering gear to this Angle
      writeServo( PIN[2], Rcvint[2] );  //Write the input Angle value to the claw head steering gear, and let the big arm steering gear turn to this Angle
      writeServo( PIN[3], Rcvint[3] );  //Write the input Angle value to the claw head steering gear, so that the turntable steering gear to this Angle
      currentAngle[0] = Rcvint[0];  //Take the input Angle value of the claw head as the current Angle
      currentAngle[1] = Rcvint[1];  //Take the input Angle value of the forearm as the current Angle
      currentAngle[2] = Rcvint[2];  //Take the input Angle value of the arm as the current Angle
      currentAngle[3] = Rcvint[3];  //Take the input Angle value of the turntable as the current Angle
      }
    if(inputString=="UP")  //If you enter the character UP
    {
      PCact[10]=1;  //PCact[10] sets 1, which is one of the conditions for a schema change for function 7: void ModeChange()
    }
    if(inputString=="DOWN")  //If the input character is DOWN
    {
      PCact[11]=1;  //PCact[11] sets 1, which is one of the conditions for function 8: void ShowAngle() to read the current coordinate value
    }
    if(inputString=="LEFT")  //f you enter the character LEFT
    {
      PCact[12]=1;  //PCact[12] sets 1, which is one of the conditions for the function 2: void Home(), return to origin function
    if(inputString=="A")  //If you enter the character A
    {
      PCact[13]=1;  //PCact[13]Set to 1, this flag is one of the conditions for function 11: void Autoplay(), autoplay mode, to start autoplay
    }
    if(inputString=="B")  //If you enter character B
    {
      PCact[14]=1;  //PCact[14]Set to 1, this flag is one of the conditions for function 9: void MemoryAction(), automatic recording mode, recording point
    }
    if(inputString=="C")  //If you enter the character C
    {
      PCact[15]=1;  //PCact[15] sets 1, which is one of the conditions for function 11: void Autoplay(), autoplay mode, to end autoplay
    }
    if(inputString=="D")  //If you enter the character D
    {
      PCact[16]=1;  //PCact[16] sets 1, which is one of the conditions for function 12: void Autodelete() todelete a record point
    }
    for ( int i = 0; i < 4; i ++ )  //for cycle, respectively set up 4 steering gear
    {
      RcvBuf[i]="";  //A null character is assigned to each steering gear to reset it
      Rcvint[i]=0;  //Assign a value of 0 to each steering gear to play a reset role
    }
    inputString = "";  //Reset serial port data
    stringlength = 0;  //Set serial port character length to 0 (reset)
    Serialindex =0;  //The serial port pointer points to the initial 0 position
    stringComplete = false;  //The serial port receiving flag is false, indicating that the port is reset, preparing for cyclic recalculation
  }
}

void serialEvent()  //Function 14: Serial port event for PC mode communication
{
  while (Serial.available())   //When the serial port has data
  {
    char inChar = (char)Serial.read();  //Read serial data (character type)
    PCcmd[Serialindex]=inChar;  //The PC instruction character that assigns the currently read character to the data bit corresponding to the current serial port pointer
    if(inChar!='\n')  //If the character is not a carriage return
    {
      inputString += inChar;  //Set the character to 1 bit
      stringlength++;  //The character length is also 1 bit
      Serialindex++;  //This bit is also 1
    }
    if (inChar == '\n')  //If the character is a carriage return
    {
      stringComplete = true;  //Indicates that the string complies with character definition rules, setting the stringComplete identifier to 1
     }
  }
}
