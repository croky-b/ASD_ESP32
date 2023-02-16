
#include <SBUS.h>  //use my fork
#include <Servo_MCPWM.h>   //use my fork

#define COMMAND_RX  39

SBUS sBus(Serial2); // SBUS object on Serial 2 port


uint16_t SBUSchannels[16];
bool     SBUSfailSafe;
bool     SBUSlostFrame;
bool     sbusInit;



const uint8_t encoder_CH1_CH2 = 27; 
const uint8_t encoder_CH3_CH4 = 33; 

float CH1X=0;
float CH2Y=0;

float CH3X=0;
float CH4Y=0;


float hyp_CH1_CH2=0;
float powhyp_CH1_CH2=0;

float angle_CH1_CH2 =0;
float angle2_CH1_CH2 =0;

int16_t power_CH1_CH2;
int16_t powerESC_CH1_CH2;

float hyp_CH3_CH4=0;
float powhyp_CH3_CH4=0;

float angle_CH3_CH4 =0;
float angle2_CH3_CH4 =0;

int16_t power_CH3_CH4;
int16_t powerESC_CH3_CH4;

Servo SERVO1;
Servo SERVO2;
Servo ESC1;
Servo ESC2;


void setup() {
disableCore0WDT();
disableCore1WDT();


  
 pinMode(COMMAND_RX, INPUT_PULLDOWN);
 
 SERVO1.attach(19, 1, 1, 0);
 SERVO2.attach(5, 1, 1, 1);
 ESC1.attach(18, 1, 2, 0);
 ESC1.attach(32, 1, 2, 1);

 // Task 1 setup (running on core 0)
  TaskHandle_t Task1;
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    Task1code,   /* Task function. */
    "Task1",     /* name of task. */
    100000,      /* Stack size of task (10000) */
    NULL,        /* parameter of the task */
    1,           /* priority of the task (1 = low, 3 = medium, 5 = highest)*/
    &Task1,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */

   

 sBus.begin(COMMAND_RX, true); 
 readSbusCommands();

Serial.begin(115200);
 
}


void readSbusCommands()
{
 
  if (sBus.read(&SBUSchannels[0], &SBUSfailSafe, &SBUSlostFrame))
  {
    sbusInit         = true;
    
  }

  
  
}



void loop() {

 readSbusCommands(); 

 

 }


void Task1code(void *pvParameters)
{
  for (;;)
  {
    GoTo_CH1_CH2(round(angle_CH1_CH2));
    ASD_CH1_CH2();
    GoTo_CH3_CH4(round(angle_CH3_CH4));
    ASD_CH3_CH4();
    }
  
}


//
// =======================================================================================================
// ASD
// =======================================================================================================
//

void ASD_CH1_CH2()
{



if (angle2_CH1_CH2>0)
 {angle2_CH1_CH2 -=360;}

angle_CH1_CH2 =360+angle2_CH1_CH2;
angle2_CH1_CH2= degrees(atan2(CH1X,CH2Y));
 
CH1X= SBUSchannels[0] -992;
CH2Y= SBUSchannels[1] -992;

powhyp_CH1_CH2=pow(CH1X,2)+pow(CH2Y,2);
hyp_CH1_CH2=sqrt(powhyp_CH1_CH2);

powerESC_CH1_CH2=(power_CH1_CH2*5)+1500;
power_CH1_CH2=(map(hyp_CH1_CH2,0,820,0,100));



ESC1.writeMicroseconds(powerESC_CH1_CH2);
}


void ASD_CH3_CH4()
{



if (angle2_CH3_CH4>0)
 {angle2_CH3_CH4 -=360;}

angle_CH3_CH4 =360+angle2_CH3_CH4;
angle2_CH3_CH4= degrees(atan2(CH3X,CH4Y));
 
CH3X= SBUSchannels[2] -992;
CH4Y= SBUSchannels[3] -992;

powhyp_CH3_CH4=pow(CH3X,2)+pow(CH4Y,2);
hyp_CH3_CH4=sqrt(powhyp_CH3_CH4);

powerESC_CH3_CH4=(power_CH3_CH4*5)+1500;
power_CH3_CH4=(map(hyp_CH1_CH2,0,820,0,100));

ESC2.writeMicroseconds(powerESC_CH3_CH4);

}



//
// =======================================================================================================
//SERVO 360
// =======================================================================================================
//

float GoTo_CH1_CH2(float setpoint){ // moves the servo to an input position
  float error_CH1_CH2=0;
  float out_CH1_CH2=0;
  

 
  float CW_error_CH1_CH2 = setpoint - getPos_CH1_CH2(); //calculates the error if moving CW
  if(CW_error_CH1_CH2 < 0){ //if CW_error is less than 0
    CW_error_CH1_CH2 = CW_error_CH1_CH2+360;
  }
  float CCW_error_CH1_CH2 = getPos_CH1_CH2()-setpoint; //calculates the error if moving CCW
  if(CCW_error_CH1_CH2 < 0){ //if CCW_error is less than 0
    CCW_error_CH1_CH2 = CCW_error_CH1_CH2+360;
  }
  
  // if CW_error is smaller than CCW_error (or both are equal) then error should be CW_error
  if(CW_error_CH1_CH2 < CCW_error_CH1_CH2 || CW_error_CH1_CH2 == CCW_error_CH1_CH2){ 
    error_CH1_CH2 = CW_error_CH1_CH2;
  }
  
  // if CCW_error is smaller then CW_error then make error CCW_error 
  else if(CCW_error_CH1_CH2 < CW_error_CH1_CH2){ 
    error_CH1_CH2 = -1*CCW_error_CH1_CH2; //makes error negative
  }

  out_CH1_CH2 = 12*error_CH1_CH2;
  out_CH1_CH2 = constrain(out_CH1_CH2, -255, 255); //constrains output to have maximum magnitude of 255 
  if(abs(out_CH1_CH2) < 25){ //if output is less than 25 make it 0
    out_CH1_CH2 = 0;
  }
 
  if(out_CH1_CH2 > 0){ //if output is positive move CW
    SERVO1.writeMicroseconds(1460);
   
  }
  else if(out_CH1_CH2 < 0){ //if output is negative move CCW
    SERVO1.writeMicroseconds(1540);
   
  }
  else if(out_CH1_CH2 == 0){
    SERVO1.writeMicroseconds(1500);
   
  }

  


 }

float getPos_CH1_CH2(){ // gets and returns encoder position
  // gets raw 10 bit position from the encoder and maps it to angle value
  // adds offset value

   
 float p_encoder_CH1_CH2= (analogRead(encoder_CH1_CH2)*0.087890625);
 
 uint16_t pos_CH1_CH2 = round (p_encoder_CH1_CH2);
  
  return pos_CH1_CH2;
  
 
}


float GoTo_CH3_CH4(float setpoint){ // moves the servo to an input position
  float error_CH3_CH4=0;
  float out_CH3_CH4=0;
  

 
  float CW_error_CH3_CH4 = setpoint - getPos_CH3_CH4(); //calculates the error if moving CW
  if(CW_error_CH3_CH4 < 0){ //if CW_error is less than 0
    CW_error_CH3_CH4 = CW_error_CH3_CH4+360;
  }
  float CCW_error_CH3_CH4 = getPos_CH3_CH4()-setpoint; //calculates the error if moving CCW
  if(CCW_error_CH3_CH4 < 0){ //if CCW_error is less than 0
    CCW_error_CH3_CH4 = CCW_error_CH3_CH4+360;
  }
  
  // if CW_error is smaller than CCW_error (or both are equal) then error should be CW_error
  if(CW_error_CH3_CH4 < CCW_error_CH3_CH4 || CW_error_CH3_CH4 == CCW_error_CH3_CH4){ 
    error_CH3_CH4 = CW_error_CH3_CH4;
  }
  
  // if CCW_error is smaller then CW_error then make error CCW_error 
  else if(CCW_error_CH3_CH4 < CW_error_CH3_CH4){ 
    error_CH3_CH4 = -1*CCW_error_CH3_CH4; //makes error negative
  }

  out_CH3_CH4 = 12*error_CH3_CH4;
  out_CH3_CH4 = constrain(out_CH3_CH4, -255, 255); //constrains output to have maximum magnitude of 255 
  if(abs(out_CH3_CH4) < 25){ //if output is less than 25 make it 0
    out_CH3_CH4 = 0;
  }
 
  if(out_CH3_CH4 > 0){ //if output is positive move CW
    SERVO1.writeMicroseconds(1460);
   
  }
  else if(out_CH3_CH4 < 0){ //if output is negative move CCW
    SERVO1.writeMicroseconds(1540);
   
  }
  else if(out_CH3_CH4 == 0){
    SERVO1.writeMicroseconds(1500);
   
  }

  


 }

float getPos_CH3_CH4(){ // gets and returns encoder position
  // gets raw 10 bit position from the encoder and maps it to angle value
  // adds offset value

   
 float p_encoder_CH3_CH4= (analogRead(encoder_CH3_CH4)*0.087890625);
 
 uint16_t pos_CH3_CH4 = round (p_encoder_CH3_CH4);
  
  return pos_CH3_CH4;
  
 
}
