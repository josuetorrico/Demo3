 /* Serial device defines for DXL1 bus */ 
#define DXL_BUS_ID 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04

// defining the IDs that we will assign to our robot
#define turn 1
#define left_leg 2
#define right_leg 3
#define cam 4
#define CLWGRP_MOTOR 5


//defining the speeds that we will be using for the servos in wheel mode
#define frwd 200
#define frwdx 192//175
#define trn 180
#define rvrs 200
#define rvrsx 192//175
#define camSpeed 100
#define stop 0
#define CLWGRP_MIN 200
#define CLWGRP_MAX 300//440


#define RGT_TRQ 1000
#define LFT_TRQ 1000

int CLAW_CMD;

Dynamixel Dxl(DXL_BUS_ID);

int RESET_FLAG;


void setup(){
  Dxl.begin(3);
  
  Dxl.jointMode(right_leg);
  Dxl.jointMode(left_leg);
  Dxl.goalPosition(right_leg, 1000);
  Dxl.goalPosition(left_leg, 950);
  delay(5000);
  Dxl.wheelMode(right_leg);
  Dxl.wheelMode(left_leg);
  Dxl.wheelMode(turn);
  Dxl.wheelMode(cam);
  Dxl.jointMode(CLWGRP_MOTOR);


  //initialize to stop
  Dxl.goalSpeed(right_leg, stop); 
  Dxl.goalSpeed(left_leg, stop);
  
  Dxl.goalSpeed(CLWGRP_MOTOR,100);
  Dxl.maxTorque(CLWGRP_MOTOR,1000);
  
  //legs
   Dxl.maxTorque(right_leg,1000);
   Dxl.maxTorque(left_leg,1000);
  
  RESET_FLAG = 0;
  
  SerialUSB.attachInterrupt(usbInterrupt);
  
}


void usbInterrupt(byte* buffer, byte nCount){
  for(unsigned int i=0; i < nCount;i++){
    SerialUSB.println((char)buffer[i]);
  }
  //move forward - button W
  if(buffer[0] == 119){         
    SerialUSB.println("moving forward");
    Dxl.goalSpeed(right_leg, frwdx | 0x400); //CW
    Dxl.goalSpeed(left_leg, frwd); //CCW 
  }
  //turn left or rotate CCW - button A  
  else if(buffer[0] == 97){       
    SerialUSB.println("turning left");
    Dxl.goalSpeed(turn, trn); //CCW
  }
  //turn right or rotate CW - button D  
  else if(buffer[0] == 100){
    SerialUSB.println("turning right");
    Dxl.goalSpeed(turn, trn | 0x400); //CW
  }
  //moving backwards - button X
  else if(buffer[0] == 120){       
    SerialUSB.println("moving backwards");
    Dxl.goalSpeed(right_leg, rvrsx); //CCW
    Dxl.goalSpeed(left_leg, rvrs | 0x400); //CW
  }
  //camera rotate right - button H
  else if(buffer[0] == 104){      
    SerialUSB.println("cam rotating right (CW)");
    Dxl.goalSpeed(right_leg, camSpeed | 0x400);   
  } 
  //camera rotate left - button G
  else if(buffer[0] == 103){      
    SerialUSB.println("cam rotating left (CCW)");
    Dxl.goalSpeed(right_leg, camSpeed);   
  }
  //only left leg - button K
  else if(buffer[0] == 107){      
    SerialUSB.println("only right leg moving");
    Dxl.goalSpeed(left_leg, 150);   
  }
  //only right leg - button L
  else if(buffer[0] == 108){      
    SerialUSB.println("only left leg moving");
    Dxl.goalSpeed(right_leg, 150 | 0x400);   
  }
  //only left leg - button j
  else if(buffer[0] == 106){      
    //SerialUSB.println("only right leg moving");
    //Dxl.goalSpeed(left_leg, 150);
   RESET_FLAG = 1;   
  }

//CLAW GRIP CLOSE "i"
  else if(buffer[0] == 105){
     Dxl.goalPosition(CLWGRP_MOTOR, 30);
    }
  
  //CLAW GRIP OPEN "o"
  else if(buffer[0] == 111){
    Dxl.goalPosition(CLWGRP_MOTOR, 350);
  }
      
  //stop command - button S
  else if(buffer[0] == 115){      
    SerialUSB.println("stopping");
    Dxl.goalSpeed(right_leg, stop); 
    Dxl.goalSpeed(left_leg, stop);
    Dxl.goalSpeed(turn, stop);
    Dxl.goalSpeed(cam, stop);    
  }
  else
    SerialUSB.println("I don't understand that command try W,A,S,D, or S.");
    
}


void loop(){
  delay(100);
 
   if(RESET_FLAG == 1){
     RESET_FLAG = 0;
       //reseting the legs - button j
      Dxl.jointMode(right_leg);
      Dxl.jointMode(left_leg);
      Dxl.goalPosition(right_leg, 500);
      Dxl.goalPosition(left_leg, 200);
      delay(5000);
      Dxl.wheelMode(right_leg);
      Dxl.wheelMode(left_leg);
  
  }
 
}
