/* 
   -- ECE590 - Robot Design --
   Chris Glomb - Demo 3
   OpenCM9.04 - Motor Control & USB Interface
   
   Controls small robotic arm with 4 Dynamixel motors
*/

 /* Serial device defines for DXL1 bus */ 
#define DXL_BUS_ID 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04

/* Dynamixel Motor IDs */
#define sh_yaw 1
#define sh_pitch 2
#define elbow 3
#define gripper 4

#define STOP_SPEED  0
#define REVERSE_BIT 0x0400 //10th bit

int16 yaw_pos, pitch_pos, elbow_pos, gripper_pos;
int data_flag = 0;

int temp;

Dynamixel Dxl(DXL_BUS_ID);

void setup(){
  //You can attach your serialUSB interrupt
  //or, also detach the interrupt by detachInterrupt(void) method
  SerialUSB.attachInterrupt(usbInterrupt);
  pinMode(BOARD_LED_PIN, OUTPUT);  //toggleLED_Pin_Out; For FUN!!
  
  // Initialize the dynamixel bus:
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  Dxl.maxTorque(sh_yaw,1020);
  Dxl.maxTorque(sh_pitch,1020);
  Dxl.maxTorque(elbow,1020);
  Dxl.maxTorque(gripper,1020);
  
  Dxl.jointMode(sh_yaw);
  Dxl.jointMode(sh_pitch);
  Dxl.jointMode(elbow);
  Dxl.jointMode(gripper);
  
  Dxl.goalSpeed(sh_yaw,100);
  Dxl.goalSpeed(sh_pitch,100);
  Dxl.goalSpeed(elbow,100);
  Dxl.goalSpeed(gripper,100);
  delay(100);
}//End setup()

//USB max packet data is maximum 64byte, so nCount can not exceeds 64 bytes
void usbInterrupt(byte* buffer, byte nCount){
  yaw_pos = (int16)buffer[0] | ((int16)buffer[1])<<8;
  pitch_pos = (int16)buffer[2] | ((int16)buffer[3])<<8;
  elbow_pos = (int16)buffer[4] | ((int16)buffer[5])<<8;
  gripper_pos = (int16)buffer[6] | ((int16)buffer[7])<<8;
  data_flag = 1;
  //SerialUSB.print(yaw_pos);
}

void loop(){
  if(data_flag == 1){
    data_flag = 0;
    toggleLED();
    Dxl.goalPosition(sh_yaw, yaw_pos);
    Dxl.goalPosition(sh_pitch, pitch_pos);
    Dxl.goalPosition(elbow, elbow_pos);
    Dxl.goalPosition(gripper, gripper_pos);
  }
}


