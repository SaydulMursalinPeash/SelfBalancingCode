
#include "I2Cdev.h"
#include"MPU6050_6Axis_MotionApps20.h"



#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define OUTPUT_READABLE_YAWPITCHROLL


//-----------------------Defining motor controller pins---------------------------//
#define M11 7
#define M12 6
#define M21 5
#define M22 4
#define E1 3
#define E2 10


//-----------------------Variable declaration for PID controller------------------//
float kp=50.0;
double ki=0.0;
float kd=20.0;
float iError=0;
float lastTime=0;
float maxVal=250;
float oldVal=0;


MPU6050 sensor;
int16_t ax,ay,az;
int16_t vx,vy,vz;



//-------------------------------MPU control---------------------------------------------//
bool dmpReady=false;
uint8_t mpuIntStatus;
uint8_t devStat;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];


//-------------------------Oriantation and motion variables-----------------//
VectorFloat gravity;  //x,y z
Quaternion q;         //w,x,y,z
float eular[3];       //psi theta phi
float ypr[3];         //yaw pitch roll




//--------------------------Interrupt detection routine--------------------//
volatile bool mpuInt=false;
void MotionDataReady(){
  mpuInt=true;
}


//-------------------------Setup-------------------------------------------//
void setup(){
  //Join I2C bus
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION==I2CDEV_BUILTIN_FASTWIRE
    FASTWIRE::setup(400,true);
  #endif

  Serial.begin(115200);
  while(!Serial);
  //Initialize device
  Serial.println("Testing device connection......");
  Serial.println("MPU6050 connection ");
  Serial.println(sensor.testConnection()?"Successfull.":"Unsuccessfull.");
  delay(000);
  Serial.println("Initializing DMP");
  devStat=sensor.dmpInitialize();

  //Gyro offset

 sensor.setXGyroOffset(+127);
 sensor.setYGyroOffset(-50);
 sensor.setXGyroOffset(-27);
 sensor.setZAccelOffset(2185);
 sensor.setXAccelOffset(-602);
 sensor.setYAccelOffset(-731);
 if(devStat==0){
  Serial.println("Enabling DMP");
  sensor.setDMPEnabled(true);

  //enable Arduino interrupt detection
  attachInterrupt(0,MotionDataReady,RISING);
  mpuIntStatus=sensor.getIntStatus();
  //set DMP ready flag
  Serial.println("DMP ready!!!!!");
  dmpReady=true;
  packetSize=sensor.dmpGetFIFOPacketSize();
  
 }
 else{
  //Error
  Serial.print(F("DMP Initialization failed (code "));
  Serial.print(devStat);
  Serial.println(F(")"));
 }
  
}



//-------------------------------Function to update yaw pitch roll-------------------------//

void updateYawPitchRoll(){
  //IF dmp not ready,nothing to do
  if(!dmpReady){
    return;
  }

  #ifdef ARDUINO_BOARD
    while(!mpuInt&& fifoCount<packetSize);
  #endif
  mpuInt=false;
  mpuIntStatus=sensor.getIntStatus();

  //--------------------Get current FIFO count------------------------//
  fifoCount=sensor.getFIFOCount();


  //--------------------check for overflow----------------------------//
  if((mpuIntStatus&0x10)||fifoCount==1024)
  {
    //reset so we can continue cleanly
    sensor.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }
  else if(mpuIntStatus&0x02){
    //Waiting for correct data available
    while(fifoCount<packetSize){
      fifoCount=sensor.getFIFOCount();
    }
    sensor.getFIFOBytes(fifoBuffer,packetSize);
    fifoCount-=packetSize;
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      sensor.dmpGetQuaternion(&q,fifoBuffer);
      sensor.dmpGetGravity(&gravity,&q);
      sensor.dmpGetYawPitchRoll(ypr,&q,&gravity);
      //printing yaw pitch roll
      Serial.print("ypr\t");
      
      Serial.print(ypr[1] * 180/M_PI);
     #endif
  }
}

//-------------------------------------PID controller---------------------------------//

float PID(float setPoint, float current){
  //calculating time since last function was called
  float thisTime=millis();
  float dT=thisTime-lastTime;
  lastTime=thisTime;
  float error=setPoint-current;
  
  //calculating integral error
  iError+=error*dT;
  
  //calculating differential error
  float dError=(oldVal-current)/dT;
  oldVal=current;

  //calculating result value
  float result=kp*error + ki*iError + kd*dError;

  //limit PID to max

  result=(result>maxVal)?maxVal:result;
  result=(result<-maxVal)?-maxVal:result;
  return result;
}

//-------------------------------------Motor handler function-------------------------//
/*
void runMotor1(float motor_speed){
  if(motor_speed>=0){
      digitalWrite(M11,HIGH);
      digitalWrite(M12,LOW);
      analogWrite(E1,fabs(motor_speed));
        
  }
  else{
      digitalWrite(M11,HIGH);
      digitalWrite(M12,LOW);
      analogWrite(E1,fabs(motor_speed));
  }
}
void runMotor2(float motor_speed){
  if(motor_speed>=0){
      digitalWrite(M21,HIGH);
      digitalWrite(M22,LOW);
      analogWrite(E2,fabs(motor_speed));
        
  }
  else{
      digitalWrite(M21,HIGH);
      digitalWrite(M22,LOW);
      analogWrite(E2,fabs(motor_speed));
  }
}*/

class Motor{
  private:
    int pin1,pin2,e;

  public:
    Motor(int pin1,int pin2 ,int e){
      this->pin1=pin1;
      this->pin2=pin2;
      this->e=e;
    }
    void run_motor(double speed_l){
      if(speed_l>=0){
        digitalWrite(pin1,HIGH);
        digitalWrite(pin2,LOW);
        analogWrite(e,fabs(speed_l));
        
      }
      else{
        digitalWrite(pin2,HIGH);
        digitalWrite(pin1,LOW);
        analogWrite(e,fabs(speed_l));
      }
    }
  
};
//-------------------------------------No movement------------------------------------//

void stay(){
  updateYawPitchRoll();
  float pitch=ypr[1] * 180/M_PI;
  float value=PID(0,pitch);
  if(fabs(value)<100){
    value=0;
  }
  Serial.print("\nres=");
  Serial.print(value);
  
  Serial.println("\n");
  Motor m1(M11,M12,E1);
  Motor m2(M21,M22,E2);
  
  m1.run_motor(value);
  m2.run_motor(value);

}
//--------------------------------------forward---------------------------------------//
void forward(){
  updateYawPitchRoll();
  float pitch=ypr[1] * 180/M_PI;
  float value=PID(2,pitch);
  if(fabs(value)<100){
    value=0;
  }
  Serial.print("\nres=");
  Serial.print(value);
  
  Serial.println("\n");
  Motor m1(M11,M12,E1);
  Motor m2(M21,M22,E2);
  
  m1.run_motor(value);
  m2.run_motor(value);

}
//-------------------------------------Backward---------------------------------------//
void backward(){
  updateYawPitchRoll();
  float pitch=ypr[1] * 180/M_PI;
  float value=PID(-2,pitch);
  if(fabs(value)<100){
    value=0;
  }
  Serial.print("\nres=");
  Serial.print(value);
  
  Serial.println("\n");
  Motor m1(M11,M12,E1);
  Motor m2(M21,M22,E2);
  
  m1.run_motor(value);
  m2.run_motor(value);

}
//-------------------------------------Main Loop--------------------------------------//
int command=48;
void loop(){
  if(Serial.available()>0){
    command=Serial.read();
    if(command==48){
      stay();
    }
    else if(command==49){
      forward();
    }
    else if(command==50){
      backward();
    }
    else{
      stay();
    }
    command=48;
  }
  
}
