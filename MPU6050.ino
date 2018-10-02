#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <MsTimer2.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
SoftwareSerial BLE(3,2);//RX=3TX2
byte uartbuff[8];
int databuff[8];
bool blinkState = false;
int16_t ax, ay, az;
int16_t gx, gy, gz;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
//VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
//float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
byte T=0;
void Timer(){
  T=1;
  //if(T==5)T=0;
  //getData();
  //sendData();
}
float Data[4];
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++){
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}

void SDS(float S_Out[])
{
  int temp[4] = {0};
  unsigned int temp1[4] = {0};
  unsigned char databuf[10] = {0};
  unsigned char i;
  unsigned short CRC16 = 0;
 // float SDS_OutData[4];
  /*for(i=0;i<4;i++) {
  SDS_OutData[i]=S_Out[i];
  }*/
  for(i=0;i<4;i++)
   {

    temp[i]  = (int)Data[i];
    temp1[i] = (unsigned int)temp[i];

   }

  for(i=0;i<4;i++)
  {
    databuf[i*2]   = (unsigned char)(temp1[i]%256);
    databuf[i*2+1] = (unsigned char)(temp1[i]/256);
  }

  CRC16 = CRC_CHECK(databuf,8);
  databuf[8] = CRC16%256;
  databuf[9] = CRC16/256;

  //SDS_UART_Init();
  for(i=0;i<10;i++)
  {

    Serial.write(databuf[i]);
  }
}


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    BLE.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    delay(20);
    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
 
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        mpuInterrupt=true;
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    
  MsTimer2::set(5, Timer); // 500ms period
  MsTimer2::start();
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void sendData(){
            databuff[0]=(int)(ypr[0]*20000)>>8;
            databuff[1]=(int)(ypr[1]*20000)>>8;
            databuff[2]=(int)(ypr[2]*20000)>>8;
            //databuff[3]=(int)(gz);
            databuff[0]=LIMIT(databuff[0],-118,120);
            databuff[1]=LIMIT(databuff[1],-118,120);
            databuff[2]=LIMIT(databuff[2],-118,120);
            //databuff[3]=LIMIT(gz,-120,120);
            Data[0]=(int16_t)databuff[0];
            Data[1]=(int16_t)databuff[1];
            Data[2]=(int16_t)databuff[2];
            /*
            BLE.write(124);
            //BLE.write((byte)(databuff[0]));
            BLE.write((byte)(databuff[1]));
            BLE.write((byte)(databuff[2]));
            BLE.write(125);
           */
            BLE.write(0x01);
            //BLE.write((byte)(databuff[0]));
            BLE.write((byte)(databuff[1]+120));
            BLE.write((byte)(databuff[2]+120));
            BLE.write(0xff);
            /*
            Serial.write(0xfe);
            //BLE.write((byte)(databuff[0]));
            Serial.write((byte)(databuff[1]));
            Serial.write((byte)(databuff[2]));
            Serial.write(0xff);
            */
            //BLE.write(uartbuff[0]++);
            //Data[3]=gz;
            //Data[3]=(int16_t)databuff[3];
/*
            Data[0]=(int16_t)(ypr[0]*20000)>>8;
            Data[1]=(int16_t)(ypr[1]*20000)>>8;
            Data[2]=(int16_t)(ypr[2]*20000)>>8;
            Data[3]=(int16_t)(gz*20)>>8;
*/
            SDS(Data); 
}
void getData(){
     // if programming failed, don't try to do anything
    if (!dmpReady) return;
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            /*
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
            */
            /*
            databuff[0]=(int)(ypr[0]*20000)>>8;
            databuff[1]=(int)(ypr[1]*20000)>>8;
            databuff[2]=(int)(ypr[2]*20000)>>8;
            //databuff[3]=(int)(gz);
            databuff[0]=LIMIT(databuff[0],-120,120);
            databuff[1]=LIMIT(databuff[1],-120,120);
            databuff[2]=LIMIT(databuff[2],-120,120);
            //databuff[3]=LIMIT(gz,-120,120);
            Data[0]=(int16_t)databuff[0];
            Data[1]=(int16_t)databuff[1];
            Data[2]=(int16_t)databuff[2];
            BLE.write(124);
            //BLE.write((byte)(databuff[0]));
            BLE.write((byte)(databuff[1]));
            BLE.write((byte)(databuff[2]));
            BLE.write(125);
        
            //BLE.write(uartbuff[0]++);
            //Data[3]=gz;
            //Data[3]=(int16_t)databuff[3];
            SDS(Data);
            */
            
}
byte cnt=0;
void loop() {
        if(T==1){
          getData();
          cnt++;
          if(cnt==3){
          blinkState = !blinkState;
          digitalWrite(LED_PIN, blinkState); 
            cnt=0,sendData();
          }

          T=0;

        }

          //sendData();
       
        //delay(2);
        //BLE.write(uartbuff[0]++);
    
}
