#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <avr/wdt.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(3,4); // RX, TX

#include <EEPROM.h>   // needed to save calibration routine
#include "writeAnything.h"

int eepromAdr = 1;
int calib = 0;

//battery monitor settings
int batMonPin = A7;    // input pin for the voltage divider
float pinVoltage = 0; // variable to hold the calculated voltage
float batteryVoltage = 0;




// timer function
unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 10000;           // interval at which to blink (milliseconds)

struct 
{
  int XGyroOff = 0;
  int YGyroOff = 0;
  int ZGyroOff = 0;
  int XAccelOff = 0;
  int YAccelOff = 0;
  int ZAccelOff = 0;
} calibrate;
 

const int analogPinX = A0;     // the number of the analog pin
const int analogPinY = A1;     // the number of the analog pin
 
const int button1 = 10;     // the number of the pushbutton pin
const int button2 = 7;     // the number of the pushbutton pin
const int button3 = 8;     // the number of the pushbutton pin
const int button4 = 9;     // the number of the pushbutton pin
const int button5 = 5;     // the number of the pushbutton pin

int analogX = 0;
int analogY = 0;
int analogXMapped = 0;
int analogYMapped = 0;

float degree = 0;
int Ba = 1;
int Bb = 1;
int Bc = 1;
int Bd = 1;
int Be = 1;
int Calibrating = 0;

// calibration defines
int buffersize=800;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

int16_t ax, ay, az,gx, gy, gz;

int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

// class default I2C address is 0x68
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  pinMode(button4, INPUT_PULLUP);
  pinMode(button5, INPUT_PULLUP);

  wdt_enable(WDTO_500MS);
  
  delay(15);
    
    // start bluetooth serial
    mySerial.begin(115200);
    mySerial.print("Starting...");

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    devStatus = mpu.dmpInitialize();

  
      // read offsets from eeprom    
      EEPROM_readAnything(0,calibrate);
       
      mpu.setXGyroOffset(calibrate.XGyroOff);
      mpu.setYGyroOffset(calibrate.YGyroOff);
      mpu.setZGyroOffset(calibrate.ZGyroOff);
      mpu.setXAccelOffset(calibrate.XAccelOff);
      mpu.setYAccelOffset(calibrate.YAccelOff);
      mpu.setZAccelOffset(calibrate.ZAccelOff);
     
       
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection    
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it       
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
}


void loop() {
  wdt_reset();
    //check and run calibration
    if (calib == 1) {
      wdt_disable() //disable watchdog whilst calibrating
      EEPROM_readAnything(0,calibrate);

        state = 0;

        mySerial.print("<");
        mySerial.print(",");
        mySerial.print("000");
        mySerial.print(",");
        mySerial.print("000");
        mySerial.print(",");
        mySerial.print("000");
        mySerial.print(",");
        mySerial.print("0");
        mySerial.print(",");
        mySerial.print("0");
        mySerial.print(",");
        mySerial.print("0");
        mySerial.print(",");
        mySerial.print("0");
        mySerial.print(",");
        mySerial.print("0");
        mySerial.print(",");
        mySerial.print("0");
        mySerial.print(",");
        mySerial.print("0");
        mySerial.print(",");
        //mySerial.print("0");
        //mySerial.print(",");
        mySerial.print(calib);
        mySerial.print(",");
        mySerial.println(">"); 
     
      if (state==0){  
        meansensors();
        state++;    
      }

      if (state==1) {
        calibration();
        state++;
      }

      if (state==2) {

        meansensors();
        calibrate.XGyroOff = gx_offset;
        calibrate.YGyroOff = gy_offset;
        calibrate.ZGyroOff = gz_offset;
        calibrate.XAccelOff = ax_offset;
        calibrate.YAccelOff = ay_offset;
        calibrate.ZAccelOff = az_offset;

        // save calibration struct to eeprom
        EEPROM_writeAnything(0,calibrate); 

        mpu.setXGyroOffset(calibrate.XGyroOff);
        mpu.setYGyroOffset(calibrate.YGyroOff);
        mpu.setZGyroOffset(calibrate.ZGyroOff);
        mpu.setXAccelOffset(calibrate.XAccelOff);
        mpu.setYAccelOffset(calibrate.YAccelOff);
        mpu.setZAccelOffset(calibrate.ZAccelOff);
        
        //Now calibrated so move straight onto normal operation
        calib = 0;
      }    
      wdt_enable()  // re-enable watchdog
    }



  

    //Normal Operation
    else{

    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
           
        // other program behavior stuff here

        //read serial port for calibration char.
        while (mySerial.available() > 0) {         
          calib = mySerial.parseInt();          
        };


       // read battery voltage
         pinVoltage = (analogRead(batMonPin)) * 0.00488;       //  Calculate the voltage on the A/D pin
                                    //  A reading of 1 for the A/D = 0.0048mV
                                    //  if we multiply the A/D reading by 0.00488 then 
                                    //  we get the voltage on the pin.                                  
                                    
  
        batteryVoltage = pinVoltage * 1.86;;    //  Use the ratio calculated for the voltage divider
            

        analogXMapped = map ((analogRead(analogPinX)),0,1024,0,200);
        analogYMapped = map ((analogRead(analogPinY)),0,1024,0,200);

        analogXMapped = constrain(analogXMapped, 0, 200);
        analogYMapped = constrain(analogYMapped, 0, 200);

        //check buttons
        Ba = ButtonConvert(button1);
        Bb = ButtonConvert(button2);
        Bc = ButtonConvert(button3);
        Bd = ButtonConvert(button4);
        Be = ButtonConvert(button5);
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
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

        mySerial.print("<");
        mySerial.print(",");

        degree = map_double((ypr[0] * 180/M_PI), -180, 180, 360, 0);

        mySerial.print(degree);
        mySerial.print(",");
        mySerial.print(ypr[1] * 180/M_PI);
        mySerial.print(",");
        mySerial.print(ypr[2] * 180/M_PI);
        mySerial.print(",");
        mySerial.print(analogXMapped);
        mySerial.print(",");
        mySerial.print(analogYMapped);
        mySerial.print(",");
        mySerial.print(Ba);
        mySerial.print(",");
        mySerial.print(Bb);
        mySerial.print(",");
        mySerial.print(Bc);
        mySerial.print(",");
        mySerial.print(Bd);
        mySerial.print(",");
        mySerial.print(Be);
        mySerial.print(",");
        //mySerial.print(batteryVoltage);
        //mySerial.print(",");
        mySerial.print(Calibrating);
        mySerial.print(",");
        mySerial.println(">");
                     
        }
      }
}

///////////////////////////////////   FUNCTIONS FOR OPERATION ////////////////////////////////////





//Buttons are using pullups, so read 1 when low and 0 when high. this function
//flips the 0 and 1 so 1 is high
int ButtonConvert (int button){

  int result = digitalRead(button);
  //flip 0 and 1
  int temp = !result;
 return temp;
};

float map_double(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
///////////////////////////////////   FUNCTIONS  FOR CALIBRATION ////////////////////////////////////
void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration(){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  while (1){
    int ready=0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);

    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

    meansensors();
    Serial.println("Calibrating...");
    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    if (ready==6) break;
  }
}

