// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>

//      2013-05-08 - added seamless Fastwire support
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;


//#define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_YAWPITCHROLL


#define INTERRUPT_PIN 7  // use pin 2 on Arduino Uno & most boards
#define FORWARD_PIN A0
#define BACKWARD_PIN A1
#define TRIGGER_PIN A2
#define SAFETY_PIN 9
//#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
//bool blinkState = false;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    pinMode (FORWARD_PIN, INPUT);
    pinMode (BACKWARD_PIN, INPUT);
    pinMode (TRIGGER_PIN, INPUT_PULLUP);
    pinMode (SAFETY_PIN, INPUT_PULLUP);
    pinMode(INTERRUPT_PIN, INPUT);
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE  // join I2C bus (I2Cdev library doesn't do this automatically)
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Keyboard.begin();
    Serial.begin(115200);
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
  //  while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);  // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    if (devStatus == 0) {    
        //Serial.println(F("Enabling DMP...")); // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)...")); // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        //Serial.println(F("DMP ready! Waiting for first interrupt...")); // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();  // get expected DMP packet size for later comparison
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    } 
    //pinMode(LED_PIN, OUTPUT);  // configure LED for output
}

void loop() {
   
    if (!dmpReady) return;   // if programming failed, don't try to do anything
    while (!mpuInterrupt && fifoCount < packetSize);   // wait for MPU interrupt or extra packet(s) available
    mpuInterrupt = false;  // reset interrupt flag and get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();  // get current FIFO count 
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {  // check for overflow (this should never happen unless our code is too inefficient)
        mpu.resetFIFO();   // reset so we can continue cleanly
        Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {  // otherwise, check for DMP data ready interrupt (this should happen frequently)
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();  // wait for correct available data length, should be a VERY short wait
        mpu.getFIFOBytes(fifoBuffer, packetSize);  // read a packet from FIFO
        fifoCount -= packetSize;  // track FIFO count here in case there is > 1 packet available   
        Serial.print(analogRead(FORWARD_PIN));
        Serial.print(",");
        Serial.print(analogRead(BACKWARD_PIN));
        Serial.print(",");
        if (digitalRead(SAFETY_PIN) == LOW ) {
          if (analogRead(FORWARD_PIN) > 530) {
            Keyboard.write('w');          
          } else if (analogRead(FORWARD_PIN) < 450) {
            Keyboard.write('s');
          }
          if (analogRead(BACKWARD_PIN) > 530) {
            Keyboard.write('d');          
          } else if (analogRead(BACKWARD_PIN) < 450) {
            Keyboard.write('a');
          }
        }
        if (digitalRead(TRIGGER_PIN) == LOW) {
          Serial.print("1");
          if (digitalRead(SAFETY_PIN) == LOW ) {
            Serial.print("2");
            Keyboard.write('e');      
          }
        
        } else  {
          Serial.print("0");          
        }     
        Serial.print(",");
        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print(",");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print(",");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        

        // blink LED to indicate activity
        //blinkState = !blinkState;
        //digitalWrite(LED_PIN, blinkState);
    }
}
