//----------------------------------------------------------------------
// Based on: https://www.luisllamas.es/arduino-orientacion-imu-mpu-6050/
//----------------------------------------------------------------------

//GND - GND
//VCC - VCC
//SDA - Pin A4
//SCL - Pin A5
//INT - Pin 2

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_SSD1306.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"

#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

void meansensors();
void calibration();
bool isStep(float accelMagnitude, float gyroY);
void toogle();

bool toggle = false;

#define INTERRUPT_PIN 19

//CALIBRATION FUNCTION VARS
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
int16_t ax, ay, az,gx, gy, gz;
 
int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;


//NORMAL FUNCTION VARS
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]
VectorInt16 aa;         // [x, y, z]
VectorInt16 aaReal;     // [x, y, z]
VectorInt16 aaWorld;    // [x, y, z]
VectorFloat gravity;    // [x, y, z]
float ypr[3];           // [yaw, pitch, roll]

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);


//STEP COUNT VARIABLES
float lastAccelMagnitude = 0;
float lastGyroY = 0;
unsigned long lastStepTime = 0;
int stepCount = 0;
static const float stepTimeThreshold = 500; 

float timeInit = 0;
float timeEnd = 0;
float timeDelta = 0;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(9600);

    // Iniciar MPU6050
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // Comprobar  conexion
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // Iniciar DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();



    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);

    // Activar DMP
    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // Activar interrupcion
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print("Error: ");
            display.println(devStatus);
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        delay(5000);
        
    }

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  display.display();
  delay(500); // Pause for 2 seconds
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setRotation(0);
}

void loop() {
      if (state==0){
    Serial.println("\nReading sensors for first time...");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Calibrando.");
    meansensors();
    display.display();
    state++;
    delay(1000);
  }
 
  if (state==1) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Calibrando..");
    display.display();
    Serial.println("\nCalculating offsets...");
    calibration();
    state++;
    delay(1000);
  }
 
  if (state==2) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Calibrando...");
    display.display();
    meansensors();
    state++;
    Serial.println("\nFINISHED!");
    Serial.print("\nSensor readings with offsets:\t");
    Serial.print(mean_ax); 
    Serial.print("\t");
    Serial.print(mean_ay); 
    Serial.print("\t");
    Serial.print(mean_az); 
    Serial.print("\t");
    Serial.print(mean_gx); 
    Serial.print("\t");
    Serial.print(mean_gy); 
    Serial.print("\t");
    Serial.println(mean_gz);
    Serial.print("Your offsets:\t");
    Serial.print(ax_offset); 
    Serial.print("\t");
    Serial.print(ay_offset); 
    Serial.print("\t");
    Serial.print(az_offset); 
    Serial.print("\t");
    Serial.print(gx_offset); 
    Serial.print("\t");
    Serial.print(gy_offset); 
    Serial.print("\t");
    Serial.println(gz_offset); 
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Calibrado");
    Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
    Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
    Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);
    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);
    Serial.println(state);
  }

  if(state==3){
    // Si fallo al iniciar, parar programa
    if (!dmpReady) return;
    
    display.clearDisplay();
    display.setCursor(0, 0);

    display.println("MPU6050");

    // Ejecutar mientras no hay interrupcion
    while (!mpuInterrupt && fifoCount < packetSize) {
          
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // Obtener datos del FIFO
    fifoCount = mpu.getFIFOCount();

    // Controlar overflow
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } 
    else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

  

  // Mostrar Yaw, Pitch, Roll
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  /*
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180/M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180/M_PI);
  Serial.print("\t");
  Serial.println(ypr[2] * 180/M_PI);
*/
  /*
  // Mostrar aceleracion
  Serial.print("areal\t");
  Serial.print(aaReal.x);
  Serial.print("\t");
  Serial.print(aaReal.y);
  Serial.print("\t");
  Serial.println(aaReal.z);
  */
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

  if(abs(ypr[2])>1.9){
    toogle();
    }
  if(toggle){
   // Calculate acceleration magnitude
            float accelMagnitude = sqrt(aaReal.x * aaReal.x + aaReal.y * aaReal.y + aaReal.z * aaReal.z);

            // Check for step based on acceleration and gyroscope data
            if (isStep(accelMagnitude, ypr[1])) {
                unsigned long currentTime = millis();
                if (currentTime - lastStepTime > stepTimeThreshold) {
                    stepCount++;
                    if(stepCount==1){
                      timeInit = millis();
                    }
                    lastStepTime = currentTime;
                }
            }

            if(stepCount>=1){
              timeDelta = millis() - timeInit;
            }
            // Display step count and other sensor data
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("Start");
            display.setCursor(0, 20);
            display.print("Step Count: ");
            display.println(stepCount*2);
            display.setCursor(0, 30);
            display.print("Time: ");
            display.print(timeDelta/1000);
            display.print(" s");
  /*
  
  display.setCursor(0, 30);
  display.println("Yaw    Pitch    Roll");
  display.setCursor(0, 40);
  display.print(ypr[0] );
  display.print("   ");
  display.print(ypr[1] );
  display.print("   ");
  display.println(ypr[2] );
  
  display.setCursor(0, 40);
  display.println("Acel x   y   z: ");
  display.setCursor(0, 50);
  display.print(" ");
  display.print(aaReal.x);
  display.print(" ");
  display.print(aaReal.y);
  display.print("   ");
  display.print(aaReal.z);
  */
  display.display();
  delay(100);
            


            

            //display.display();

  }else{    display.clearDisplay();
            display.setCursor(0, 0);
            display.println("Stop");
            display.print("Step Count: ");
            display.println(stepCount*2);
            display.setCursor(0, 20);
            display.print("Time: ");
            display.print(timeDelta/1000);
            display.print(" s");
            display.display();
  }


  } 
}
}


///////////////////////////////////   FUNCTIONS   ////////////////////////////////////

//toogle function
void toogle(){
  if(toggle){
    toggle = false;
  }
  else{
    toggle = true;
    timeInit = 0;
    timeDelta = 0;
    stepCount = 0;
  }
  delay(1000);
}

// Function to check if a step is detected
bool isStep(float accelMagnitude, float gyroY) {
    static const float accelThreshold = 5000;  // Adjust this threshold as needed
    static const float gyroThreshold = 0.4;   // Adjust this threshold as needed

    Serial.print("accelMagnitude: ");
    Serial.println(accelMagnitude);
    Serial.print("gyroY: ");
    Serial.println(gyroY);

    // Check for conditions to detect a step
    if (accelMagnitude > accelThreshold && lastAccelMagnitude <= accelThreshold &&
        abs(gyroY) > gyroThreshold) {
        // Additional checks using gyroscope data may be added here to improve step detection
        return true;
    }

    return false;
}


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
    Serial.println("...");
 
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