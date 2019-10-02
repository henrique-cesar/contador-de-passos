#define BLYNK_PRINT Serial
#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include "I2Cdev.h"
BlynkTimer timer;

#include "MPU6050_6Axis_MotionApps20.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define BLYNK_PRINT Serial

char auth[] = "m1SYTF8Rz5UPYDf8oHK6VFAilbJcVHKh";

char ssid[] = "Bruno E Malf";
char pass[] = "MALFLPV8";


// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define INTERRUPT_PIN D7  // use pin 2 on Arduino Uno & most boards
#define LED_PIN D4 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

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

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void ICACHE_RAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

float getPitch(float val) {
    return val * 180/M_PI;
}

// ================================================================
// ===                      INITIAL VARIABLES                   ===
// ================================================================
// CONTADOR DE PASSOS
int passos = 0;
float base;
float variation = 10;
bool inicio = true;
bool sentido = false;

// VARIAVEIS ADICIONAIS
float altura;
float peso;
int sexo;
float cp;
float velocidade;
float distancia;
float segundos;
float calorias;
double MET;
int modoController;
String stringModo;
String stringValor;

//BUTTONS TO BLYNK
int start;
float input;
int botaoModo;
WidgetLCD lcd(V8);


// ================================================================
// ===                      PRINT FUNCTIONS                     ===
// ================================================================
void imprimeLcd(){
    segundos++;
    lcd.clear();
    lcd.print(0, 0, stringModo);
    lcd.print(0, 1, stringValor);
}
void imprimePassos()
{
    lcd.clear();
    lcd.print(0,0, "Passos: ");
    lcd.print(0,1, passos);
    Serial.print("Passos: ");
    Serial.printf("%d",passos);
}

void imprimeCalorias(){
    lcd.clear();
    lcd.print(0,0, "Calorias: ");
    lcd.print(0,1, calorias);
    Serial.print(calorias);
}

void imprimeVelocidade(){
    lcd.clear();
    lcd.print(0,0, "Velocidade: ");
    lcd.print(0,1, velocidade);
    Serial.print(velocidade);
}

void imprimeDistancia(){
    lcd.clear();
    lcd.print(0,0, "Distancia: ");
    lcd.print(0,1, distancia);
    Serial.print(distancia);
}
// ================================================================
// ===                      BLYNK CONFIG                        ===
// ================================================================

BLYNK_WRITE(V0) {
    start = param.asInt();
}

BLYNK_WRITE(V1){
    botaoModo = param.asInt();
}

BLYNK_WRITE(V2){
    input = param.asDouble();
}

BLYNK_WRITE(V4){
    sexo = param.asInt();
}
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    timer.setInterval(1000L,imprimeLcd);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(D5, D6);
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    Blynk.begin(auth, ssid, pass);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
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
}




// ================================================================
// ===                    FUNCTION TO COUNT STEPS               ===
// ================================================================

void footStepsSetup() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

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
            
            if (inicio) {
                base = getPitch(ypr[1]);
                inicio = false;
            }

            if ((getPitch(ypr[1]) > base + variation && !sentido) || (getPitch(ypr[1]) < base - variation && sentido)){
                passos++;
                base = getPitch(ypr[1]);
                sentido = !sentido;
                Serial.print("Passos: ");
                Serial.println(passos);
           }
           

        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void sendUpTime(){
    segundos++;
}
void calculateMET(){
    if(velocidade >= 2){
        MET = 7.5;
    } else if (velocidade > 0 && velocidade < 2 )
    {
        MET = 3.8;
    } else
    {
       MET = 1.3;
    }
}
void alteraModo(){
    if(start == 1){
        if(modoController % 4 == 0){
            modoController++;
            imprimePassos();
        }
        if(modoController %4 == 1){
            modoController++;
            stringModo = "Calorias: ";
            stringValor = String(calorias);
            //imprimeCalorias();
        }
        if(modoController %4 == 2){
            modoController++;
            stringModo = "Velocidade: ";
            stringValor = String(velocidade);
            //imprimeVelocidade();
        }
        if(modoController %4 == 3){
            modoController++;
            stringModo = "Distancia: ";
            stringValor = String(distancia);
            imprimeDistancia();
        }
        else{
            modoController = 0;
        }
    }
}
void sexoFeminino(){
    cp = altura * 0.415;
}
void sexoMasculino(){
    cp = altura * 0.413;
}



void loop(){
    Blynk.run();
    
    if (altura == 0){
        Serial.println("altura não definida");
        Serial.println("Informe a altura");
        lcd.clear();
        lcd.print(0,0, "informe: ");
        lcd.print(0,1, "altura (m)");

        while (altura == 0)
        {
            Blynk.run();
            altura = input;
            delay(150);
            input = 0;
            Serial.printf("altura %f\n", altura);
        }
    }

    if(peso == 0){
        Serial.println("peso não definido");
        Serial.println("Informe o peso");
        lcd.clear();
        lcd.print(0,0, "informe: ");
        lcd.print(0,1, "peso (kg)");

        while (peso == 0)
        {
            Blynk.run();
            peso = input;
            delay(150);
            input = 0;
            Serial.printf("peso %f\n", peso);
        }
    }
    if(sexo == 0){
        lcd.clear();
        while (sexo == 0)
        {
            lcd.print(0,0, "informe: ");
            lcd.print(0,1, "sexo");
            delay(150);
        }
    }
    if(sexo == 1){
        sexoMasculino();
    }
    if(sexo == 2){
        sexoFeminino();
    }

    if(start == 1){
        Serial.print(botaoModo);

        lcd.clear();
        footStepsSetup();
        calorias = MET * peso * (segundos / 3600);
        calculateMET();
        distancia = cp * passos;
        velocidade = distancia / segundos;

        }     
    else {
        passos = 0;
        segundos = 0;
        calorias = 0;
        velocidade = 0;
        distancia = 0;
        modoController = 0;
    }
    
}
