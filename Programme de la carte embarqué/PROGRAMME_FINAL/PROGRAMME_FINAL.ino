    #include <Servo.h> 
    #include <SPI.h>
    #include <Mirf.h>
    #include <nRF24L01.h>
    #include <MirfHardwareSpiDriver.h>
    #include <Adafruit_Sensor.h>
    #include <Adafruit_LSM303_U.h>
    #include <Adafruit_L3GD20_U.h>
    #include <Adafruit_10DOF.h>
    #include <Adafruit_BMP085.h>
    Adafruit_BMP085 bmp;
    #include "I2Cdev.h"
    #include "MPU6050_6Axis_MotionApps20.h"
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
    #endif
    // class default I2C address is 0x68
    // specific I2C addresses may be passed as a parameter here
    // AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
    // AD0 high = 0x69
    MPU6050 mpu;
    #define OUTPUT_READABLE_YAWPITCHROLL
    #define LED_PIN 12 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
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
    
    /* Assign a unique ID to the sensors */
    Adafruit_10DOF                dof   = Adafruit_10DOF();
    Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
    Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
    //Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
    
    /* Update this with the correct SLP for accurate altitude measurements */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    
    // ================================================================
    // ===               INTERRUPT DETECTION ROUTINE                ===
    // ================================================================
    
    volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
    void dmpDataReady() {
        mpuInterrupt = true;
    }
    
    
    Servo ailreonRH;
    Servo ailreonLH;
    Servo directionRH;
    Servo directionLH;
    Servo elevator;
    Servo ESC;

    float ailreonRH1;
    float ailreonLH1;
    float directionRH1;
    float directionLH1;
    float elevator1;
    float GAZ1;
    
    int ledPin = 5;  
    
    void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
        #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
            Wire.begin();
            TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
        #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
            Fastwire::setup(400, true);
        #endif
     // initialize device
        //Serial.println(F("Initializing I2C devices..."));
        mpu.initialize();
    // load and configure the DMP
       // Serial.println(F("Initializing DMP..."));
        devStatus = mpu.dmpInitialize();
         // supply your own gyro offsets here, scaled for min sensitivity
        mpu.setXGyroOffset(83);
        mpu.setYGyroOffset(49);
        mpu.setZGyroOffset(-26);
        mpu.setZAccelOffset(1003); // 1688 factory default for my test chip
    // make sure it worked (returns 0 if so)
        if (devStatus == 0) {
            // turn on the DMP, now that it's ready
          //  Serial.println(F("Enabling DMP..."));
            mpu.setDMPEnabled(true);
    
            // enable Arduino interrupt detection
           // Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
            attachInterrupt(0, dmpDataReady, RISING);
            mpuIntStatus = mpu.getIntStatus();
    
            // set our DMP Ready flag so the main loop() function knows it's okay to use it
          //  Serial.println(F("DMP ready! Waiting for first interrupt..."));
            dmpReady = true;
    
            // get expected DMP packet size for later comparison
            packetSize = mpu.dmpGetFIFOPacketSize();
        } else {
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            // (if it's going to break, usually the code will be 1)
           // Serial.print(F("DMP Initialization failed (code "));
           // Serial.print(devStatus);
           // Serial.println(F(")"));
        }
        
    if (!bmp.begin()) {
     // Serial.println("Could not find a valid BMP085 sensor, check wiring!");
      while (1) {}
      }
    
    if(!mag.begin())
      {
        /* There was a problem detecting the LSM303 ... check your connections */
        //Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
        while(1);
      }
      if(!bmp.begin())
      {
        /* There was a problem detecting the BMP180 ... check your connections */
        //Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
        while(1);
      }
    /*-----------------------------------------------------------------------*/
      
     ailreonRH.attach(22);
     ailreonLH.attach(23);
     directionRH.attach(24);
     directionLH.attach(25);
     elevator.attach(26);
     ESC.attach(27);
     ESC.writeMicroseconds(1000);
     
      pinMode(ledPin, OUTPUT);
      Serial.begin(9600);  // pour le dÃƒÆ’Ã‚Â©bogage
      
          Mirf.cePin = 7; // Broche CE sur D9
      Mirf.csnPin = 8; // Broche CSN sur D10
      Mirf.spi = &MirfHardwareSpi; // On veut utiliser le port SPI hardware
      Mirf.init(); // Initialise la bibliothèque
    
      Mirf.channel = 0; // Choix du canal de communication (128 canaux disponibles, de 0 à 127)
      Mirf.payload = 10; // Taille d'un message (maximum 32 octets)
      Mirf.config(); // Sauvegarde la configuration dans le module radio
    
      Mirf.setTADDR((byte *) "nrf01"); // Adresse de transmission
      Mirf.setRADDR((byte *) "nrf02"); // Adresse de réception
      
    }

       
    void loop(){
        float directionRH1;
        float directionLH2;
        float elevator1;
        float GAZ1;
        
        
          
      int numero1;
      int numero2;
      int numero3;
      int numero4;
      int numero8; 
      byte numero;
      int pairimpair;
      byte data[Mirf.payload]; // Tableau de byte qui va stocker le message recu
      
      if(!Mirf.isSending() && Mirf.dataReady()){ // Si un message a ÃƒÆ’Ã‚Â©tÃƒÆ’Ã‚Â© recu et qu'un autre n'est pas en cours d'emission
        
       Mirf.getData(data); // on rÃƒÆ’Ã‚Â©cupÃƒÆ’Ã‚Â©re le message 
      
      // la suite de 4 bytes est convertie en 2 int   
       numero1 = ((long )data[0]) << 8;
       numero1 |= data[1];
       numero2 = ((long )data[2]) << 8;
       numero2 |= data[3];
       numero3 = ((long )data[4]) << 8;
       numero3 |= data[5];
       numero4 = ((long )data[6]) << 8;
       numero4 |= data[7];
       numero8 = ((long )data[8]) << 8;
       numero8 |= data[9]; 

        
       
        
         ailreonRH1 = map(numero1, 0, 255, 0, 180);
         ailreonLH1 = map(numero1, 0, 255, 180, 0);
         elevator1 = map(numero2, 0, 255, 180, 0);
         directionRH1 = map(numero3, 0, 255, 0, 180);
         directionLH2 = map(numero3, 0, 255, 180,0);
         GAZ1 =map(numero4, 0, 255, 2000, 1000);

     
         analogWrite(ledPin, numero8); 
         ailreonRH.write(ailreonRH1 );
         ailreonLH.write(ailreonLH1);
         elevator.write(elevator1);
         directionRH.write(directionRH1);
         directionLH.write(directionLH2);
         ESC.write(GAZ1);// tell servo to go to position in variable 'pos' 
        
      
        
    }
    
          sensors_event_t accel_event;
          sensors_event_t mag_event;
          sensors_event_t bmp_event;
          sensors_vec_t   orientation;
    
     // if programming failed, don't try to do anything
         if (!dmpReady) {return;}
        // wait for MPU interrupt or extra packet(s) available
        while (!mpuInterrupt && fifoCount < packetSize) {
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
          if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
              // reset so we can continue cleanly
              mpu.resetFIFO();
              //Serial.println(F("FIFO overflow!"));
      
          // otherwise, check for DMP data ready interrupt (this should happen frequently)
          } else if (mpuIntStatus & 0x02) {
              // wait for correct available data length, should be a VERY short wait
              while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    
              // read a packet from FIFO
              mpu.getFIFOBytes(fifoBuffer, packetSize);
              
              // track FIFO count here in case there is > 1 packet available
              // (this lets us immediately read more without waiting for an interrupt)
              fifoCount -= packetSize;
        
    
              #ifdef OUTPUT_READABLE_YAWPITCHROLL
    
                // display Euler angles in degrees
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                Serial.print(" yaw; ");
               Serial.print(ypr[0] * 180/M_PI);
                Serial.print(" ; ");
                Serial.print(ypr[1] * 180/M_PI);
                Serial.print(" ; ");
                Serial.print(ypr[2] * 180/M_PI);
                
                #endif
                /* Calculate the heading using the magnetometer */
                mag.getEvent(&mag_event);
                if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
                {
                  /* 'orientation' should have valid .heading data now */
                  //Serial.print(F("Heading: "));
                  Serial.print(" ; ");
                  Serial.print(orientation.heading);
                  
                }
                //Serial.print("Temperature = ");
                Serial.print(" ; ");
                Serial.print(bmp.readTemperature());
                Serial.print(" ; ");
                Serial.print(bmp.readAltitude());

                       //ailero
                Serial.print(" pal*");  
                       Serial.print(ailreonRH1);
                      //elevator
                       Serial.print(" * ");
                       Serial.print(elevator1);
                       //directionRH
                       Serial.print(" * ");
                       Serial.print(directionRH1);
                       //GAZ
                       Serial.print(" * ");
                       Serial.println(GAZ1);
                              
                              
            // blink LED to indicate activity
            blinkState = !blinkState;
            digitalWrite(LED_PIN, blinkState);
        }
    
          //delay(50);
    }


