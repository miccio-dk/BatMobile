
#include "EEPROM.h"
template <class T> boolean inRange (T val, T min, T max) {
  return ((min <= val) && (val <= max));
}


//debugging settings
boolean DEBUG_MOTORS = true;
boolean DEBUG_SENSORS = true;
boolean DEBUG_SENSORS_THRES = false;
boolean DEBUG_LINEPARAM = false;
boolean DEBUG_DISTANCE = false;

boolean REMOTE_CONTROL = true;
boolean SENSORS_READING = true;
boolean LINE_FOLLOWING = true;
boolean WALL_FOLLOWING = false;
boolean PATH = LINE_FOLLOWING && false;
boolean XBEE = false;
boolean START_STILL = false;
unsigned int DebugFreq = 5;
unsigned int steeringDuration = 50;
unsigned int steeringAmount = 100;


//Loop settings
unsigned int initialRef = 250;
float Kw = 0.75;                 // line sensors weight
float Ks = 0.5;                 // wheels synchronization
float Ki = 0.45;               // constant for integral term
float Kp[2] = {0.77, 0.77};   // constant for proportional term
const unsigned int avgMax = 50;
unsigned int avgNum = 10;    // smoothing factor
unsigned int Fs = 100;       // sampling frequency (Hz)


//debugging variables
const int LedPin = 13;
String msg = "";
unsigned long currentDebugTime, previousDebugTime;
boolean manualSteering = false;
boolean manualStill = false;
unsigned int steeringCount = 0;
enum Direction {FORWARD, BACKWARD};


//path variables
unsigned int stage = 0;
unsigned long SaltoDellaFede = 0;



//Floor sensors pins setup
const int MuxOut = A2;
const int MuxPin1 = A3;
const int MuxPin2 = A4;
const int MuxPin3 = A5;

//Motor 1 pins setup
const int Enable1 = 5;
const int InputA1 = 6;
const int InputB1 = 7;
const int EncoderA1 = 3;
const int EncoderB1 = 4;

//Motor 2 pins setup
const int Enable2 = 9;
const int InputA2 = 10;
const int InputB2 = 11;
const int EncoderA2 = 2;
const int EncoderB2 = 8;

//Distance sensors pins setup
const int SensFront = A0;
const int SensSide = A1;



//sensor variables
const int inputsNr = 8;                                       //number of sensors
int sensors[inputsNr] = {0, 0, 0, 0, 0, 0, 0, 0};             //sensors values
int sensorsPos[inputsNr] = {4, 5, 6, 7, 0, 1, 2, 3};          //sensor order
int sensorsWeight[inputsNr] = {-7, -5, -3, -1, 1, 3, 5, 7};   //sensor weight

int sensorsMinThresh[inputsNr] = {725, 627, 586, 607, 586, 727, 719, 600};
int sensorsMaxThresh[inputsNr] = {986, 974, 968, 974, 968, 983, 981, 966};
int sensorsSum = 0;
int sensorsSumWeighed = 0;

//distance sensors variables
int distanceFront = 0;
int distanceSide = 0;

//Motor Constants
const int gearRatio = 172;                  // 17.2:1
const int linesPerRev = 15;                 // 16 for 21B encoders

//interrupt variables (used to read tachometer)
volatile unsigned long TPeriod1, TPeriod2;   // time periods
volatile unsigned long Time1, Time2;         // running time
volatile unsigned int avgCount1, avgCount2;
volatile unsigned long avgTot1, avgTot2;
volatile unsigned long avgVals[2][avgMax];

//loop variables (used to control the motors)
unsigned long currentTime, previousTime, deltaTime;
float Ref;                           // reference for motor speed (RPM)
float Speed1, Speed2;                // wheels speed (RPM)
float Error1, Error2, Error3;        // difference between reference and measured value (RPM)
float Integral;                      // integral sum
float Bias;                          // speed bias given by the I-controller
float LineOut;                       // output from the sensors
unsigned int CSignal1, CSignal2;     // power to the motor



void quickStop() {
  //Motor 1
  digitalWrite(InputA1, LOW);
  digitalWrite(InputB1, LOW);
  digitalWrite(Enable1, HIGH);

  //Motor 2
  digitalWrite(InputA2, LOW);
  digitalWrite(InputB2, LOW);
  digitalWrite(Enable2, HIGH);
}

void quickStop(int n) {
  switch (n) {
      case 1:
        digitalWrite(InputA1, LOW);
        digitalWrite(InputB1, LOW);
        digitalWrite(Enable1, HIGH);
        break;
      case 2:
        digitalWrite(InputA2, LOW);
        digitalWrite(InputB2, LOW);
        digitalWrite(Enable2, HIGH);
        break;
      default:
        break;
  }

}


void setDirection(int motor, Direction a) {
  switch (motor) {
    case 0:
      switch (a) {
        case FORWARD:
          //Motor 1 direction
          digitalWrite(InputA1, HIGH);
          digitalWrite(InputB1, LOW);

          //Motor 2 direction
          digitalWrite(InputA2, HIGH);
          digitalWrite(InputB2, LOW);
          break;
        case BACKWARD:
          //Motor 1 direction
          digitalWrite(InputA1, LOW);
          digitalWrite(InputB1, HIGH);

          //Motor 2 direction
          digitalWrite(InputA2, LOW);
          digitalWrite(InputB2, HIGH);
          break;
      }
      break;
    case 1:
      switch (a) {
        case FORWARD:
          //Motor 1 direction
          digitalWrite(InputA1, HIGH);
          digitalWrite(InputB1, LOW);
          break;
        case BACKWARD:
          //Motor 1 direction
          digitalWrite(InputA1, LOW);
          digitalWrite(InputB1, HIGH);
          break;
      }
      break;
    case 2:
      switch (a) {
        case FORWARD:
          //Motor 2 direction
          digitalWrite(InputA2, HIGH);
          digitalWrite(InputB2, LOW);
          break;
        case BACKWARD:
          //Motor 2 direction
          digitalWrite(InputA2, LOW);
          digitalWrite(InputB2, HIGH);
          break;
      }
      break;
    default:
      break;
  }
}

void loadK() {
  if(EEPROM.read(0)==0) {
    if(DEBUG_MOTORS || DEBUG_SENSORS || REMOTE_CONTROL) {
      Serial.println("Overriding constants...");
    }
    EEPROM.write(0,max(initialRef,50)-50);
    EEPROM.write(1,map(Kw*1000,0,2000,0,255));
    EEPROM.write(2,map(Ks*1000,0,2000,0,255));
    EEPROM.write(3,map(Ki*1000,0,2000,0,255));
    EEPROM.write(4,map(Kp[0]*1000,0,2000,0,255));
    EEPROM.write(5,map(Kp[1]*1000,0,2000,0,255));
    EEPROM.write(6,min(avgNum,avgMax));
    EEPROM.write(7,min(sensorsWeight[inputsNr/2],255));
    EEPROM.write(8,min(sensorsWeight[inputsNr/2+1],255));
    EEPROM.write(9,min(sensorsWeight[inputsNr/2+2],255));
    EEPROM.write(10,min(sensorsWeight[inputsNr/2+3],255));
  }

  initialRef = EEPROM.read(0)+50;
  Kw =     ((float)map(EEPROM.read(1),0,255,0,2000))/1000;
  Ks =     ((float)map(EEPROM.read(2),0,255,0,2000))/1000;
  Ki =     ((float)map(EEPROM.read(3),0,255,0,2000))/1000;
  Kp[0] =  ((float)map(EEPROM.read(4),0,255,0,2000))/1000;
  Kp[1] =  ((float)map(EEPROM.read(5),0,255,0,2000))/1000; 
  avgNum = min(EEPROM.read(6),avgMax);
  for(int i=0; i<inputsNr/2; i++){
    int a = EEPROM.read(7+i);
    sensorsWeight[inputsNr/2+i] = a;
    sensorsWeight[inputsNr/2-i-1] = -a;
  }

  if(DEBUG_MOTORS || DEBUG_SENSORS || REMOTE_CONTROL) {
    msg = "Memory raw dump:";
    msg += "\nKw\t\t\t";
    msg += EEPROM.read(1);
    msg += "\nKs\t\t\t";
    msg += EEPROM.read(2);
    msg += "\nKi\t\t\t";
    msg += EEPROM.read(3);
    msg += "\nKp (left)\t";
    msg += EEPROM.read(4);
    msg += "\nKp (right)\t";
    msg += EEPROM.read(4);
    msg += "\nInit. Speed\t";
    msg += EEPROM.read(0);
    msg += "\n# Avg.\t\t";
    msg += EEPROM.read(6);
    msg += "\nSensor wt.\t";
    for(int i=0; i<inputsNr/2; i++){
      int a = EEPROM.read(7+i);
      msg += a;
      msg += " ";
    }

    msg += "\n\nLoop constants:";
    msg += "\nKw\t\t\t";
    msg += (int)(Kw*1000);
    msg += "\nKs\t\t\t";
    msg += (int)(Ks*1000);
    msg += "\nKi\t\t\t";
    msg += (int)(Ki*1000);
    msg += "\nKp (left)\t";
    msg += (int)(Kp[0]*1000);
    msg += "\nKp (right)\t";
    msg += (int)(Kp[1]*1000);
    msg += "\nInit. Speed\t";
    msg += (int)initialRef;
    msg += "rpm\n# Avg.\t\t";
    msg += (int)avgNum;
    msg += "smp\nSensor wt.\t";
    for(int i=0; i<inputsNr; i++){
      msg += sensorsWeight[i];
      msg += " ";
    }
    Serial.println(msg);
  }
}





/* SETUP */
void setup() {
  //initialize serial only if needed
  if(DEBUG_MOTORS || DEBUG_SENSORS || REMOTE_CONTROL) {
    // serial comm setup
    Serial.begin(9600);
    
    //initial message
    Serial.println("## HELLO! ##\n\n");

    if(XBEE) {
      delay(15000);
    }
  }
  //parameters setup
  loadK();
  Ref = initialRef;

  if(stage == 6) {
      Ref *= (1.1*1.3);
  }

  manualStill = START_STILL;
  delay(5000);
  CSignal1 = 10;
  CSignal2 = 10;

  
  // multiplexer pins
  pinMode(MuxOut, INPUT);
  pinMode(MuxPin1, OUTPUT);
  pinMode(MuxPin2, OUTPUT);
  pinMode(MuxPin3, OUTPUT);
  
  //Motor 1
  pinMode(Enable1, OUTPUT);
  pinMode(InputA1, OUTPUT);
  pinMode(InputB1, OUTPUT);
  pinMode(EncoderA1, INPUT);
  pinMode(EncoderB1, INPUT);

  //Motor 2
  pinMode(Enable2, OUTPUT);
  pinMode(InputA2, OUTPUT);
  pinMode(InputB2, OUTPUT);
  pinMode(EncoderA2, INPUT);
  pinMode(EncoderB2, INPUT);

  //Distance sensors
  pinMode(SensFront, INPUT);
  pinMode(SensSide, INPUT);


  //Led pin
  pinMode(LedPin, OUTPUT);

  digitalWrite(Enable1, LOW);
  digitalWrite(Enable2, LOW);  
  setDirection(0, FORWARD);

  //variables initialization
  TPeriod1 = 0;
  TPeriod2 = 0;
  Time1 = 0;
  Time2 = 0;
  avgCount1 = 0;
  avgCount2 = 0;
  avgTot1 = 0;
  avgTot2 = 0;
  for(int i=0 ; i<avgMax ; i++) {
    avgVals[0][i] = 0;
    avgVals[1][i] = 0;
  }
  currentTime = 0;
  previousTime = 0;
  
  //loop vars setup
  Integral = 0;
  Error1 = 0;
  Error2 = 0;
  Error3 = 0;
  LineOut = 0;
  Bias = 0;
  
  //interrups for TPeriod reading
  //called on rising edge for encoders channel A
  attachInterrupt(1, motor1, RISING);
  attachInterrupt(0, motor2, RISING);
  
  //turn on motors
  analogWrite(Enable1, CSignal1);
  analogWrite(Enable2, CSignal2);
  delay(1000);

  //increases PWM freqs
  //TCCR0B = TCCR0B & 0b11111000 | 0x01;
  //TCCR1B = TCCR1B & 0b11111000 | 0x01;
}



/* LOOP */
void loop() {


  /***************/
  /* motors loop */
  /***************/
  //update current time
  currentTime = micros();

  //runs loop with fixed time interval
  if((currentTime-previousTime) > (1000000UL/Fs)) {

    //find the loop time period (debug)
    deltaTime = currentTime-previousTime;
    
    //update loop timer
    previousTime = currentTime;

    //toggle led
    digitalWrite(LedPin, !digitalRead(LedPin));

    // read distance sensors using main loop timing
    distanceFront = analogRead(SensFront);

    distanceSide = analogRead(SensSide);

    /****************/
    /* floor sensor */
    /****************/
    if(SENSORS_READING) {
      //reset the sensors output to the loop
      LineOut = 0;
      sensorsSum = 0;
      sensorsSumWeighed = 0;
      //cycle through the sensors
      for(int i=0; i<inputsNr; i++) {
        //set the multiplexer to read one of the sensors
        digitalWrite(MuxPin1, bitRead(i,0));
        digitalWrite(MuxPin2, bitRead(i,1));
        digitalWrite(MuxPin3, bitRead(i,2));
        
        //read the value
        int reading = analogRead(MuxOut);
        
        //save and scale the value(s) using the mapping vector
        int realPos = sensorsPos[i];
        sensors[realPos] = constrain(map(reading, sensorsMinThresh[i], sensorsMaxThresh[i], 10, 0), 0, 10);


        //updates the thresholds
        //if(reading < minThresh)  minThresh = reading;
        //if(reading > maxThresh)  maxThresh = reading;
        
        //scale the value
        //int val = map(reading, minThresh, maxThresh, 0, 10);
        //int val = map(reading, 100, 900, 0, 10);      


        //calculate sum of sensors with scaled value
        sensorsSumWeighed += sensors[realPos]*sensorsWeight[i];

        //calculate sum of sensors (tells how many sensors are above the line)
        sensorsSum += sensors[realPos];

        //wait for better ADC accuracy
        delayMicroseconds(5);
      }

      if(LINE_FOLLOWING) {
        //calculate real weighed sensors output
        LineOut = sensorsSumWeighed/(80 - sensorsSum + 1)*10;
      }
      else if(stage == 4) {
        LineOut = 27;
      }
      else {
        LineOut = 0;
      }
    }
    
    //calculate speed
    Speed1 = (1000000 * 10 * 60 * 1.0) / (TPeriod1 * gearRatio * linesPerRev);
    Speed2 = (1000000 * 10 * 60 * 1.0) / (TPeriod2 * gearRatio * linesPerRev);
    
    if(manualSteering) {
      //if remote cotrols for steering are sent, ignore loop I-controller for a limited time
      steeringCount++;
      //after steeringDuration*2, disable the manual steering and reanable loop control
      if(steeringCount>=(steeringDuration*2)) {
        manualSteering = false;
        steeringCount = 0;
      }
      //reset bias and keep running for steeringDuration
      if(steeringCount>=(steeringDuration)) {
        Bias = 0;
      }
    } else {
      //calculations for the I-controller
      Error3 = (LineOut*Kw)+((Speed1-Speed2)*Ks);
      Integral += Error3;
      Bias = Integral*Ki;
    }
    
    //calculations for the P-controller(s)
    Error1 = (Ref-Speed1-Bias);
    Error2 = (Ref-Speed2+Bias);

    //calculation of the outputs
    CSignal1 = constrain((Error1*Kp[0]), 0, 255);
    CSignal2 = constrain((Error2*Kp[1]), 0, 255);
    
    if(manualStill) {
      //if manual still mode, power to the motor is 0
      quickStop();
    } else if(Error1>=0 && Error2>=0) {
      //update motor power
      setDirection(0,FORWARD);
      analogWrite(Enable1, CSignal1);
      analogWrite(Enable2, CSignal2);
    } else if(Error1<0 && Error2>=0) {
      quickStop(1);
      setDirection(2,FORWARD);
      analogWrite(Enable2, CSignal2);
    } else if(Error1>=0 && Error2<0) {
      setDirection(1,FORWARD);
      analogWrite(Enable1, CSignal1);
      quickStop(2);
    }
  }
  
  
  /******************/
  /* remote control */
  /******************/
  if(REMOTE_CONTROL) {
    char val = 0;
    int dest = 0;
    int param = 0;
    int value = 0;
    
    if (Serial.available()) {
      Integral = 0;
      Bias = 0;
      LINE_FOLLOWING = false;
      val = Serial.read();
      switch(val) {
        case 'c': //change setting
        dest = Serial.parseInt();
        param = Serial.parseInt();
        value = Serial.parseInt();

        //parameters setting 
        switch(param) {
          case 0:
          if(dest==0) {
            Kp[0] = value/1000.0;
            Kp[1] = value/1000.0;
            EEPROM.write(4,map(value,0,2000,0,255));
            EEPROM.write(5,map(value,0,2000,0,255));
            } else {
              Kp[dest-1] = value/1000.0;
              EEPROM.write(4+dest-1,map(value,0,2000,0,255));
              Serial.print(EEPROM.read(4+dest-1));
              Serial.println(" written in EEPROM.");
            }
            Serial.print(EEPROM.read(4));
            Serial.println(" written in EEPROM (twice).");
            break;

            case 1:
            if(dest==0) {
              Ki = value/1000.0;
              EEPROM.write(3,map(value,0,2000,0,255));
              Serial.print(EEPROM.read(3));
              Serial.println(" written in EEPROM.");
            }
            break;

            case 2:
            if(dest==0) {
              Kw = value/1000.0;
              EEPROM.write(1,map(value,0,2000,0,255));
              Serial.print(EEPROM.read(1));
              Serial.println(" written in EEPROM.");
            }
            break;

            case 3:
            if(dest==0) {
              Ks = value/1000.0;
              EEPROM.write(2,map(value,0,2000,0,255));
              Serial.print(EEPROM.read(2));
              Serial.println(" written in EEPROM.");
            }
            break;

            case 4:
            if(dest==0) {
              Fs = value;
            }
            break;

            case 5:
            if(dest==0 && value<=avgMax) {
              avgNum = value;
              EEPROM.write(6,value);
              Serial.print(EEPROM.read(6));
              Serial.println(" written in EEPROM.");
            }
            break;

            case 6:
            if(dest==0) {
              initialRef = value;
              Ref = initialRef;
              EEPROM.write(0,max(initialRef,50)-50);
              Serial.print(EEPROM.read(0));
              Serial.println(" written in EEPROM.");
            }
            break;

            case 10:
            case 11:
            case 12:
            case 13:
            if(dest==3) {
              sensorsWeight[inputsNr/2+(param-10)] = value;
              sensorsWeight[inputsNr/2-(param-10)-1] = -value;
              EEPROM.write(7+param-10,value);
              Serial.print(EEPROM.read(7+param-10));
              Serial.println(" written in EEPROM.");
            }
            break;

            case 20:
            if(dest==0) {
              stage = value;
            }
            break;

          }
          break;
        case 'p': //increase speed
        Ref *= 1.2;
        break;
        case 'm': //decrease speed
        Ref *= 0.8;
        break;
        case 'l': //turn left
        manualSteering = true;
        Bias = -(steeringAmount/Ki);
        break;
        case 'r': //turn right
        manualSteering = true;
        Bias = (steeringAmount/Ki);
        break;
        case 'f': //go forward
        setDirection(0, FORWARD);
        manualSteering = false;
        manualStill = false;
        Ref = initialRef;
        DEBUG_MOTORS = true;
        break;
        case 's': //break
        quickStop();
        manualStill = true;
        DEBUG_MOTORS = false;
        break;
        case 'b': //go backward
        setDirection(0, BACKWARD);
        manualSteering = false;
        manualStill = false;
        Ref = initialRef;
        break;
        case 'i': //info
        msg = "Loop constants:";
        msg += "\nKw\t\t\t";
        msg += (int)(Kw);
        msg += ".";
        msg += (int)((Kw-(int)Kw)*10);
        msg += (int)((Kw*10-(int)(Kw*10))*10);
        msg += (int)((Kw*100-(int)(Kw*100))*10);

        msg += "\nKs\t\t\t";
        msg += (int)(Ks);
        msg += ".";
        msg += (int)((Ks-(int)Ks)*10);
        msg += (int)((Ks*10-(int)(Ks*10))*10);
        msg += (int)((Ks*100-(int)(Ks*100))*10);

        msg += "\nKi\t\t\t";
        msg += (int)(Ki);
        msg += ".";
        msg += (int)((Ki-(int)Ki)*10);
        msg += (int)((Ki*10-(int)(Ki*10))*10);
        msg += (int)((Ki*100-(int)(Ki*100))*10);

        msg += "\nKp (left)\t";
        msg += (int)(Kp[0]);
        msg += ".";
        msg += (int)((Kp[0]-(int)Kp[0])*10);
        msg += (int)((Kp[0]*10-(int)(Kp[0]*10))*10);
        msg += (int)((Kp[0]*100-(int)(Kp[0]*100))*10);

        msg += "\nKp (right)\t";
        msg += (int)(Kp[1]);
        msg += ".";
        msg += (int)((Kp[1]-(int)Kp[1])*10);
        msg += (int)((Kp[1]*10-(int)(Kp[1]*10))*10);
        msg += (int)((Kp[1]*100-(int)(Kp[1]*100))*10);

        msg += "\nInit. Speed\t";
        msg += (int)initialRef;

        msg += " rpm\n# Avg.\t\t";
        msg += (int)avgNum;

        msg += " smp\nSensor wt.\t";
        for(int i=0; i<inputsNr; i++){
          msg += sensorsWeight[i];
          msg += " ";
        }
        Serial.println(msg);
        break;
      }
    }
  }
  

  /*****************/
  /* path handling */
  /*****************/
  
  // TODO testing!!
  if(PATH) {
    switch (stage) {
    case 0: //outside the path
      if(inRange(sensorsSumWeighed,-40,20) && (sensorsSum>=60)) {
        Serial.println("stage 0 entered!");
        LINE_FOLLOWING = false;
        stage = 1;
        Serial.println("stage 0 cleared!");
      }
      break;
    case 1: //entering the path
      if(inRange(sensorsSumWeighed,-20,20) && (sensorsSum<=20)) {
        Serial.println("stage 1 entered!");
        //turn
        quickStop();
        setDirection(1, BACKWARD);
        analogWrite(Enable1, 255);
        setDirection(2, FORWARD);
        analogWrite(Enable2, 255);
        delay(300);

        //restore
        quickStop();
        Integral = 0;
        delay(500);
        LINE_FOLLOWING = true;
        setDirection(0, FORWARD);
        stage = 2;
        Serial.println("stage 1 cleared!");
      }
      break;
    case 2: //line gap: go straight :D
      if((sensorsSum>=65)) {
        Serial.println("stage 2 entered!");
        //sensorsWeight[0] = 0;
        //sensorsWeight[inputsNr-1] = 0;
        SaltoDellaFede = millis();
        LINE_FOLLOWING = false;
        stage = 21;
        Serial.println("stage 2 cleared!");
      }
      break;
    case 21: //end of line gap
      if((millis()-SaltoDellaFede) >= 2500) {
        Serial.println("stage 2.1 entered!");
        LINE_FOLLOWING = true;
        Integral = 0;
        stage = 3;
        Serial.println("stage 2.1 cleared!");
      }
      break;
    case 3: //start semicircle
      if(inRange(sensorsSumWeighed,-10,10) && (sensorsSum<=10)) {
        Serial.println("stage 3 entered!");
        //turn
        quickStop();
        setDirection(1, FORWARD);
        analogWrite(Enable1, 255);
        setDirection(2, BACKWARD);
        analogWrite(Enable2, 255);
        delay(420);

        //circle
        setDirection(0, FORWARD);
        LINE_FOLLOWING = false;
        LineOut = 27;
        stage = 4;
        Serial.println("stage 3 cleared!");
      }
      break;
    case 4: //circle finished
      if(inRange(sensorsSumWeighed,-20,20) && (sensorsSum<=20)) {
        Serial.println("stage 4 entered!");
        //turn
        quickStop();
        setDirection(1, FORWARD);
        analogWrite(Enable1, 255);
        setDirection(2, BACKWARD);
        analogWrite(Enable2, 255);
        delay(420);

        //restore
        quickStop();
        Integral = 0;
        delay(500);
        LINE_FOLLOWING = true;
        Ref *= 1.1;
        setDirection(0, FORWARD);
        stage = 5;
        Serial.println("stage 4 cleared!");
      }
      break;
    case 5: //speed increment
      if(inRange(sensorsSumWeighed,-160,-140) && inRange(sensorsSum,20,40)) {
        Serial.println("stage 5 entered!");
        Ref *= 1.3;
        Ki *= 1.15;
        sensorsWeight[0] = -27;
        sensorsWeight[1] = -22;
        sensorsWeight[2] = -8;

        sensorsWeight[inputsNr-3] = 8;
        sensorsWeight[inputsNr-2] = 22;
        sensorsWeight[inputsNr-1] = 27;


        stage = 6;
        Serial.println("stage 5 cleared!");
      }
      break;
    case 6: //stop and start wall
      if(inRange(sensorsSumWeighed,-10,10) && (sensorsSum<=10)) {
        Serial.println("stage 6 entered!");
        //stop
        quickStop();
        delay(3000);

        //restore
        Integral = 0;
        Ref *=0.5 ;
        LINE_FOLLOWING = false;
        //DEBUG_DISTANCE = true;
        setDirection(0, FORWARD);
        stage = 7;
        Serial.println("stage 6 cleared!");
      }
      break;
    case 7: // front wall: turn right
      if(distanceFront>=65) {
        Serial.println("stage 7 entered!");
        //turn
        quickStop();
        setDirection(1, BACKWARD);
        analogWrite(Enable1, 255);
        setDirection(2, FORWARD);
        analogWrite(Enable2, 255);
        delay(300);

        //restore
        quickStop();
        Integral = 0;
        delay(750);
        LINE_FOLLOWING = false;
        //WALL_FOLLOWING = true;
        setDirection(0, FORWARD);
        stage = 8;
        Serial.println("stage 7 cleared!");
      }
      break;
    
    case 8: // no side wall: turn left
      if(distanceSide<=5) {
        Serial.println("stage 8 entered!");
        //turn
        quickStop();
        delay(150);
        setDirection(1, FORWARD);
        analogWrite(Enable1, 255);
        setDirection(2, BACKWARD);
        analogWrite(Enable2, 255);
        delay(600);

        //restore
        quickStop();
        Integral = 0;
        delay(750);
        LINE_FOLLOWING = false;
        setDirection(0, FORWARD);
        stage = 9;
        Serial.println("stage 8 cleared!");
      }
      break;
    case 9: // side wall again: keep it cool
      if(distanceSide>10) {
        Serial.println("stage 9 entered!");
        stage = 10;
        Serial.println("stage 9 cleared!");
      }
      break;
    case 10: // no side wall again: turn left
      if(distanceSide<=5) {
        Serial.println("stage 10 entered!");
        //turn
        quickStop();
        delay(150);
        setDirection(1, FORWARD);
        analogWrite(Enable1, 255);
        setDirection(2, BACKWARD);
        analogWrite(Enable2, 100);
        delay(800);

        //restore
        quickStop();
        Integral = 0;
        delay(750);
        LINE_FOLLOWING = false;
        setDirection(0, FORWARD);
        stage = 11;
        Serial.println("stage 10 cleared!");
      }
      break;
    
    case 11: // front wall: turn right
      if(distanceFront>=90) {
        Serial.println("stage 11 entered!");
        //turn
        quickStop();
        setDirection(1, BACKWARD);
        analogWrite(Enable2, 200);
        setDirection(2, FORWARD);
        analogWrite(Enable2, 200);
        delay(450);

        //restore
        quickStop();
        Integral = 0;
        delay(750);
        LINE_FOLLOWING = false;
        WALL_FOLLOWING = false;
        setDirection(0, FORWARD);
        stage = 12;
        Serial.println("stage 11 cleared!");
      }
      break;
    case 12: // front wall: stop!!
      if(distanceFront>=70) {
        Serial.println("stage 12 entered!");
        //stop
        quickStop();
        stage = 13;
        Serial.println("stage 12 cleared! You made it!!");
      }
      break;
    case 13:
    quickStop();
      while(true);
      break;
    default:
      //quickStop();
      break;
    }
  }


  /**************/
  /* DEBUG DATA */
  /**************/
  if(DEBUG_MOTORS || DEBUG_SENSORS || DEBUG_SENSORS_THRES || DEBUG_LINEPARAM || DEBUG_DISTANCE) {
    //update current time
    currentDebugTime = micros();
    //runs loop with fixed time interval
    if((currentDebugTime-previousDebugTime) > (1000000UL / DebugFreq)) {
      //update debug communication timer
      previousDebugTime = currentDebugTime;
      
      
      //construct the message in a single string
      msg = "";

      //gather sensors info
      if(DEBUG_SENSORS) {
        for(int i=0; i<inputsNr; i++){
          int realPos = sensorsPos[i];
          msg += "s ";
          msg += realPos;
          msg += " ";
          msg += sensors[realPos];
          msg += "\n";
        }
      }

      //gather sensors tuning info
      if(DEBUG_SENSORS_THRES) {
        for(int i=0; i<inputsNr; i++){
          int realPos = sensorsPos[i];
          msg += sensors[realPos];
          msg += "\t";
        }
        msg += "\n";
      }


      //gathers motors/loop info
      if(DEBUG_MOTORS) {
        msg += "m 1 ";
        msg += (int)Speed1;
        msg += " ";
        msg += (int)Error1;
        msg += " ";
        msg += (int)CSignal1;
        
        msg += "\nm 2 ";
        msg += (int)Speed2;
        msg += " ";
        msg += (int)Error2;
        msg += " ";
        msg += (int)CSignal2;
        
        msg += "\nm 0 ";
        msg += (int)LineOut;
        msg += " ";
        msg += (int)Bias;
        msg += "\n";
      }

      //gathers line parameters
      if(DEBUG_LINEPARAM) {
        msg += "\nWTsum\t";
        msg += sensorsSumWeighed;
        msg += "\t\tsum\t";
        msg += sensorsSum;
      }

      //gathers distance sensors
      if(DEBUG_DISTANCE) {
        msg += "\nFront\t";
        msg += distanceFront;
        msg += "\t\tSide\t";
        msg += distanceSide;
      }

      //send the string
      Serial.print(msg);

      //debug debugging :D
      /*
      msg = "loop freq: ";
      msg += (int)(1000000UL/deltaTime);
      msg += " Hz\t time taken: ";
      msg += (micros()-currentDebugTime);
      msg += " us\n";
      Serial.print(msg);
      */
    }
  }

  if(distanceFront>200) {
    quickStop();
  }
}



//motor1 interrupt
void motor1() {
  //subtract the last reading
  avgTot1 -= avgVals[0][avgCount1];
  //obtain new reading
  avgVals[0][avgCount1] = abs(micros()-Time1);
  //update time counter
  Time1 = micros();
  //add the reading to the total
  avgTot1 += avgVals[0][avgCount1];
  //advance to the next position in the array
  avgCount1++;
  //reset counter
  if(avgCount1 >= avgNum)  avgCount1 = 0;
  //calculate the average time period
  TPeriod1 = avgTot1 / avgNum;
}



//motor2 interrupt
void motor2() {
  avgTot2 -= avgVals[1][avgCount2];
  avgVals[1][avgCount2] = abs(micros()-Time2);
  Time2 = micros();
  avgTot2 += avgVals[1][avgCount2];
  avgCount2++;
  if(avgCount2 >= avgNum)  avgCount2 = 0;
  TPeriod2 = avgTot2 / avgNum;
}

