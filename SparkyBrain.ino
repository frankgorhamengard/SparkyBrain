/*This is code for the Sparky Robot brain with IR ball sensor
 * From SparkyProjects

This sketch recieves command values from the control Panel to control the Sparky robot.

It has an LED to indcate link status

Developed by Miss Daisy FRC Team 341
*/

#include <Servo.h>
#include <EasyTransfer.h>
#include <SparkyXfrBuffers.h>
#include <EEPROM.h>

// declare local mode routines
void enabledState(void);
void notEnabledState(void);
void calibrationAndTests(void);

//create two transfer objects
EasyTransfer ETin, ETout; 

//  global timing variables
unsigned long lastUpdateTime = 0; // asynchronous link verification 
unsigned long lastBlinkToggle = 0; // link status LED, follows its own definite timing
unsigned long afterCalDwellTimeEnd;
unsigned long nextBallCheckTime;
unsigned long loopTimeNow;
 
// filter accumulators
int VIN_accum;

// Message count tracking
long int lastMessageCounter = 0;
long int messageDropCounter = 1000;
unsigned long commOpTime = millis();

// Enable flags
boolean commGoodFlag = false;

//  declare servo objects
Servo leftDriveMotor;
Servo rightDriveMotor;
Servo intakeMotor;
Servo conveyorMotor;
Servo shooterMotor;

// Servo ouptut is from 0 to 180
const int servoHaltVal     = 90;   // 90 is no motion
const int servoFullForeVal = 180;  // 180 is full forward
const int servoFullBackVal = 0;    // 0 is full reverse

// Stick input is from 0 to 1023
const int stickHaltVal     = 512;   // this is center, no motion

// declare EEPROM addresses and variables to store values locally
const int maxKnobAddr = 0;
int16_t shootSpeedKnobMax;  // size of int16_t is always 2
const int minKnobAddr = 2;
int16_t shootSpeedKnobMin;

//  declare the transfer buffers
TO_SPARKY_DATA_STRUCTURE rxdata;
FROM_SPARKY_DATA_STRUCTURE txdata;

const int BALL_OVERRIDE_2      = 2;
const int TEST_SWITCH2_PIN_4      = 4;
const int HC05_POWER_LOW_ON_8    = 8;
const int BALL_SEEN_10     = 10;     // IR receiver, HIGH when no IR seen (when ball present)
const int LINK_STATUS_LED_11 = 11;   // IR transmitter is plugged in to 11 to get power, no connection to signal
const int GREEN_SHOOT_LED_12  = 12;
const int LINK_DATA_LED_13   = 13;
const int VIN_PIN_A0          = A0;


///////////////////// SETUP, called once at start ///////////////////////////////////////////////
void setup(){
  // Serial is connected throught the Bluetooth modules to the master
  Serial.begin(9600);
  while (!Serial) ; // wait for serial port to connect. Needed for native USB
  // print this before the HC-05 is connected so we can connect a computer and see what code it is.
  Serial.write("Code for the Sparky Robot brain with IR ball sensor 20190824");

  //start the library, pass in the data details and the name of the serial port.
  ETin.begin(details(rxdata), &Serial);
  ETout.begin(details(txdata), &Serial);

  // set transmitter buffer to default values
  txdata.buttonstate = HIGH;
  txdata.supplyvoltagereading = analogRead(VIN_PIN_A0);
  txdata.ballready = false;
  txdata.packetreceivedcount = 0;

  // init rxdata to safe values, in case they are used before first packet sets them
  rxdata.stickLx = stickHaltVal;
  rxdata.stickLy = stickHaltVal;
  rxdata.stickLbutton = LOW;
  rxdata.stickRx = stickHaltVal;
  rxdata.stickRy = stickHaltVal;
  rxdata.stickRbutton = LOW;
  rxdata.intake = LOW;      // the low active switches on the panel are inverted before sending
  rxdata.shoot = LOW;       // so now all 4 are high active
  rxdata.drivemode = LOW;
  rxdata.enabled = LOW;
  rxdata.counter = -1;

const int myPulseWidthMax = 2000;
const int myPulseWidthMin =1000;

// pin 0 is rx, 1 is tx - for serial port, not used as DIO
///   Servo pins   //////////     digital pins   /////////////////////////////////////
                            pinMode(BALL_OVERRIDE_2, INPUT_PULLUP);
  leftDriveMotor.attach(3,myPulseWidthMin,myPulseWidthMax);
                            pinMode( TEST_SWITCH2_PIN_4, INPUT_PULLUP);
  rightDriveMotor.attach(5,myPulseWidthMin,myPulseWidthMax);
  intakeMotor.attach(6,myPulseWidthMin,myPulseWidthMax);
  shooterMotor.attach(7,myPulseWidthMin,myPulseWidthMax);
                           digitalWrite( HC05_POWER_LOW_ON_8, HIGH);  // make sure the HC-05 power starts out off 
                           pinMode(HC05_POWER_LOW_ON_8, OUTPUT);  //  pin 8 used to power on Bluetooth module
  conveyorMotor.attach(9,myPulseWidthMin,myPulseWidthMax);
                            pinMode(BALL_SEEN_10, INPUT_PULLUP);  //  
                            pinMode(LINK_STATUS_LED_11, OUTPUT);  
                            pinMode(GREEN_SHOOT_LED_12, OUTPUT); // drive signal for spike on green LEDs
                            pinMode(LINK_DATA_LED_13, OUTPUT);       // link data test output

  notEnabledState();  // make sure everything is off

  /////   sync with EEPROM
  EEPROM.get( minKnobAddr, shootSpeedKnobMin);  // read int in
  if ( shootSpeedKnobMin < 0 || shootSpeedKnobMin > 256 ) { // if not in range, probably never calibrated
    shootSpeedKnobMin = 256; // min must be positive, start high, calibrate down at runtime
    EEPROM.put( minKnobAddr, shootSpeedKnobMin);
  }
  EEPROM.get( maxKnobAddr, shootSpeedKnobMax);
  if ( shootSpeedKnobMax < 768 || shootSpeedKnobMax > 1023 ) { // if not in range, probably never calibrated
    shootSpeedKnobMax = 768; // Max must be positive, start low, calibrate up at runtime
    EEPROM.put( maxKnobAddr, shootSpeedKnobMax);
  }

  rxdata.shooterspeed = shootSpeedKnobMin; //  lowest known speed
  afterCalDwellTimeEnd = millis();  // setting to now means the dwell is over, no dwell required

  commGoodFlag = false;

  digitalWrite( HC05_POWER_LOW_ON_8, LOW);  // this turns on power to the HC-05
}

///////    the MAIN asynchronous loop, called repeatedly    /////////////////////////////
void loop(){

  unsigned long now = millis();
  if ( now > commOpTime ) {
    commOpTime = now + 60;
      
    // we run the recieve function more often then the transmit function. 
    // This is important due to the slight differences in 
    // the clock speed of different Arduinos. If we didn't do this, messages 
    // would build up in the buffer and appear to cause a delay.
  
    // first Check if new data packets were received
    if ( ETin.receiveData() ) {
      txdata.packetreceivedcount++;
      lastUpdateTime = millis();
    } else {
      delay(10);   // wait a bit if no packet yet, resync to control
      if (ETin.receiveData()){
        txdata.packetreceivedcount++;
        lastUpdateTime = millis();
      }
    }
    // Check again in case 2 packets were waiting
    if (ETin.receiveData()){
      txdata.packetreceivedcount++;
      lastUpdateTime = millis();
    }
      
    // each time, we will unconditionally go ahead and send the data out
    txdata.transmitpacketcount++;
    ETout.sendData();

    //  check the received data
    if ((rxdata.counter - lastMessageCounter) == 0){
      messageDropCounter += 1;
    }else{
      messageDropCounter = 0;
    }
    
    commGoodFlag = messageDropCounter <= 10;
    if ( !commGoodFlag ) {
      notEnabledState();     // comm lost stop everything
    } else {
      if ( rxdata.enabled ) {   // receiveing enable
        // An update was received recently, process the data packet
        enabledState();
      } else {
        // There are no new data packets to control the robot, or not enabled
        // turn everything off so the robot doesn't continue issuing the last received commands
        notEnabledState();
      }
    }
    lastMessageCounter = rxdata.counter;
  }   //  end of comm op
  
  //  check for TEST mode
  if ( !digitalRead( TEST_SWITCH2_PIN_4)  ) {  // LOW is active
    txdata.buttonstate = MCUCR; // instead of buttonstate, reuse to read register , for no particular reason
    calibrationAndTests();      // run testing routine, returns immediately unless cal is signaled
  } else {
    txdata.buttonstate = -1;  // TEST not active
  }

  // scale and filter the voltage reading, accum is 16 times reading
  VIN_accum = VIN_accum - (VIN_accum>>4)  + ((analogRead(VIN_PIN_A0)*32)/20);
  txdata.supplyvoltagereading = VIN_accum>>4;

  loopTimeNow = millis();
  // this is where we determine whether a ball is in the sensor, every 100 milliseconds synchronous
  if ( loopTimeNow > nextBallCheckTime) {
    static byte count;
    nextBallCheckTime = loopTimeNow + 50;   //check every tenth of second
    if ( !digitalRead( BALL_SEEN_10) ) {  // if ball not seen
      if ( txdata.ballready == true ) {
        if ( ++count > 1 ) {
          txdata.ballready = false;;
        } 
      } else {                                  //// debouncing code, one count
        count = 0;  
      }
    } else {   // ball seen
      if ( txdata.ballready == false ) {
        if ( ++count > 1 ) {
          txdata.ballready = true;;
        } 
      } else {
        count = 0;  
      }
    }
  }
} // end of loop

///////////////////  apply stick profile  //////////////////
// argument:   stick value from receive buffer, 0 - 1023
// Return:     servo value, 0-179, scaled and profile applied
// profile:   Apply a deadband to all joystick values so that 
//            anything between +/-50 from stick center is converted to servo center.
const long int deadband = 10;
//------------------------------------------------
int convertStickToServo(int stickValue) {
  long int longServoValue;  // use longs, 180*1024 > 32768

  if ( stickValue > (stickHaltVal + deadband) ) {
    longServoValue = ( ((long)(stickValue - deadband)) * 180) >> 10; // /1024; // servo range / stick range
  } else {
    if ( stickValue < (stickHaltVal - deadband) ) {
      longServoValue = ( ((long)(stickValue + deadband)) * 180) >> 10; //  /1024;
    } else {
      longServoValue = servoHaltVal; //else stick in deadband, set Halt value
    }
  }
  return (int) longServoValue;
}


//////////////  stop all activity if communications not working or disabled
void notEnabledState(){
  static unsigned long disTimeNow;
  static unsigned long periodLength;
  // One or more conditions are not satisfied to allow the sparky to operate, disable all motors

  // Set all speed controllers to output 0V.
  leftDriveMotor.write(servoHaltVal);
  txdata.leftmotorcommand = servoHaltVal;
  rightDriveMotor.write(servoHaltVal);
  txdata.rightmotorcommand = servoHaltVal;
  intakeMotor.write(servoHaltVal);
  conveyorMotor.write(servoHaltVal);
  shooterMotor.write(servoHaltVal);
  txdata.shooterspeedecho = -rxdata.shooterspeed; 
  digitalWrite( GREEN_SHOOT_LED_12, 0 );   // 0 is off
 
  disTimeNow = millis();
  if (  lastBlinkToggle + periodLength < disTimeNow ) {
    lastBlinkToggle = disTimeNow;  // triggered and reset.
    if ( commGoodFlag )  {  // its just the switch off
      if ( bitRead( PORTB,3) ) {   // this how to read an output pin
        digitalWrite( LINK_STATUS_LED_11, LOW);
        periodLength = 100;
      } else {
        digitalWrite( LINK_STATUS_LED_11, HIGH);
        periodLength = 700;
      }  
    } else { // comm down, switch is off, slow blink
      if ( bitRead( PORTB,3) ) {   // this how to read an output pin
        digitalWrite( LINK_STATUS_LED_11, LOW);
        periodLength = 500;
      } else {
        digitalWrite( LINK_STATUS_LED_11, HIGH);
        periodLength = 1500;
      }
    }
  }
}  // end of disabled state

 
/////////////////////  enabledState   /////////////////////////////
void enabledState(){
  int shooterSpeed, rawShooterSpeed;
  static unsigned long shootReleaseTime;
  static int updownVal;

//FOR NOW DON'T DO THE FILTER  
  updownVal = rxdata.stickLx;
//  if (txdata.leftmotorcommand == servoHaltVal) {
//    updownVal = 512;  //reset, it looks like we just entered enabled mode
//  }
//   
//   // If in the enabled state, the sparky bot is allowed to move 
//
//  // get and filter the stick values
//  int newupdownValDif = rxdata.stickLx - updownVal;
//  if ( newupdownValDif > 1 ) newupdownValDif = 1;
//  if ( newupdownValDif < -1 ) newupdownValDif = -1;
//  if ( updownVal < 522 && updownVal >502 )
//    updownVal += newupdownValDif*64;
//  else
//    updownVal += newupdownValDif;
  
  // Steer the robot based on selected drive mode
  int leftMotorSpeed = servoHaltVal;
  int rightMotorSpeed = servoHaltVal;
  if (rxdata.drivemode < 1){
    // Tank Mode - left joystick control left drive, right joystick controls right drive
    leftMotorSpeed  = convertStickToServo(updownVal); 
    rightMotorSpeed = convertStickToServo(rxdata.stickRx);
  } else {
    // Arcade Mode - left joystick controls speed, right joystick controls turning
    int speedVal = convertStickToServo(updownVal);
    int  turnVal = convertStickToServo(rxdata.stickRy);
    leftMotorSpeed  = speedVal + turnVal - 90;
    rightMotorSpeed = speedVal - turnVal + 90;
    
    if ( leftMotorSpeed < 0  ) leftMotorSpeed  = 0;    // eg.   0   + 0   - 90
    if ( leftMotorSpeed > 180) leftMotorSpeed  = 180;  // eg.   180 + 180 - 90
    if (rightMotorSpeed < 0  ) rightMotorSpeed = 0;    // eg.   0   - 180 + 90
    if (rightMotorSpeed > 180) rightMotorSpeed = 180;  // eg.   180 - 0   + 90
  }
  // Issue the commanded speed to the drive motors
  // both motors spin full clockwise for 180, left motor mounted opposite direction, so
  if ( txdata.leftmotorcommand != leftMotorSpeed ) {
    leftDriveMotor.write(leftMotorSpeed); // left wheel must spin opposite
    txdata.leftmotorcommand = leftMotorSpeed;
  }
  if ( txdata.rightmotorcommand != rightMotorSpeed ) {
    rightDriveMotor.write(180 - rightMotorSpeed);
    txdata.rightmotorcommand = rightMotorSpeed;
  }
  
///////  INTAKE FUNCTIONS: INTAKE ALWAYS ENABLED, LOADER ONLY ENABLED WHEN BALL IS NOT IN SHOOTER  ////////////
//      SHOOTER FUNCTIONS: LOADER ONLY ENABLED WHEN BALL IS IN SHOOTER

  ///////  calculate shooter wheel speed    ////////////
  // 1023 is highest speed, 0 is lowest
  rawShooterSpeed = rxdata.shooterspeed; // a natural 0 low, 1023 high
  if ( rawShooterSpeed > shootSpeedKnobMax  ) { //recalibrate, first time seeing value this high
    shootSpeedKnobMax = rawShooterSpeed;
    EEPROM.put( maxKnobAddr, shootSpeedKnobMax);
  }
  if ( rawShooterSpeed < shootSpeedKnobMin  ) { //recalibrate, first time seeing value this low
    shootSpeedKnobMin = rawShooterSpeed;
    EEPROM.put( minKnobAddr, shootSpeedKnobMin);
  }
  // Map the potentiometer dial (min to max) to a valid positive shooter speed (90+20 to 180), 20 is min speed
  shooterSpeed = map(rawShooterSpeed, shootSpeedKnobMin, shootSpeedKnobMax, servoHaltVal-20, servoFullBackVal);
  txdata.shooterspeedecho = shooterSpeed;      // assigning shooterSpeed to echo for testing
  
  // both operands converted to boolean and compared, this is a logical XOR operation, override inverts ballready
  if ( (boolean)txdata.ballready == (boolean)digitalRead(BALL_OVERRIDE_2) ) {  // txdata.ballready is the stored synchronous result
    //  light the green ballLEDs   TBD
    digitalWrite( GREEN_SHOOT_LED_12, 1 );   // 1 is on
    // Run the shooter
    shooterMotor.write(shooterSpeed);
    if (rxdata.shoot > 0 ) {    // shooter button pressed
      shootReleaseTime = millis() + 1500;   // trigger and hold shoot even if button released
    } else {
      if ( shootReleaseTime < millis() ) {   // if shoot was not triggered yet
        // Stop the conveyor
        conveyorMotor.write(servoHaltVal);
      }
      else
      {     // shoot in progress, button not pressed, but ball still present = extend time ubtil after ball is moved
        shootReleaseTime = millis() + 1500; 
      }
    }
    if (rxdata.intake > 0){  // intake button pressed 
      // Run the intake for loading extra balls, control only the intake, not the conveyor
      intakeMotor.write(servoFullBackVal);
    } else {
      // Stop the intake
      intakeMotor.write(servoHaltVal);
    }
  } else {   // no ball   //////////////
    // turn green LEDs off
    digitalWrite( GREEN_SHOOT_LED_12, 0 );   // 0 is off
    shooterMotor.write(servoHaltVal);   ///off shooterSpeed);
    txdata.shooterspeedecho = servoHaltVal;
    if (rxdata.intake > 0){  // intake button pressed 
      // Run the intake feed
      intakeMotor.write(servoFullBackVal);
      conveyorMotor.write((servoFullBackVal*3)/4);  // (.75 speed)
      digitalWrite(LINK_DATA_LED_13, LOW); 
    } else {
      // Stop the intake
      intakeMotor.write(servoHaltVal);
      conveyorMotor.write(servoHaltVal);
      digitalWrite(LINK_DATA_LED_13, HIGH); 
    }
  }

  // if a shoot is in progress
  if ( shootReleaseTime >= millis() ) {   // if shoot release is in the future
    // Run the conveyor forward
    conveyorMotor.write(servoFullBackVal);
    // ball may be gone from sensor, but shoot still in progress, run shooter motor
    shooterMotor.write(shooterSpeed);  
    digitalWrite( GREEN_SHOOT_LED_12, 1 );   // 1 is on
  }
  
  // do a slow blink to show enabled and running
  if ( lastBlinkToggle < millis()-1000 ) { //if more than a second ago
    lastBlinkToggle = millis();
    if ( bitRead( PORTB, 3) ) {   // check the output pin
      digitalWrite( LINK_STATUS_LED_11, LOW);
    } else {
      digitalWrite( LINK_STATUS_LED_11, HIGH);
    }
  }
}   // end of enabledState()

////////////   doCalibrationSweep   //////////////
void doCalibrationSweep( Servo *theservo ) {   // note that this routine is blocking
                                               // to prevent changes to calibration timing
  // make a calibration start sound  TBD

  // sweep the servo command up and down in case the CAL button wasn't correctly pushed,
  //  in which case the motors will move, we don't want to jerk them around
  for (int i = 90; i >= 5; i -= 5 ) {
    theservo->write( i );
    delay(50);
    digitalWrite( LINK_STATUS_LED_11,  !bitRead( PORTB,3) ); // flip the LED fast
  }
  theservo->write(0); 
  delay(1000);  // allow time for controller to store minimum value
  for (int i = 0; i <= 175; i += 5 ) {
    theservo->write( i );
    delay(50);
    digitalWrite( LINK_STATUS_LED_11,  !bitRead( PORTB,3) ); // flip the LED fast
  }
  theservo->write(180); 
  delay(1000);   // allow time for controller to store maximum value
  for (int i = 180; i >= 95; i -= 5 ) {
    theservo->write( i );
    delay(50);
    digitalWrite( LINK_STATUS_LED_11,  !bitRead( PORTB,3) ); // flip the LED fast
 }
  theservo->write(90);   //servoHaltVal
  delay(1000);   // allow controller to store center value
  digitalWrite( LINK_STATUS_LED_11,  HIGH ); // flip the LED off to end
  delay(3000);    //before going back to test mode allow time for operator to release the CAL button
   
  // make a calibration done sound  TBD
}

//  ////////////////   SW 2 is on, test mode.     //////////////
// the sweep routine blocks for timing, so afterward we need to not check for tests again 
//   until the rest of the system has recovered from being blocked
void calibrationAndTests(){
  static int shootButtonCount, intakeButtonCount;
  /////// code for calibrating computer controlled servos
  //  the CAL button on the controller is depressed before pushing intake and/or shoot
  if ( afterCalDwellTimeEnd < millis() ) {// ok to start a test
    if (rxdata.intake > 0) {  // intake button pressed 
      if ( ++intakeButtonCount >= 2 ) {   // intake button still pressed, start a cal
        intakeButtonCount = 0;  // count was seen, reset and do a CAL
        if (rxdata.shoot > 0 ) { // shooter button now also pressed
          doCalibrationSweep( &shooterMotor );
        } else {   // only the intake pressed
          doCalibrationSweep( &intakeMotor );
        }
        afterCalDwellTimeEnd = millis() + 2000; // don't start another test for 2 seconds
      }
    } else {
      intakeButtonCount = 0;  
    }
    if (rxdata.shoot > 0){  // shoot button pressed 
      if ( ++shootButtonCount >= 2 ) {   // intake button still pressed, start a cal
        shootButtonCount = 0;  // count was seen, reset 
        if (rxdata.intake > 0 ) { // intake button now also pressed
          doCalibrationSweep( &shooterMotor );
        } else {   // only the intake pressed
          doCalibrationSweep( &conveyorMotor );
        }
        afterCalDwellTimeEnd = millis() + 2000; // don't start another test for 2 seconds
      }
    } else {
      shootButtonCount = 0;  
    }
  }
 
  // do a quick blink
  if ( lastBlinkToggle < millis()-100 ) { //if more than a 1/5 second ago
    lastBlinkToggle = millis();
    static int interval = 0;
    if ( !bitRead( PORTB,3) ) {   // this how to read an output pin
      digitalWrite( LINK_STATUS_LED_11, HIGH);
    } else {
      if ( ++interval > 20 ) {
        digitalWrite( LINK_STATUS_LED_11, LOW);
        interval = 0;
      }
    }
  }  // end of blink
} // end of function calibrationAndTests
