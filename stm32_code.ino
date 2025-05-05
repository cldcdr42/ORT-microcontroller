#include <SimpleFOC.h>


// Create instance of magnetic sensor
// For this case - as5600 with I2C
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);


// Create instance of BLDC driver
// BLDCDriver3PWM( pin_pwmA, pin_pwmB, pin_pwmC, enable (optional))
//      See pins on the backside of the driver board
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 10, 6, 8);


// Create instance of BLDC motor
// BLDCMotor(int pp, (optional R, KV))
// - pp  - pole pair number
// - R   - phase resistance value - optional
// - KV  - motor KV rating [rpm/V] - optional
//      Number of pole pairs (14/2=7), phase resistance is taken from documentation
//      KV_rating is choosen using example utils -> calibration -> find_kv_rating -> magnetic_sensor
int const motor_pole_pairs = 7;
float const motor_phase_resistance = 15.3;
int const motor_KV_rating = 60;
BLDCMotor motor = BLDCMotor(motor_pole_pairs, motor_phase_resistance, motor_KV_rating);


// Commander interface constructor
// - serial  - optionally receives HardwareSerial/Stream instance
// - eol     - optionally receives eol character - by default it is the newline: "\n" 
// - echo    - option echo last typed character (for command line feedback) - defualt false
Commander commander = Commander(Serial);
void onMotor(char* cmd){ commander.motor(&motor,cmd); }

// Example of a custom function for a commander interface
/*
void holdAngle(char* cmd){
  // do something useful

  if (cmd[0] == '0') {
    motor.target = 0;
  }

  else if (cmd[0] == '1') {
    if ((motor.shaft_angle * 180 / PI) > (starting_angle + 360)) { motor.target = 0.05; }
    else if ((motor.shaft_angle * 180 / PI) < (starting_angle - 360)) { motor.target = -0.05; }
    else { motor.target = 0; } 
  }
};

*/

float limit_angle = 360;
float limit_torque = 0.05;

float rotation_angle = 0;
float current_angle;

long timestamp_us = _micros();

// Initialize starting angle value
float starting_angle = 0;

void setAngleLimit(char* cmd){
  String myString = String(cmd);
  limit_angle = myString.toFloat();
};

void setTorqueLimit(char* cmd){
  String myString = String(cmd);
  limit_torque = myString.toFloat();
};

void rotateForAngle(char* cmd) {
  String myString = String(cmd);
  rotation_angle = myString.toFloat();
  
  current_angle = motor.shaft_angle * 180 / PI;
  
  if (rotation_angle > 0) {
    while (motor.shaft_angle * 180 / PI < current_angle + rotation_angle) {

      Serial.print("A;");
      Serial.print(millis());
      Serial.print(";");
      Serial.print(motor.shaft_angle * 180 / PI);
      Serial.print("\r\n");

      Serial.print("V;");
      Serial.print(millis());
      Serial.print(";");
      Serial.print(motor.shaft_velocity);
      Serial.print("\r\n");
      
      motor.target = limit_torque;
      motor.loopFOC();
      motor.move();
    } 
  }

  if (rotation_angle < 0) {
    while (motor.shaft_angle * 180 / PI > current_angle + rotation_angle) {

      Serial.print("A;");
      Serial.print(millis());
      Serial.print(";");
      Serial.print(motor.shaft_angle * 180 / PI);
      Serial.print("\r\n");

      Serial.print("V;");
      Serial.print(millis());
      Serial.print(";");
      Serial.print(motor.shaft_velocity);
      Serial.print("\r\n");

      motor.target = -limit_torque;
      motor.loopFOC();
      motor.move();
    }
  }
  
  rotation_angle = 0;
  motor.target = 0;
}


void setup() {
  Serial.begin(115200);
  //SimpleFOCDebug::enable(&Serial);


  // initialize the sensor
  sensor.init();


  // Initialize the driver
  // pwm frequency to be used [Hz]
  //driver.pwm_frequency = 10000;
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // Max DC voltage allowed - default is voltage_power_supply
  driver.voltage_limit = 12;
  // driver init
  driver.init();

  // Link the driver and sensor to a motor
  motor.linkSensor(&sensor);
  motor.linkDriver(&driver);

  // Motion control type
  motor.controller = MotionControlType::torque;
  // Torque control type (foc-current/current/voltage)
  motor.torque_controller = TorqueControlType::voltage;
  
  // Specify number of pole pairs in motor to avoid mistakes from calibration
  motor.pole_pairs = 7;

  // Set limit for speed of rotating (WORKS ONLY IN VELOCITY MODE)
  motor.velocity_limit = 0.1;
  
  // Calibrate and srart FOC algorithm
  motor.init();
  motor.initFOC();
  

  // Command interface to interact with the motor
  /*
      MMG0      get value target
      MMG6      get value angle
      
      ME0       disable motor
      ME1       enable motor
      
      M1        set target to 1 [torque mode - from 0 to 1]
      M0.05     set target to 0.05 [torque mode - from 0 to 1]
  
      @0        no verbouse output
      @2        user-friendly verbose output

      MLV5      set limit to max voltage
  */
  commander.add('M',onMotor,"Configure motor");
  
  // 
  commander.add('A',setAngleLimit,"Set allowed angle in degrees [A70]");
  commander.add('D',setTorqueLimit,"Set allowed torque [D0.05]");
  commander.add('Z',rotateForAngle,"Rotate for a specific angle Z[30]");

  // Custom command for command interface
  //    command.add('L', myFunc, "descr");
  //    'L' - label, any letter
  //    myFunc - name of your dunction
  //    "descr" - description (prompted when typing ?)
  // commander.add('K', holdAngle, "Enter hold angle mode");

  _delay(1000);

  // Update starting angle value since it may not be at 0 due to magnet position
  starting_angle = motor.shaft_angle * 180 / PI;
}

/*
Access angle and velocity:

motor.shaft_angle;      // motor angle
motor.shaft_velocity;   //motor velocity
*/

/*

 !!!!!!!!!

  Target changes from 0 to 1 in torque mode

  I do not know why, because it supposed to change the current through the motor.
  Consider value 1 as maximum possible torque, while 0 - minimun (no torque)

 !!!!!!!!!

*/
//float target_voltage = 0.025;
// float target_voltage = 1;


/*
  default units in this library:


    Position/angle 	    Radians
    Velocity 	          Rad/s
    Torque/Current 	    Amps
*/


void loop() {

  // Printing value (send them to python script to collect in a file)
  Serial.print("A;");
  Serial.print(millis());
  Serial.print(";");
  Serial.print(motor.shaft_angle * 180 / PI);
  Serial.print("\r\n");

  Serial.print("V;");
  Serial.print(millis());
  Serial.print(";");
  Serial.print(motor.shaft_velocity);
  Serial.print("\r\n");
  
  // Holding specific angle
  if ((motor.shaft_angle * 180 / PI) > (starting_angle + limit_angle)) { motor.target = -limit_torque; }
  else if ((motor.shaft_angle * 180 / PI) < (starting_angle - limit_angle)) { motor.target = limit_torque; }
  else { motor.target = 0; } 

  motor.loopFOC();
  motor.move();

  commander.run();
}


/*
How data should be formatted before printing

//printf("X;%d;%i;%i;%i\r\n", int((Kernel::Clock::now() - start).count()), (int16_t)accelerations[0], (int16_t)accelerations[1], (int16_t)accelerations[2]);
//printf("W;%d;%f\r\n", int((Kernel::Clock::now() - start).count()), weight);

// "X;Time_date;Acc1_int;Acc2_int;Acc3_int\r\n"
// "W;Time_date;Weight_flt\r\n"
*/


//serialReceiveUserCommand();

/*
void serialReceiveUserCommand() {

  // a string to hold incoming data
  static String received_chars;

  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;
    // end of user input
    if (inChar == '\n') {

      // change the motor target
      target_voltage = received_chars.toFloat();
      Serial.print("Target voltage: ");
      Serial.println(target_voltage);

      // reset the command buffer
      received_chars = "";
    }
  }
}

*/
