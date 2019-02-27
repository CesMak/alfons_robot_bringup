/**************************************************************************//**
   ROS NODE: VOLTAGE MONITORING SENSOR


   @author  Duncan Iglesias
   @company Arkbro Inc.
   @date    February 2017

   @link

 *****************************************************************************/

// Problems:
// 1) limited memory:
// Do not use more than 89% of dynamic memory otherwise connection fails!!!
//[ERROR] [1551272209.509575]: Unable to sync with device;
//possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino
// 2)
// sometimes servo is referenced instead of motors....


// Connection Battery:
//// Red (of Battery measurement) to A0 on right side
//// Black1 (of Battery measurement) to Gnd on right side of arduino

// Connection Sharp:
//// To test sharp sensor just connect to USB to PC no additional power necessary.
//// Setup connect:
////  Red    --> PIN: A2 (left if green motor connectors are pointing to you)
////  Black  --> PIN: A2 (rightest)
////  Yellow --> PIN: A2 (middle)
//
//// Links:
//// https://www.youtube.com/watch?v=GL8dkw1NbMc

// Connection Servo
//// To test Servo just connect to USB to PC no additional power necessary.
//// Setup connect Servo:
////  Red    --> PIN: to Vin
////  Black  --> PIN: ~5 (rightest)
////  Yellow --> PIN: ~5 (middle)
//
//// The Limits of the Servo are 0 - 180 (byte)
//// My Softlimits are 30 -150
//// Press STR+SHIFT+M to see angle of motor

#include <ros.h>
#include <SharpIR.h>
#include <Servo.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

// SharpSensor:
#define IR 2        // define signal pin of Sharp Sensor
#define model 1080  // used 1080 because model GP2Y0A21YK0F is used 10 to 80 cm

// Battery:
#define K           0.00472199
#define CELLS       1
#define MAX_CELLS   12
#define CRITICAL    0.30

// Servo (to turn camera)
#define SERVO     5    // Servo wird an Pin 5 angesteuert

#define SERVO_RIGHT       60              // Mit welchem Winkel stößt der Sensor auf dem Servo Rechts an?
#define SERVO_LEFT        150             // Das gleiche mit Links
#define LEFT              LOW             // Links drehen bei LOW 0
#define RIGHT             HIGH            // ... und Rechts bei HIGH 1

// Motors:
#define DIR_B     13   // Richtung B
#define DIR_A     12   // Richtung A
#define PWM_B     11   // Geschwindigkeit B
#define BRAKE_A   9    // Bremse A
#define BRAKE_B   8    // Bremse B
#define PWM_A     3    // Geschwindigkeit A
 
// Nützliche Einstellungen:
#define FULL_SPEED        190             // Vollgas ist 255, höher geht nicht. Erstmal nur die halbe Geschwindigkeit...

#define SPEED_TURN_A      150
#define SPEED_BACK_B      80

// SharpSensor:
SharpIR SharpIR(IR, model);
std_msgs::Float64 sharp_dis;
ros::Publisher sharp_dis_pub("robot/sharp_dis", &sharp_dis);

// Servo:
Servo SensorServo;  // Mit diesem Element wird der Servo gesteuert
byte ServoPosition = 90;
boolean TurnServo = RIGHT;
std_msgs::Int32 servo_pos;
ros::Publisher servo_pos_pub("robot/servo_pos", &servo_pos);

// Battery:
double cell_const[MAX_CELLS] =
{
  1.0000, 2.1915, 2.6970, 4.1111,
  4.7333, 6.6000, 6.6000, 7.8293,
  8.4667, 9.2353, 11.0000, 11.0000
};

ros::NodeHandle nh;
sensor_msgs::BatteryState batt_state;

ros::Publisher batteryState("/robot/battery/info", &batt_state);

int dt_ms = 1000;

// setup methods:
void setupBattery()
{
  // Populate battery parameters.
  batt_state.design_capacity          = 2200;  // mAh
  batt_state.power_supply_status      = 2;     // discharging
  batt_state.power_supply_health      = 0;     // unknown
  batt_state.power_supply_technology  = 3;     // LiPo
  batt_state.present                  = 1;     // battery present

  batt_state.location      = "Crawler";        // unit location
  batt_state.serial_number = "ABC_0001";       // unit serial number

  batt_state.cell_voltage = new float[CELLS];  // individual cell health
}

void setupMotors()
{
  //Motor A (rechts) initialisieren
  pinMode( DIR_A, OUTPUT );    // Pin für Richtung Motor A als Ausgang definieren
  pinMode( BRAKE_A, OUTPUT );  // Pin für Bremse Motor A als Ausgang definieren

  //Motor B (links) initialisieren
  pinMode( DIR_B, OUTPUT );    // Pin für Richtung Motor B als Ausgang definieren
  pinMode( BRAKE_B, OUTPUT );  // Pin für Bremse Motor B als Ausgang definieren

  // Beide Bremsen anziehen, HIGH = Bremsen!
  digitalWrite( BRAKE_A, HIGH );
  digitalWrite( BRAKE_B, HIGH );
}

// loop methods:
void loopBattery()
{
  // Battery status.
  double battVoltage = 0.0;
  double prevVoltage = 0.0;

  // Reset Power Supply Health.
  batt_state.power_supply_health = 0;

  // Populate battery state message.
  for (int i = 0; i < CELLS; i++)
  {
    // Read raw voltage from analog pin.
    double cellVoltage = analogRead(i) * K;

    // Scale reading to full voltage.
    cellVoltage *= cell_const[i];
    double tmp = cellVoltage;

    // Isolate current cell voltage.
    cellVoltage -= prevVoltage;
    battVoltage += cellVoltage;
    prevVoltage = tmp;

    // Set current cell voltage to message.
    batt_state.cell_voltage[i] = (float)cellVoltage;

    // Check if battery is attached.
    if (batt_state.cell_voltage[i] >= 2.0)
    {
      if (batt_state.cell_voltage[i] <= 3.2)
        batt_state.power_supply_health = 5; // Unspecified failure.
      batt_state.present = 1;
    }
    else
      batt_state.present = 0;
  }

  // Update battery health.
  if (batt_state.present)
  {
    batt_state.voltage = (float)battVoltage;
    float volt = batt_state.voltage;
    float low  = 3.0 * CELLS;
    float high = 4.2 * CELLS;
    batt_state.percentage = constrain((volt - low) / (high - low), 0.0, 1.0);
  }
  else
  {
    batt_state.voltage = 0.0;
    batt_state.percentage = 0.0;
  }

  // Update power supply health if not failed.
  if (batt_state.power_supply_health == 0 && batt_state.present)
  {
    if (batt_state.voltage > CELLS * 4.2)
      batt_state.power_supply_health = 4; // overvoltage
    else if (batt_state.voltage < CELLS * 3.0)
      batt_state.power_supply_health = 3; // dead
    else
      batt_state.power_supply_health = 1; // good
  }

  // Publish data to ROSSERIAL.
  batteryState.publish( &batt_state );
}

void loopReadSharp()
{
  sharp_dis.data = SharpIR.distance();  // this returns the distance to the object you're measuring in m
  sharp_dis_pub.publish( &sharp_dis );
}

void loopReadServo()
{
  servo_pos.data = SensorServo.read();  // int value
  servo_pos_pub.publish( &servo_pos);
}


// callbacks:
void servoPosCb(const std_msgs::Int32& msg)
{
  SensorServo.write( msg.data );
}

void setDtCb(const std_msgs::Int32& msg)
{
  dt_ms  = msg.data;
}

void setThrottleCb(const std_msgs::Int32 &msg)
{
  if (msg.data > 0)
  {
    //Motoren auf Geradeaus stellen, ...
    digitalWrite( DIR_A, HIGH );
    analogWrite( PWM_A, msg.data );
    // ..Bremsen lösen!
    digitalWrite( BRAKE_A, LOW );
    nh.logwarn(">0");
  }
  else if(msg.data < 0)   // drive backward:
  {
    digitalWrite( DIR_A, LOW );
    analogWrite( PWM_A, (msg.data*-1) );
    digitalWrite( BRAKE_A, LOW );
    nh.logwarn("<0");
  }
  else // stop 
  {
    analogWrite( PWM_A, FULL_SPEED );
    digitalWrite( BRAKE_A, HIGH );
    nh.logwarn("=0");
  }
}

/**************************************************************************//**
  Main Setup

  Initiates the ROS nodes and subscribes to the ROS topics over ROSSERIAL.

******************************************************************************/
ros::Subscriber<std_msgs::Int32> servoPos_sub("/robot/set_servo_pos", &servoPosCb );
ros::Subscriber<std_msgs::Int32> setDtCb_sub("/robot/set_dt", &setDtCb );
ros::Subscriber<std_msgs::Int32> setThrottleCb_sub("/robot/throttle", &setThrottleCb );
void setup()
{
  // Launch ROS node and set parameters.
  nh.initNode();

  // Setup publishers.
  nh.advertise(batteryState);
  nh.advertise(sharp_dis_pub);
  nh.advertise(servo_pos_pub);

  // Setup subscribers:
  nh.subscribe(servoPos_sub);
  //nh.subscribe(setDtCb_sub);
  nh.subscribe(setThrottleCb_sub);

  setupBattery();
  setupMotors();

  // setup Servo:
  SensorServo.attach( SERVO );
  SensorServo.write( 105 );

}


/**************************************************************************//**
  Main Function

  Initiates the ROS nodes and subscribes to the ROS topics to respond to
  incoming motor communication requests. Instantiates the motor controller
  and publishes the current motor position.

  @param argc   --> Number of input arguements.
  @aram argv**  --> Array of input arguments.

******************************************************************************/
void loop()
{
  loopBattery();
  loopReadSharp();
  loopReadServo();

  nh.spinOnce();
  delay(dt_ms);  // wait 1000 ms

}

/*****************************************************************************/


//rostopic echo /crawler/battery/info
//header:
//  seq: 23
//  stamp:
//    secs: 0
//    nsecs:         0
//  frame_id: ''
//voltage: 3.95230555534
//current: 0.0
//charge: 0.0
//capacity: 0.0
//design_capacity: 2200.0
//percentage: 0.793588101864
//power_supply_status: 2
//power_supply_health: 1
//power_supply_technology: 3
//present: True
//cell_voltage: []
//location: "Crawler"
//serial_number: "ABC_0001"

