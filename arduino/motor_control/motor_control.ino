#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <Servo.h>

typedef struct {
  int pin;
  Servo motor;
} Motor;

ros::NodeHandle nodehandle;
const int motor_amount = 8; // = (sizeof motor_pins)/(sizeof motor_pins[0]);
int motor_pins[motor_amount] = {3, 5, 6, 7, 8, 9, 10, 11};
Motor motors[motor_amount];

void setSpeedCallback(const geometry_msgs::Vector3& msg){
  int id = msg.x;
  int value = msg.y;
  if (abs(value-187) > 9) {
    return;
  } 
  analogWrite(id, value);
//  for (int i = 0; i < motor_amount; i++) {
//    if (motors[i].pin == id) {
//      nodehandle.logwarn("Moving motor");
//      motors[i].motor.write(value);
//      break;
//    }
//  }
}

ros::Subscriber<geometry_msgs::Vector3> set_speed("set_motor_pwm", &setSpeedCallback);

void setup() {
  Serial.begin(57600);
  Serial.write("starting");
  nodehandle.initNode();
  nodehandle.subscribe(set_speed);
  while (!nodehandle.connected()) {
    nodehandle.spinOnce();
    delay(1);
  }
  nodehandle.logdebug("connected");
}

void loop() {   
  while (!nodehandle.connected()) {
      for (int i = 0; i < 8; i++) {
          analogWrite(motor_pins[i], 187);
      }
    delay(1);
  }
  nodehandle.spinOnce();
  delay(1);
}
