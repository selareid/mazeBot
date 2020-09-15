#include <MeMCore.h>
#include <SoftwareSerial.h>

int robotSpeed = 200;
int state = -1;
int lastState = -1;
int cyclesInState = 0;
int path[23]; //-1, 0, 1, 2 (left, forward, right, backward/default)

int currentPathPosition = 0;

MeLineFollower lineFollowSensor = MeLineFollower(PORT_2);
MeUltrasonicSensor ultrasonic = MeUltrasonicSensor(PORT_3);
MeDCMotor m1 = MeDCMotor(M1);
MeDCMotor m2 = MeDCMotor(M2);
MeRGBLed led = MeRGBLed();

void setup() {
  Serial.begin(9600);

  for (byte i = 0; i < sizeof(path) / sizeof(path[0]); i++) { //set each path item to default of 2
    path[i] = 2;
  }

  led.setpin(13);
  led.setColor(255, 255, 255);
  led.show();
}

void loop() {
  Serial.println(state);
  if (path[23] != 2) led.setColor(255, 0, 255);

  switch (state) {
    case -1: //prestart state
      if (lineFollowSensor.readSensors() == 0) {
        led.setColor(255, 0, 0);
        changeState(0);
      }
      break;
    case 0: //move forward on line
      switch (lineFollowSensor.readSensors()) {
        case 1: //right white
          changeState(1);
          break;
        case 2: //left white
          changeState(2);
          break;
        case 3: //both white, at intersection
          changeState(3);
          break;
        default:
          robot_forward();
          break;
      }

      break;
    case 1: //tweak right
//      m1.run(robotSpeed);
      m2.run(robotSpeed);
      if (lineFollowSensor.readSensors() == 0 || lineFollowSensor.readSensors() == 3) changeState(0);
      break;
    case 2: //tweak left
      m1.run(-robotSpeed);
//      m2.run(robotSpeed);
      if (lineFollowSensor.readSensors() == 0 || lineFollowSensor.readSensors() == 3) changeState(0);
      break;
    case 3: //go to end of intersection
      robot_forward();
      if (lineFollowSensor.readSensors() != 3) {
        if (floor(random(0, 2)) == 1) { //options for state change: 4, 5, 8 - in this case we have 50% change of straight and 50% change of left/right
          changeState(floor(random(0, 2)) /*random between 0 and 1*/ + 4); //4 or 5
        }
        else {
          changeState(8);
        }
      }
      break;
    case 4: // rotate left
      led.setColorAt(0, 255, 0, 0);
      led.setColorAt(1, 0, 0, 255);
      m1.run(robotSpeed);
      m2.run(robotSpeed);
      if (lineFollowSensor.readSensors() == 3) changeState(6);
      break;
    case 5: // rotate right
      led.setColorAt(1, 255, 0, 0);
      led.setColorAt(0, 0, 0, 255);
      m1.run(-robotSpeed);
      m2.run(-robotSpeed);
      if (lineFollowSensor.readSensors() == 3) changeState(7);
      break;
    case 6: //continue left rotation and detect end
      led.setColorAt(0, 255, 0, 0);
      led.setColorAt(1, 0, 0, 255);
      m1.run(robotSpeed);
      m2.run(robotSpeed);
      if (lineFollowSensor.readSensors() == 0) changeState(10);
      break;
    case 7: //continue right rotation and detect end
      led.setColorAt(1, 255, 0, 0);
      led.setColorAt(0, 0, 0, 255);
      m1.run(-robotSpeed);
      m2.run(-robotSpeed);
      if (lineFollowSensor.readSensors() == 0) changeState(10);
      break;
    case 8: //continue straight - the reason for this is so that we have an intermediary between end of intersection case and check ultrasonic sensor case for consistency
      changeState(10); //check ultrasonic sensor before continuing forward
      break;
    case 9: //turn around
      changeState(path[currentPathPosition] == -1 ? 4 : 5);
      path[currentPathPosition] = 2;
      break;
    case 10: //check ultrasonic sensor
        led.setColor(0, 255, 0);
    
        int sensorRead; //for some reason got error when this was one line
        sensorRead = ultrasonic.distanceCm(400); //400 default if too close or too far
        
        if (sensorRead > 25 && sensorRead != 400) {
          changeState(0);
          path[currentPathPosition] = lastState == 6 ? -1 : lastState == 7 ? 1 : 0; //add to saved path depending on last state
          currentPathPosition++;
        }
        else {
          if (lastState == 6 || lastState == 7) {
            path[currentPathPosition] = lastState == 6 ? -1 : lastState == 7 ? 1 : 0;
            changeState(9);
          }
        }
      break;
    default:
      changeState(0);
  }

  cyclesInState++;
  led.show();
}

int findLeftTurn() { // find left turn based on current turn position
  switch (path[currentPathPosition]) {
    case 2:

      break;
  }
}

void changeState(int newState) {
  lastState = state;
  state = newState;
  cyclesInState = 0;
  robot_stop();
}

void robot_stop() {
  m1.stop();
  m2.stop();
}

void robot_forward() {
  m1.run(-robotSpeed);
  m2.run(robotSpeed);
}

void robot_backward() {
  m1.run(robotSpeed);
  m2.run(-robotSpeed);
}
