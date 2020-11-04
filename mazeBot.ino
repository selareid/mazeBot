#include <MeMCore.h>

int robotSpeed = 200;
int state = -1;
int path[255]; //-1, 0, 1, 2 (left, forward, right, backward/default)
boolean backTracking = false; //is robot going back through maze (due to dead end)
boolean facingIn = false; //to account for certain rotations when backTracking

int currentPathPosition = 0; //current intersection number

//Motors and Sensors
MeDCMotor m1 = MeDCMotor(M1);
MeDCMotor m2 = MeDCMotor(M2);
MeLineFollower lineFollowSensor = MeLineFollower(PORT_2);
MeUltrasonicSensor ultrasonic = MeUltrasonicSensor(PORT_3);
MeRGBLed led = MeRGBLed();

void setup() {
  //set each path item to default of 2
  for (byte i = 0; i < sizeof(path) / sizeof(path[0]); i++) {
    path[i] = 2;
  }

  //set the led pin, color, and show
  led.setpin(13);
  led.setColor(255, 255, 255);
  led.show();

  pinMode(7, INPUT); //button on the robot - set as input
}

void loop() {
  //check for button press at end of maze
  if (state != -2 && analogRead(7) == 0) {
    changeState(-2);
    delay(1000); //delay so that button press isn't read twice/later by the -2 state
  }

  switch (state) {
    case -2: //waiting after button press (end of maze)          
        led.setColor(0, 255, 0);
        
        backTracking = false;
        facingIn = false;
        currentPathPosition = 0;
        
        if (analogRead(7) == 0) { //check for button press
          changeState(0);
          led.setColor(0, 0, 255);
          delay(1000);
        }
      break;
    case -1: //prestart state
      if (lineFollowSensor.readSensors() == 0) { //check for black line
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
    case 1: //tweak right, get back on line
      m2.run(robotSpeed);
      if (lineFollowSensor.readSensors() == 0 || lineFollowSensor.readSensors() == 3) changeState(0); //either on line or at intersection
      break;
    case 2: //tweak left, get back on line
      m1.run(-robotSpeed);
      if (lineFollowSensor.readSensors() == 0 || lineFollowSensor.readSensors() == 3) changeState(0); //either on line or at intersection
      break;
    case 3: //go to end of intersection
      robot_forward();
      
      if (lineFollowSensor.readSensors() != 3) { //check for black line (end of intersection)
        if (path[currentPathPosition] == 2 || backTracking) { //check not using saved path (first run through)
          //new intersection - turn left, update path turn position thing, and reset rotation (facingIn)
          changeState(4);
          facingIn = false;
          path[currentPathPosition]++;
  
          if (path[currentPathPosition] >= 3) { //constrain saved rotation
            path[currentPathPosition] = -1;
          }
        }
        else { //use saved path to do turn
          changeState(path[currentPathPosition] == -1 ? 4 : path[currentPathPosition] == 0 ? 8 : 5); //change state based on saved path
        }
      }
      break;
    case 4: // rotate left
      led.setColorAt(0, 255, 0, 0);
      led.setColorAt(1, 0, 0, 255);
      
      m1.run(robotSpeed);
      m2.run(robotSpeed);
      
      if (lineFollowSensor.readSensors() == 3) changeState(6); //is off the black line
      break;
    case 5: // rotate right
      led.setColorAt(1, 255, 0, 0);
      led.setColorAt(0, 0, 0, 255);
      
      m1.run(-robotSpeed);
      m2.run(-robotSpeed);
      
      if (lineFollowSensor.readSensors() == 3) changeState(7); //is off the black line
      break;
    case 6: //continue left rotation and detect end
      led.setColorAt(0, 255, 0, 0);
      led.setColorAt(1, 0, 0, 255);
      
      m1.run(robotSpeed);
      m2.run(robotSpeed);
      
      if (lineFollowSensor.readSensors() == 0) changeState(10); //is on the next black line
      break;
    case 7: //continue right rotation and detect end
      led.setColorAt(1, 255, 0, 0);
      led.setColorAt(0, 0, 0, 255);
      
      m1.run(-robotSpeed);
      m2.run(-robotSpeed);
      
      if (lineFollowSensor.readSensors() == 0) changeState(10); //is on the next black line
      break;
    case 8: //continue straight - the reason for this is so that we have an intermediary between end of intersection case and check ultrasonic sensor case for consistency
      changeState(10); //check ultrasonic sensor before continuing forward
      break;
    //case 9: //this state was made redundant, kept here to reduce reader confusion (due to missing state 9)
    //  break;
    case 10: //check ultrasonic sensor
        int sensorRead; //for some reason got error when declaraction and assignment were on one line
        sensorRead = ultrasonic.distanceCm(400); //400 default if too close or too far
        
        if (sensorRead > 25) { //no wall detected
          //increase path position, unless backtracking (going backward)
          if (path[currentPathPosition] == 2) {
            currentPathPosition--;
            led.setColor(0, 255, 50);
          }
          else {
            //reset rotation and backTracking
            backTracking = false;
            facingIn = false;
            
            led.setColor(0, 50, 255);
            currentPathPosition++;
          }
          
          changeState(0);
        }
        else {
          if (backTracking) { //going back through the maze (due to dead end)
            switch (path[currentPathPosition]) { //saved positions are from starting position (when approaching intersection from maze start)
              case -1: //facing right, unless facingin is false
                //try next path (top)
                changeState(facingIn ? 4 : 5); //rotate based on current rotation
                path[currentPathPosition] = 0;
                facingIn = false;
                break;
              case 0: //facing down, unless facingin is false
                //try next path (right)
                changeState(facingIn ? 4 : 5); //rotate based on current rotation
                path[currentPathPosition] = 1;
                facingIn = false;
                break;
              case 1: //facing left, unless facingin is false
                //have to backtrack more (go further back down the maze)
                changeState(facingIn ? 4 : 5); //rotate based on current rotation
                path[currentPathPosition] = 2;
                facingIn = false;
                break;
            }
          }
          else {  
            switch (path[currentPathPosition]) {
              case -1: //try next intersection (clockwise)
                changeState(5);
                path[currentPathPosition] = 0;
                break;
              case 0: //try next intersection (clockwise)
                changeState(5);
                path[currentPathPosition] = 1;
                break;
              case 1: //no available intersections, now reversing
                changeState(5);
                path[currentPathPosition] = 2;
                backTracking = true;
                facingIn = true;
                break;
            }
          }
        }
      break;
    default:
      changeState(0); //go straight
  }

  led.show();
}

void changeState(int newState) {
  state = newState;
  robot_stop(); //chose to stop the robot each state change for consistency
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
