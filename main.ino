#include <AFMotor.h>
#include <Servo.h>
#include <math.h> 

AF_DCMotor motor1(1); // top right motor
AF_DCMotor motor2(2); // top left motor
AF_DCMotor motor3(3); // bottom right motor
AF_DCMotor motor4(4); // bottom left motor

const int servoPin = 9;    // Pin for the servo motor
const int triggerPin = A0;   // Trigger pin for the ultrasonic sensor
const int echoPin = A1;      // Echo pin for the ultrasonic sensor
const int detectionDistance = 30;  // Distance threshold for object detection in centimeters
const int yellowPin = A3; // Pin for yellow LED
const int redPin = A4;    // Pin for red LED
const int greenPin = A5;   // Pin for green LED
const int degreeTurn = 3;  // degree for each turn of servo motor

int loopDelay = 500;
char data;  // variable for incoming bluetooth data
int oneFullTurnTime = 3800;
const int distanceToPass = 40;
int cluster;
int maxSpeed = 130;


Servo myServo;


void setup(){
  Serial.begin(9600);
  
  pinMode(yellowPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);

  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);

  myServo.attach(servoPin);
  myServo.write(90);

  cluster = calculateCluster(detectionDistance,distanceToPass);
}

void ledState(char color){

  if (color == 'R'){
    digitalWrite(A3, LOW);
    digitalWrite(A4, HIGH); 
    digitalWrite(A5, LOW); 
  }

  else if (color == 'G'){
    digitalWrite(A3, LOW);
    digitalWrite(A4, LOW);
    digitalWrite(A5, HIGH); 
  }

  else if (color == 'Y'){
    digitalWrite(A3, HIGH); 
    digitalWrite(A4, LOW); 
    digitalWrite(A5, LOW); 

  }
  else{

  }
}

void forward(){
    myServo.write(90);
    ledState('G');

    // motor1.setSpeed(maxSpeed);
    // motor2.setSpeed(maxSpeed);
    // motor3.setSpeed(maxSpeed);
    // motor4.setSpeed(maxSpeed);

    // motor1.run(FORWARD);
    // motor2.run(FORWARD);
    // motor3.run(FORWARD);
    // motor4.run(FORWARD);
      for(int i = 0; i < maxSpeed; i+=3){
        motor1.run(FORWARD);      
        motor2.run(FORWARD);
        motor3.run(FORWARD); 
        motor4.run(FORWARD);  
        motor1.setSpeed(i);
        motor2.setSpeed(i);
        motor3.setSpeed(i);
        motor4.setSpeed(i);
        delay(5);
  }
}

void stop(){
    myServo.write(90);
    ledState('R');

    motor1.setSpeed(0);
    motor2.setSpeed(0);
    motor3.setSpeed(0);
    motor4.setSpeed(0);

    motor1.run(RELEASE);
    motor2.run(RELEASE);
    motor3.run(RELEASE);
    motor4.run(RELEASE);
}

void left(){
  myServo.write(90);
  ledState('Y');

  motor1.setSpeed(130);
  motor2.setSpeed(80);
  motor3.setSpeed(130);
  motor4.setSpeed(80);

  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
}
void right(){
  myServo.write(90);
  ledState('Y');

  motor1.setSpeed(80);
  motor2.setSpeed(130);
  motor3.setSpeed(80);
  motor4.setSpeed(130);

  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
}

void back(){    
  myServo.write(90);
  ledState('R');

  motor1.setSpeed(80);
  motor2.setSpeed(80);
  motor3.setSpeed(80);
  motor4.setSpeed(80);

  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

bool checkValid(char temp){
  char valid[] = {'F','f','S','s','L','l','R','r','B','b'};
  bool isValid = false;
  for (int i = 0; i < sizeof(valid);i++){
    if (temp == valid[i]){
      isValid = true;
      break;
    }
  }
  return isValid;
}


int calculateCluster(int obstacleDistance,int minDistanceToPass){
  float equalSide = pow((pow(obstacleDistance,2)+pow(minDistanceToPass,2)),0.5);
  float angle = 2* (asin((minDistanceToPass/2)/equalSide));
  float angleDegree = (180/PI) * angle;
  int cluster = angleDegree/degreeTurn;
  return cluster;
}


bool primaryCheck() {
  myServo.write(90);
  long distance = calculateDistance();
  if (distance < detectionDistance){
    long distance1 = calculateDistance();
    delay(100);
    long distance2 = calculateDistance();
    delay(100);
    long avgDistance = (distance1 + distance2) / 2;
    if (avgDistance < detectionDistance){
      return true;
    }
    else{
      return false;
    }
  }
}

void execute(char temp){
  if (checkValid(temp)){
    char lower_command = tolower(temp);
    if (lower_command == 'f'){ forward(); }
    else if (lower_command == 's'){ stop(); }
    else if (lower_command == 'l'){ left(); }
    else if (lower_command == 'r'){ right(); }
    else if (lower_command == 'b'){ back(); }
    else{ stop(); }
  }
  else{
    stop();
  }
  primaryCheck();
}

long calculateDistance(){
  long duration, distance;
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;  // Convert duration to distance in cm
  if(distance==0){
    return 200;
  }
  return distance;

}

void scan(long* distanceArr){
    for (int angle = 0; angle <= 180; angle += degreeTurn) {
      myServo.write(angle);
      long distance = calculateDistance();
      distanceArr[angle/degreeTurn] = distance;
      delay(50);
    }
}

int analyse(){
  int arraySize = (180 / degreeTurn) + 1;
  long distanceArr[arraySize];
  long averageArr[arraySize];
  scan(distanceArr);
  for(int i = 0;i <= (arraySize-1);i++){
    int sum = 0;
    for(int j = 1;j <= (arraySize-1);j++){
      if((j-i) == cluster){
        break;
      }
      sum += distanceArr[i];
    }
    int average = sum/cluster;
    averageArr[i] = average;
  }
  int max = averageArr[0];
  int index = 0;
  for(int i = 0; i < (arraySize-1);i++){
    if(averageArr[i] > max){
      max = averageArr[i];
      index = i;
    }
  }
  return index;
}

void turnLeftRight(int index){
  if(index<30){
    int degree = 90-(3*index);
    float tempTime = (oneFullTurnTime / 360.0) * degree;
    holdFunction(2,tempTime+200);
  }
  else{
    int degree = (3*index)-90;
    int tempTime = (oneFullTurnTime / 360.0) * degree;
    holdFunction(3,tempTime+200);
  }
}


void holdFunction(int identifier,int duration){
  long start = millis();
  while (millis()-start < duration){
    if (identifier == 0){
      stop();  
    }
    else if (identifier == 1){
      back();
    }
    else if (identifier == 2){
      right();
    }
    else if (identifier == 3){
      left();
    }
    else{
    }
  }
}

void loop() {
  if (Serial.available()) {
      data = Serial.read();
      if (data == 's' || data == 'S'){
        stop();
      }
      else{
          execute(data);
      }  
    }

  if (primaryCheck()) {
    holdFunction(1,1000);
    myServo.write(90);
    int index = analyse();
    Serial.println(index);
    turnLeftRight(index);
    } 
  else {
    if (Serial.available()) {
      data = Serial.read();
      delay(loopDelay);
      execute(data);
    }
    execute(data);
  }
  execute(data);
}
