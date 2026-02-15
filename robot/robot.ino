// IMPORTANT: SERIAL MONITOR MUST BE SET TO "NO LINE ENDING"
int IN1 = 2; int IN2 = 3;
int IN3 = 4; int IN4 = 5;
int ENA = 9; int ENB = 10;
int trigPin = 8;
int echoPin = 7;

float duration, distance;
int baseSpeed =220;

void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  distance = readDistance();
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(100);
  moveForward();
  
  // Obstacle Safety Stop
  if (distance < 10 && distance > 0) {
    stopRobot(); 
    delay(10000);
  }
  if (Serial.available() > 0) {
    int percent = Serial.parseInt(); 
    char command = Serial.read();   
    
    baseSpeed = map(percent, 0, 100, 0, 255);

    switch (command) {
      case 'F': moveForward(); break;
      case 'B': moveBackward(); break;
      case 'L': turnLeft(); break;
      case 'R': turnRight(); break;
      case 'S': stopRobot(); break;
    }
  }
  delay(50); 
}

float readDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH); // 30ms timeout
  return (duration * 0.0343) / 2;
}

void moveForward() {
  analogWrite(ENA, baseSpeed); // left 
  analogWrite(ENB, baseSpeed*0.); // 
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void turnLeft() {

  int boostedSpeed = constrain(baseSpeed * 2, 0, 255);

  analogWrite(ENA, baseSpeed);   
  analogWrite(ENB, boostedSpeed*0.65); 
  
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void turnRight() {

  int boostedSpeed = constrain(baseSpeed * 2, 0, 255);

  analogWrite(ENA, boostedSpeed); 
  analogWrite(ENB, baseSpeed*0.65);   
  
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void moveBackward() {
  analogWrite(ENA, baseSpeed);
  analogWrite(ENB, baseSpeed);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void stopRobot() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}
