#include <Servo.h>
#include <SPI.h>

/*-------Defining Inputs------*/

int LS = 6;      // left sensor //10
int RS = 7;      // right sensor //11
int BLS = 5;      // back left sensor //9
int BRS = 4;      // back right sensor //6

int IRF = 12;
int IRB = 13;
int com;   //serial comm jet to arduino
int valk = 0;

long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement
long duration1; // variable for the duration of sound wave travel
int distance1; // variable for the distance measurement

#define echoPin 48 // attach pin D43 Arduino to pin Echo of HC-SR04
#define trigPin 46 //attach pin D44 Arduino to pin Trig of HC-SR04
#define echoPin1 42 // attach pin D42 Arduino to back pin Echo of HC-SR04 //43
#define trigPin1 44 //attach pin D44 Arduino to back pin Trig of HC-SR04 //45

const int voltageSensor = A0;

float vOUT = 0.0;
float vIN = 0.0;
float R1 = 30000.0;
float R2 = 7500.0;
int value = 0;

char text[32] = "";

const int l1 = 2;
const int r1 = 3;
const int d1 = 32; //22
const int d2 = 34; //24

Servo myservo;  // create servo object to control a servo

int pos = 0;
int stopBot = 0;
int started = 0;
int movement = 0;

void setup()
{
  pinMode(LS, INPUT);
  pinMode(RS, INPUT);
  pinMode(IRF, INPUT);
  pinMode(IRB, INPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);

  pinMode(32, OUTPUT); //22
  pinMode(34, OUTPUT); //24
  pinMode(38, OUTPUT); //35
  pinMode(40, INPUT); //41
  pinMode(trigPin, OUTPUT); // Front Ultrasonic
  pinMode(echoPin, INPUT); //  Front Ultrasonic
  pinMode(trigPin1, OUTPUT); // Sets the trigPin back as an OUTPUT
  pinMode(echoPin1, INPUT); // Sets the echoPin back as an INPUT

  pinMode(A7, INPUT);

  myservo.attach(36); //32
  myservo.write(45);
  Serial.begin(9600);
}

void bigdelay() {
  delay(10000);
  delay(10000);
  delay(10000);
  delay(10000);
  delay(10000);
  delay(10000);
  delay(10000);
  delay(10000);
  delay(10000);
}

void Ffronttoleft() {
  for (pos = 50; pos <= 75; pos += 1) {
    myservo.write(pos);
    delay(15);
  }
  digitalWrite(d1, LOW);
  digitalWrite(d2, HIGH);
}

void Ffronttoright() {
  for (pos = 50; pos >= 20; pos -= 1) {
    myservo.write(pos);
    delay(15);
  }
  digitalWrite(d1, LOW);
  digitalWrite(d2, HIGH);
}

void Flefttofront() {
  for (pos = 65; pos >= 50; pos -= 1) {
    myservo.write(pos);
    delay(15);
  }
  digitalWrite(d1, LOW);
  digitalWrite(d2, HIGH);
}

void Frighttofront() {

  myservo.write(50);
  delay(15);
  digitalWrite(d1, LOW);
  digitalWrite(d2, HIGH);
}

void Bfronttoleft() {
  for (pos = 50; pos <= 75; pos += 1) {
    myservo.write(pos);
    delay(15);
  }
  digitalWrite(d1, HIGH);
  digitalWrite(d2, LOW);
}

void Bfronttoright() {
  for (pos = 50; pos >= 20; pos -= 1) {
    myservo.write(pos);
    delay(15);
  }
  digitalWrite(d1, HIGH);
  digitalWrite(d2, LOW);
}

void Blefttofront() {
  for (pos = 65; pos >= 50; pos -= 1) {
    myservo.write(pos);
    delay(15);
  }
  digitalWrite(d1, HIGH);
  digitalWrite(d2, LOW);
}

void Brighttofront() {

  myservo.write(40);
  delay(15);

  digitalWrite(d1, HIGH);
  digitalWrite(d2, LOW);
}

void backward() {
  digitalWrite(d1, HIGH);
  digitalWrite(d2, LOW);
}

void forward() {
  digitalWrite(d1, LOW);
  digitalWrite(d2, HIGH);
}

void stopmotor() {
  myservo.write(45);
  analogWrite(l1, 0);
  analogWrite(r1, 0);
}

void startmotor() {
  myservo.write(45);
  analogWrite(l1, 40);
  analogWrite(r1, 51);
}

void jetsonSeq1() {
  //Notify Jetson Nano
  delay(1000); //was 7000
}

void jetsonSeq2() {

  //Notify Jetson Nano
  delay(1000); //was 7000
}

void backwardMotion() {

  while (1) {
    digitalWrite(trigPin1, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin1, LOW);
    duration1 = pulseIn(echoPin1, HIGH);
    distance1 = duration1 * 0.034 / 2;

    //startmotor();
    if (digitalRead(BRS) && digitalRead(BLS))    // Move Forward
    {
      backward();
    }

    if (!(digitalRead(BRS)) && digitalRead(BLS))    // Turn right
    {
      Bfronttoright();
      delay(1000);
      Brighttofront();
    }

    if (digitalRead(BRS) && !(digitalRead(BLS)))    // turn left
    {
      Bfronttoleft();
      delay(1000);
      Blefttofront();
    }

    if (distance1 < 15)    // stop
    {
      stopmotor();
      Serial.println("2");
      Serial.println("2");
      delay(50);
      Serial.println("2");
      Serial.println("2");
      delay(50);
      Serial.println("2");
      Serial.println("2");
      delay(50);
      Serial.println("2");
      Serial.println("2");
      jetsonSeq2();
      delay(5000);
      Serial.println("0");
      Serial.println("0");
      delay(50);
      Serial.println("0");
      Serial.println("0");
      delay(50);
      Serial.println("0");
      Serial.println("0");
      delay(50);
      Serial.println("0");
      Serial.println("0");
      break;
    }
  }
}


int batteryCheck() {
  value = analogRead(voltageSensor);
  vOUT = (value * 5.0) / 1024.0;
  vIN = vOUT / (R2 / (R1 + R2));
  //Serial.print("Input = ");
  //Serial.println(vIN);
  delay(1000);
  if (vIN > 22)return 1;
  else {
    digitalWrite(38, HIGH); //35
    return 0;
  }
}

int nrfStatus() {

  Serial.println("0");
  Serial.println("0");
  delay(50);
  Serial.println("0");
  Serial.println("0");
  delay(50);
  Serial.println("0");
  Serial.println("0");
  delay(50);
  Serial.println("0");
  Serial.println("0");
  routine();
  delay(5);
  return 0;
}

void routine() {
  while (1)
  {
    com = analogRead(A7);

    if (com < 300) {
      stopmotor();
      Serial.println("1");
      Serial.println("1");
      delay(50);
      Serial.println("1");
      Serial.println("1");
      delay(50);
      Serial.println("1");
      Serial.println("1");
      delay(50);
      Serial.println("1");
      Serial.println("1");
      delay(10000);
      delay(10000);
      delay(10000);
      Serial.println("0");
      Serial.println("0");
      delay(50);
      Serial.println("0");
      Serial.println("0");
      delay(50);
      Serial.println("0");
      Serial.println("0");
      delay(50);
      Serial.println("0");
      Serial.println("0");
    }

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;

    startmotor();
    //Serial.println("Motor running");

    if (digitalRead(LS) && digitalRead(RS))    // Move Forward
    {
      //Serial.println("Motor running");
      //Serial.println("Going FORWARD");
      forward();
    }

    if (!(digitalRead(LS)) && digitalRead(RS))    // Turn right
    {
      //Serial.println("Turning RIGHT");
      Ffronttoright();
      delay(1000);
      Frighttofront();
    }

    if (digitalRead(LS) && !(digitalRead(RS)))    // turn left
    {
      //Serial.println("Turning LEFT");
      Ffronttoleft();
      delay(1000);
      Flefttofront();
    }

    if (distance < 7)    // stop //Front Ultrasonic
    {

      stopmotor();
      delay(2000);
      startmotor();
      backwardMotion();
      delay(1000);
      break;
    }
  }
}

void loop() {

  //       Serial.println("Checking Battery Status");
  if (batteryCheck()) {
    //      Serial.println("Done");
  }
  delay(500);
  valk = digitalRead(40); //41
  if (  valk == HIGH ) {
    nrfStatus();
    // if (nrfStatus()) {
    // Serial.println("Waiting for you");
  }

  delay(1000);
}
