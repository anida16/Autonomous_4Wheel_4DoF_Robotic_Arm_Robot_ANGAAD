#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

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

//RF24 radio(7, 8); // CE, CSN
char text[32] = "";

const int l1 = 2;
//const int l2 = 4;
const int r1 = 3;
//const int r2 = 5;
const int d1 = 32; //22
const int d2 = 34; //24
//const int d3 = 26;
//const int d4 = 28;

Servo myservo;  // create servo object to control a servo

int pos = 0;
int stopBot = 0;
int started = 0;
int movement = 0;


//Functions

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
  //digitalWrite(d3, LOW);
  //digitalWrite(d4, HIGH);

}

void Ffronttoright() {
  for (pos = 50; pos >= 20; pos -= 1) {
    myservo.write(pos);
    delay(15);
  }
  digitalWrite(d1, LOW);
  digitalWrite(d2, HIGH);
  //digitalWrite(d3, LOW);
  //digitalWrite(d4, HIGH);
}

void Flefttofront() {
  for (pos = 65; pos >= 50; pos -= 1) {
    myservo.write(pos);
    delay(15);
  }
  digitalWrite(d1, LOW);
  digitalWrite(d2, HIGH);
  //digitalWrite(d3, LOW);
  //digitalWrite(d4, HIGH);
}

void Frighttofront() {

  myservo.write(50);
  delay(15);

  digitalWrite(d1, LOW);
  digitalWrite(d2, HIGH);
  //digitalWrite(d3, LOW);
  //digitalWrite(d4, HIGH);
}

void Bfronttoleft() {
  for (pos = 50; pos <= 75; pos += 1) {
    myservo.write(pos);
    delay(15);
  }
  digitalWrite(d1, HIGH);
  digitalWrite(d2, LOW);
  //digitalWrite(d3, HIGH);
  //digitalWrite(d4, LOW);

}

void Bfronttoright() {
  for (pos = 50; pos >= 20; pos -= 1) {
    myservo.write(pos);
    delay(15);
  }
  digitalWrite(d1, HIGH);
  digitalWrite(d2, LOW);
  //digitalWrite(d3, HIGH);
  //digitalWrite(d4, LOW);


}

void Blefttofront() {
  for (pos = 65; pos >= 50; pos -= 1) {
    myservo.write(pos);
    delay(15);
  }
  digitalWrite(d1, HIGH);
  digitalWrite(d2, LOW);
  //digitalWrite(d3, HIGH);
  //digitalWrite(d4, LOW);
}

void Brighttofront() {

  myservo.write(40);
  delay(15);

  digitalWrite(d1, HIGH);
  digitalWrite(d2, LOW);
  //digitalWrite(d3, HIGH);
  //digitalWrite(d4, LOW);
}

void backward() {
  digitalWrite(d1, HIGH);
  digitalWrite(d2, LOW);
  //digitalWrite(d3, HIGH);
  //digitalWrite(d4, LOW);
}

void forward() {
  digitalWrite(d1, LOW);
  digitalWrite(d2, HIGH);
  //digitalWrite(d3, LOW);
  //digitalWrite(d4, HIGH);
}

void stopmotor() {
  myservo.write(45);
  analogWrite(l1, 0);
  //analogWrite(l2, 0);
  analogWrite(r1, 0);
  //analogWrite(r2, 0);
}

void startmotor() {
  myservo.write(45);
  analogWrite(l1, 40);
  //analogWrite(l2, 40);
  analogWrite(r1, 51);
  //analogWrite(r2, 51);
}

void jetsonSeq1() {

  //Notify Jetson Nano
  delay(7000);
}

void jetsonSeq2() {

  //Notify Jetson Nano
  delay(7000);
}

void backwardMotion() {

  while (1) {
    //com = analogRead(A7);
    //    if (com < 300) {
    //      stopmotor();
    //      Serial.println("1");
    //      Serial.println("1");
    //      delay(10000);
    //      delay(10000);
    //      delay(10000);
    //    }
    //    Serial.println("0");
    // Clears the trigPin condition
    digitalWrite(trigPin1, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trigPin1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin1, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration1 = pulseIn(echoPin1, HIGH);
    // Calculating the distance
    distance1 = duration1 * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
    // Displays the distance on the Serial Monitor

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


//void forwardMotion() {
//
//  while (1) {
//    startmotor();
//    if (digitalRead(LS) && digitalRead(RS))    // Move Forward
//    {
//      forward();
//    }
//
//    if (!(digitalRead(LS)) && digitalRead(RS))    // Turn right
//    {
//      Ffronttoright();
//      delay(1000);
//      Frighttofront();
//    }
//
//    if (digitalRead(LS) && !(digitalRead(RS)))    // turn left
//    {
//      Ffronttoleft();
//      delay(1000);
//      Flefttofront();
//    }
//    if (digitalRead(IRF) == LOW)  // stop
//    {
//      stopmotor();
//      jetsonSeq1();
//      backwardMotion();
//      delay(1000);
//      Serial.print("fwdstoptest");
//      break;
//
//    }
//  }
//}

int batteryCheck() {
  value = analogRead(voltageSensor);
  vOUT = (value * 5.0) / 1024.0;
  vIN = vOUT / (R2 / (R1 + R2));
  //  Serial.print("Input = ");
  //    Serial.println(vIN);
  delay(1000);
  if (vIN > 22)return 1;
  else {
    digitalWrite(38, HIGH); //35
    return 0;
  }
}

//void recvData()
//{
//  while(radio.available()){
//    radio}
//  }
int nrfStatus() {

  //  if (radio.available()) {
  //
  //
  //    char text[32] = "";
  //    const char text2 = text;
  //    char text3[32] = "abc";
  //
  //
  //    radio.read(&text, sizeof(text));
  //    //    Serial.println(text);
  //    //    char text[32] = "";                 //Saving the incoming data
  //    //    radio.read(&text, sizeof(text));    //Reading the data
  //
  //    //  if (strcmp(&text2, &text3)== 0){
  //    if (text[0] == 'r' && text[1] == 'u' && text[2] == 'n') {
  //      //      Serial.println("Performing routine");
  //      delay(1000);
  //      radio.flush_rx();
  //      radio.flush_tx();
  //      delay(50);
  //      radio.flush_rx();
  //      radio.flush_tx();
  //      delay(50);
  //      delay(50);
  //      radio.flush_rx();
  //      radio.flush_tx();
  //      delay(50);
  //      radio.flush_rx();
  //      radio.flush_tx();


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
    //    Serial.println("0");
    // Clears the trigPin condition
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
    // Displays the distance on the Serial Monitor


    startmotor();
    // Serial.println("Motor running");

    if (digitalRead(LS) && digitalRead(RS))    // Move Forward
    {
      //      Serial.println("Motor running");
      //      Serial.println("Going FORWARD");
      forward();
    }

    if (!(digitalRead(LS)) && digitalRead(RS))    // Turn right
    {
      //      Serial.println("Turning RIGHT");
      Ffronttoright();
      delay(1000);
      Frighttofront();
    }

    if (digitalRead(LS) && !(digitalRead(RS)))    // turn left
    {
      //      Serial.println("Turning LEFT");
      Ffronttoleft();
      delay(1000);
      Flefttofront();
    }

    if (distance < 7)    // stop //Front Ultrasonic
    {
      //      Serial.println("Motor STOPPED");
      //      Serial.println("1");
      //      Serial.println("1");
      stopmotor();
      delay(2000);
      startmotor();
      //      Serial.println("Going BACK");
      //      Serial.println("0");
      //      Serial.println("0");
      //      delay(50);
      //      Serial.println("0");
      //      Serial.println("0");
      //      delay(50);
      //      Serial.println("0");
      //      Serial.println("0");
      //      delay(50);
      //      Serial.println("0");
      //      Serial.println("0");
      backwardMotion();
      //      Serial.println("Gonna Stop");
      delay(1000);

      break;
    }

    //if(!digitalRead(IRB)){
    //    stopmotor();
    //      break;
    //   }
  }
}



void setup()
{
  pinMode(LS, INPUT);
  pinMode(RS, INPUT);
  pinMode(IRF, INPUT);
  pinMode(IRB, INPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  //pinMode(31, OUTPUT);

  //pinMode(4, OUTPUT);
  //pinMode(5, OUTPUT);
  pinMode(32, OUTPUT); //22
  pinMode(34, OUTPUT); //24
  pinMode(38, OUTPUT); //35
  pinMode(40, INPUT); //41
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  pinMode(trigPin1, OUTPUT); // Sets the trigPin back as an OUTPUT
  pinMode(echoPin1, INPUT); // Sets the echoPin back as an INPUT
  //pinMode(26, OUTPUT);
  //pinMode(28, OUTPUT);

  pinMode(A7, INPUT);
  //pinMode(18, OUTPUT);

//  radio.begin();
  //radio.setChannel(0x55);
  //radio.openReadingPipe(0, 0xF0F0F0F0AA);
  //radio.setPALevel(RF24_PA_HIGH);
  //radio.startListening();
  myservo.attach(36); //32
  myservo.write(45);
  Serial.begin(9600);
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
    //    if (nrfStatus()) {
    //            Serial.println("Waiting for you");



  }

  delay(1000);
}
