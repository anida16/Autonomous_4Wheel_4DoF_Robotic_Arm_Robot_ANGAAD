
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
// Include RadioHead Amplitude Shift Keying Library
#include <RH_ASK.h>

// Create Amplitude Shift Keying Object
RH_ASK rf_driver;

//Byte of array representing the address. This is the address where we will send the data. This should be same on the receiving side.
int button_pin = 2;
boolean button_state = 0;

long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

#define echoPin 43 // attach pin D43 Arduino to back pin Echo of HC-SR04
#define trigPin 45 //attach pin D45 Arduino to back pin Trig of HC-SR04

void setup() {
  // Initialize ASK Object
  rf_driver.init();
  pinMode(button_pin, INPUT);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  //  digitalWrite(11,LOW);
  Serial.begin(9600);   //start serial to communicate process
}

void loop()
{
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

  button_state = digitalRead(button_pin);
  if (distance < 10)
  {

    const char *msg = "run";
    rf_driver.send((uint8_t *)msg, strlen(msg));
    rf_driver.waitPacketSent();
    Serial.println("Your Button State is HIGH");
    delay(2000);
  }
  else
  {
    //const char text[] = "Your Button State is LOW";
    //radio.write(&text, sizeof(text));                  //Sending the message to receiver
    Serial.println("Your Button State is LOW");

  }
  //radio.write(&button_state, sizeof(button_state));  //Sending the message to receiver

  delay(100);
}
