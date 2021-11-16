#include <RH_ASK.h>
#include <SPI.h> // Not actualy used but needed to compile

int trigger = 9;
//RH_ASK driver;
RH_ASK rf_driver;

void setup()
{
  pinMode(9, OUTPUT);
  Serial.begin(9600);  // Debugging only
  rf_driver.init();
  //    if (!driver.init())
  //         Serial.println("init failed");
}

void loop()
{
  uint8_t buf[11];
  uint8_t buflen = sizeof(buf);
  // Check if received packet is correct size
  if (rf_driver.recv(buf, &buflen))
  {
    if (buf[0] == 'r' && buf[1] == 'u' && buf[2] == 'n') {
      //            Serial.println("Performing routine");
      delay(100);

      Serial.println((char*)buf);
      memset(buf, 0, sizeof(buf));
      //             Serial.println((char*)buf);
      digitalWrite(trigger, HIGH);
      delay(4000);
      digitalWrite(trigger, LOW);

    }
  }
  else
  {
    digitalWrite(trigger, LOW);
    Serial.println("Nothing");
  }
  delay(1000);
}
