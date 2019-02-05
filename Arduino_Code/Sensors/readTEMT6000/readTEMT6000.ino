#include <Wire.h>

#define LAPIN A0 // PIN with Light sensor analog output 
#define LPPIN 4 // PIN with Light power input


static float light;

void setup() {
    Serial.begin(115200);   
    
}

void loop() {

    // temperature is an integer in hundredths
    light = readLight();
    
    Serial.print("Light :");  
    Serial.print(light);
    Serial.println("Lx");
    
    delay(500);      
}

float readLight() {
  float result;
  // Light sensor Voltage
digitalWrite(LPPIN, HIGH); // Power the sensor
delay(10);
int sensorValue = analogRead(LAPIN);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 3.3V):
  float voltage = sensorValue * (3.3 / 1023.0); // Batvalue is 3.3V
result = voltage*200; // multiply by 2000 to have Lx
digitalWrite(LPPIN, LOW); // switch off the sensor
  return result;
}

