#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

#define SDA_PIN 8
#define SCL_PIN 9

int bluePin = 0;
int greenPin = 1;
int redPin = 3;//not sure why pin 2 reads high
int buttonPin = 10;

// Temp values
float offsetAx = 0, offsetAy = 0, offsetAz = 0;

MPU6050 mpu;

void setup() {
  pinMode(bluePin,OUTPUT);
  pinMode(greenPin,OUTPUT);
  pinMode(redPin,OUTPUT);
  pinMode(buttonPin, INPUT);

  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);  // Initialize I2C with custom pins

  Serial.println("Initializing MPU6050...");
  mpu.initialize();  // Initialize MPU6050

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed! Check wiring.");
    while (1);  // Stop execution if MPU6050 is not detected
  }

  Serial.println("MPU6050 Initialized!");

  // Disable Gyroscope and DLPF to save power
  mpu.setSleepEnabled(false);  // Ensure the sensor is awake
  Wire.beginTransmission(0x68);
  Wire.write(0x6C);  // PWR_MGMT_2 register
  Wire.write(0x07);  // Disable Gyroscope (X, Y, Z)
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);  // CONFIG register
  Wire.write(0x07);  // Disable DLPF
  Wire.endTransmission();
}

void loop() {
  // Check if the button is pressed
  if (digitalRead(buttonPin) == HIGH) {
    calibrateMPU();
    delay(100);  // Debounce delay
  }

  digitalWrite(2,LOW);
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);  // Get accelerometer data

  // Convert to 'g' values (1g = 9.81 m/s^2)
  float Ax = (ax / 16384.0) - offsetAx;
  float Ay = (ay / 16384.0) - offsetAy;
  float Az = (az / 16384.0) - offsetAz;

  // Calculate roll (Y-axis)
  float roll = atan2(Ax, Az) * 180.0 / PI;  // Roll around the Y-axis

  // Print roll and pitch values
  Serial.print("Roll (Y-axis): "); Serial.print(roll); Serial.println();

  delay(500);  // Delay to make output more readable
  
  // Conditions for LEDs
  // hard left therefore blue only
  if(roll > 45 && roll < 180){//(5,180)
    digitalWrite(bluePin,HIGH);
    digitalWrite(greenPin,LOW);
    digitalWrite(redPin,LOW);
  }

  // soft left therefore blue and green
  if(roll > 5 && roll <= 30){ //(5,30]
    digitalWrite(bluePin,HIGH);
    digitalWrite(greenPin,HIGH);
    digitalWrite(redPin,LOW);
  }

  // dead level therefore green only
  if(roll >= -5 && roll <= 5){ //[-5,5]
    digitalWrite(bluePin,LOW);
    digitalWrite(greenPin,HIGH);
    digitalWrite(redPin,LOW);
  }

  // soft right therefore red only
  if(roll < -5 && roll >= -30){ //(-5,-30]
    digitalWrite(bluePin,LOW);
    digitalWrite(greenPin,HIGH);
    digitalWrite(redPin,HIGH);
  }

  // hard right therefore red only
  if(roll < -30 && roll > -180){ //(-30,-180)
    digitalWrite(bluePin,LOW);
    digitalWrite(greenPin,LOW);
    digitalWrite(redPin,HIGH);
  }
}
//calibrates MPU
void calibrateMPU() {
  Serial.println("Calibrating MPU6050... Keep it level and steady!");
  delay(1000);

  float sumAx = 0, sumAy = 0, sumAz = 0;
  int samples = 100;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    
    sumAx += ax / 16384.0;
    sumAy += ay / 16384.0;
    sumAz += az / 16384.0;

    delay(10);
  }

  offsetAx = sumAx / samples;
  offsetAy = sumAy / samples;
  offsetAz = sumAz / samples - 1.0;  // Subtract 1g to account for gravity
    
  Serial.println("Calibration complete!");
  //Serial.print("Offsets: "); Serial.print(offsetAy);
}