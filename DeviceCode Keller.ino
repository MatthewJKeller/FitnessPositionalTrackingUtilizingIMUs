//Matthew Keller
//4-30-25
#include <ReefwingAHRS.h>
#include <Arduino_BMI270_BMM150.h>

#define BUTTON_PIN 8
#define BUZZER_PIN 4

ReefwingAHRS ahrs;
SensorData data;

bool running = false;
unsigned long previousMillis = 0;
const long displayPeriod = 50;

// Neutral angles with mounting considerations
const float NEUTRAL_ROLL = -5.0; // -5 degrees since tilted when mounted on glove
const float NEUTRAL_PITCH = 0.0;
const float TOLERANCE_DEGREES = 20.0; // 20 degree range of error to account for sensor error

void setup() {
  pinMode(BUTTON_PIN,INPUT_PULLUP);
  pinMode(BUZZER_PIN,OUTPUT);
  // Power on
  Serial.begin(115200);
  delay(7000);

  ahrs.begin();
  ahrs.setFusionAlgorithm(SensorFusion::MADGWICK);

  // Greenville, SC Declination for calibration

  ahrs.setDeclination(-7.65); 
  if (!IMU.begin()) {
    Serial.println("IMU not found.");
    while (1);
  }

  Serial.println("IMU ready. Press button to toggle output.");
}

void loop() {
  //Button activation toggle
  if (digitalRead(BUTTON_PIN) == LOW) {
    while (digitalRead(BUTTON_PIN) == LOW);
    running = !running;
    Serial.print("Toggled: ");
    Serial.println(running ? "ON" : "OFF");
    delay(200);
  }
  //Main algorithm
  if (running) {
    if (IMU.gyroscopeAvailable()) IMU.readGyroscope(data.gx, data.gy, data.gz);
    if (IMU.accelerationAvailable()) IMU.readAcceleration(data.ax,data.ay, data.az);
    if (IMU.magneticFieldAvailable()) IMU.readMagneticField(data.mx, data.my, data.mz);

    ahrs.setData(data);
    ahrs.update();

    float roll = ahrs.angles.roll;
    float pitch = ahrs.angles.pitch;

    // Check if roll or pitch exceeds tolerance
    bool deviated = abs(roll -NEUTRAL_ROLL) > TOLERANCE_DEGREES ||
                    abs(pitch - NEUTRAL_PITCH) >TOLERANCE_DEGREES;

    if (deviated) {
      analogWrite(BUZZER_PIN,5);
    } else {
      analogWrite(BUZZER_PIN,0);
    }
    //Printing for debugging and calibration
    if (millis() - previousMillis >=displayPeriod) {
      Serial.print("Roll: ");
      Serial.print(roll, 2);
      Serial.print("  Pitch: ");
      Serial.println(pitch, 2);
      previousMillis = millis();
    }
  } else {
    analogWrite(BUZZER_PIN, 0);
  }
}
