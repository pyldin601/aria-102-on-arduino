#define DEBUG_MODE            false

#define SENSOR_A              A1
#define SENSOR_B              A0

#define OUTPUT_ANCHOR_A       3
#define OUTPUT_ANCHOR_B       9
#define OUTPUT_ANCHOR_C       10
#define OUTPUT_ANCHOR_D       11

#define SENSOR_A_MIN          190
#define SENSOR_A_MAX          869
#define SENSOR_B_MIN          186
#define SENSOR_B_MAX          867
#define SENSOR_CAL_MIN        -255
#define SENSOR_CAL_MAX        255

#define PWM_LEVEL_MIN         0
#define PWM_LEVEL_MAX         255

struct SensorCheckpoint {
  byte sensorId;
  int triggerLevel;
  char triggerDirection;
  byte offsetPercent;
};

struct Speed {
  float rpm;
  float fineTune;
};

const byte sensorCheckpointsCount = 80;
const byte speedsCount = 3;
const byte sensorsCount = 2;
const byte roundsCount = 4;

SensorCheckpoint checkpoints[sensorCheckpointsCount] = {
  { 1, 0,  1, 110 }, { 1,  100,  1, 114 }, { 1,  200,  1, 118 }, { 0, -200,  1,  98 }, { 0, -100,  1, 129 },
  { 0, 0,  1, 110 }, { 0,  100,  1, 118 }, { 0,  200,  1, 122 }, { 1,  200, -1, 161 }, { 1,  100, -1, 118 },
  { 1, 0, -1, 114 }, { 1, -100, -1, 114 }, { 1, -200, -1, 122 }, { 0,  200, -1, 149 }, { 0,  100, -1, 126 },
  { 0, 0, -1, 118 }, { 0, -100, -1, 118 }, { 0, -200, -1, 122 }, { 1,  -200, 1, 181 }, { 1, -100,  1, 118 },

  { 1, 0,  1, 110 }, { 1,  100,  1, 110 }, { 1,  200,  1, 114 }, { 0, -200,  1, 114 }, { 0, -100,  1, 126 },
  { 0, 0,  1, 110 }, { 0,  100,  1, 114 }, { 0,  200,  1, 118 }, { 1,  200, -1, 196 }, { 1,  100, -1, 114 },
  { 1, 0, -1, 110 }, { 1, -100, -1, 110 }, { 1, -200, -1, 110 }, { 0,  200, -1, 173 }, { 0,  100, -1, 118 },
  { 0, 0, -1, 114 }, { 0, -100, -1, 110 }, { 0, -200, -1, 122 }, { 1, -200,  1, 232 }, { 1, -100,  1, 106 },

  { 1, 0,  1, 110 }, { 1,  100,  1, 102 }, { 1,  200,  1, 114 }, { 0, -200,  1, 122 }, { 0, -100,  1, 122 },
  { 0, 0,  1, 110 }, { 0,  100,  1, 110 }, { 0,  200,  1, 118 }, { 1,  200, -1, 181 }, { 1,  100, -1, 114 },
  { 1, 0, -1, 110 }, { 1, -100, -1, 110 }, { 1, -200, -1, 110 }, { 0,  200, -1, 169 }, { 0,  100, -1, 122 },
  { 0, 0, -1, 118 }, { 0, -100, -1, 106 }, { 0, -200, -1, 126 }, { 1, -200,  1, 189 }, { 1, -100,  1, 110 },

  { 1, 0,  1, 110 }, { 1,  100,  1, 102 }, { 1,  200,  1, 114 }, { 0, -200,  1, 126 }, { 0, -100,  1, 118 },
  { 0, 0,  1, 110 }, { 0,  100,  1, 110 }, { 0,  200,  1, 114 }, { 1,  200, -1, 204 }, { 1,  100, -1, 114 },
  { 1, 0, -1, 106 }, { 1, -100, -1, 110 }, { 1, -200, -1, 114 }, { 0,  200, -1, 173 }, { 0,  100, -1, 118 },
  { 0, 0, -1, 114 }, { 0, -100, -1, 106 }, { 0, -200, -1, 126 }, { 1, -200,  1, 189 }, { 1, -100,  1, 118 },
};

unsigned int rawSensorValue[sensorsCount] = { 0, 0 };
int calibratedSensorValue[sensorsCount] = { 0, 0 };
unsigned long fullRoundCheckpoints[roundsCount] = { 0, 0, 0, 0 };

Speed rotationSpeeds[speedsCount] = {
  { 33.0 + 1 / 3.0, 40.0 },
  { 45.0, 10.0 },
};

float pwmValue = 128.0;
byte currentCheckpoint = 0;
float speedAccuracyRatio = 0.0;

byte speed = 0;

float getExpectedRoundTime() {
  return (1 / (rotationSpeeds[speed].rpm / 60.0) * 1000.0);
}

float getSpeedFineTune() {
  return rotationSpeeds[speed].fineTune;
}

void setup() {
  pinMode(OUTPUT_ANCHOR_A, OUTPUT);
  pinMode(OUTPUT_ANCHOR_B, OUTPUT);
  pinMode(OUTPUT_ANCHOR_C, OUTPUT);
  pinMode(OUTPUT_ANCHOR_D, OUTPUT);

  if (DEBUG_MODE == false) {
    TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
    TCCR1B = TCCR1B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
  }
}

void loop() {
  taskReadSensors();
  taskControlRotation();
  taskSyncAnchorsPwmValues();
}

void taskReadSensors() {
  rawSensorValue[0] = analogRead(SENSOR_A);
  rawSensorValue[1] = analogRead(SENSOR_B);

  calibratedSensorValue[0] = constrain(
                               map(
                                 rawSensorValue[0],
                                 SENSOR_A_MIN,
                                 SENSOR_A_MAX,
                                 SENSOR_CAL_MIN,
                                 SENSOR_CAL_MAX
                               ),
                               SENSOR_CAL_MIN,
                               SENSOR_CAL_MAX
                             );
  calibratedSensorValue[1] = constrain(
                               map(
                                 rawSensorValue[1],
                                 SENSOR_B_MIN,
                                 SENSOR_B_MAX,
                                 SENSOR_CAL_MIN,
                                 SENSOR_CAL_MAX
                               ),
                               SENSOR_CAL_MIN,
                               SENSOR_CAL_MAX
                             );
}

void taskControlRotation() {
  int expectedRoundTime = getExpectedRoundTime();

  SensorCheckpoint sc = checkpoints[currentCheckpoint];
  int level = sc.triggerLevel;
  int sign = sc.triggerDirection;
  int sensorValue = calibratedSensorValue[sc.sensorId];

  if (sign * sensorValue >= sign * level) {
    unsigned long now = millis();
    unsigned int expectedPercent = currentCheckpoint == 0
                                   ? checkpoints[sensorCheckpointsCount - 1].offsetPercent
                                   : sc.offsetPercent;

    currentCheckpoint = (currentCheckpoint + 1) % sensorCheckpointsCount;

    if (currentCheckpoint % 20 == 0) {
      unsigned long previousRound = fullRoundCheckpoints[currentCheckpoint / 20];
      long actualRoundTime = now - previousRound;
      fullRoundCheckpoints[currentCheckpoint / 20] = now;

      speedAccuracyRatio = 100 / (float) actualRoundTime * (float) expectedRoundTime;
      taskSyncPwmToRotationSpeed();
    }
  }
}

void taskSyncPwmToRotationSpeed() {
  if (speedAccuracyRatio < 80) {
    pwmValue = PWM_LEVEL_MAX;
    return;
  }
  float fineTune = getSpeedFineTune();
  pwmValue = constrain(map(speedAccuracyRatio * 10.0 + fineTune, 800, 1050, 100, 64), 0, 255);
}

void taskSyncAnchorsPwmValues() {
  int amplifiedValueA = min(255, pwmValue) / 255.0 * calibratedSensorValue[0];
  int amplifiedValueB = min(255, pwmValue) / 255.0 * calibratedSensorValue[1];

  analogWrite(OUTPUT_ANCHOR_A, constrain(max(0, amplifiedValueA), 0, 255));
  analogWrite(OUTPUT_ANCHOR_B, constrain(-min(0, amplifiedValueB), 0, 255));
  analogWrite(OUTPUT_ANCHOR_C, constrain(-min(0, amplifiedValueA), 0, 255));
  analogWrite(OUTPUT_ANCHOR_D, constrain(max(0, amplifiedValueB), 0, 255));
}

float amplifySensorValue(float sensorValue) {
  return min(255, pwmValue) / 255.0 * sensorValue;
}
