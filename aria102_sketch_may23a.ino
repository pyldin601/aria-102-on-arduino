#define DEBUG_MODE false

#define SENSOR_A A1
#define SENSOR_B A0

#define SENSOR_A_MIN 190
#define SENSOR_A_MAX 869
#define SENSOR_B_MIN 186
#define SENSOR_B_MAX 867

#define DC_MIN_LEVEL 0
#define DC_MID_LEVEL 64
#define DC_MAX_LEVEL 255

#define FIRST_SPEED 33 + 1 / 3
#define SECOND_SPEED 45
#define MOON_SPEED 500

#define MODE_STOP 0
#define MODE_RUN 1
#define MODE_BREAK 2

struct SensorCheckpoint {
  byte sensorId;
  int triggerLevel;
  char triggerDirection;
  byte offsetPercent;
};

struct StabilizerState {
  byte maxValue;
  byte minValue;
  float currentValue;
};


const byte sensorCheckpointsCount = 80;
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

unsigned int rawSensorValue[2] = { 0, 0 };
int calibratedSensorValue[2] = { 0, 0 };

StabilizerState stabilizerState = { DC_MIN_LEVEL, DC_MAX_LEVEL, DC_MAX_LEVEL };
int acceleration = 0;

byte currentCheckpoint = 0;

// current selected speed
int currentMode = MODE_RUN;
int currentSpeed = SECOND_SPEED;

void setup() {
  pinMode(3, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  if (DEBUG_MODE == false) {
    //        TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
    //        TCCR1B = TCCR1B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
  }

  Serial.begin(115200);
}

void loop() {
  taskReadSensors();
  taskCheckpointCross();
  taskSyncMotorCoils();
  taskSyncAverages();
  taskAdjustDcPower();
}

void taskReadSensors() {
  rawSensorValue[0] = analogRead(SENSOR_A);
  rawSensorValue[1] = analogRead(SENSOR_B);

  calibratedSensorValue[0] = constrain(map(rawSensorValue[0], SENSOR_A_MIN, SENSOR_A_MAX, -255, 255), -255, 255);
  calibratedSensorValue[1] = constrain(map(rawSensorValue[1], SENSOR_B_MIN, SENSOR_B_MAX, -255, 255), -255, 255);
}

void taskSyncMotorCoils() {
  int amplifiedValueA = min(255, stabilizerState.currentValue) / 255.0 * calibratedSensorValue[0];
  int amplifiedValueB = min(255, stabilizerState.currentValue) / 255.0 * calibratedSensorValue[1];

  analogWrite(3, constrain(max(0, amplifiedValueA), 0, 255));
  analogWrite(9, constrain(-min(0, amplifiedValueB), 0, 255));
  analogWrite(10, constrain(-min(0, amplifiedValueA), 0, 255));
  analogWrite(11, constrain(max(0, amplifiedValueB), 0, 255));
}

unsigned long lastCheckpointTime = 0;

float speedAccuracyRatio = 0.0;
float speedAccuracyRatioAverage = 0.0;

void taskCheckpointCross() {
  int expectedRoundTime = (1 / (currentSpeed / 60.0) * 1000.0);

  SensorCheckpoint sc = checkpoints[currentCheckpoint];
  int level = sc.triggerLevel;
  int sign = sc.triggerDirection;
  int sensorValue = calibratedSensorValue[sc.sensorId];

  if (sign * sensorValue >= sign * level) {
    unsigned long now = millis();
    unsigned int expectedPercent = currentCheckpoint == 0
                                   ? checkpoints[sensorCheckpointsCount - 1].offsetPercent
                                   : sc.offsetPercent;

    float expectedCheckpointTime = expectedPercent / 10000.0 * expectedRoundTime;
    int actualCheckpointTime = now - lastCheckpointTime;

    speedAccuracyRatio = 100 / actualCheckpointTime * expectedCheckpointTime;




    lastCheckpointTime = now;
    currentCheckpoint = (currentCheckpoint + 1) % sensorCheckpointsCount;
  }
}

unsigned long prevSyncAveragesTime = 0;
void taskSyncAverages() {
  unsigned long now = millis();
  if (now - prevSyncAveragesTime > 10) {
    speedAccuracyRatioAverage += (speedAccuracyRatio - speedAccuracyRatioAverage) * 0.01;
    prevSyncAveragesTime = now;
  }
}

long n = 0;

void taskAdjustDcPower() {
  int dc = map(speedAccuracyRatioAverage * 10.0, 300.0, 1300.0, DC_MAX_LEVEL, DC_MIN_LEVEL);
  dc = constrain(dc, DC_MIN_LEVEL, DC_MAX_LEVEL);
  stabilizerState.currentValue += (dc - stabilizerState.currentValue) * 0.1;

  n += 1;
  if (n % 10 == 0) {
    Serial.print(speedAccuracyRatioAverage);
    Serial.print('\t');
    Serial.println(stabilizerState.currentValue);
  }
}
