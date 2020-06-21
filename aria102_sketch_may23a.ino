#define DEBUG_MODE false

#define SENSOR_A A1
#define SENSOR_B A0

#define SENSOR_A_MIN 190
#define SENSOR_A_MAX 869
#define SENSOR_B_MIN 186
#define SENSOR_B_MAX 867

#define DC_MIN_LEVEL 0
#define DC_MAX_LEVEL 255

#define FIRST_SPEED 33 + 1 / 3
#define SECOND_SPEED 45
#define MOON_SPEED 500

#define MODE_STOP 0
#define MODE_RUN 1
#define MODE_BREAK 2

unsigned int rawSensorValue[2] = { 0, 0 };
int calibratedSensorValue[2] = { 0, 0 };

struct SensorCheckpoint {
  byte sensorId;
  int triggerLevel;
  char triggerDirection;
  unsigned int offsetPercent;
};

const byte sensorCheckpointsCount = 80;
SensorCheckpoint checkpoints[sensorCheckpointsCount] = {
  { 1, 0,  1,    0 }, { 1,  100,  1,  105 }, { 1,  200,  1,  138 }, { 0, -200,  1,  180 }, { 0, -100,  1,  308 },
  { 0, 0,  1,  413 }, { 0,  100,  1,  518 }, { 0,  200,  1,  638 }, { 1,  200, -1,  833 }, { 1,  100, -1,  953 },
  { 1, 0, -1, 1058 }, { 1, -100, -1, 1163 }, { 1, -200, -1, 1283 }, { 0,  200, -1, 1455 }, { 0,  100, -1, 1575 },
  { 0, 0, -1, 1688 }, { 0, -100, -1, 1800 }, { 0, -200, -1, 1928 }, { 1,  -200, 1, 2108 }, { 1, -100,  1, 2228 },

  { 1, 0,  1, 2341 }, { 1,  100,  1, 2446 }, { 1,  200,  1, 2573 }, { 0, -200,  1, 2678 }, { 0, -100,   1, 2806 },
  { 0, 0,  1, 2918 }, { 0,  100,  1, 3023 }, { 0,  200,  1, 3151 }, { 1,  200, -1, 3308 }, { 1,  100,  -1, 3436 },
  { 1, 0, -1, 3548 }, { 1, -100, -1, 3653 }, { 1, -200, -1, 3773 }, { 0,  200, -1, 3931 }, { 0,  100,  -1, 4059 },
  { 0, 0, -1, 4171 }, { 0, -100, -1, 4291 }, { 0, -200, -1, 4419 }, { 1, -200,  1, 4599 }, { 1,  -100,  1, 4719 },

  { 1, 0,  1, 4824 }, { 1,  100,  1, 4929 }, { 1,  200,  1, 5056 }, { 0, -200,  1, 5161 }, { 0, -100,  1, 5289 },
  { 0, 0,  1, 5401 }, { 0,  100,  1, 5514 }, { 0,  200,  1, 5626 }, { 1,  200, -1, 5821 }, { 1,  100, -1, 5941 },
  { 1, 0, -1, 6054 }, { 1, -100, -1, 6159 }, { 1, -200, -1, 6272 }, { 0,  200, -1, 6444 }, { 0,  100, -1, 6564 },
  { 0, 0, -1, 6684 }, { 0, -100, -1, 6789 }, { 0, -200, -1, 6909 }, { 1, -200,  1, 7134 }, { 1, -100,  1, 7247 },

  { 1, 0,  1, 7359 }, { 1,  100,  1, 7464 }, { 1,  200,  1, 7577 }, { 0, -200,  1, 7704 }, { 0, -100,  1, 7817 },
  { 0, 0,  1, 7937 }, { 0,  100,  1, 8042 }, { 0,  200,  1, 8155 }, { 1,  200, -1, 8335 }, { 1,  100, -1, 8447 },
  { 1, 0, -1, 8567 }, { 1, -100, -1, 8672 }, { 1, -200, -1, 8785 }, { 0,  200, -1, 8957 }, { 0,  100, -1, 9077 },
  { 0, 0, -1, 9190 }, { 0, -100, -1, 9302 }, { 0, -200, -1, 9422 }, { 1, -200,  1, 9717 }, { 1, -100,  1, 9822 },
};
float checkpointTime[sensorCheckpointsCount];
byte currentCheckpoint = 0;
unsigned long lastCheckpointTime = 0;

// current selected speed
int currentMode = MODE_RUN;
int currentSpeed = SECOND_SPEED;

// current sensors raw values (0..1023)
int sensorPhaseA = 0;
int sensorPhaseB = 0;

// current sensors calibrated values (-256..256)
int calSensorPhaseA = 0;
int calSensorPhaseB = 0;


// sensors checkpoints
const int sensorsCheckpointCount = 32;
int sensorsCheckpointMap[sensorsCheckpointCount] = {
  0, 100, 200, 100, 0, -100, -200, -100,
  0, 100, 200, 100, 0, -100, -200, -100,
  0, 100, 200, 100, 0, -100, -200, -100,
  0, 100, 200, 100, 0, -100, -200, -100,
};
int sensorsCheckpointDirMap[sensorsCheckpointCount] = {
  1, 1, 1, -1, -1, -1, -1, 1,
  1, 1, 1, -1, -1, -1, -1, 1,
  1, 1, 1, -1, -1, -1, -1, 1,
  1, 1, 1, -1, -1, -1, -1, 1,
};
int sensorACheckpointOffsets[sensorsCheckpointCount] = {
  0, 2, 3, 100, 114, 130, 145, 267,
  282, 296, 312, 434, 449, 464, 480, 597,
  612, 626, 642, 766, 782, 796, 813, 929,
  945, 960, 976, 1097, 1113, 1128, 1144, 1261,
};

unsigned long sensorACheckpointTime[sensorsCheckpointCount];
unsigned long sensorBCheckpointTime[sensorsCheckpointCount];
byte currentSensorACheckpoint = 0;
byte currentSensorBCheckpoint = 0;

// power
float dcValue = DC_MAX_LEVEL;
float dcCorrection = 0.0;

void setup() {
  pinMode(3, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  initializeSensorsCheckpoints();

  if (DEBUG_MODE == false) {
    TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
    TCCR1B = TCCR1B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
  }

  Serial.begin(115200);
}

void loop() {
  if (currentMode == MODE_RUN) {
    readSensors();
    waitNextSensorCheckpoint();
    sendLevelsToSoils();
    runStopDetectionTask();
    listenToSensorCheckpointCrossing();
  }
  if (currentMode == MODE_STOP) {
    cleanLevels();
  }
}

void initializeSensorsCheckpoints() {
  for (byte i = 0; i < sensorsCheckpointCount; i += 1) {
    sensorACheckpointTime[i] = 0;
    sensorACheckpointTime[i] = 0;
  }
}

void waitNextSensorCheckpoint() {
  int expectedRoundTime = (1 / (currentSpeed / 60.0) * 1000.0);

  int sensorACheckpointValue = sensorsCheckpointMap[currentSensorACheckpoint];
  int sensorACheckpointDir = sensorsCheckpointDirMap[currentSensorACheckpoint];

  if (sensorACheckpointDir * calSensorPhaseA >= sensorACheckpointDir * sensorACheckpointValue) {
    unsigned long currentTime = millis();
    long actualRoundTime = currentTime - sensorACheckpointTime[currentSensorACheckpoint];

    int computedDcValue = map(actualRoundTime, expectedRoundTime - 100, expectedRoundTime + 100, DC_MIN_LEVEL, DC_MAX_LEVEL);

    dcValue = constrain(computedDcValue, DC_MIN_LEVEL, DC_MAX_LEVEL);

    applyMicroCorrection(actualRoundTime, expectedRoundTime);

    sensorACheckpointTime[currentSensorACheckpoint] = currentTime;
    currentSensorACheckpoint = (currentSensorACheckpoint + 1) % sensorsCheckpointCount;
  }

  int sensorBCheckpointValue = sensorsCheckpointMap[currentSensorBCheckpoint];
  int sensorBCheckpointDir = sensorsCheckpointDirMap[currentSensorBCheckpoint];

  if (sensorACheckpointDir * calSensorPhaseA >= sensorACheckpointDir * sensorACheckpointValue) {
    unsigned long currentTime = millis();
    long actualRoundTime = currentTime - sensorBCheckpointTime[currentSensorBCheckpoint];

    int computedDcValue = map(actualRoundTime, expectedRoundTime - 100, expectedRoundTime + 100, DC_MIN_LEVEL, DC_MAX_LEVEL);

    dcValue = constrain(computedDcValue, DC_MIN_LEVEL, DC_MAX_LEVEL);

    applyMicroCorrection(actualRoundTime, expectedRoundTime);

    sensorBCheckpointTime[currentSensorBCheckpoint] = currentTime;
    currentSensorBCheckpoint = (currentSensorBCheckpoint + 1) % sensorsCheckpointCount;
  }
}

void readSensors() {
  sensorPhaseA = analogRead(SENSOR_A);
  sensorPhaseB = analogRead(SENSOR_B);

  calSensorPhaseA = map(sensorPhaseA, SENSOR_A_MIN, SENSOR_A_MAX, -256, 255);
  calSensorPhaseB = map(sensorPhaseB, SENSOR_B_MIN, SENSOR_B_MAX, -256, 255);

  rawSensorValue[0] = sensorPhaseA;
  rawSensorValue[1] = sensorPhaseB;

  calibratedSensorValue[0] = calSensorPhaseA;
  calibratedSensorValue[1] = calSensorPhaseB;
}


void applyMicroCorrection(long actualRoundTime, long expectedRoundTime) {
  int timeDifference = abs(actualRoundTime - expectedRoundTime);
  float correctionRate = constrain(map(timeDifference, 0, 50, 0, 20) / 100.0, .0, .2);

  if (actualRoundTime < expectedRoundTime) {
    // too fast
    dcCorrection = max(-100.0, dcCorrection - correctionRate);
  }

  if (actualRoundTime > expectedRoundTime) {
    // too slow
    dcCorrection = min(100.0, dcCorrection + correctionRate);
  }
}

void cleanLevels() {
  analogWrite(3, 0);
  analogWrite(9, 0);
  analogWrite(10, 0);
  analogWrite(11, 0);
}

void sendLevelsToSoils() {
  int amplifiedValueA = min(255.0, dcCorrection + dcValue) / 255.0 * calSensorPhaseA;
  int amplifiedValueB = min(255.0, dcCorrection + dcValue) / 255.0 * calSensorPhaseB;

  analogWrite(3, constrain(max(0, amplifiedValueA), 0, 255));
  analogWrite(9, constrain(-min(0, amplifiedValueB), 0, 255));
  analogWrite(10, constrain(-min(0, amplifiedValueA), 0, 255));
  analogWrite(11, constrain(max(0, amplifiedValueB), 0, 255));
}

void runStopDetectionTask() {
  if (millis() - sensorACheckpointTime[0] > 2000) {
    dcValue = DC_MAX_LEVEL;
  }
}

void listenToSensorCheckpointCrossing() {
  int expectedRoundTime = (1 / (currentSpeed / 60.0) * 1000.0);

  SensorCheckpoint sc = checkpoints[currentCheckpoint];
  int level = sc.triggerLevel;
  int sign = sc.triggerDirection;
  int sensorValue = calibratedSensorValue[sc.sensorId];

  if (sign * sensorValue >= sign * level) {
    unsigned long now = millis();
    unsigned int expectedPercent = currentCheckpoint == 0
      ? (10000 - checkpoints[sensorCheckpointsCount - 1].offsetPercent)
      : (sc.offsetPercent - checkpoints[currentCheckpoint - 1].offsetPercent);

    int expectedCheckpointTime = expectedPercent / 10000.0 * expectedRoundTime;
    int actualCheckpointTime = now - lastCheckpointTime;

    Serial.print(currentCheckpoint);
    Serial.print('\t');
    Serial.print(expectedPercent);
    Serial.print('\t');
    Serial.println(actualCheckpointTime - expectedCheckpointTime);

    lastCheckpointTime = now;
    currentCheckpoint = (currentCheckpoint + 1) % sensorCheckpointsCount;
  }
}
