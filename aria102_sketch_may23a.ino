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

unsigned long sensorACheckpointTime[sensorsCheckpointCount];
unsigned long sensorBCheckpointTime[sensorsCheckpointCount];
byte currentSensorACheckpoint = 0;
byte currentSensorBCheckpoint = 0;

// power
int dcValue = DC_MAX_LEVEL;
float dcCorrection = 0.0;

void setup() {
  pinMode(3, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  initializeSensorsCheckpoints();

  if (DEBUG_MODE == false) {
    // TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
    // TCCR1B = TCCR1B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
  }

  Serial.begin(9600);
}

void loop() {
  if (currentMode == MODE_RUN) {
    readSensors();
    waitNextSensorCheckpoint();
    sendLevelsToSoils();
    runStopDetectionTask();
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

    Serial.print(expectedRoundTime);
    Serial.print('\t');
    Serial.println(actualRoundTime);

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
