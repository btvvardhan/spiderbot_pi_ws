#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ===================== PCA9685 DRIVERS =====================
Adafruit_PWMServoDriver pca1 = Adafruit_PWMServoDriver(0x40); // First PCA9685
Adafruit_PWMServoDriver pca2 = Adafruit_PWMServoDriver(0x41); // Second PCA9685

// ===================== SERVO PULSE LIMITS ==================
#define SERVOMIN 110
#define SERVOMAX 590
#define SERVO_FREQ 50     // 50 Hz for most hobby servos

// ===================== LEG / JOINT NAMES ===================
enum Leg  : uint8_t { FL=0, FR=1, RL=2, RR=3 };            // Front-Left, Front-Right, Rear-Left, Rear-Right
enum Joint: uint8_t { HIP=0, KNEE=1, ANKLE=2 };            // 3 joints per leg

// ===================== SERVO MAPPING =======================
struct ServoID {
  uint8_t board;    // 0 -> pca1, 1 -> pca2
  uint8_t channel;  // 0..15
  int8_t  trim;     // degrees (+ CW, - CCW)
  bool    reversed; // true => angle command is mirrored around 90°
};

// 4 legs x 3 joints = 12 entries — edit to match your wiring.
ServoID mapTable[4][3] = {
  /* FL */ { {0,0, 0,false}, {0,1, 0,false}, {0,2, 0,false} },       // HIP,KNEE,ANKLE
  /* FR */ { {1,3, 0,false}, {1,4, 0,false}, {1,5, 0,false}  },
  /* RL */ { {0,3, 0,false}, {0,4, 0,false}, {0,5, 0,false} },
  /* RR */ { {1,0, 0,false}, {1,1, 0,false}, {1,2, 0,false}  }
};

// ===================== ANGLE → PULSE =======================
int angleToPulseCalibrated(int angleDeg, const ServoID& s) {
  int a = constrain(angleDeg, 0, 180);
  if (s.reversed) a = 180 - a;
  a = constrain(a + s.trim, 0, 180);
  int commandedAngle = map(a, 0, 180, -5, 175);
  return map(commandedAngle, 0, 180, SERVOMIN, SERVOMAX);
}

// ===================== LOW-LEVEL SET =======================
void setServoPulse(const ServoID& s, int pulse) {
  if (s.board == 0) {
    pca1.setPWM(s.channel, 0, pulse);
  } else {
    pca2.setPWM(s.channel, 0, pulse);
  }
}

// ===================== HIGH-LEVEL HELPERS ==================
void setJointAngle(Leg leg, Joint joint, int angleDeg) {
  const ServoID& s = mapTable[(uint8_t)leg][(uint8_t)joint];
  int pulse = angleToPulseCalibrated(angleDeg, s);
  setServoPulse(s, pulse);

  Serial.print("Leg ");
  switch(leg){ case FL: Serial.print("FL"); break; case FR: Serial.print("FR"); break; case RL: Serial.print("RL"); break; case RR: Serial.print("RR"); break; }
  Serial.print(" | Joint ");
  switch(joint){ case HIP: Serial.print("HIP"); break; case KNEE: Serial.print("KNEE"); break; case ANKLE: Serial.print("ANKLE"); break; }
  Serial.print(" -> "); Serial.print(angleDeg); Serial.print("° (pulse "); Serial.print(pulse); Serial.println(")");
}

void setLegAngles(Leg leg, int hipDeg, int kneeDeg, int ankleDeg) {
  setJointAngle(leg, HIP,   hipDeg);
  setJointAngle(leg, KNEE,  kneeDeg);
  setJointAngle(leg, ANKLE, ankleDeg);
}

void setAllNeutral(int angleDeg = 90) {
  for (uint8_t L = 0; L < 4; L++) {
    for (uint8_t J = 0; J < 3; J++) {
      setJointAngle((Leg)L, (Joint)J, angleDeg);
      delay(10);
    }
  }
}

// ===================== WALKING GAIT SEQUENCE =====================
// gaitSteps[step][leg][joint] = angleDeg
int gaitSteps[][4][3] = {
    
  // { { 90, 90, 35},   // FL
  //   { 90, 90, 35},   // FR
  //   { 90, 90, 35},   // RL
  //  {90, 90, 35} },
  
   { { 145, 20, 35},   // FL lifting
    { 90, 55, 45},   
    { 35, 55, 45},   
    {90, 55, 45} },

    { { 90, 20, 35},   // FL hover
    { 90, 55, 45},   
    { 35, 55, 45},   
    {90, 55, 45} },

    { { 90, 45, 35},   // FL landing
    { 90, 45, 35},   
    { 35, 45, 35},   
    {90, 45, 35} },

    //step 1
    { { 90, 45, 35},  
    { 35, 45, 35},   
    { 90, 45, 35},   
    {145, 20, 35} }, // & RR hover

    { { 90, 55, 45},  
    { 35, 20, 35},   //FR lifting
    { 90, 55, 45},   
    {145, 55, 45} },

    { { 90, 55, 45},  
    { 90, 20, 35},   //FR hover
    { 90, 55, 45},   
    {145, 55, 45} },

    { { 90, 45, 35},  
    { 90, 45, 35},   //FR landing
    { 90, 45, 35},   
    {145, 45, 35} },

    //step 2
    { { 145, 45, 35},   // FL
    { 90, 45, 35},   // FR
    { 35, 20, 35},   // RL hover
    {90, 45, 35} },   //RR

};

const size_t NUM_STEPS = sizeof(gaitSteps) / sizeof(gaitSteps[0]);

// ===================== GAIT EXECUTION ======================
unsigned long stepDelayMs = 450;   // hold time per pose
bool pauseBetweenCycles = false;   // pause between gait cycles

void blendAndSetPose(uint8_t mix, size_t fromIdx, size_t toIdx) {
  for (uint8_t leg = 0; leg < 4; leg++) {
    int hip   = map(mix, 0, 255, gaitSteps[fromIdx][leg][HIP],   gaitSteps[toIdx][leg][HIP]);
    int knee  = map(mix, 0, 255, gaitSteps[fromIdx][leg][KNEE],  gaitSteps[toIdx][leg][KNEE]);
    int ankle = map(mix, 0, 255, gaitSteps[fromIdx][leg][ANKLE], gaitSteps[toIdx][leg][ANKLE]);
    setLegAngles((Leg)leg, hip, knee, ankle);
  }
}

void walkLoop(unsigned long holdMsPerStep, uint8_t smoothSteps = 0) {
  for (size_t step = 0; step < NUM_STEPS; step++) {
    Serial.print("Executing Step "); Serial.println(step);

    // Hold current step
    for (uint8_t leg = 0; leg < 4; leg++) {
      setLegAngles((Leg)leg,
                   gaitSteps[step][leg][HIP],
                   gaitSteps[step][leg][KNEE],
                   gaitSteps[step][leg][ANKLE]);
    }
    delay(holdMsPerStep);

    // Smooth blend to next step
    if (smoothSteps > 0) {
      size_t nextStep = (step + 1) % NUM_STEPS;
      for (uint8_t b = 1; b <= smoothSteps; b++) {
        uint8_t mix = map(b, 0, smoothSteps, 0, 255);
        blendAndSetPose(mix, step, nextStep);
        delay(max(holdMsPerStep / (smoothSteps + 1), 1UL)); // <-- FIXED LINE
      }
    }
  }
}

// ===================== SETUP / LOOP ========================
void setup() {
  Serial.begin(115200);
  while(!Serial) {;}
  Serial.println("=== Two PCA9685 Servo Controller (4 legs x 3 joints) ===");

  pca1.begin();
  pca2.begin();
  pca1.setPWMFreq(SERVO_FREQ);
  pca2.setPWMFreq(SERVO_FREQ);
  delay(500);

  Serial.println("Moving all joints to neutral 90°...");
  setAllNeutral(90);
  delay(800);

  Serial.print("Loaded gait with "); Serial.print(NUM_STEPS); Serial.println(" steps.");
}

void loop() {
  // Continuous walking loop
  walkLoop(stepDelayMs, 20); // holdMsPerStep, smoothSteps (3 = smooth blending)
  if (pauseBetweenCycles) delay(1000);
}