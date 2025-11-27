#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ====== PCA9685 SETUP (SERVOS) ======
Adafruit_PWMServoDriver pca1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pca2 = Adafruit_PWMServoDriver(0x41);

#define SERVOMIN 110
#define SERVOMAX 590

// ====== MPU9250 SETUP (IMU) ======
#define MPU9250_ADDR  0x68
#define AK8963_ADDR   0x0C

// MPU9250 registers
#define MPU9250_PWR_MGMT_1  0x6B
#define MPU9250_CONFIG      0x1A
#define MPU9250_GYRO_CONFIG 0x1B
#define MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_INT_PIN_CFG 0x37
#define MPU9250_WHO_AM_I    0x75

// AK8963 registers
#define AK8963_CNTL1        0x0A

// IMU timing
unsigned long lastIMUTime = 0;
const int IMU_RATE = 50; // 50ms = 20Hz
bool imuAvailable = false;

// Serial buffer (like serial_bridge.ino)
char rxbuf[128];
uint8_t rxidx = 0;

// Persistent angles
int currentAngles[12];

// ====== I2C HELPERS ======
void writeByte(uint8_t addr, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t readByte(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.read();
}

void readBytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t *dest) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, count);
  uint8_t i = 0;
  while (Wire.available() && i < count) {
    dest[i++] = Wire.read();
  }
}

// ====== IMU FUNCTIONS ======
bool initMPU9250() {
  // Check WHO_AM_I
  uint8_t whoami = readByte(MPU9250_ADDR, MPU9250_WHO_AM_I);
  if (whoami != 0x71 && whoami != 0x73) {
    return false; // IMU not found
  }
  
  writeByte(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x00);
  delay(100);
  writeByte(MPU9250_ADDR, MPU9250_CONFIG, 0x03);
  writeByte(MPU9250_ADDR, MPU9250_GYRO_CONFIG, 0x00);
  writeByte(MPU9250_ADDR, MPU9250_ACCEL_CONFIG, 0x00);
  writeByte(MPU9250_ADDR, MPU9250_INT_PIN_CFG, 0x02);
  
  writeByte(AK8963_ADDR, AK8963_CNTL1, 0x00);
  delay(10);
  writeByte(AK8963_ADDR, AK8963_CNTL1, 0x16);
  delay(10);
  
  return true;
}

void readMPU9250(int16_t &ax, int16_t &ay, int16_t &az,
                 int16_t &gx, int16_t &gy, int16_t &gz,
                 int16_t &mx, int16_t &my, int16_t &mz) {
  uint8_t rawData[6];
  
  readBytes(MPU9250_ADDR, 0x3B, 6, rawData);
  ax = ((int16_t)rawData[0] << 8) | rawData[1];
  ay = ((int16_t)rawData[2] << 8) | rawData[3];
  az = ((int16_t)rawData[4] << 8) | rawData[5];
  
  readBytes(MPU9250_ADDR, 0x43, 6, rawData);
  gx = ((int16_t)rawData[0] << 8) | rawData[1];
  gy = ((int16_t)rawData[2] << 8) | rawData[3];
  gz = ((int16_t)rawData[4] << 8) | rawData[5];
  
  readBytes(AK8963_ADDR, 0x03, 6, rawData);
  mx = ((int16_t)rawData[1] << 8) | rawData[0];
  my = ((int16_t)rawData[3] << 8) | rawData[2];
  mz = ((int16_t)rawData[5] << 8) | rawData[4];
}

void publishIMU() {
  if (!imuAvailable) return;
  
  int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
  readMPU9250(ax, ay, az, gx, gy, gz, mx, my, mz);
  
  // Convert to physical units
  float ax_ms2 = (ax / 16384.0) * 9.81;
  float ay_ms2 = (ay / 16384.0) * 9.81;
  float az_ms2 = (az / 16384.0) * 9.81;
  
  float gx_rads = (gx / 131.0) * (PI / 180.0);
  float gy_rads = (gy / 131.0) * (PI / 180.0);
  float gz_rads = (gz / 131.0) * (PI / 180.0);
  
  Serial.print("IMU,");
  Serial.print(ax_ms2, 4); Serial.print(",");
  Serial.print(ay_ms2, 4); Serial.print(",");
  Serial.print(az_ms2, 4); Serial.print(",");
  Serial.print(gx_rads, 4); Serial.print(",");
  Serial.print(gy_rads, 4); Serial.print(",");
  Serial.print(gz_rads, 4); Serial.print(",");
  Serial.print(mx); Serial.print(",");
  Serial.print(my); Serial.print(",");
  Serial.print(mz);
  Serial.println();
}

// ====== SERVO FUNCTIONS ======
int angleToPulse(int realAngle, int servo_index) {
  // Invert for specific servos (match serial_bridge.ino)
  if (servo_index == 2 || servo_index == 5 || servo_index == 8 || servo_index == 11) {
    realAngle = 180 - realAngle;
  }
  int commandedAngle = map(realAngle, 0, 180, -5, 175);
  return map(commandedAngle, 0, 180, SERVOMIN, SERVOMAX);
}

void writeServo(uint8_t servoNum, int angle) {
  if (servoNum > 11) return;
  int pulse = angleToPulse(angle, servoNum);
  
  if (servoNum < 6) {
    pca1.setPWM(servoNum, 0, pulse);
  } else {
    pca2.setPWM(servoNum - 6, 0, pulse);
  }
}

// ====== SETUP ======
void setup() {
  Serial.begin(115200);
  // DON'T wait for serial connection - start immediately
  
  Wire.begin();
  Wire.setClock(400000);
  
  pca1.begin();
  pca2.begin();
  pca1.setPWMFreq(50);
  pca2.setPWMFreq(50);
  delay(100);
  
  // Initialize all servos to 90 degrees
  for (int i = 0; i < 12; i++) {
    currentAngles[i] = 90;
    writeServo(i, 90);
  }
  
  delay(200);
  
  // Try to init IMU (non-blocking if it fails)
  imuAvailable = initMPU9250();
  
  Serial.println("READY");
  Serial.flush();
  
  if (imuAvailable) {
    Serial.println("IMU_OK");
  }
}

// ====== LOOP ======
void loop() {
  // CRITICAL: Process ALL available serial data FIRST
  // This prevents buffer overflow and ensures responsiveness
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    
    // Ignore carriage return
    if (c == '\r') continue;
    
    // Process complete line
    if (c == '\n') {
      rxbuf[rxidx] = '\0'; // Null terminate
      
      // Parse CSV: expecting exactly 12 comma-separated integers
      int vals[12];
      int found = 0;
      
      char *ptr = rxbuf;
      char *endptr;
      
      // Fast parsing using strtol (like serial_bridge.ino)
      while (found < 12 && *ptr) {
        // Skip whitespace and commas
        while (*ptr == ' ' || *ptr == ',') ptr++;
        if (!*ptr) break;
        
        // Parse integer
        long val = strtol(ptr, &endptr, 10);
        if (ptr == endptr) break; // No valid number
        
        vals[found++] = (int)val;
        ptr = endptr;
      }
      
      // Apply ONLY if we got exactly 12 values
      if (found == 12) {
        // Validate range
        bool valid = true;
        for (int i = 0; i < 12; i++) {
          if (vals[i] < 0 || vals[i] > 180) {
            valid = false;
            break;
          }
        }
        
        if (valid) {
          // Apply to hardware IMMEDIATELY
          for (int i = 0; i < 12; i++) {
            currentAngles[i] = vals[i];
            writeServo(i, vals[i]);
          }
        }
      }
      
      // Reset buffer for next line
      rxidx = 0;
      
    } else {
      // Add to buffer (with overflow protection)
      if (rxidx < sizeof(rxbuf) - 1) {
        rxbuf[rxidx++] = c;
      } else {
        // Buffer overflow - reset
        rxidx = 0;
      }
    }
  }
  
  // Publish IMU data at 20Hz (AFTER processing serial)
  if (millis() - lastIMUTime >= IMU_RATE) {
    publishIMU();
    lastIMUTime = millis();
  }
  
  // CRITICAL: Minimal delay to prevent blocking
  delayMicroseconds(100);
}