// VCC - 3.3V
// GND - GND
// SCL - GPIO22
// SDA - GPIO21
// AD0 - GND (Set I2C address to 0x68)

#include <Wire.h>

// MPU6500 I2C address
const uint8_t MPU_ADDR = 0x68;

// Accelerometer and gyroscope sensitivity 
const float ACCEL_SENSITIVITY = 16384.0; // ±2g (LSB/g)
const float GYRO_SENSITIVITY = 131.0; // ±250dps (LSB/dps)

// Register addresses
const uint8_t PWR_MGMT_1 = 0x6B; // Pwr management register (For waking up)
const uint8_t ACCEL_XOUT_H = 0x3B; // High byte of X-axis acceleration
const uint8_t GYRO_XOUT_H = 0x43; // High byte of X-axis angular velocity

// Collision detection settings
const float COLLISION_THRESHOLD = 0.4; // Lower for slower speed collisions
const int   GYRO_Z_THRESHOLD = 5; // Threshold for z-axis rotation
const unsigned long COLLISION_COOLDOWN = 500; // ms between allowed collisions

// Global variables for collision detection:
float prev_ax = 0.0, prev_ay = 0.0, prev_az = 0.0; // Previous acceleration values
unsigned long last_collision_time = 0;



void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Wake up the MPU6500
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0); // Clear sleep bit to start sensor
  Wire.endTransmission(true);

  // Allow sensor to settle then read initial ax value
  delay(100);
  prev_ax = readAccelX() / ACCEL_SENSITIVITY;
}




void loop() {
  // Read accelerometer data (2 bytes each)
  int16_t accelX = readSensor(ACCEL_XOUT_H); // X-axis (forward)
  int16_t accelY = readSensor(ACCEL_XOUT_H + 2); // Y-axis
  int16_t accelZ = readSensor(ACCEL_XOUT_H + 4); // Z-axis
  
  // Read gyroscope data
  int16_t gyroX = readSensor(GYRO_XOUT_H); // X-axis
  int16_t gyroY = readSensor(GYRO_XOUT_H + 2); // Y-axis
  int16_t gyroZ = readSensor(GYRO_XOUT_H + 4); // Z-axis
  
  // Convert raw readings to physical units 
  // (g)
  float ax = accelX / ACCEL_SENSITIVITY;
  float ay = accelY / ACCEL_SENSITIVITY;
  float az = accelZ / ACCEL_SENSITIVITY;
  // (dps)
  float gx = gyroX / GYRO_SENSITIVITY;
  float gy = gyroY / GYRO_SENSITIVITY;
  float gz = gyroZ / GYRO_SENSITIVITY;
  
  // Print values formatted as: ax, ay, az, gx, gy, gz
  Serial.print(ax, 2); Serial.print(", ");
  Serial.print(ay, 2); Serial.print(", ");
  Serial.print(az, 2); Serial.print(", ");
  Serial.print(gx, 2); Serial.print(", ");
  Serial.print(gy, 2); Serial.print(", ");
  Serial.print(gz, 2);
  
  // --- Collision Detection ---
  unsigned long current_time = millis();
  if (current_time - last_collision_time > COLLISION_COOLDOWN) {
    // Calculate change in acceleration for all axes
    float delta_ax = abs(prev_ax - ax);
    float delta_ay = abs(prev_ay - ay);
    float delta_az = abs(prev_az - az);
    
    // Check if any acceleration change exceeds threshold
    if ((delta_ax > COLLISION_THRESHOLD || delta_ay > COLLISION_THRESHOLD || delta_az > COLLISION_THRESHOLD) && abs(gz) > GYRO_Z_THRESHOLD) {
      // Determine collision side based on z axis reading
      String collisionSide;
      if (gz > 0) {
        collisionSide = "Front Right";
      } else if (gz < 0) {
        collisionSide = "Front Left";
      } else {
        collisionSide = "Unknown";
      }
      
      Serial.print(" BUMP: ");
      Serial.print(collisionSide);
      
      last_collision_time = current_time;  // Start cooldown
    }
  }
  Serial.println();
  
  // Save acceleration values for next loop
  prev_ax = ax;
  prev_ay = ay;
  prev_az = az;
  
  delay(100);
}

// Function to read 16 bit sensor register
int16_t readSensor(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);  // Get high and low bytes
  
  uint8_t highByte = Wire.read();
  uint8_t lowByte = Wire.read();
  
  return (highByte << 8) | lowByte;
}

// Quickly read accel X (used once in setup)
int16_t readAccelX() {
  return readSensor(ACCEL_XOUT_H);
}
