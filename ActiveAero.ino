#include <Arduino_LSM9DS1.h>

// Global var
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float roll, pitch;

// Thresholds for canard modes in (m/s^2) for now
#define ACCEL_THRESHOLD 2.0
#define BRAKE_THRESHOLD -2.0

void setup() {
    Serial.begin(115200); // Start Serial Monitor at 115200 baud rate
    initializeIMU();      // Set up the IMU
}

void loop() {
    // Step 1: Read IMU data
    readIMUData();

    // Step 2: Process IMU data with complementary filter algorithm
    roll = calculateRoll(accelX, accelY, accelZ);
    pitch = calculatePitch(accelX, accelY, accelZ);

    // Step 3: Determine canard mode and print results
    determineCanardMode(pitch, accelX);

    delay(10); // For now
}

// Function to initialize the IMU
void initializeIMU() {
    Serial.println("Initializing IMU...");
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU! Check connections.");
        while (1); // Stop execution if IMU fails
    }
    Serial.println("IMU initialized successfully.");
}

// Function to read accelerometer and gyroscope data
void readIMUData() {
    if (IMU.readAcceleration(accelX, accelY, accelZ)) {
        Serial.print("Accel (m/s^2) X:");
        Serial.print(accelX);
        Serial.print(" Y:");
        Serial.print(accelY);
        Serial.print(" Z:");
        Serial.println(accelZ);
    }

    if (IMU.readGyroscope(gyroX, gyroY, gyroZ)) {
        Serial.print("Gyro (rad/s) X:");
        Serial.print(gyroX);
        Serial.print(" Y:");
        Serial.print(gyroY);
        Serial.print(" Z:");
        Serial.println(gyroZ);
    }
}

// Complementary filter algorithm to combine accelerometer and gyroscope data
float complementaryFilter(float accelAngle, float gyroRate, float alpha = 0.98) {
    static float filteredAngle = 0.0; // Preserve filtered value between calls
    filteredAngle = alpha * (filteredAngle + gyroRate * 0.01) + (1 - alpha) * accelAngle;
    return filteredAngle;
}

// Function to calculate roll angle from accelerometer data
float calculateRoll(float ax, float ay, float az) {
    return atan2(ay, az) * RAD_TO_DEG; // Convert radians to degrees
}

// Function to calculate pitch angle from accelerometer data
float calculatePitch(float ax, float ay, float az) {
    return atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG; // Convert radians to degrees
}

// Function to determine canard mode and print results
void determineCanardMode(float pitch, float accelX) {
    if (accelX > ACCEL_THRESHOLD) {
        // Acceleration Mode: Canards parallel to forward motion
        Serial.println("Mode: Acceleration");
        Serial.println("Canard Position: 0° (Parallel to motion)");
    } else if (accelX < BRAKE_THRESHOLD) {
        // Braking Mode: Canards angled for air braking
        Serial.println("Mode: Braking");
        Serial.println("Canard Position: 75° (Air braking with downforce)");
    } else {
        // Downforce Mode: Canards angled for grip and stability
        Serial.println("Mode: Downforce");
        Serial.println("Canard Position: 45° (Stability)");
    }

    // Print processed data for debugging
    Serial.print("Pitch: ");
    Serial.print(pitch);
    Serial.print(" | Acceleration X: ");
    Serial.println(accelX);
}

