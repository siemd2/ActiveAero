#include <Arduino_LSM9DS1.h>
#include <Servo.h> // Include the Servo library

// Global variables
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float roll, pitch;

// Thresholds for canard modes in (m/s^2) for now
#define ACCEL_THRESHOLD 2.0
#define BRAKE_THRESHOLD -2.0
#define ROLL_THRESHOLD 5.0 // Threshold in degrees to detect left or right turns

// Servo objects for controlling the canards
Servo leftCanardServo;
Servo rightCanardServo;

void setup() {
    Serial.begin(115200); // Start Serial Monitor at 115200 baud rate
    initializeIMU();      // Set up the IMU

    // Initialize the servo motors
    Serial.println("Initializing servos...");
    leftCanardServo.attach(3);  // Left canard on pin 3
    rightCanardServo.attach(5); // Right canard on pin 5

    // Set initial positions to Downforce (45°)
    leftCanardServo.write(45);
    rightCanardServo.write(45);
    Serial.println("Servos initialized.");
}

void loop() {
    // Step 1: Read IMU data
    readIMUData();

    // Step 2: Process IMU data with complementary filter algorithm
    roll = calculateRoll(accelX, accelY, accelZ);
    pitch = calculatePitch(accelX, accelY, accelZ);

    // Step 3: Determine canard mode and control servos
    determineCanardMode(pitch, accelX, roll);

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

// Function to determine canard mode and control servos
void determineCanardMode(float pitch, float accelX, float roll) {
    if (accelX > ACCEL_THRESHOLD) {
        // Acceleration Mode: Canards parallel to forward motion
        Serial.println("Mode: Acceleration");
        leftCanardServo.write(0); // Left canard at 0°
        rightCanardServo.write(0); // Right canard at 0°
    } else if (accelX < BRAKE_THRESHOLD) {
        // Braking Mode: Canards angled for air braking
        Serial.println("Mode: Braking");
        leftCanardServo.write(90); // Both canards at 90°
        rightCanardServo.write(90);
    } else if (roll < -ROLL_THRESHOLD) {
        // Left Turn Mode: Adjust for left turn grip
        Serial.println("Mode: Left Turn");
        leftCanardServo.write(60); // Left canard adds more downforce
        rightCanardServo.write(30); // Right canard reduces downforce
    } else if (roll > ROLL_THRESHOLD) {
        // Right Turn Mode: Adjust for right turn grip
        Serial.println("Mode: Right Turn");
        leftCanardServo.write(30); // Left canard reduces downforce
        rightCanardServo.write(60); // Right canard adds more downforce
    } else {
        // Downforce Mode: Canards angled for grip and stability
        Serial.println("Mode: Downforce");
        leftCanardServo.write(45); // Both canards at 45° for stability
        rightCanardServo.write(45);
    }

    // Print processed data for debugging
    Serial.print("Pitch: ");
    Serial.print(pitch);
    Serial.print(" | Acceleration X: ");
    Serial.print(accelX);
    Serial.print(" | Roll: ");
    Serial.println(roll);
}


