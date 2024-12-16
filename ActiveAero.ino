#include <Arduino_LSM9DS1.h>

// Global var
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float roll, pitch;

// Thresholds for canard modes in (m/s^2) for now
#define ACCEL_THRESHOLD 2.0
#define BRAKE_THRESHOLD -2.0

#define RAD_TO_DEG 57.296  // Conversion factor from radians to degrees
#define SCALE_FACTOR 10000.0  // Scaling factor used for fixed-point arithmetic

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

// // Complementary filter algorithm to combine accelerometer and gyroscope data
// float complementaryFilter(float accelAngle, float gyroRate, float alpha = 0.98) {
//     static float filteredAngle = 0.0; // Preserve filtered value between calls
//     filteredAngle = alpha * (filteredAngle + gyroRate * 0.01) + (1 - alpha) * accelAngle;
//     return filteredAngle;
// }
float complementaryFilter(float accelAngle, float gyroRate, float alpha) {
    static float filteredAngle = 0.0; // Preserve value between calls
    float result;
    asm volatile (
        "fmuls %1, %3         \n\t"  // alpha * gyroRate
        "fadds %4, r0         \n\t"  // Add filteredAngle
        "fmuls %2, %5         \n\t"  // (1 - alpha) * accelAngle
        "fadds r0, r0, r1     \n\t"  // Combine results
        "mov %0, r0           \n\t"  // Move result to output
        : "=r" (result)              // Output
        : "r" (gyroRate), "r" (accelAngle), "r" (alpha), "r" (filteredAngle), "r" (1.0f - alpha) // Inputs
        : "r0"                       // Clobbers
    );
    filteredAngle = result; // Update static value
    return result;
}

// // Function to calculate roll angle from accelerometer data
// float calculateRoll(float ax, float ay, float az) {
//     return atan2(ay, az) * RAD_TO_DEG; // Convert radians to degrees
// }
float calculateRoll(float ax, float ay, float az) {
    float result;

    // Convert the float inputs (ay, az) to integers (scaled by 10000)
    int ay_scaled = ay * SCALE_FACTOR;
    int az_scaled = az * SCALE_FACTOR;

    // Assembly to compute atan2 approximation and convert to degrees
    asm volatile(
        "mov r24, %[ay_scaled]         \n\t"   // Load ay_scaled into r24
        "mov r25, %[az_scaled]         \n\t"   // Load az_scaled into r25
        
        "cp r24, r25                    \n\t"   // Compare ay and az
        "BRGE atan_positive             \n\t"   // If ay >= az, go to positive angle
        
        "atan_negative:                 \n\t"   // Else, negative angle
        "mov r26, r24                   \n\t"   // Copy ay_scaled (negative roll)
        "mul r26, %[rad_to_deg]         \n\t"   // Multiply by RAD_TO_DEG
        "mov %[result], r0              \n\t"   // Store lower byte of result
        "rjmp end_calculation           \n\t"   // Jump to end of calculation
        
        "atan_positive:                 \n\t"   // Positive result handling
        "mov r26, r25                   \n\t"   // Copy az_scaled (positive roll)
        "mul r26, %[rad_to_deg]         \n\t"   // Multiply by RAD_TO_DEG
        "mov %[result], r0              \n\t"   // Store lower byte of result

        "end_calculation:               \n\t"
        : [result] "=r" (result)         // Output operand
        : [ay_scaled] "r" (ay_scaled), [az_scaled] "r" (az_scaled), [rad_to_deg] "r" (RAD_TO_DEG)  // Input operands
        : "r24", "r25", "r26", "r0"  // Clobbered registers
    );
    result = result / SCALE_FACTOR;  // Undo the scaling

    return result;  // Return the final roll value in degrees
}

// Function to calculate pitch angle from accelerometer data
// float calculatePitch(float ax, float ay, float az) {
//     return atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG; // Convert radians to degrees
// }
float calculatePitch(float ax, float ay, float az) {
    float result;

    // Convert the float inputs (ax, ay, az) to integers (scaled by 10000)
    int ax_scaled = ax * SCALE_FACTOR;
    int denominator_scaled = sqrt(ay * ay + az * az) * SCALE_FACTOR;  // Precompute denominator

    // Assembly to compute atan2 approximation for pitch
    // Multiply by RAD_TO_DEG and unscale (divide by SCALE_FACTOR) in assembly
    asm volatile(
        "mov r24, %[ax_scaled]          \n\t"  // Load ax_scaled into r24
        "mov r25, %[denominator_scaled]  \n\t"  // Load denominator_scaled into r25
        
        "cp r24, r25                    \n\t"   // Compare ax and denominator
        "BRGE atan_positive_pitch       \n\t"   // If ax >= denominator, go to positive pitch
        
        "atan_negative_pitch:           \n\t"
        "mov r26, r24                   \n\t"  // Copy ax_scaled (negative pitch)
        "mul r26, %[rad_to_deg]         \n\t"  // Multiply by RAD_TO_DEG
        "mov %[result], r0              \n\t"  // Move lower byte of result to result variable
        "rjmp end_pitch_calculation     \n\t"   // Jump to end of calculation
        
        "atan_positive_pitch:           \n\t"   // Positive pitch handling
        "mov r26, r25                   \n\t"  // Copy denominator_scaled (positive pitch)
        "mul r26, %[rad_to_deg]         \n\t"  // Multiply by RAD_TO_DEG
        "mov %[result], r0              \n\t"  // Move lower byte of result to result variable

        "end_pitch_calculation:         \n\t"
        : [result] "=r" (result)         // Output operand
        : [ax_scaled] "r" (ax_scaled), [denominator_scaled] "r" (denominator_scaled), [rad_to_deg] "r" (RAD_TO_DEG)  // Input operands
        : "r24", "r25", "r26", "r0"  // Clobbered registers
    );
    result = result / SCALE_FACTOR; // Undo the scaling

    return result;
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
