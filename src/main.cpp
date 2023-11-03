#include <Zumo32U4.h>

Zumo32U4Encoders encoders;
Zumo32U4Motors motors;

const float wheel_diameter = 39.0; // Wheel diameter in millimeters
const float gear_ratio = 75.81;   // 75.81:1 gear ratio

unsigned long last_update_time = 0;
int prev_left_count = 0;
int prev_right_count = 0;

float left_distance = 0.0;  // in millimeters
float right_distance = 0.0; // in millimeters

void setup() {
  encoders.init();
  motors.setSpeeds(0, 0); // Stop the motors initially
}

void loop() {
  unsigned long current_time = millis();
  unsigned long delta_time = current_time - last_update_time;

  if (delta_time >= 100) {
    last_update_time = current_time;

    int left_count = encoders.getCountsLeft();
    int right_count = encoders.getCountsRight();

    int left_count_diff = left_count - prev_left_count;
    int right_count_diff = right_count - prev_right_count;

    prev_left_count = left_count;
    prev_right_count = right_count;

    float left_distance_diff = (float)left_count_diff / gear_ratio * PI * wheel_diameter / 12.0;  // Calculate left wheel distance in millimeters
    float right_distance_diff = (float)right_count_diff / gear_ratio * PI * wheel_diameter / 12.0; // Calculate right wheel distance in millimeters

    left_distance += left_distance_diff;
    right_distance += right_distance_diff;

    float velocity_left = left_distance_diff / (delta_time / 1000.0);   // Calculate left wheel velocity in mm/s
    float velocity_right = right_distance_diff / (delta_time / 1000.0); // Calculate right wheel velocity in mm/s

    Serial.print("Left Distance: ");
    Serial.print(left_distance);
    Serial.print(" mm, Right Distance: ");
    Serial.print(right_distance);
    Serial.print(" mm, Left Velocity: ");
    Serial.print(velocity_left);
    Serial.print(" mm/s, Right Velocity: ");
    Serial.print(velocity_right);
    Serial.println(" mm/s");
  }
}

