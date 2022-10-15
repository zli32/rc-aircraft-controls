#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#define PID_SETPOINT 0
#define ALPHA 0.5
#define Kp 40.5
#define Ki 0.004
#define MAX_ROC = 20


Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Servo servo;
sensors_event_t event;
float alpha = 0.5;
float filtered_y_accel;
float last_y_accel;
float integral_term = 0;
float pid_output;
float current_servo_angle = 90;
uint32_t servo_control_period = 0;
elapsedMillis time_passed;
elapsedMillis servo_control_timer = 0;

void setup() {
  if(!accel.begin())
  {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }
  accel.setRange(ADXL345_RANGE_16_G);
  accel.setDataRate(ADXL345_DATARATE_25_HZ);
  accel.getEvent(&event);
  last_y_accel = event.acceleration.y;
  servo.attach(0);
  servo.write(90);
  time_passed = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  accel.getEvent(&event);
  filter();
  pid_output = pid(filtered_y_accel);
  time_passed = 0;
  set_servo_control_period();
  if (servo_control_timer > servo_control_period) {
    rotate_servo();
    servo_control_timer = 0;
  }
  Serial.print("PI Output: "); Serial.println(pid_output);
  Serial.print("Current servo angle: "); Serial.println(current_servo_angle);
  Serial.print("Current servo control period: "); Serial.println(servo_control_period);
  Serial.print("Current filtered y accel: "); Serial.println(filtered_y_accel);
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
//  Serial.print("Generated angle: "); Serial.println(map(event.acceleration.y, -20, 20, 20 , 160), DEC);
//  angle = alpha * map(event.acceleration.y, -20, 20, 20 , 160) + (1 - alpha) * last_output_angle;
//  Serial.print("Filtered angle: "); Serial.println(angle, DEC);
}

void filter() {
  filtered_y_accel = ALPHA * event.acceleration.y + (1 - ALPHA) * last_y_accel;
  last_y_accel = filtered_y_accel;
}

float pid(float process_var) {
  float error = PID_SETPOINT - process_var;
  integral_term += error * time_passed;
  return Kp * (error) + Ki * (integral_term);
}

void rotate_servo() {
  if (current_servo_angle > 160) {
    servo.write(current_servo_angle = 160);
    return;
  }
  if (current_servo_angle < 20) {
    servo.write(current_servo_angle = 20);
  }
  if (filtered_y_accel > 0.5) {
    servo.write(current_servo_angle -= 1);
  }
  if (filtered_y_accel < -0.5) {
    servo.write(current_servo_angle += 1);
  }
}
void set_servo_control_period() {
  servo_control_period = 1000 / abs(pid_output);
}
