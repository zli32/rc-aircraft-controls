#include <Wire.h>
#include <elapsedMillis.h>
#include "bno055.h"

#define ADDRESS_PIN 21
#define INT_PIN 22

struct bno055_t orientation_sensor;
struct bno055_quaternion_t quaternion_data;
struct bno055_euler_float_t eulerData;
ellapsedMillis print_timer;

u8 operation_mode = 0;
u8 power_mode = 0;
u8 axis_remap_value;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.setClock(400000);
  Wire.begin();

  orientation_sensor.bus_read = bno055_read;
  orientation_sensor.bus_write = bno055_write;
  orientation_sensor.delay_msec = delay_wrapper;
  orientation_sensor.dev_addr = BNO055_I2C_ADDR1;
  Serial.println("Intitialize BNO055");
  bno055_init(&orientation_sensor);
  Serial.println("Set operation mode");
  bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
  Serial.println("Readback operation mode");
  bno055_get_operation_mode(&operation_mode);
  bno055_get_axis_remap_value(&axis_remap_value);
  Serial.println("Axis configuration:");
  Serial.println(axis_remap_value, HEX);
  Serial.println("END SETUP\n");
}

void loop() {
  // put your main code here, to run repeatedly:
  bno055_convert_float_euler_hpr_deg(&eulerData);
}

s8 bno055_write(u8 dev_addr, u8 reg_addr, u8 * reg_data, u8 wr_len) {
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.write(reg_data, wr_len);
  s8 code = Wire.endTransmission();
  return code;
}

s8 bno055_read(u8 dev_addr, u8 reg_addr, u8 * reg_data, u8 r_len) {
  u8 index = 0;
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  u8 code = Wire.endTransmission();
  s8 bytes_returned = Wire.requestFrom(dev_addr, r_len);
  while (Wire.available()) {
    u8 data = Wire.read();
    reg_data[index] = data;
    index++;
  }
  Serial.println();
  return 0;
}

void delay_wrapper(u32 ms) {
  delay(ms);
}

void data_debug_print() {
  if (print_timer > 200) {
    Serial.println("Euler Data:");
    Serial.print("h: "); Serial.print(eulerData.h); Serial.print("\t");
    Serial.print("r: "); Serial.print(eulerData.r); Serial.print("\t");
    Serial.print("p: "); Serial.print(eulerData.p); Serial.print("\t");
    Serial.println();
    print_timer = 0;
  }
}
