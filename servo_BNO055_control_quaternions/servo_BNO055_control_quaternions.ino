#include <i2c_driver_wire.h>
#include "bno055.h"

#define ADDRESS_PIN 21
#define INT_PIN 22

struct bno055_t orientation_sensor;
struct bno055_quaternion_t quaternion_data;
struct bno055_euler_float_t eulerData;

u8 operation_mode = 0;
u8 power_mode = 0;

float w;
float x;
float y;
float z;
float theta;
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
  bno055_read_quaternion_wxyz(&quaternion_data);
  bno055_convert_float_euler_hpr_deg(&eulerData);
  Serial.println("Raw Quaternion Data:");
  w = (float) quaternion_data.w / (1 << 14);
  x = (float) quaternion_data.x / (1 << 14);
  y = (float) quaternion_data.y / (1 << 14);
  z = (float) quaternion_data.z / (1 << 14);
  theta = 2 * acos(w);
  Serial.print("w: "); Serial.print(w, DEC); Serial.print("\t"); 
  Serial.print("x: "); Serial.print(x, DEC); Serial.print("\t"); 
  Serial.print("y: "); Serial.print(y, DEC); Serial.print("\t"); 
  Serial.print("z: "); Serial.print(z, DEC); Serial.print("\t"); 
  Serial.println("");
  Serial.println("Decomposed Angle and Vector:");
  Serial.print("theta: "); Serial.print(theta); Serial.print("\t");
  Serial.print("angle: "); Serial.print(theta * 180 / PI); Serial.print("\t");
  Serial.print("x: "); Serial.print(x / sin(theta/2)); Serial.print("\t");
  Serial.print("y: "); Serial.print(y / sin(theta/2)); Serial.print("\t");
  Serial.print("z: "); Serial.print(z / sin(theta/2)); Serial.print("\t");
  Serial.println("");
  Serial.println("Euler Data:");
  Serial.print("h: "); Serial.print(eulerData.h); Serial.print("\t");
  Serial.print("r: "); Serial.print(eulerData.r); Serial.print("\t");
  Serial.print("p: "); Serial.print(eulerData.p); Serial.print("\t");
  Serial.println();
  delay(200);
}

s8 bno055_write(u8 dev_addr, u8 reg_addr, u8 * reg_data, u8 wr_len) {
//  Serial.println("BEGIN WRITE");
//  Serial.print("device addr: "); Serial.print(dev_addr, HEX); Serial.print("\t");
//  Serial.print("register addr: "); Serial.print(reg_addr, HEX); Serial.print("\t");
//  Serial.print("request length: "); Serial.print(wr_len); Serial.println("\t");
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.write(reg_data, wr_len);
  s8 code = Wire.endTransmission();
//  Serial.print("write code: "); Serial.println(code);
//  Serial.println();
  return code;
}

s8 bno055_read(u8 dev_addr, u8 reg_addr, u8 * reg_data, u8 r_len) {
  u8 index = 0;
//  Serial.println("BEGIN READ");
//  Serial.print("device addr: "); Serial.print(dev_addr, HEX); Serial.print("\t");
//  Serial.print("register addr: "); Serial.print(reg_addr, HEX);  Serial.print("\t");
//  Serial.print("request length: "); Serial.print(r_len); Serial.println("\t");
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  u8 code = Wire.endTransmission();
//  Serial.print("write address code in read: "); Serial.println(code);
  s8 bytes_returned = Wire.requestFrom(dev_addr, r_len);
//  Serial.print("read # bytes returned: "); Serial.println(bytes_returned);
  while (Wire.available()) {
//    Serial.print("index: "); Serial.println(index);
    u8 data = Wire.read();
//    Serial.print("data read in: "); Serial.println(data, HEX);
    reg_data[index] = data;
    index++;
  }
  Serial.println();
  return 0;
}

void delay_wrapper(u32 ms) {
  delay(ms);
}
