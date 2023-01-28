#include <Wire.h>
#include "bno055.h"

#define RESET_PIN 13
#define ADDRESS_PIN 21
#define INT_PIN 22

struct bno055_t myBNO;
struct bno055_euler_float_t eulerData;
u8 accel_cal_status = 0;
u8 gyro_cal_status = 0;
u8 mag_cal_status = 0;
u8 sys_cal_status = 0;
u8 operation_mode = 0;
u8 power_mode = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.setClock(100000);
  Wire.begin();

  // Note: using pinMode on RESET_PIN seems to completely brick the I2C interface
//  pinMode(RESET_PIN, OUTPUT);
  pinMode(ADDRESS_PIN, OUTPUT);
  pinMode(INT_PIN, INPUT);
//  digitalWrite(RESET_PIN, HIGH);
  digitalWrite(ADDRESS_PIN, LOW);

  myBNO.bus_read = bno055_read;
  myBNO.bus_write = bno055_write;
  myBNO.delay_msec = delay_wrapper;
  myBNO.dev_addr = BNO055_I2C_ADDR1;
  Serial.println("Intitialize BNO055");
  bno055_init(&myBNO);
  Serial.println("Set operation mode");
  bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
  Serial.println("Readback operation mode");
  bno055_get_operation_mode(&operation_mode);
  Serial.println("END SETUP\n");
}

void loop() {
  // put your main code here, to run repeatedly:
  s8 sys_code = bno055_get_sys_calib_stat(&sys_cal_status);  
  s8 gyro_code = bno055_get_gyro_calib_stat(&gyro_cal_status);
  s8 accel_code = bno055_get_accel_calib_stat(&accel_cal_status);
  s8 mag_code = bno055_get_mag_calib_stat(&mag_cal_status);
  Serial.print("sys status retrieval code: "); Serial.println(sys_code);
  Serial.print("gyro status retrieval code: "); Serial.println(gyro_code);
  Serial.print("accel status retrieval code: "); Serial.println(accel_code);
  Serial.print("mag status retrieval code: "); Serial.println(mag_code);
  Serial.println();
  Serial.println("Caibration Status: ");
  Serial.print(sys_cal_status); Serial.print("\t");
  Serial.print(gyro_cal_status); Serial.print("\t");
  Serial.print(accel_cal_status); Serial.print("\t");
  Serial.print(mag_cal_status); Serial.print("\t");
  Serial.println();
  bno055_convert_float_euler_hpr_deg(&eulerData);
  Serial.println("Euler Data:");
  Serial.print("h: "); Serial.print(eulerData.h); Serial.print("\t");
  Serial.print("r: "); Serial.print(eulerData.r); Serial.print("\t");
  Serial.print("p: "); Serial.print(eulerData.p); Serial.print("\t");
  Serial.println();
  bno055_get_power_mode(&power_mode);
  Serial.println("Power mode: "); Serial.print(power_mode);
  Serial.println("\n\n");
  delay_wrapper(500);
}

s8 bno055_write(u8 dev_addr, u8 reg_addr, u8 * reg_data, u8 wr_len) {
  Serial.println("BEGIN WRITE");
  Serial.print("device addr: "); Serial.print(dev_addr, HEX); Serial.print("\t");
  Serial.print("register addr: "); Serial.print(reg_addr, HEX); Serial.print("\t");
  Serial.print("request length: "); Serial.print(wr_len); Serial.println("\t");
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.write(reg_data, wr_len);
  s8 code = Wire.endTransmission();
  Serial.print("write code: "); Serial.println(code);
  Serial.println();
  return code;
}

s8 bno055_read(u8 dev_addr, u8 reg_addr, u8 * reg_data, u8 r_len) {
  u8 index = 0;
  Serial.println("BEGIN READ");
  Serial.print("device addr: "); Serial.print(dev_addr, HEX); Serial.print("\t");
  Serial.print("register addr: "); Serial.print(reg_addr, HEX);  Serial.print("\t");
  Serial.print("request length: "); Serial.print(r_len); Serial.println("\t");
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  u8 code = Wire.endTransmission();
  Serial.print("write address code in read: "); Serial.println(code);
  s8 bytes_returned = Wire.requestFrom(dev_addr, r_len);
  Serial.print("read # bytes returned: "); Serial.println(bytes_returned);
  while (Wire.available()) {
    Serial.print("index: "); Serial.println(index);
    u8 data = Wire.read();
    Serial.print("data read in: "); Serial.println(data, HEX);
    reg_data[index] = data;
    index++;
  }
  Serial.println();
  return 0;
}

void delay_wrapper(u32 ms) {
  delay(ms);
}
