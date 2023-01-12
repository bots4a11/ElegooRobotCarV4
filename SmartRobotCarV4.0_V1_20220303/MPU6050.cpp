#include "MPU6050.h"
MPU6050::MPU6050() { devAddr = MPU6050_DEFAULT_ADDRESS; }
MPU6050::MPU6050(uint8_t address) { devAddr = address; }
void MPU6050::initialize() {
  setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  setSleepEnabled(false);
}
bool MPU6050::testConnection() { return getDeviceID() == 0x34; }
uint8_t MPU6050::getAuxVDDIOLevel() {
  I2Cdev::readBit(devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_PWR_MODE_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setAuxVDDIOLevel(uint8_t level) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_PWR_MODE_BIT,
                   level);
}
uint8_t MPU6050::getRate() {
  I2Cdev::readByte(devAddr, MPU6050_RA_SMPLRT_DIV, buffer);
  return buffer[0];
}
void MPU6050::setRate(uint8_t rate) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_SMPLRT_DIV, rate);
}
uint8_t MPU6050::getExternalFrameSync() {
  I2Cdev::readBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT,
                   MPU6050_CFG_EXT_SYNC_SET_LENGTH, buffer);
  return buffer[0];
}
void MPU6050::setExternalFrameSync(uint8_t sync) {
  I2Cdev::writeBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT,
                    MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync);
}
uint8_t MPU6050::getDLPFMode() {
  I2Cdev::readBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT,
                   MPU6050_CFG_DLPF_CFG_LENGTH, buffer);
  return buffer[0];
}
void MPU6050::setDLPFMode(uint8_t mode) {
  I2Cdev::writeBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT,
                    MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}
uint8_t MPU6050::getFullScaleGyroRange() {
  I2Cdev::readBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT,
                   MPU6050_GCONFIG_FS_SEL_LENGTH, buffer);
  return buffer[0];
}
void MPU6050::setFullScaleGyroRange(uint8_t range) {
  I2Cdev::writeBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT,
                    MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}
bool MPU6050::getAccelXSelfTest() {
  I2Cdev::readBit(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_XA_ST_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setAccelXSelfTest(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_XA_ST_BIT,
                   enabled);
}
bool MPU6050::getAccelYSelfTest() {
  I2Cdev::readBit(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_YA_ST_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setAccelYSelfTest(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_YA_ST_BIT,
                   enabled);
}
bool MPU6050::getAccelZSelfTest() {
  I2Cdev::readBit(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ZA_ST_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setAccelZSelfTest(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ZA_ST_BIT,
                   enabled);
}
uint8_t MPU6050::getFullScaleAccelRange() {
  I2Cdev::readBits(devAddr, MPU6050_RA_ACCEL_CONFIG,
                   MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH,
                   buffer);
  return buffer[0];
}
void MPU6050::setFullScaleAccelRange(uint8_t range) {
  I2Cdev::writeBits(devAddr, MPU6050_RA_ACCEL_CONFIG,
                    MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH,
                    range);
}
uint8_t MPU6050::getDHPFMode() {
  I2Cdev::readBits(devAddr, MPU6050_RA_ACCEL_CONFIG,
                   MPU6050_ACONFIG_ACCEL_HPF_BIT,
                   MPU6050_ACONFIG_ACCEL_HPF_LENGTH, buffer);
  return buffer[0];
}
void MPU6050::setDHPFMode(uint8_t bandwidth) {
  I2Cdev::writeBits(devAddr, MPU6050_RA_ACCEL_CONFIG,
                    MPU6050_ACONFIG_ACCEL_HPF_BIT,
                    MPU6050_ACONFIG_ACCEL_HPF_LENGTH, bandwidth);
}
uint8_t MPU6050::getFreefallDetectionThreshold() {
  I2Cdev::readByte(devAddr, MPU6050_RA_FF_THR, buffer);
  return buffer[0];
}
void MPU6050::setFreefallDetectionThreshold(uint8_t threshold) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_FF_THR, threshold);
}
uint8_t MPU6050::getFreefallDetectionDuration() {
  I2Cdev::readByte(devAddr, MPU6050_RA_FF_DUR, buffer);
  return buffer[0];
}
void MPU6050::setFreefallDetectionDuration(uint8_t duration) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_FF_DUR, duration);
}
uint8_t MPU6050::getMotionDetectionThreshold() {
  I2Cdev::readByte(devAddr, MPU6050_RA_MOT_THR, buffer);
  return buffer[0];
}
void MPU6050::setMotionDetectionThreshold(uint8_t threshold) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_MOT_THR, threshold);
}
uint8_t MPU6050::getMotionDetectionDuration() {
  I2Cdev::readByte(devAddr, MPU6050_RA_MOT_DUR, buffer);
  return buffer[0];
}
void MPU6050::setMotionDetectionDuration(uint8_t duration) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_MOT_DUR, duration);
}
uint8_t MPU6050::getZeroMotionDetectionThreshold() {
  I2Cdev::readByte(devAddr, MPU6050_RA_ZRMOT_THR, buffer);
  return buffer[0];
}
void MPU6050::setZeroMotionDetectionThreshold(uint8_t threshold) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_ZRMOT_THR, threshold);
}
uint8_t MPU6050::getZeroMotionDetectionDuration() {
  I2Cdev::readByte(devAddr, MPU6050_RA_ZRMOT_DUR, buffer);
  return buffer[0];
}
void MPU6050::setZeroMotionDetectionDuration(uint8_t duration) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_ZRMOT_DUR, duration);
}
bool MPU6050::getTempFIFOEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_FIFO_EN, MPU6050_TEMP_FIFO_EN_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setTempFIFOEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_FIFO_EN, MPU6050_TEMP_FIFO_EN_BIT,
                   enabled);
}
bool MPU6050::getXGyroFIFOEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_FIFO_EN, MPU6050_XG_FIFO_EN_BIT, buffer);
  return buffer[0];
}
void MPU6050::setXGyroFIFOEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_FIFO_EN, MPU6050_XG_FIFO_EN_BIT,
                   enabled);
}
bool MPU6050::getYGyroFIFOEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_FIFO_EN, MPU6050_YG_FIFO_EN_BIT, buffer);
  return buffer[0];
}
void MPU6050::setYGyroFIFOEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_FIFO_EN, MPU6050_YG_FIFO_EN_BIT,
                   enabled);
}
bool MPU6050::getZGyroFIFOEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_FIFO_EN, MPU6050_ZG_FIFO_EN_BIT, buffer);
  return buffer[0];
}
void MPU6050::setZGyroFIFOEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_FIFO_EN, MPU6050_ZG_FIFO_EN_BIT,
                   enabled);
}
bool MPU6050::getAccelFIFOEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setAccelFIFOEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT,
                   enabled);
}
bool MPU6050::getSlave2FIFOEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_FIFO_EN, MPU6050_SLV2_FIFO_EN_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setSlave2FIFOEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_FIFO_EN, MPU6050_SLV2_FIFO_EN_BIT,
                   enabled);
}
bool MPU6050::getSlave1FIFOEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_FIFO_EN, MPU6050_SLV1_FIFO_EN_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setSlave1FIFOEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_FIFO_EN, MPU6050_SLV1_FIFO_EN_BIT,
                   enabled);
}
bool MPU6050::getSlave0FIFOEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_FIFO_EN, MPU6050_SLV0_FIFO_EN_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setSlave0FIFOEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_FIFO_EN, MPU6050_SLV0_FIFO_EN_BIT,
                   enabled);
}
bool MPU6050::getMultiMasterEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_MULT_MST_EN_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setMultiMasterEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_MULT_MST_EN_BIT,
                   enabled);
}
bool MPU6050::getWaitForExternalSensorEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_WAIT_FOR_ES_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setWaitForExternalSensorEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_WAIT_FOR_ES_BIT,
                   enabled);
}
bool MPU6050::getSlave3FIFOEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_SLV_3_FIFO_EN_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setSlave3FIFOEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_SLV_3_FIFO_EN_BIT,
                   enabled);
}
bool MPU6050::getSlaveReadWriteTransitionEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_I2C_MST_P_NSR_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setSlaveReadWriteTransitionEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_I2C_MST_P_NSR_BIT,
                   enabled);
}
uint8_t MPU6050::getMasterClockSpeed() {
  I2Cdev::readBits(devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_I2C_MST_CLK_BIT,
                   MPU6050_I2C_MST_CLK_LENGTH, buffer);
  return buffer[0];
}
void MPU6050::setMasterClockSpeed(uint8_t speed) {
  I2Cdev::writeBits(devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_I2C_MST_CLK_BIT,
                    MPU6050_I2C_MST_CLK_LENGTH, speed);
}
uint8_t MPU6050::getSlaveAddress(uint8_t num) {
  if (num > 3)
    return 0;
  I2Cdev::readByte(devAddr, MPU6050_RA_I2C_SLV0_ADDR + num * 3, buffer);
  return buffer[0];
}
void MPU6050::setSlaveAddress(uint8_t num, uint8_t address) {
  if (num > 3)
    return;
  I2Cdev::writeByte(devAddr, MPU6050_RA_I2C_SLV0_ADDR + num * 3, address);
}
uint8_t MPU6050::getSlaveRegister(uint8_t num) {
  if (num > 3)
    return 0;
  I2Cdev::readByte(devAddr, MPU6050_RA_I2C_SLV0_REG + num * 3, buffer);
  return buffer[0];
}
void MPU6050::setSlaveRegister(uint8_t num, uint8_t reg) {
  if (num > 3)
    return;
  I2Cdev::writeByte(devAddr, MPU6050_RA_I2C_SLV0_REG + num * 3, reg);
}
bool MPU6050::getSlaveEnabled(uint8_t num) {
  if (num > 3)
    return 0;
  I2Cdev::readBit(devAddr, MPU6050_RA_I2C_SLV0_CTRL + num * 3,
                  MPU6050_I2C_SLV_EN_BIT, buffer);
  return buffer[0];
}
void MPU6050::setSlaveEnabled(uint8_t num, bool enabled) {
  if (num > 3)
    return;
  I2Cdev::writeBit(devAddr, MPU6050_RA_I2C_SLV0_CTRL + num * 3,
                   MPU6050_I2C_SLV_EN_BIT, enabled);
}
bool MPU6050::getSlaveWordByteSwap(uint8_t num) {
  if (num > 3)
    return 0;
  I2Cdev::readBit(devAddr, MPU6050_RA_I2C_SLV0_CTRL + num * 3,
                  MPU6050_I2C_SLV_BYTE_SW_BIT, buffer);
  return buffer[0];
}
void MPU6050::setSlaveWordByteSwap(uint8_t num, bool enabled) {
  if (num > 3)
    return;
  I2Cdev::writeBit(devAddr, MPU6050_RA_I2C_SLV0_CTRL + num * 3,
                   MPU6050_I2C_SLV_BYTE_SW_BIT, enabled);
}
bool MPU6050::getSlaveWriteMode(uint8_t num) {
  if (num > 3)
    return 0;
  I2Cdev::readBit(devAddr, MPU6050_RA_I2C_SLV0_CTRL + num * 3,
                  MPU6050_I2C_SLV_REG_DIS_BIT, buffer);
  return buffer[0];
}
void MPU6050::setSlaveWriteMode(uint8_t num, bool mode) {
  if (num > 3)
    return;
  I2Cdev::writeBit(devAddr, MPU6050_RA_I2C_SLV0_CTRL + num * 3,
                   MPU6050_I2C_SLV_REG_DIS_BIT, mode);
}
bool MPU6050::getSlaveWordGroupOffset(uint8_t num) {
  if (num > 3)
    return 0;
  I2Cdev::readBit(devAddr, MPU6050_RA_I2C_SLV0_CTRL + num * 3,
                  MPU6050_I2C_SLV_GRP_BIT, buffer);
  return buffer[0];
}
void MPU6050::setSlaveWordGroupOffset(uint8_t num, bool enabled) {
  if (num > 3)
    return;
  I2Cdev::writeBit(devAddr, MPU6050_RA_I2C_SLV0_CTRL + num * 3,
                   MPU6050_I2C_SLV_GRP_BIT, enabled);
}
uint8_t MPU6050::getSlaveDataLength(uint8_t num) {
  if (num > 3)
    return 0;
  I2Cdev::readBits(devAddr, MPU6050_RA_I2C_SLV0_CTRL + num * 3,
                   MPU6050_I2C_SLV_LEN_BIT, MPU6050_I2C_SLV_LEN_LENGTH, buffer);
  return buffer[0];
}
void MPU6050::setSlaveDataLength(uint8_t num, uint8_t length) {
  if (num > 3)
    return;
  I2Cdev::writeBits(devAddr, MPU6050_RA_I2C_SLV0_CTRL + num * 3,
                    MPU6050_I2C_SLV_LEN_BIT, MPU6050_I2C_SLV_LEN_LENGTH,
                    length);
}
uint8_t MPU6050::getSlave4Address() {
  I2Cdev::readByte(devAddr, MPU6050_RA_I2C_SLV4_ADDR, buffer);
  return buffer[0];
}
void MPU6050::setSlave4Address(uint8_t address) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_I2C_SLV4_ADDR, address);
}
uint8_t MPU6050::getSlave4Register() {
  I2Cdev::readByte(devAddr, MPU6050_RA_I2C_SLV4_REG, buffer);
  return buffer[0];
}
void MPU6050::setSlave4Register(uint8_t reg) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_I2C_SLV4_REG, reg);
}
void MPU6050::setSlave4OutputByte(uint8_t data) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_I2C_SLV4_DO, data);
}
bool MPU6050::getSlave4Enabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_EN_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setSlave4Enabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_EN_BIT,
                   enabled);
}
bool MPU6050::getSlave4InterruptEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_I2C_SLV4_CTRL,
                  MPU6050_I2C_SLV4_INT_EN_BIT, buffer);
  return buffer[0];
}
void MPU6050::setSlave4InterruptEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_I2C_SLV4_CTRL,
                   MPU6050_I2C_SLV4_INT_EN_BIT, enabled);
}
bool MPU6050::getSlave4WriteMode() {
  I2Cdev::readBit(devAddr, MPU6050_RA_I2C_SLV4_CTRL,
                  MPU6050_I2C_SLV4_REG_DIS_BIT, buffer);
  return buffer[0];
}
void MPU6050::setSlave4WriteMode(bool mode) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_I2C_SLV4_CTRL,
                   MPU6050_I2C_SLV4_REG_DIS_BIT, mode);
}
uint8_t MPU6050::getSlave4MasterDelay() {
  I2Cdev::readBits(devAddr, MPU6050_RA_I2C_SLV4_CTRL,
                   MPU6050_I2C_SLV4_MST_DLY_BIT,
                   MPU6050_I2C_SLV4_MST_DLY_LENGTH, buffer);
  return buffer[0];
}
void MPU6050::setSlave4MasterDelay(uint8_t delay) {
  I2Cdev::writeBits(devAddr, MPU6050_RA_I2C_SLV4_CTRL,
                    MPU6050_I2C_SLV4_MST_DLY_BIT,
                    MPU6050_I2C_SLV4_MST_DLY_LENGTH, delay);
}
uint8_t MPU6050::getSlate4InputByte() {
  I2Cdev::readByte(devAddr, MPU6050_RA_I2C_SLV4_DI, buffer);
  return buffer[0];
}
bool MPU6050::getPassthroughStatus() {
  I2Cdev::readBit(devAddr, MPU6050_RA_I2C_MST_STATUS,
                  MPU6050_MST_PASS_THROUGH_BIT, buffer);
  return buffer[0];
}
bool MPU6050::getSlave4IsDone() {
  I2Cdev::readBit(devAddr, MPU6050_RA_I2C_MST_STATUS,
                  MPU6050_MST_I2C_SLV4_DONE_BIT, buffer);
  return buffer[0];
}
bool MPU6050::getLostArbitration() {
  I2Cdev::readBit(devAddr, MPU6050_RA_I2C_MST_STATUS,
                  MPU6050_MST_I2C_LOST_ARB_BIT, buffer);
  return buffer[0];
}
bool MPU6050::getSlave4Nack() {
  I2Cdev::readBit(devAddr, MPU6050_RA_I2C_MST_STATUS,
                  MPU6050_MST_I2C_SLV4_NACK_BIT, buffer);
  return buffer[0];
}
bool MPU6050::getSlave3Nack() {
  I2Cdev::readBit(devAddr, MPU6050_RA_I2C_MST_STATUS,
                  MPU6050_MST_I2C_SLV3_NACK_BIT, buffer);
  return buffer[0];
}
bool MPU6050::getSlave2Nack() {
  I2Cdev::readBit(devAddr, MPU6050_RA_I2C_MST_STATUS,
                  MPU6050_MST_I2C_SLV2_NACK_BIT, buffer);
  return buffer[0];
}
bool MPU6050::getSlave1Nack() {
  I2Cdev::readBit(devAddr, MPU6050_RA_I2C_MST_STATUS,
                  MPU6050_MST_I2C_SLV1_NACK_BIT, buffer);
  return buffer[0];
}
bool MPU6050::getSlave0Nack() {
  I2Cdev::readBit(devAddr, MPU6050_RA_I2C_MST_STATUS,
                  MPU6050_MST_I2C_SLV0_NACK_BIT, buffer);
  return buffer[0];
}
bool MPU6050::getInterruptMode() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setInterruptMode(bool mode) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_INT_PIN_CFG,
                   MPU6050_INTCFG_INT_LEVEL_BIT, mode);
}
bool MPU6050::getInterruptDrive() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setInterruptDrive(bool drive) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT,
                   drive);
}
bool MPU6050::getInterruptLatch() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_PIN_CFG,
                  MPU6050_INTCFG_LATCH_INT_EN_BIT, buffer);
  return buffer[0];
}
void MPU6050::setInterruptLatch(bool latch) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_INT_PIN_CFG,
                   MPU6050_INTCFG_LATCH_INT_EN_BIT, latch);
}
bool MPU6050::getInterruptLatchClear() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_PIN_CFG,
                  MPU6050_INTCFG_INT_RD_CLEAR_BIT, buffer);
  return buffer[0];
}
void MPU6050::setInterruptLatchClear(bool clear) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_INT_PIN_CFG,
                   MPU6050_INTCFG_INT_RD_CLEAR_BIT, clear);
}
bool MPU6050::getFSyncInterruptLevel() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_PIN_CFG,
                  MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT, buffer);
  return buffer[0];
}
void MPU6050::setFSyncInterruptLevel(bool level) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_INT_PIN_CFG,
                   MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT, level);
}
bool MPU6050::getFSyncInterruptEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_PIN_CFG,
                  MPU6050_INTCFG_FSYNC_INT_EN_BIT, buffer);
  return buffer[0];
}
void MPU6050::setFSyncInterruptEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_INT_PIN_CFG,
                   MPU6050_INTCFG_FSYNC_INT_EN_BIT, enabled);
}
bool MPU6050::getI2CBypassEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_PIN_CFG,
                  MPU6050_INTCFG_I2C_BYPASS_EN_BIT, buffer);
  return buffer[0];
}
void MPU6050::setI2CBypassEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_INT_PIN_CFG,
                   MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}
bool MPU6050::getClockOutputEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_CLKOUT_EN_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setClockOutputEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_INT_PIN_CFG,
                   MPU6050_INTCFG_CLKOUT_EN_BIT, enabled);
}
uint8_t MPU6050::getIntEnabled() {
  I2Cdev::readByte(devAddr, MPU6050_RA_INT_ENABLE, buffer);
  return buffer[0];
}
void MPU6050::setIntEnabled(uint8_t enabled) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_INT_ENABLE, enabled);
}
bool MPU6050::getIntFreefallEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FF_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setIntFreefallEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FF_BIT,
                   enabled);
}
bool MPU6050::getIntMotionEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_MOT_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setIntMotionEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_MOT_BIT,
                   enabled);
}
bool MPU6050::getIntZeroMotionEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_ZMOT_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setIntZeroMotionEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_ZMOT_BIT,
                   enabled);
}
bool MPU6050::getIntFIFOBufferOverflowEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_ENABLE,
                  MPU6050_INTERRUPT_FIFO_OFLOW_BIT, buffer);
  return buffer[0];
}
void MPU6050::setIntFIFOBufferOverflowEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_INT_ENABLE,
                   MPU6050_INTERRUPT_FIFO_OFLOW_BIT, enabled);
}
bool MPU6050::getIntI2CMasterEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_ENABLE,
                  MPU6050_INTERRUPT_I2C_MST_INT_BIT, buffer);
  return buffer[0];
}
void MPU6050::setIntI2CMasterEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_INT_ENABLE,
                   MPU6050_INTERRUPT_I2C_MST_INT_BIT, enabled);
}
bool MPU6050::getIntDataReadyEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_ENABLE,
                  MPU6050_INTERRUPT_DATA_RDY_BIT, buffer);
  return buffer[0];
}
void MPU6050::setIntDataReadyEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_INT_ENABLE,
                   MPU6050_INTERRUPT_DATA_RDY_BIT, enabled);
}
uint8_t MPU6050::getIntStatus() {
  I2Cdev::readByte(devAddr, MPU6050_RA_INT_STATUS, buffer);
  return buffer[0];
}
bool MPU6050::getIntFreefallStatus() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_FF_BIT,
                  buffer);
  return buffer[0];
}
bool MPU6050::getIntMotionStatus() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_MOT_BIT,
                  buffer);
  return buffer[0];
}
bool MPU6050::getIntZeroMotionStatus() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_ZMOT_BIT,
                  buffer);
  return buffer[0];
}
bool MPU6050::getIntFIFOBufferOverflowStatus() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_STATUS,
                  MPU6050_INTERRUPT_FIFO_OFLOW_BIT, buffer);
  return buffer[0];
}
bool MPU6050::getIntI2CMasterStatus() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_STATUS,
                  MPU6050_INTERRUPT_I2C_MST_INT_BIT, buffer);
  return buffer[0];
}
bool MPU6050::getIntDataReadyStatus() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_STATUS,
                  MPU6050_INTERRUPT_DATA_RDY_BIT, buffer);
  return buffer[0];
}
void MPU6050::getMotion9(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx,
                         int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my,
                         int16_t *mz) {
  getMotion6(ax, ay, az, gx, gy, gz);
}
void MPU6050::getMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx,
                         int16_t *gy, int16_t *gz) {
  I2Cdev::readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
  *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
  *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
  *az = (((int16_t)buffer[4]) << 8) | buffer[5];
  *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
  *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
  *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}
void MPU6050::getAcceleration(int16_t *x, int16_t *y, int16_t *z) {
  I2Cdev::readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 6, buffer);
  *x = (((int16_t)buffer[0]) << 8) | buffer[1];
  *y = (((int16_t)buffer[2]) << 8) | buffer[3];
  *z = (((int16_t)buffer[4]) << 8) | buffer[5];
}
int16_t MPU6050::getAccelerationX() {
  I2Cdev::readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}
int16_t MPU6050::getAccelerationY() {
  I2Cdev::readBytes(devAddr, MPU6050_RA_ACCEL_YOUT_H, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}
int16_t MPU6050::getAccelerationZ() {
  I2Cdev::readBytes(devAddr, MPU6050_RA_ACCEL_ZOUT_H, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}
int16_t MPU6050::getTemperature() {
  I2Cdev::readBytes(devAddr, MPU6050_RA_TEMP_OUT_H, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU6050::getRotation(int16_t *x, int16_t *y, int16_t *z) {
  I2Cdev::readBytes(devAddr, MPU6050_RA_GYRO_XOUT_H, 6, buffer);
  *x = (((int16_t)buffer[0]) << 8) | buffer[1];
  *y = (((int16_t)buffer[2]) << 8) | buffer[3];
  *z = (((int16_t)buffer[4]) << 8) | buffer[5];
}
int16_t MPU6050::getRotationX() {
  I2Cdev::readBytes(devAddr, MPU6050_RA_GYRO_XOUT_H, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}
int16_t MPU6050::getRotationY() {
  I2Cdev::readBytes(devAddr, MPU6050_RA_GYRO_YOUT_H, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}
int16_t MPU6050::getRotationZ() {
  I2Cdev::readBytes(devAddr, MPU6050_RA_GYRO_ZOUT_H, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}
uint8_t MPU6050::getExternalSensorByte(int position) {
  I2Cdev::readByte(devAddr, MPU6050_RA_EXT_SENS_DATA_00 + position, buffer);
  return buffer[0];
}
uint16_t MPU6050::getExternalSensorWord(int position) {
  I2Cdev::readBytes(devAddr, MPU6050_RA_EXT_SENS_DATA_00 + position, 2, buffer);
  return (((uint16_t)buffer[0]) << 8) | buffer[1];
}
uint32_t MPU6050::getExternalSensorDWord(int position) {
  I2Cdev::readBytes(devAddr, MPU6050_RA_EXT_SENS_DATA_00 + position, 4, buffer);
  return (((uint32_t)buffer[0]) << 24) | (((uint32_t)buffer[1]) << 16) |
         (((uint16_t)buffer[2]) << 8) | buffer[3];
}
bool MPU6050::getXNegMotionDetected() {
  I2Cdev::readBit(devAddr, MPU6050_RA_MOT_DETECT_STATUS,
                  MPU6050_MOTION_MOT_XNEG_BIT, buffer);
  return buffer[0];
}
bool MPU6050::getXPosMotionDetected() {
  I2Cdev::readBit(devAddr, MPU6050_RA_MOT_DETECT_STATUS,
                  MPU6050_MOTION_MOT_XPOS_BIT, buffer);
  return buffer[0];
}
bool MPU6050::getYNegMotionDetected() {
  I2Cdev::readBit(devAddr, MPU6050_RA_MOT_DETECT_STATUS,
                  MPU6050_MOTION_MOT_YNEG_BIT, buffer);
  return buffer[0];
}
bool MPU6050::getYPosMotionDetected() {
  I2Cdev::readBit(devAddr, MPU6050_RA_MOT_DETECT_STATUS,
                  MPU6050_MOTION_MOT_YPOS_BIT, buffer);
  return buffer[0];
}
bool MPU6050::getZNegMotionDetected() {
  I2Cdev::readBit(devAddr, MPU6050_RA_MOT_DETECT_STATUS,
                  MPU6050_MOTION_MOT_ZNEG_BIT, buffer);
  return buffer[0];
}
bool MPU6050::getZPosMotionDetected() {
  I2Cdev::readBit(devAddr, MPU6050_RA_MOT_DETECT_STATUS,
                  MPU6050_MOTION_MOT_ZPOS_BIT, buffer);
  return buffer[0];
}
bool MPU6050::getZeroMotionDetected() {
  I2Cdev::readBit(devAddr, MPU6050_RA_MOT_DETECT_STATUS,
                  MPU6050_MOTION_MOT_ZRMOT_BIT, buffer);
  return buffer[0];
}
void MPU6050::setSlaveOutputByte(uint8_t num, uint8_t data) {
  if (num > 3)
    return;
  I2Cdev::writeByte(devAddr, MPU6050_RA_I2C_SLV0_DO + num, data);
}
bool MPU6050::getExternalShadowDelayEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_I2C_MST_DELAY_CTRL,
                  MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT, buffer);
  return buffer[0];
}
void MPU6050::setExternalShadowDelayEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_I2C_MST_DELAY_CTRL,
                   MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT, enabled);
}
bool MPU6050::getSlaveDelayEnabled(uint8_t num) {
  if (num > 4)
    return 0;
  I2Cdev::readBit(devAddr, MPU6050_RA_I2C_MST_DELAY_CTRL, num, buffer);
  return buffer[0];
}
void MPU6050::setSlaveDelayEnabled(uint8_t num, bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_I2C_MST_DELAY_CTRL, num, enabled);
}
void MPU6050::resetGyroscopePath() {
  I2Cdev::writeBit(devAddr, MPU6050_RA_SIGNAL_PATH_RESET,
                   MPU6050_PATHRESET_GYRO_RESET_BIT, true);
}
void MPU6050::resetAccelerometerPath() {
  I2Cdev::writeBit(devAddr, MPU6050_RA_SIGNAL_PATH_RESET,
                   MPU6050_PATHRESET_ACCEL_RESET_BIT, true);
}
void MPU6050::resetTemperaturePath() {
  I2Cdev::writeBit(devAddr, MPU6050_RA_SIGNAL_PATH_RESET,
                   MPU6050_PATHRESET_TEMP_RESET_BIT, true);
}
uint8_t MPU6050::getAccelerometerPowerOnDelay() {
  I2Cdev::readBits(devAddr, MPU6050_RA_MOT_DETECT_CTRL,
                   MPU6050_DETECT_ACCEL_ON_DELAY_BIT,
                   MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH, buffer);
  return buffer[0];
}
void MPU6050::setAccelerometerPowerOnDelay(uint8_t delay) {
  I2Cdev::writeBits(devAddr, MPU6050_RA_MOT_DETECT_CTRL,
                    MPU6050_DETECT_ACCEL_ON_DELAY_BIT,
                    MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH, delay);
}
uint8_t MPU6050::getFreefallDetectionCounterDecrement() {
  I2Cdev::readBits(devAddr, MPU6050_RA_MOT_DETECT_CTRL,
                   MPU6050_DETECT_FF_COUNT_BIT, MPU6050_DETECT_FF_COUNT_LENGTH,
                   buffer);
  return buffer[0];
}
void MPU6050::setFreefallDetectionCounterDecrement(uint8_t decrement) {
  I2Cdev::writeBits(devAddr, MPU6050_RA_MOT_DETECT_CTRL,
                    MPU6050_DETECT_FF_COUNT_BIT, MPU6050_DETECT_FF_COUNT_LENGTH,
                    decrement);
}
uint8_t MPU6050::getMotionDetectionCounterDecrement() {
  I2Cdev::readBits(devAddr, MPU6050_RA_MOT_DETECT_CTRL,
                   MPU6050_DETECT_MOT_COUNT_BIT,
                   MPU6050_DETECT_MOT_COUNT_LENGTH, buffer);
  return buffer[0];
}
void MPU6050::setMotionDetectionCounterDecrement(uint8_t decrement) {
  I2Cdev::writeBits(devAddr, MPU6050_RA_MOT_DETECT_CTRL,
                    MPU6050_DETECT_MOT_COUNT_BIT,
                    MPU6050_DETECT_MOT_COUNT_LENGTH, decrement);
}
bool MPU6050::getFIFOEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setFIFOEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT,
                   enabled);
}
bool MPU6050::getI2CMasterModeEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_USER_CTRL,
                  MPU6050_USERCTRL_I2C_MST_EN_BIT, buffer);
  return buffer[0];
}
void MPU6050::setI2CMasterModeEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL,
                   MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}
void MPU6050::switchSPIEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL,
                   MPU6050_USERCTRL_I2C_IF_DIS_BIT, enabled);
}
void MPU6050::resetFIFO() {
  I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL,
                   MPU6050_USERCTRL_FIFO_RESET_BIT, true);
}
void MPU6050::resetI2CMaster() {
  I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL,
                   MPU6050_USERCTRL_I2C_MST_RESET_BIT, true);
}
void MPU6050::resetSensors() {
  I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL,
                   MPU6050_USERCTRL_SIG_COND_RESET_BIT, true);
}
void MPU6050::reset() {
  I2Cdev::writeBit(devAddr, MPU6050_RA_PWR_MGMT_1,
                   MPU6050_PWR1_DEVICE_RESET_BIT, true);
}
bool MPU6050::getSleepEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setSleepEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT,
                   enabled);
}
bool MPU6050::getWakeCycleEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setWakeCycleEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT,
                   enabled);
}
bool MPU6050::getTempSensorEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT,
                  buffer);
  return buffer[0] == 0;
}
void MPU6050::setTempSensorEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT,
                   !enabled);
}
uint8_t MPU6050::getClockSource() {
  I2Cdev::readBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT,
                   MPU6050_PWR1_CLKSEL_LENGTH, buffer);
  return buffer[0];
}
void MPU6050::setClockSource(uint8_t source) {
  I2Cdev::writeBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT,
                    MPU6050_PWR1_CLKSEL_LENGTH, source);
}
uint8_t MPU6050::getWakeFrequency() {
  I2Cdev::readBits(devAddr, MPU6050_RA_PWR_MGMT_2,
                   MPU6050_PWR2_LP_WAKE_CTRL_BIT,
                   MPU6050_PWR2_LP_WAKE_CTRL_LENGTH, buffer);
  return buffer[0];
}
void MPU6050::setWakeFrequency(uint8_t frequency) {
  I2Cdev::writeBits(devAddr, MPU6050_RA_PWR_MGMT_2,
                    MPU6050_PWR2_LP_WAKE_CTRL_BIT,
                    MPU6050_PWR2_LP_WAKE_CTRL_LENGTH, frequency);
}
bool MPU6050::getStandbyXAccelEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XA_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setStandbyXAccelEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XA_BIT,
                   enabled);
}
bool MPU6050::getStandbyYAccelEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YA_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setStandbyYAccelEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YA_BIT,
                   enabled);
}
bool MPU6050::getStandbyZAccelEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZA_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setStandbyZAccelEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZA_BIT,
                   enabled);
}
bool MPU6050::getStandbyXGyroEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XG_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setStandbyXGyroEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XG_BIT,
                   enabled);
}
bool MPU6050::getStandbyYGyroEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YG_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setStandbyYGyroEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YG_BIT,
                   enabled);
}
bool MPU6050::getStandbyZGyroEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZG_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setStandbyZGyroEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZG_BIT,
                   enabled);
}
uint16_t MPU6050::getFIFOCount() {
  I2Cdev::readBytes(devAddr, MPU6050_RA_FIFO_COUNTH, 2, buffer);
  return (((uint16_t)buffer[0]) << 8) | buffer[1];
}
uint8_t MPU6050::getFIFOByte() {
  I2Cdev::readByte(devAddr, MPU6050_RA_FIFO_R_W, buffer);
  return buffer[0];
}
void MPU6050::getFIFOBytes(uint8_t *data, uint8_t length) {
  I2Cdev::readBytes(devAddr, MPU6050_RA_FIFO_R_W, length, data);
}
void MPU6050::setFIFOByte(uint8_t data) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_FIFO_R_W, data);
}
uint8_t MPU6050::getDeviceID() {
  I2Cdev::readBits(devAddr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT,
                   MPU6050_WHO_AM_I_LENGTH, buffer);
  return buffer[0];
}
void MPU6050::setDeviceID(uint8_t id) {
  I2Cdev::writeBits(devAddr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT,
                    MPU6050_WHO_AM_I_LENGTH, id);
}
uint8_t MPU6050::getOTPBankValid() {
  I2Cdev::readBit(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setOTPBankValid(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT,
                   enabled);
}
int8_t MPU6050::getXGyroOffsetTC() {
  I2Cdev::readBits(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
                   MPU6050_TC_OFFSET_LENGTH, buffer);
  return buffer[0];
}
void MPU6050::setXGyroOffsetTC(int8_t offset) {
  I2Cdev::writeBits(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
                    MPU6050_TC_OFFSET_LENGTH, offset);
}
int8_t MPU6050::getYGyroOffsetTC() {
  I2Cdev::readBits(devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
                   MPU6050_TC_OFFSET_LENGTH, buffer);
  return buffer[0];
}
void MPU6050::setYGyroOffsetTC(int8_t offset) {
  I2Cdev::writeBits(devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
                    MPU6050_TC_OFFSET_LENGTH, offset);
}
int8_t MPU6050::getZGyroOffsetTC() {
  I2Cdev::readBits(devAddr, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
                   MPU6050_TC_OFFSET_LENGTH, buffer);
  return buffer[0];
}
void MPU6050::setZGyroOffsetTC(int8_t offset) {
  I2Cdev::writeBits(devAddr, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
                    MPU6050_TC_OFFSET_LENGTH, offset);
}
int8_t MPU6050::getXFineGain() {
  I2Cdev::readByte(devAddr, MPU6050_RA_X_FINE_GAIN, buffer);
  return buffer[0];
}
void MPU6050::setXFineGain(int8_t gain) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_X_FINE_GAIN, gain);
}
int8_t MPU6050::getYFineGain() {
  I2Cdev::readByte(devAddr, MPU6050_RA_Y_FINE_GAIN, buffer);
  return buffer[0];
}
void MPU6050::setYFineGain(int8_t gain) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_Y_FINE_GAIN, gain);
}
int8_t MPU6050::getZFineGain() {
  I2Cdev::readByte(devAddr, MPU6050_RA_Z_FINE_GAIN, buffer);
  return buffer[0];
}
void MPU6050::setZFineGain(int8_t gain) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_Z_FINE_GAIN, gain);
}
int16_t MPU6050::getXAccelOffset() {
  I2Cdev::readBytes(devAddr, MPU6050_RA_XA_OFFS_H, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU6050::setXAccelOffset(int16_t offset) {
  I2Cdev::writeWord(devAddr, MPU6050_RA_XA_OFFS_H, offset);
}
int16_t MPU6050::getYAccelOffset() {
  I2Cdev::readBytes(devAddr, MPU6050_RA_YA_OFFS_H, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU6050::setYAccelOffset(int16_t offset) {
  I2Cdev::writeWord(devAddr, MPU6050_RA_YA_OFFS_H, offset);
}
int16_t MPU6050::getZAccelOffset() {
  I2Cdev::readBytes(devAddr, MPU6050_RA_ZA_OFFS_H, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU6050::setZAccelOffset(int16_t offset) {
  I2Cdev::writeWord(devAddr, MPU6050_RA_ZA_OFFS_H, offset);
}
int16_t MPU6050::getXGyroOffset() {
  I2Cdev::readBytes(devAddr, MPU6050_RA_XG_OFFS_USRH, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU6050::setXGyroOffset(int16_t offset) {
  I2Cdev::writeWord(devAddr, MPU6050_RA_XG_OFFS_USRH, offset);
}
int16_t MPU6050::getYGyroOffset() {
  I2Cdev::readBytes(devAddr, MPU6050_RA_YG_OFFS_USRH, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU6050::setYGyroOffset(int16_t offset) {
  I2Cdev::writeWord(devAddr, MPU6050_RA_YG_OFFS_USRH, offset);
}
int16_t MPU6050::getZGyroOffset() {
  I2Cdev::readBytes(devAddr, MPU6050_RA_ZG_OFFS_USRH, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU6050::setZGyroOffset(int16_t offset) {
  I2Cdev::writeWord(devAddr, MPU6050_RA_ZG_OFFS_USRH, offset);
}
bool MPU6050::getIntPLLReadyEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_ENABLE,
                  MPU6050_INTERRUPT_PLL_RDY_INT_BIT, buffer);
  return buffer[0];
}
void MPU6050::setIntPLLReadyEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_INT_ENABLE,
                   MPU6050_INTERRUPT_PLL_RDY_INT_BIT, enabled);
}
bool MPU6050::getIntDMPEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DMP_INT_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setIntDMPEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_INT_ENABLE,
                   MPU6050_INTERRUPT_DMP_INT_BIT, enabled);
}
bool MPU6050::getDMPInt5Status() {
  I2Cdev::readBit(devAddr, MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_5_BIT,
                  buffer);
  return buffer[0];
}
bool MPU6050::getDMPInt4Status() {
  I2Cdev::readBit(devAddr, MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_4_BIT,
                  buffer);
  return buffer[0];
}
bool MPU6050::getDMPInt3Status() {
  I2Cdev::readBit(devAddr, MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_3_BIT,
                  buffer);
  return buffer[0];
}
bool MPU6050::getDMPInt2Status() {
  I2Cdev::readBit(devAddr, MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_2_BIT,
                  buffer);
  return buffer[0];
}
bool MPU6050::getDMPInt1Status() {
  I2Cdev::readBit(devAddr, MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_1_BIT,
                  buffer);
  return buffer[0];
}
bool MPU6050::getDMPInt0Status() {
  I2Cdev::readBit(devAddr, MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_0_BIT,
                  buffer);
  return buffer[0];
}
bool MPU6050::getIntPLLReadyStatus() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_STATUS,
                  MPU6050_INTERRUPT_PLL_RDY_INT_BIT, buffer);
  return buffer[0];
}
bool MPU6050::getIntDMPStatus() {
  I2Cdev::readBit(devAddr, MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_DMP_INT_BIT,
                  buffer);
  return buffer[0];
}
bool MPU6050::getDMPEnabled() {
  I2Cdev::readBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT,
                  buffer);
  return buffer[0];
}
void MPU6050::setDMPEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT,
                   enabled);
}
void MPU6050::resetDMP() {
  I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL,
                   MPU6050_USERCTRL_DMP_RESET_BIT, true);
}
void MPU6050::setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank) {
  bank &= 0x1F;
  if (userBank)
    bank |= 0x20;
  if (prefetchEnabled)
    bank |= 0x40;
  I2Cdev::writeByte(devAddr, MPU6050_RA_BANK_SEL, bank);
}
void MPU6050::setMemoryStartAddress(uint8_t address) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_MEM_START_ADDR, address);
}
uint8_t MPU6050::readMemoryByte() {
  I2Cdev::readByte(devAddr, MPU6050_RA_MEM_R_W, buffer);
  return buffer[0];
}
void MPU6050::writeMemoryByte(uint8_t data) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_MEM_R_W, data);
}
void MPU6050::readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank,
                              uint8_t address) {
  setMemoryBank(bank);
  setMemoryStartAddress(address);
  uint8_t chunkSize;
  for (uint16_t i = 0; i < dataSize;) {
    chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;
    if (i + chunkSize > dataSize)
      chunkSize = dataSize - i;
    if (chunkSize > 256 - address)
      chunkSize = 256 - address;
    I2Cdev::readBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, data + i);
    i += chunkSize;
    address += chunkSize;
    if (i < dataSize) {
      if (address == 0)
        bank++;
      setMemoryBank(bank);
      setMemoryStartAddress(address);
    }
  }
}
bool MPU6050::writeMemoryBlock(const uint8_t *data, uint16_t dataSize,
                               uint8_t bank, uint8_t address, bool verify,
                               bool useProgMem) {
  setMemoryBank(bank);
  setMemoryStartAddress(address);
  uint8_t chunkSize;
  uint8_t *verifyBuffer;
  uint8_t *progBuffer;
  uint16_t i;
  uint8_t j;
  if (verify)
    verifyBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
  if (useProgMem)
    progBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
  for (i = 0; i < dataSize;) {
    chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;
    if (i + chunkSize > dataSize)
      chunkSize = dataSize - i;
    if (chunkSize > 256 - address)
      chunkSize = 256 - address;
    if (useProgMem) {
      for (j = 0; j < chunkSize; j++)
        progBuffer[j] = pgm_read_byte(data + i + j);
    } else {
      progBuffer = (uint8_t *)data + i;
    }
    I2Cdev::writeBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, progBuffer);
    if (verify && verifyBuffer) {
      setMemoryBank(bank);
      setMemoryStartAddress(address);
      I2Cdev::readBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, verifyBuffer);
      if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {
        free(verifyBuffer);
        if (useProgMem)
          free(progBuffer);
        return false;
      }
    }
    i += chunkSize;
    address += chunkSize;
    if (i < dataSize) {
      if (address == 0)
        bank++;
      setMemoryBank(bank);
      setMemoryStartAddress(address);
    }
  }
  if (verify)
    free(verifyBuffer);
  if (useProgMem)
    free(progBuffer);
  return true;
}
bool MPU6050::writeProgMemoryBlock(const uint8_t *data, uint16_t dataSize,
                                   uint8_t bank, uint8_t address, bool verify) {
  return writeMemoryBlock(data, dataSize, bank, address, verify, true);
}
bool MPU6050::writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize,
                                       bool useProgMem) {
  uint8_t *progBuffer, success, special;
  uint16_t i, j;
  if (useProgMem) {
    progBuffer = (uint8_t *)malloc(8);
  }
  uint8_t bank, offset, length;
  for (i = 0; i < dataSize;) {
    if (useProgMem) {
      bank = pgm_read_byte(data + i++);
      offset = pgm_read_byte(data + i++);
      length = pgm_read_byte(data + i++);
    } else {
      bank = data[i++];
      offset = data[i++];
      length = data[i++];
    }
    if (length > 0) {
      if (useProgMem) {
        if (sizeof(progBuffer) < length)
          progBuffer = (uint8_t *)realloc(progBuffer, length);
        for (j = 0; j < length; j++)
          progBuffer[j] = pgm_read_byte(data + i + j);
      } else {
        progBuffer = (uint8_t *)data + i;
      }
      success = writeMemoryBlock(progBuffer, length, bank, offset, true);
      i += length;
    } else {
      if (useProgMem) {
        special = pgm_read_byte(data + i++);
      } else {
        special = data[i++];
      }
      if (special == 0x01) {
        I2Cdev::writeByte(devAddr, MPU6050_RA_INT_ENABLE, 0x32);
        success = true;
      } else {
        success = false;
      }
    }
    if (!success) {
      if (useProgMem)
        free(progBuffer);
      return false;
    }
  }
  if (useProgMem)
    free(progBuffer);
  return true;
}
bool MPU6050::writeProgDMPConfigurationSet(const uint8_t *data,
                                           uint16_t dataSize) {
  return writeDMPConfigurationSet(data, dataSize, true);
}
uint8_t MPU6050::getDMPConfig1() {
  I2Cdev::readByte(devAddr, MPU6050_RA_DMP_CFG_1, buffer);
  return buffer[0];
}
void MPU6050::setDMPConfig1(uint8_t config) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_DMP_CFG_1, config);
}
uint8_t MPU6050::getDMPConfig2() {
  I2Cdev::readByte(devAddr, MPU6050_RA_DMP_CFG_2, buffer);
  return buffer[0];
}
void MPU6050::setDMPConfig2(uint8_t config) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_DMP_CFG_2, config);
}
