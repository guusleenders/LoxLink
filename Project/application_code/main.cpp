#include "system.hpp"

#include "LED.hpp"
#include "Watchdog.hpp"

#include "LoxCANDriver_STM32.hpp"
#include "LoxBusTreeExtension.hpp"
//#include "LoxBusDIExtension.hpp"
//#include "LoxBusTreeAlarmSiren.hpp"
//#include "LoxBusTreeRoomComfortSensor.hpp"
//#include "LoxBusTreeTouch.hpp"
//#include "LoxLegacyDMXExtension.hpp"
//#include "LoxLegacyRS232Extension.hpp"
//#include "LoxLegacyModbusExtension.hpp"
//#include "LoxLegacyRelayExtension.hpp"
#include "LoxBusTreeRgbwDimmer.hpp"
#include "LoxBusNano2Relay.hpp"

I2C_HandleTypeDef hi2c2;


int main(void) {
  system_init();
  simpleInitI2C(&hi2c2);

  static CTL_TASK_t main_task;
  ctl_task_init(&main_task, 255, "main"); // create subsequent tasks whilst running at the highest priority.

  // Warning: be aware that two relay extension need two different serial numbers!
  uint32_t serial_base = serialnumber_24bit();

  static LoxCANDriver_STM32 gLoxCANDriver(tLoxCANDriverType_LoxoneLink);
  //static LoxCANDriver_STM32 gLoxCANDriver(tLoxCANDriverType_TreeBus);
#if EXTENSION_RS232
//  static LoxLegacyRS232Extension gLoxLegacyRS232Extension(gLoxCANDriver, serial_base);
#endif
#if EXTENSION_MODBUS
//  static LoxLegacyModbusExtension gLoxLegacyModbusExtension(gLoxCANDriver, serial_base);
#endif
  //static LoxBusDIExtension gDIExtension(gLoxCANDriver, serial_base, gResetReason);
  //static LoxLegacyRelayExtension gRelayExtension(gLoxCANDriver, serial_base);
  //static LoxLegacyDMXExtension gDMXExtension(gLoxCANDriver, serial_base);
  static LoxBusTreeExtension gTreeExtension(gLoxCANDriver, serial_base, gResetReason);
  //static LoxBusTreeRoomComfortSensor gTreeRoomComfortSensor(gTreeExtension.Driver(eTreeBranch_rightBranch), 0xb0112233, gResetReason);
  //static LoxBusTreeRoomComfortSensor gTreeRoomComfortSensor(gLoxCANDriver, 0xb011223e, gResetReason);
  //gTreeExtension.AddDevice(&gTreeRoomComfortSensor, eTreeBranch_rightBranch);
  //static LoxBusTreeTouch gLoxBusTreeTouch(gTreeExtension.Driver(eTreeBranch_rightBranch), 0xb010035b, gResetReason);
  //gTreeExtension.AddDevice(&gLoxBusTreeTouch, eTreeBranch_rightBranch);
  //  static LoxBusTreeAlarmSiren gLoxBusTreeAlarmSiren(gTreeExtension.Driver(eTreeBranch_leftBranch), 0xb010035c, gResetReason);
  //  static LoxBusTreeAlarmSiren gLoxBusTreeAlarmSiren(gLoxCANDriver, 0xb010035c, gResetReason);
  //  gTreeExtension.AddDevice(&gLoxBusTreeAlarmSiren, eTreeBranch_leftBranch);
  static LoxBusTreeRgbwDimmer gLoxBusTreeRgbwDimmer0(gTreeExtension.Driver(eTreeBranch_rightBranch), 0xb0200001, gResetReason, &hi2c2, 0);
  gTreeExtension.AddDevice(&gLoxBusTreeRgbwDimmer0, eTreeBranch_rightBranch);

  static LoxBusTreeRgbwDimmer gLoxBusTreeRgbwDimmer1(gTreeExtension.Driver(eTreeBranch_rightBranch), 0xb0200002, gResetReason, &hi2c2, 1);
  gTreeExtension.AddDevice(&gLoxBusTreeRgbwDimmer1, eTreeBranch_rightBranch);

  static LoxBusTreeRgbwDimmer gLoxBusTreeRgbwDimmer2(gTreeExtension.Driver(eTreeBranch_rightBranch), 0xb0200003, gResetReason, &hi2c2, 2);
  gTreeExtension.AddDevice(&gLoxBusTreeRgbwDimmer2, eTreeBranch_rightBranch);

  static LoxBusNano2Relay gLoxBusNano2Relay(gTreeExtension.Driver(eTreeBranch_rightBranch), 0xb010035b, gResetReason);
  gTreeExtension.AddDevice(&gLoxBusNano2Relay, eTreeBranch_rightBranch);

#if DEBUG
  MX_print_cpu_info();
#endif
  gLED.Startup();
  gLoxCANDriver.Startup();
  gLoxBusTreeRgbwDimmer0.Startup();
  gLoxBusTreeRgbwDimmer1.Startup();
  gLoxBusTreeRgbwDimmer2.Startup();

  Start_Watchdog();
  ctl_task_set_priority(&main_task, 0); // drop to lowest priority to start created tasks running.

  while (1) {
  }
  return 0;
}


int debug_printf(const char *format, ...){
  return 0;
}
void debug_break(void){
  return;
}

extern "C" void I2C2_EV_IRQHandler(){
    HAL_I2C_EV_IRQHandler(&hi2c2);
}
extern "C" void I2C2_ER_IRQHandler(){
    HAL_I2C_ER_IRQHandler(&hi2c2);
}