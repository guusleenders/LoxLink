//
//  LoxBusNano2Relay.hpp
//
//  Created by Guus Leenders on 04.11.22.
//

#include "LoxBusNano2Relay.hpp"
#include <__cross_studio_io.h>

/***
 *  Constructor
 ***/
LoxBusNano2Relay::LoxBusNano2Relay(LoxCANBaseDriver &driver, uint32_t serial, eAliveReason_t alive)
  : LoxBusTreeDevice(driver, serial, eDeviceType_t_Nano2RelayTree, 0, 12010401, 1, sizeof(config), &config, alive) {
}

void LoxBusNano2Relay::ConfigUpdate(void) {
  debug_printf("unknown = %d\n", config.unknown0);
}

void LoxBusNano2Relay::ConfigLoadDefaults(void) {
}

void LoxBusNano2Relay::ReceiveDirect(LoxCanMessage &message) {
  switch (message.commandNat) {
    case Digital_Value: {
      for(uint8_t i = 0; i < NANO2_RELAY_NUMBER_OF_CHANNELS; i++){
        if(message.value32 & 1<<i){
          UpdateRelayChannel(i, true);
        }else{
          UpdateRelayChannel(i, false);
        }
      }
    }  break;
    default:
      LoxBusTreeDevice::ReceiveDirect(message);
      break;
  }
}

void LoxBusNano2Relay::ReceiveDirectFragment(LoxMsgNATCommand_t command, uint8_t extensionNAT, uint8_t deviceNAT, const uint8_t *data, uint16_t size) {
    if(command != Config_Data) {
        debug_printf("frag %02x\n", command);
    }
    LoxBusTreeDevice::ReceiveDirectFragment(command, extensionNAT, deviceNAT, data, size);
}

void LoxBusNano2Relay::UpdateRelayChannel(uint8_t channel, bool value){
  // Do stuff here
}