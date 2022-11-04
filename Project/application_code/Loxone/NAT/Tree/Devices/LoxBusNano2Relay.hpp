//
//  LoxBusNano2Relay.hpp
//
//  Created by Guus Leenders on 04.11.22.
//

#ifndef LoxBusNano2Relay_hpp
#define LoxBusNano2Relay_hpp

#include "LoxBusTreeDevice.hpp"

#define NANO2_RELAY_NUMBER_OF_CHANNELS 2

class __attribute__((__packed__)) tLoxBusNano2RelayConfig : public tConfigHeader {
public:
  uint32_t unknown0;
  uint32_t unknown1;
  uint32_t unknown2;
  uint32_t unknown3;
  uint32_t unknown4;  

private:
  tConfigHeaderFiller filler;
};

class LoxBusNano2Relay : public LoxBusTreeDevice {
  tLoxBusNano2RelayConfig config;
  
  // Protocol methods
  virtual void ConfigUpdate(void);
  virtual void ConfigLoadDefaults(void);
  virtual void ReceiveDirect(LoxCanMessage &message);
  virtual void ReceiveDirectFragment(LoxMsgNATCommand_t command, uint8_t extensionNAT, uint8_t deviceNAT, const uint8_t *data, uint16_t size);
  
  // Application methods
  void UpdateRelayChannel(uint8_t channel, bool value);
public:
  LoxBusNano2Relay(LoxCANBaseDriver &driver, uint32_t serial, eAliveReason_t alive);
};

#endif /* LoxBusNano2Relay_hpp */