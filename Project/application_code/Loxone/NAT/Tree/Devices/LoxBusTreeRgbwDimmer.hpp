//
//  LoxBusTreeRgbwDimmer.hpp
//
//  Created by Markus Fritze on 03.03.19.
//  Copyright (c) 2019 Markus Fritze. All rights reserved.
//

#ifndef LoxBusTreeRgbwDimmer_hpp
#define LoxBusTreeRgbwDimmer_hpp

#include "LoxBusTreeDevice.hpp"

#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_i2c.h"

#include "pca9685.hpp"

#define RGBW_DIMMER_NUMBER_OF_CHANNELS            4
#define RGBW_DIMMER_BITS                          12
#define RGBW_DIMMER_RESOLUTION                    4096
#define RGBW_DIMMER_LOXONE_MAX                    100

#define RGBW_DIMMER_CONSTANT_FADE_TIME            1
#define RGBW_DIMMER_CONSTANT_FADE_RATE            0

#define RGBW_DIMMER_CONFIG_GAMMA_CORRECTION_MASK  0b10000000
#define RGBW_DIMMER_CONFIG_FADE_RATE_MASK         0b01111111
#define RGBW_DIMMER_SMART_FADE_RATE_MASK          0b0011111111111111
#define RGBW_DIMMER_SMART_FADE_RATE_UNIT_MASK     0b0100000000000000

#define RGBW_DIMMER_WARM_WHITE_LIMIT              2700
#define RGBW_DIMMER_COLD_WHITE_LIMIT              6500

#define RGBW_DIMMER_COMPOSITE_VALUE_START         2
#define RGBW_DIMMER_COMPOSITE_FADE_RATE_START     6
#define RGBW_DIMMER_COMPOSITE_IGNORE              0xFF


class __attribute__((__packed__)) tLoxBusTreeRgbwDimmerConfig : public tConfigHeader {
public:
  uint32_t lossOfConnectionState; // RGBW, 101 = retain value, otherwise 0..100%
  uint32_t fadeRate; // in %/s, bit 7: perception correction active
  uint32_t ledType1;
  uint32_t unknown1;
  uint32_t unknown2;

private:
  tConfigHeaderFiller filler;
};

// Led states, need one per channel to make fading possible
typedef struct ledState{
    uint16_t ledTargetValue;
    uint16_t ledCurrentValue;
    uint16_t ledStartValue;
    uint16_t ledFadeCount;
    uint16_t ledFadeCountMax;
    bool ledFadeDone;
    bool ledGammaCorrection;
} ledState_t;

class LoxBusTreeRgbwDimmer : public LoxBusTreeDevice {
  tLoxBusTreeRgbwDimmerConfig config;
  
  // Protocol methods
  virtual void Timer10ms(void);
  virtual void ConfigUpdate(void);
  virtual void ConfigLoadDefaults(void);
  virtual void ReceiveDirect(LoxCanMessage &message);
  virtual void ReceiveDirectFragment(LoxMsgNATCommand_t command, uint8_t extensionNAT, uint8_t deviceNAT, const uint8_t *data, uint16_t size);
  virtual void SetState(eDeviceState state);

  // Application methods
  void SetLedChannel(uint8_t i, uint16_t targetValue);
  void SetLedChannel(uint8_t i, uint16_t targetValue, uint16_t ledFadeCountMax, bool gammaCorrection);
  void UpdateLedFade(uint8_t i);
  uint16_t CalculateSmartFadeRate(uint16_t value16);
  
public:
  LoxBusTreeRgbwDimmer(LoxCANBaseDriver &driver, uint32_t serial, eAliveReason_t alive, I2C_HandleTypeDef *_i2c_handle, uint8_t _baseAddress);
  void Startup(void);

private:
  I2C_HandleTypeDef* i2c_handle; // Local instance of i2c handle
  ledState_t ledStates[RGBW_DIMMER_NUMBER_OF_CHANNELS]; // Array of led states
  bool fadeMode; // 1 = constant fade time, 0 = constant fade speed
  uint8_t baseAddress;
};

#endif /* LoxBusTreeRgbwDimmer_hpp */