//
//  LoxBusTreeRgbwDimmer.cpp
//
//  Created by Markus Fritze on 03.03.19.
//  Copyright (c) 2019 Markus Fritze. All rights reserved.
//

#include "LoxBusTreeRgbwDimmer.hpp"
#include <__cross_studio_io.h>

#include "system.hpp"



pca9685_handle_t pca9685 = {
    .i2c_handle = NULL,
    .device_address = PCA9865_I2C_DEFAULT_DEVICE_ADDRESS,
    .inverted = false
};


/***
 *  Constructor
 ***/
LoxBusTreeRgbwDimmer::LoxBusTreeRgbwDimmer(LoxCANBaseDriver &driver, uint32_t serial, eAliveReason_t alive, I2C_HandleTypeDef *_i2c_handle, uint8_t _baseAddress)
  : LoxBusTreeDevice(driver, serial, eDeviceType_t_RGBW24VDimmerTree, 0, 12040624, 1, sizeof(config), &config, alive) {
  i2c_handle = _i2c_handle;
  pca9685.i2c_handle = _i2c_handle;
  baseAddress = _baseAddress*RGBW_DIMMER_NUMBER_OF_CHANNELS;
}


void LoxBusTreeRgbwDimmer::Startup(void) {
  pca9685_init(&pca9685); 
  
  // Set state vars to default values
  for(uint8_t i = 0; i < RGBW_DIMMER_NUMBER_OF_CHANNELS; i++){
    ledStates[i].ledCurrentValue = 0;
    ledStates[i].ledTargetValue = 0;
    ledStates[i].ledStartValue = 0;
    ledStates[i].ledFadeCount = 0;
    ledStates[i].ledFadeCountMax = 0;
    ledStates[i].ledFadeDone = true;
  } 
}

void LoxBusTreeRgbwDimmer::Timer10ms(void) {
  LoxNATExtension::Timer10ms();
  // Update dimming for all channels
  for(uint8_t i = 0; i < RGBW_DIMMER_NUMBER_OF_CHANNELS; i++){
    UpdateLedFade(i);
  }
}

void LoxBusTreeRgbwDimmer::ConfigUpdate(void) {
  // Config has been updated, is automatically reflected in config struct
}

void LoxBusTreeRgbwDimmer::ConfigLoadDefaults(void) {
  // Non recognized config received, load default values
  config.lossOfConnectionState = 0x65656565; // Set to last state when loss of connection happens (=101dec per channel)
  config.fadeRate = 0x64646464; // Set fade rate to default 100%/s (=100dec per channel)
  // Led type does not really matter

}

void LoxBusTreeRgbwDimmer::ReceiveDirect(LoxCanMessage &message) {
  switch (message.commandNat) {
    case RGBW: {
      // Standard 4 channel device, standard RGBW or  standard 4 channel
      for(uint8_t i = 0; i < RGBW_DIMMER_NUMBER_OF_CHANNELS; i++){
        uint16_t targetValue = ((uint8_t)(message.value32>>(i*8))) * RGBW_DIMMER_RESOLUTION/RGBW_DIMMER_LOXONE_MAX;
        SetLedChannel(i, targetValue); // Use config channels for dimming and gamma correction
      }

    }  break;
    case Composite_RGBW: {
      {
        // Smart RGB device
        fadeMode = RGBW_DIMMER_CONSTANT_FADE_TIME; // Smart devices use constant fade time
        uint16_t fadeRate = CalculateSmartFadeRate(message.value16); // Set fadeRate that is transmitted in each message with smart devices
        for(uint8_t i = 0; i < RGBW_DIMMER_NUMBER_OF_CHANNELS; i++){
          uint16_t targetValue = ((uint8_t)(message.value32>>(i*8))) * RGBW_DIMMER_RESOLUTION/RGBW_DIMMER_LOXONE_MAX;
          SetLedChannel(i, targetValue, fadeRate, true); // Smart devices always use gamma correction
        }
      }
    }  break;
    case 0xA8: {
      // Smart Tunable white  
      fadeMode = RGBW_DIMMER_CONSTANT_FADE_TIME; // Smart devices use constant fade time
      uint16_t fadeRate = CalculateSmartFadeRate(message.value16); // Set fadeRate that is transmitted in each message with smart devices

      uint16_t targetValue = (uint16_t)(message.value32 >> 16) * RGBW_DIMMER_RESOLUTION/RGBW_DIMMER_LOXONE_MAX;
      uint16_t temperature = message.value32 & 0xFFFF;
      
      // Calculate values of channels
      uint16_t targetValueWarm = (temperature - RGBW_DIMMER_WARM_WHITE_LIMIT) * (0 - targetValue) / (RGBW_DIMMER_COLD_WHITE_LIMIT - RGBW_DIMMER_WARM_WHITE_LIMIT) + targetValue ; // Map warm white value
      uint16_t targetValueCold = (temperature - RGBW_DIMMER_WARM_WHITE_LIMIT) * (targetValue - 0) / (RGBW_DIMMER_COLD_WHITE_LIMIT - RGBW_DIMMER_WARM_WHITE_LIMIT) + 0 ; // Map warm white value
      
      // Message arrive in two messages: value8 makes distinction between the two, == 1 for second channel
      SetLedChannel(message.value8*2+0, targetValueWarm, fadeRate, true); // First channel
      SetLedChannel(message.value8*2+1, targetValueCold, fadeRate, true); // Second channel
    }  break;
    default:
      LoxBusTreeDevice::ReceiveDirect(message);
      break;
  }
}

void LoxBusTreeRgbwDimmer::ReceiveDirectFragment(LoxMsgNATCommand_t command, uint8_t extensionNAT, uint8_t deviceNAT, const uint8_t *data, uint16_t size) {
    if(command != Config_Data) {
        debug_printf("frag %02x\n", command);
    }
    if(command == Composite_White){ // 50 and 80
      // Smart with individual channels 
      for(uint8_t i = 0; i < RGBW_DIMMER_NUMBER_OF_CHANNELS; i++){
        if(data[RGBW_DIMMER_COMPOSITE_VALUE_START+i] != RGBW_DIMMER_COMPOSITE_IGNORE){
          uint16_t targetValue = data[RGBW_DIMMER_COMPOSITE_VALUE_START+i]* RGBW_DIMMER_RESOLUTION/RGBW_DIMMER_LOXONE_MAX;
          fadeMode = RGBW_DIMMER_CONSTANT_FADE_TIME; // Smart devices use constant fade time
          uint16_t fadeRate = CalculateSmartFadeRate(((uint16_t)data[RGBW_DIMMER_COMPOSITE_FADE_RATE_START+i*2+1]<<8) | data[RGBW_DIMMER_COMPOSITE_FADE_RATE_START+i*2]); // Set fadeRate that is transmitted in each message with smart devices
          SetLedChannel(i, targetValue, fadeRate, true);
        }
      }
    }
    LoxBusTreeDevice::ReceiveDirectFragment(command, extensionNAT, deviceNAT, data, size);
}

void LoxBusTreeRgbwDimmer::SetState(eDeviceState state) {
  LoxNATExtension::SetState(state);
  if (state == eDeviceState_offline) {
    for(uint8_t i = 0; i < RGBW_DIMMER_NUMBER_OF_CHANNELS; i++){
      uint8_t targetValue = config.lossOfConnectionState>>(i*8);
      if(targetValue <= 101) // When 101 is selected, keep last value
        SetLedChannel(i, config.lossOfConnectionState>>(i*8));
    }
  }
}

void LoxBusTreeRgbwDimmer::SetLedChannel(uint8_t i, uint16_t targetValue) {
  fadeMode = RGBW_DIMMER_CONSTANT_FADE_RATE;
  bool gammaCorrection = false;
  // If gamme correction is enabled in config (bit 7 of faderate per channel)
  if((config.fadeRate>>(i*8)) & RGBW_DIMMER_CONFIG_GAMMA_CORRECTION_MASK == RGBW_DIMMER_CONFIG_GAMMA_CORRECTION_MASK)
    gammaCorrection = true;

  // If jump is selected: set new value now
  if(((config.fadeRate>>(i*8)) & RGBW_DIMMER_CONFIG_FADE_RATE_MASK) == 0)
    SetLedChannel(i, targetValue, 0, gammaCorrection);
  else
    SetLedChannel(i, targetValue, 100*100 / ((config.fadeRate>>(i*8)) & RGBW_DIMMER_CONFIG_FADE_RATE_MASK), gammaCorrection ); 
    
}

void LoxBusTreeRgbwDimmer::SetLedChannel(uint8_t i, uint16_t targetValue, uint16_t ledFadeCountMax, bool gammaCorrection) {
  // Set channel to specific channel i. Will fade or jump to targetValue depending on config state.
  if(targetValue != ledStates[i].ledCurrentValue){
    
    if(!ledStates[i].ledFadeDone){
      if( fadeMode == RGBW_DIMMER_CONSTANT_FADE_RATE){ // For constant fade time: ignore new value, for constant fade speed: adjust target value
        //if in constant speed the new val is in same direction and not passed yet
        if(( (ledStates[i].ledStartValue < ledStates[i].ledTargetValue) && (ledStates[i].ledCurrentValue < targetValue)) || //up
                ( (ledStates[i].ledStartValue > ledStates[i].ledTargetValue) && (ledStates[i].ledCurrentValue > targetValue)) ){ //down
          //just set a new val
          ledStates[i].ledTargetValue = targetValue;
        }
      }

    }else{
      ledStates[i].ledTargetValue = targetValue;
      ledStates[i].ledStartValue = ledStates[i].ledCurrentValue;
      ledStates[i].ledFadeCount = 0;
      ledStates[i].ledGammaCorrection = gammaCorrection;

      // If jump is selected: set new value now
      if(ledFadeCountMax == 0){
        ledStates[i].ledCurrentValue = ledStates[i].ledTargetValue;
        pca9685_set_channel_duty_cycle(&pca9685, baseAddress + i, ledStates[i].ledCurrentValue, ledStates[i].ledGammaCorrection);
      }else{
        ledStates[i].ledFadeCountMax = ledFadeCountMax;
      }
    }
  }
}

void LoxBusTreeRgbwDimmer::UpdateLedFade(uint8_t i) {
  // Update fade values for channel i, needs to be called every 10ms 

  // Adapted from https://github.com/septillion-git/FadeLed/blob/master/src/FadeLed.cpp
  uint16_t newVal;
  if(ledStates[i].ledCurrentValue == ledStates[i].ledTargetValue){
    ledStates[i].ledFadeDone = true;
  }
  // Fade up
  else if(ledStates[i].ledCurrentValue < ledStates[i].ledTargetValue){

    if(fadeMode == RGBW_DIMMER_CONSTANT_FADE_TIME){
      //for constant fade time we add the difference over countMax steps
      newVal = ledStates[i].ledStartValue + ledStates[i].ledFadeCount * (ledStates[i].ledTargetValue - ledStates[i].ledStartValue) / ledStates[i].ledFadeCountMax;
    }else{
      //for constant fade speed we add the full resolution over countMax steps
      newVal = ledStates[i].ledStartValue + ledStates[i].ledFadeCount * RGBW_DIMMER_RESOLUTION / ledStates[i].ledFadeCountMax;
    }

    if(newVal != ledStates[i].ledCurrentValue){
      //check for overflow
      if(newVal < ledStates[i].ledCurrentValue){
        ledStates[i].ledCurrentValue = RGBW_DIMMER_RESOLUTION;
      }
      //Check for overshoot
      else if(newVal > ledStates[i].ledTargetValue){
        ledStates[i].ledCurrentValue = ledStates[i].ledTargetValue;
      }
      //Only if the new value is good we use that
      else{
        ledStates[i].ledCurrentValue = newVal;
      }
        
      pca9685_set_channel_duty_cycle(&pca9685, baseAddress + i, ledStates[i].ledCurrentValue, ledStates[i].ledGammaCorrection);
    }
    ledStates[i].ledFadeCount++;
  }
  //need to fade down
  else if(ledStates[i].ledCurrentValue > ledStates[i].ledTargetValue){
    //we always start at the current level saved in _startVal
    if(fadeMode == RGBW_DIMMER_CONSTANT_FADE_TIME){
      //for constant fade time we subtract the difference over countMax steps
      newVal = ledStates[i].ledStartValue - ledStates[i].ledFadeCount * (ledStates[i].ledStartValue - ledStates[i].ledTargetValue) / ledStates[i].ledFadeCountMax;
    }else{
      //for constant fade speed we subtract the full resolution over countMax steps
      newVal = ledStates[i].ledStartValue - ledStates[i].ledFadeCount * RGBW_DIMMER_RESOLUTION / ledStates[i].ledFadeCountMax;
    }
  
    //check if new
    if(newVal != ledStates[i].ledCurrentValue){
      //check for overflow
      if(newVal > ledStates[i].ledCurrentValue){
        ledStates[i].ledCurrentValue = 0;
      }
      //Check for overshoot
      else if(newVal < ledStates[i].ledTargetValue){
        ledStates[i].ledCurrentValue = ledStates[i].ledTargetValue;
      }
      //Only if the new value is good we use that
      else{
        ledStates[i].ledCurrentValue = newVal;
      }
        
      pca9685_set_channel_duty_cycle(&pca9685, baseAddress + i, ledStates[i].ledCurrentValue, ledStates[i].ledGammaCorrection);
    }
    ledStates[i].ledFadeCount++;
  }
}

uint16_t LoxBusTreeRgbwDimmer::CalculateSmartFadeRate(uint16_t value16) {
  // Set fadeRate that is transmitted in each message with smart devices
  uint16_t fadeRate = value16 & RGBW_DIMMER_SMART_FADE_RATE_MASK; // fade unit is transmitted in byte 14: 1=>in seconds, 0=>in 1/10 seconds
  if((value16 & RGBW_DIMMER_SMART_FADE_RATE_UNIT_MASK) == RGBW_DIMMER_SMART_FADE_RATE_UNIT_MASK){ // in seconds
    fadeRate = fadeRate * 100; 
  }else{ // in 1/10s (100ms)
    fadeRate = fadeRate * 10;
  }
  return fadeRate;
}

