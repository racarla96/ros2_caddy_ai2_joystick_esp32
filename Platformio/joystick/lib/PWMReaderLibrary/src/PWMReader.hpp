#pragma once

#include "Arduino.h"
#include "interrupt_pins.h"

class PWMReader
{
public:
  // Create a empty instance to allocate memory
  PWMReader() {}

  // Public method to get the duty cycle in microseconds
  unsigned long getDutyCicle_us(){
    return _duty_time_us;
  }
  
  // Public method to get the period in microseconds
  unsigned long getPeriod_us(){
    return _period_time_us;
  }

  // Public method to get the time of the last rising edge in microseconds
  unsigned long getTimeLastRisingEdge_us(){
    return _rising_time_us;
  }

  bool isEnable(){
    if(micros() - _rising_time_us > _period_time_us) return false;
    return true;
  }

private:
  int _readPWMPin;   // Private member variable to store the PWM pin number

    // Volatile variables to store time information (in microseconds)
  volatile unsigned long _prev_rising_time_us;
  volatile unsigned long _rising_time_us;
  volatile unsigned long _falling_time_us;
  volatile unsigned long _duty_time_us;
  volatile unsigned long _period_time_us;

  // Private method to handle the PWM signal changes
  void Trigger()
  {
    // Disable interrupts to ensure atomicity of the critical section
    #if !defined(ESP32)
    noInterrupts();
    #endif 

    // Check if the PWM signal is currently LOW (FALLING EDGE)
    if(digitalRead(_readPWMPin) == LOW) { 
      // Record the falling time and calculate duty cycle
      _falling_time_us = micros();
      _duty_time_us = _falling_time_us - _rising_time_us;
    }
    else {
      // Record the rising time and calculate the period
      _prev_rising_time_us = _rising_time_us;
      _rising_time_us = micros();
      _period_time_us = _rising_time_us - _prev_rising_time_us;
    }

    // Re-enable interrupts
    #if !defined(ESP32)
    interrupts();
    #endif 
  }

  // Glue
  static PWMReader *instances[CORE_NUM_INTERRUPT];
#ifdef CORE_INT0_PIN
  static void TriggerExt0()
  {
    PWMReader::instances[0]->Trigger();
  }
#endif
#ifdef CORE_INT1_PIN
  static void TriggerExt1()
  {
    PWMReader::instances[1]->Trigger();
  }
#endif
#ifdef CORE_INT2_PIN
  static void TriggerExt2()
  {
    PWMReader::instances[2]->Trigger();
  }
#endif
#ifdef CORE_INT3_PIN
  static void TriggerExt3()
  {
    PWMReader::instances[3]->Trigger();
  }
#endif
#ifdef CORE_INT4_PIN
  static void TriggerExt4()
  {
    PWMReader::instances[4]->Trigger();
  }
#endif
#ifdef CORE_INT5_PIN
  static void TriggerExt5()
  {
    PWMReader::instances[5]->Trigger();
  }
#endif
#ifdef CORE_INT6_PIN
  static void TriggerExt6()
  {
    PWMReader::instances[6]->Trigger();
  }
#endif
#ifdef CORE_INT7_PIN
  static void TriggerExt7()
  {
    PWMReader::instances[7]->Trigger();
  }
#endif
#ifdef CORE_INT8_PIN
  static void TriggerExt8()
  {
    PWMReader::instances[8]->Trigger();
  }
#endif
#ifdef CORE_INT9_PIN
  static void TriggerExt9()
  {
    PWMReader::instances[9]->Trigger();
  }
#endif
#ifdef CORE_INT10_PIN
  static void TriggerExt10()
  {
    PWMReader::instances[10]->Trigger();
  }
#endif
#ifdef CORE_INT11_PIN
  static void TriggerExt11()
  {
    PWMReader::instances[11]->Trigger();
  }
#endif
#ifdef CORE_INT12_PIN
  static void TriggerExt12()
  {
    PWMReader::instances[12]->Trigger();
  }
#endif
#ifdef CORE_INT13_PIN
  static void TriggerExt13()
  {
    PWMReader::instances[13]->Trigger();
  }
#endif
#ifdef CORE_INT14_PIN
  static void TriggerExt14()
  {
    PWMReader::instances[14]->Trigger();
  }
#endif
#ifdef CORE_INT15_PIN
  static void TriggerExt15()
  {
    PWMReader::instances[15]->Trigger();
  }
#endif
#ifdef CORE_INT16_PIN
  static void TriggerExt16()
  {
    PWMReader::instances[16]->Trigger();
  }
#endif
#ifdef CORE_INT17_PIN
  static void TriggerExt17()
  {
    PWMReader::instances[17]->Trigger();
  }
#endif
#ifdef CORE_INT18_PIN
  static void TriggerExt18()
  {
    PWMReader::instances[18]->Trigger();
  }
#endif
#ifdef CORE_INT19_PIN
  static void TriggerExt19()
  {
    PWMReader::instances[19]->Trigger();
  }
#endif
#ifdef CORE_INT20_PIN
  static void TriggerExt20()
  {
    PWMReader::instances[20]->Trigger();
  }
#endif
#ifdef CORE_INT21_PIN
  static void TriggerExt21()
  {
    PWMReader::instances[21]->Trigger();
  }
#endif
#ifdef CORE_INT22_PIN
  static void TriggerExt22()
  {
    PWMReader::instances[22]->Trigger();
  }
#endif
#ifdef CORE_INT23_PIN
  static void TriggerExt23()
  {
    PWMReader::instances[23]->Trigger();
  }
#endif
#ifdef CORE_INT24_PIN
  static void TriggerExt24()
  {
    PWMReader::instances[24]->Trigger();
  }
#endif
#ifdef CORE_INT25_PIN
  static void TriggerExt25()
  {
    PWMReader::instances[25]->Trigger();
  }
#endif
#ifdef CORE_INT26_PIN
  static void TriggerExt26()
  {
    PWMReader::instances[26]->Trigger();
  }
#endif
#ifdef CORE_INT27_PIN
  static void TriggerExt27()
  {
    PWMReader::instances[27]->Trigger();
  }
#endif
#ifdef CORE_INT28_PIN
  static void TriggerExt28()
  {
    PWMReader::instances[28]->Trigger();
  }
#endif
#ifdef CORE_INT29_PIN
  static void TriggerExt29()
  {
    PWMReader::instances[29]->Trigger();
  }
#endif
#ifdef CORE_INT30_PIN
  static void TriggerExt30()
  {
    PWMReader::instances[30]->Trigger();
  }
#endif
#ifdef CORE_INT31_PIN
  static void TriggerExt31()
  {
    PWMReader::instances[31]->Trigger();
  }
#endif
#ifdef CORE_INT32_PIN
  static void TriggerExt32()
  {
    PWMReader::instances[32]->Trigger();
  }
#endif
#ifdef CORE_INT33_PIN
  static void TriggerExt33()
  {
    PWMReader::instances[33]->Trigger();
  }
#endif
#ifdef CORE_INT34_PIN
  static void TriggerExt34()
  {
    PWMReader::instances[34]->Trigger();
  }
#endif
#ifdef CORE_INT35_PIN
  static void TriggerExt35()
  {
    PWMReader::instances[35]->Trigger();
  }
#endif
#ifdef CORE_INT36_PIN
  static void TriggerExt36()
  {
    PWMReader::instances[36]->Trigger();
  }
#endif
#ifdef CORE_INT37_PIN
  static void TriggerExt37()
  {
    PWMReader::instances[37]->Trigger();
  }
#endif
#ifdef CORE_INT38_PIN
  static void TriggerExt38()
  {
    PWMReader::instances[38]->Trigger();
  }
#endif
#ifdef CORE_INT39_PIN
  static void TriggerExt39()
  {
    PWMReader::instances[39]->Trigger();
  }
#endif
#ifdef CORE_INT40_PIN
  static void TriggerExt40()
  {
    PWMReader::instances[40]->Trigger();
  }
#endif
#ifdef CORE_INT41_PIN
  static void TriggerExt41()
  {
    PWMReader::instances[41]->Trigger();
  }
#endif
#ifdef CORE_INT42_PIN
  static void TriggerExt42()
  {
    PWMReader::instances[42]->Trigger();
  }
#endif
#ifdef CORE_INT43_PIN
  static void TriggerExt43()
  {
    PWMReader::instances[43]->Trigger();
  }
#endif
#ifdef CORE_INT44_PIN
  static void TriggerExt44()
  {
    PWMReader::instances[44]->Trigger();
  }
#endif
#ifdef CORE_INT45_PIN
  static void TriggerExt45()
  {
    PWMReader::instances[45]->Trigger();
  }
#endif
#ifdef CORE_INT46_PIN
  static void TriggerExt46()
  {
    PWMReader::instances[46]->Trigger();
  }
#endif
#ifdef CORE_INT47_PIN
  static void TriggerExt47()
  {
    PWMReader::instances[47]->Trigger();
  }
#endif
#ifdef CORE_INT48_PIN
  static void TriggerExt48()
  {
    PWMReader::instances[48]->Trigger();
  }
#endif
#ifdef CORE_INT49_PIN
  static void TriggerExt49()
  {
    PWMReader::instances[49]->Trigger();
  }
#endif
#ifdef CORE_INT50_PIN
  static void TriggerExt50()
  {
    PWMReader::instances[50]->Trigger();
  }
#endif
#ifdef CORE_INT51_PIN
  static void TriggerExt51()
  {
    PWMReader::instances[51]->Trigger();
  }
#endif
#ifdef CORE_INT52_PIN
  static void TriggerExt52()
  {
    PWMReader::instances[52]->Trigger();
  }
#endif
#ifdef CORE_INT53_PIN
  static void TriggerExt53()
  {
    PWMReader::instances[53]->Trigger();
  }
#endif
  /*
    // Best practice, but overload check instance if null
    // we need a faster interrupt routine
    static void TriggerExt0()
    {
      if (PWMReader::instances[0] != NULL)
      PWMReader::instances[0]->Trigger();
    } // end of PWMReader::TriggerExt0
  */
public:
  bool begin(int readPWMPin)
  {
    _readPWMPin = readPWMPin;
    pinMode(readPWMPin, INPUT_PULLUP);
    switch (readPWMPin)
    {
#ifdef CORE_INT0_PIN
    case CORE_INT0_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT0_PIN), TriggerExt0, CHANGE);
      instances[0] = this;
      break;
#endif
#ifdef CORE_INT1_PIN
    case CORE_INT1_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT1_PIN), TriggerExt1, CHANGE);
      instances[1] = this;
      break;
#endif
#ifdef CORE_INT2_PIN
    case CORE_INT2_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT2_PIN), TriggerExt2, CHANGE);
      instances[2] = this;
      break;
#endif
#ifdef CORE_INT3_PIN
    case CORE_INT3_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT3_PIN), TriggerExt3, CHANGE);
      instances[3] = this;
      break;
#endif
#ifdef CORE_INT4_PIN
    case CORE_INT4_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT4_PIN), TriggerExt4, CHANGE);
      instances[4] = this;
      break;
#endif
#ifdef CORE_INT5_PIN
    case CORE_INT5_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT5_PIN), TriggerExt5, CHANGE);
      instances[5] = this;
      break;
#endif
#ifdef CORE_INT6_PIN
    case CORE_INT6_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT6_PIN), TriggerExt6, CHANGE);
      instances[6] = this;
      break;
#endif
#ifdef CORE_INT7_PIN
    case CORE_INT7_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT7_PIN), TriggerExt7, CHANGE);
      instances[7] = this;
      break;
#endif
#ifdef CORE_INT8_PIN
    case CORE_INT8_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT8_PIN), TriggerExt8, CHANGE);
      instances[8] = this;
      break;
#endif
#ifdef CORE_INT9_PIN
    case CORE_INT9_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT9_PIN), TriggerExt9, CHANGE);
      instances[9] = this;
      break;
#endif
#ifdef CORE_INT10_PIN
    case CORE_INT10_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT10_PIN), TriggerExt10, CHANGE);
      instances[10] = this;
      break;
#endif
#ifdef CORE_INT11_PIN
    case CORE_INT11_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT11_PIN), TriggerExt11, CHANGE);
      instances[11] = this;
      break;
#endif
#ifdef CORE_INT12_PIN
    case CORE_INT12_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT12_PIN), TriggerExt12, CHANGE);
      instances[12] = this;
      break;
#endif
#ifdef CORE_INT13_PIN
    case CORE_INT13_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT13_PIN), TriggerExt13, CHANGE);
      instances[13] = this;
      break;
#endif
#ifdef CORE_INT14_PIN
    case CORE_INT14_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT14_PIN), TriggerExt14, CHANGE);
      instances[14] = this;
      break;
#endif
#ifdef CORE_INT15_PIN
    case CORE_INT15_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT15_PIN), TriggerExt15, CHANGE);
      instances[15] = this;
      break;
#endif
#ifdef CORE_INT16_PIN
    case CORE_INT16_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT16_PIN), TriggerExt16, CHANGE);
      instances[16] = this;
      break;
#endif
#ifdef CORE_INT17_PIN
    case CORE_INT17_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT17_PIN), TriggerExt17, CHANGE);
      instances[17] = this;
      break;
#endif
#ifdef CORE_INT18_PIN
    case CORE_INT18_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT18_PIN), TriggerExt18, CHANGE);
      instances[18] = this;
      break;
#endif
#ifdef CORE_INT19_PIN
    case CORE_INT19_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT19_PIN), TriggerExt19, CHANGE);
      instances[19] = this;
      break;
#endif
#ifdef CORE_INT20_PIN
    case CORE_INT20_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT20_PIN), TriggerExt20, CHANGE);
      instances[20] = this;
      break;
#endif
#ifdef CORE_INT21_PIN
    case CORE_INT21_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT21_PIN), TriggerExt21, CHANGE);
      instances[21] = this;
      break;
#endif
#ifdef CORE_INT22_PIN
    case CORE_INT22_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT22_PIN), TriggerExt22, CHANGE);
      instances[22] = this;
      break;
#endif
#ifdef CORE_INT23_PIN
    case CORE_INT23_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT23_PIN), TriggerExt23, CHANGE);
      instances[23] = this;
      break;
#endif
#ifdef CORE_INT24_PIN
    case CORE_INT24_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT24_PIN), TriggerExt24, CHANGE);
      instances[24] = this;
      break;
#endif
#ifdef CORE_INT25_PIN
    case CORE_INT25_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT25_PIN), TriggerExt25, CHANGE);
      instances[25] = this;
      break;
#endif
#ifdef CORE_INT26_PIN
    case CORE_INT26_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT26_PIN), TriggerExt26, CHANGE);
      instances[26] = this;
      break;
#endif
#ifdef CORE_INT27_PIN
    case CORE_INT27_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT27_PIN), TriggerExt27, CHANGE);
      instances[27] = this;
      break;
#endif
#ifdef CORE_INT28_PIN
    case CORE_INT28_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT28_PIN), TriggerExt28, CHANGE);
      instances[28] = this;
      break;
#endif
#ifdef CORE_INT29_PIN
    case CORE_INT29_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT29_PIN), TriggerExt29, CHANGE);
      instances[29] = this;
      break;
#endif
#ifdef CORE_INT30_PIN
    case CORE_INT30_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT30_PIN), TriggerExt30, CHANGE);
      instances[30] = this;
      break;
#endif
#ifdef CORE_INT31_PIN
    case CORE_INT31_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT31_PIN), TriggerExt31, CHANGE);
      instances[31] = this;
      break;
#endif
#ifdef CORE_INT32_PIN
    case CORE_INT32_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT32_PIN), TriggerExt32, CHANGE);
      instances[32] = this;
      break;
#endif
#ifdef CORE_INT33_PIN
    case CORE_INT33_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT33_PIN), TriggerExt33, CHANGE);
      instances[33] = this;
      break;
#endif
#ifdef CORE_INT34_PIN
    case CORE_INT34_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT34_PIN), TriggerExt34, CHANGE);
      instances[34] = this;
      break;
#endif
#ifdef CORE_INT35_PIN
    case CORE_INT35_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT35_PIN), TriggerExt35, CHANGE);
      instances[35] = this;
      break;
#endif
#ifdef CORE_INT36_PIN
    case CORE_INT36_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT36_PIN), TriggerExt36, CHANGE);
      instances[36] = this;
      break;
#endif
#ifdef CORE_INT37_PIN
    case CORE_INT37_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT37_PIN), TriggerExt37, CHANGE);
      instances[37] = this;
      break;
#endif
#ifdef CORE_INT38_PIN
    case CORE_INT38_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT38_PIN), TriggerExt38, CHANGE);
      instances[38] = this;
      break;
#endif
#ifdef CORE_INT39_PIN
    case CORE_INT39_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT39_PIN), TriggerExt39, CHANGE);
      instances[39] = this;
      break;
#endif
#ifdef CORE_INT40_PIN
    case CORE_INT40_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT40_PIN), TriggerExt40, CHANGE);
      instances[40] = this;
      break;
#endif
#ifdef CORE_INT41_PIN
    case CORE_INT41_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT41_PIN), TriggerExt41, CHANGE);
      instances[41] = this;
      break;
#endif
#ifdef CORE_INT42_PIN
    case CORE_INT42_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT42_PIN), TriggerExt42, CHANGE);
      instances[42] = this;
      break;
#endif
#ifdef CORE_INT43_PIN
    case CORE_INT43_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT43_PIN), TriggerExt43, CHANGE);
      instances[43] = this;
      break;
#endif
#ifdef CORE_INT44_PIN
    case CORE_INT44_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT44_PIN), TriggerExt44, CHANGE);
      instances[44] = this;
      break;
#endif
#ifdef CORE_INT45_PIN
    case CORE_INT45_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT45_PIN), TriggerExt45, CHANGE);
      instances[45] = this;
      break;
#endif
#ifdef CORE_INT46_PIN
    case CORE_INT46_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT46_PIN), TriggerExt46, CHANGE);
      instances[46] = this;
      break;
#endif
#ifdef CORE_INT47_PIN
    case CORE_INT47_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT47_PIN), TriggerExt47, CHANGE);
      instances[47] = this;
      break;
#endif
#ifdef CORE_INT48_PIN
    case CORE_INT48_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT48_PIN), TriggerExt48, CHANGE);
      instances[48] = this;
      break;
#endif
#ifdef CORE_INT49_PIN
    case CORE_INT49_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT49_PIN), TriggerExt49, CHANGE);
      instances[49] = this;
      break;
#endif
#ifdef CORE_INT50_PIN
    case CORE_INT50_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT50_PIN), TriggerExt50, CHANGE);
      instances[50] = this;
      break;
#endif
#ifdef CORE_INT51_PIN
    case CORE_INT51_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT51_PIN), TriggerExt51, CHANGE);
      instances[51] = this;
      break;
#endif
#ifdef CORE_INT52_PIN
    case CORE_INT52_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT52_PIN), TriggerExt52, CHANGE);
      instances[52] = this;
      break;
#endif
#ifdef CORE_INT53_PIN
    case CORE_INT53_PIN:
      attachInterrupt(digitalPinToInterrupt(CORE_INT53_PIN), TriggerExt53, CHANGE);
      instances[53] = this;
      break;
#endif
    default:
      return false;
    } // end of switch
    return true;
  } // end of PWMReader::begin

}; // end of class PWMReader

PWMReader *PWMReader::instances[CORE_NUM_INTERRUPT];