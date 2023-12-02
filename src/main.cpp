#include <Arduino.h>
#include <NextionLCD.h>
#include <shifterCalcs.h>
#include <tachometer.h>

/**
 * @brief Code to interpret CAN messages from the MoTeC M150 to Display on the Nextion NX4827T043 LCD
 * Derived from https://github.com/tonton81/FlexCAN_T4/blob/master/examples/CAN2.0_example_FIFO_with_interrupts/CAN2.0_example_FIFO_with_interrupts.ino
 *
 * To be used with a Teensy 4.1
 *
 * @author Marvin Sevilla (Bronco Motorsports), 2021-23
 */

#warning TODO: Marv, add an instruction screen on how to use the dashboard, an idling (no ign) animation, and a demo mode

/**
 * @brief Command that is required at the end of every serial message to the LCD to indicate the end of the command
 */
void endCommand()
{
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
}

/**
 * @brief Outputs the data of a CAN message recieved
 * Copied from the FlexCANT4 CAN2.0_example_FIFO_with_interrupts.ino example
 *
 * @param msg the memory address of the CAN message recieved
 */
void canSniff(const CAN_message_t &msg)
{
  Serial.print("MB   ");
  Serial.print(msg.mb);
  Serial.print("    OVERRUN:   ");
  Serial.print(msg.flags.overrun);
  Serial.print("    LEN:   ");
  Serial.print(msg.len);
  Serial.print("   EXT:   ");
  Serial.print(msg.flags.extended);
  Serial.print("   TS:   ");
  Serial.print(msg.timestamp);
  Serial.print("       ID:   ");
  Serial.print(msg.id, HEX);
  Serial.print("   Buffer:            ");
  for (uint8_t i = 0; i < msg.len; i++)
  {
    Serial.print(msg.buf[i], HEX);
    Serial.print("      ");
  }
  Serial.println();
}

/**
 * @brief Outputs and analog signal to the MoTeC M150 (from 0-3.3V) indicating the
 * current gear of the vehicle based on the ratio of CurrentWheelSpeed / CurrentRPM
 *
 * @param ratio a floating point variable whose value is the CurrentWheelSpeed / CurrentRPM
 */
void checkShiftRatio(long unsigned ratio)
{
  float analogOut_ratio = 255 / 6;
  if (ratio == 0)
  {
    Serial.print("Neutral");
    analogWrite(analogOutputPin, 0);
  }
  if (ratio >= gear1 - t12 || ratio < gear1 + t12)
  {
    Serial.print("Gear 1");
    analogWrite(analogOutputPin, analogOut_ratio);
  }
  if (ratio >= gear2 - t12 || ratio < gear2 + t23)
  {
    Serial.print("Gear 2");
    analogWrite(analogOutputPin, analogOut_ratio * 2);
  }
  if (ratio >= gear3 - t23 || ratio < gear3 + t34)
  {
    Serial.print("Gear 3");
    analogWrite(analogOutputPin, analogOut_ratio * 3);
  }
  if (ratio >= gear4 - t34 || ratio < gear4 + t45)
  {
    Serial.print("Gear 4");
    analogWrite(analogOutputPin, analogOut_ratio * 4);
  }
  if (ratio >= gear5 - t45 || ratio < gear5 + t56)
  {
    Serial.print("Gear 5");
    analogWrite(analogOutputPin, analogOut_ratio * 5);
  }
  if (ratio >= gear6 - t56 || ratio < gear6 + t56)
  {
    Serial.print("Gear 6");
    analogWrite(analogOutputPin, analogOut_ratio);
  }
}

/**
 * @brief Reads ID of the latest CAN message, extracts its data, and sends updated parameter data to the LCD
 * Copied from the FlexCANT4 CAN2.0_example_FIFO_with_interrupts.ino example
 *
 * @param msg the memory address of the CAN message recieved
 */
void CANmsgRecieve(const CAN_message_t &msg)
{
  // canSniff(msg);

  /* Redundant code to ensure flip flop of isSilentTime */
  if (getTime() - timeHolder >= silenceTime)
  {
    isSilentTime = !isSilentTime;
    timeHolder = getTime();
  }

  /*
    if isSilentTime == false, process all CAN messages like normal
    if isSilentTime == true, process the message if it's for GearP, otherwise return
  */
  if (isSilentTime == false)
  {
    if (currScreen != Params)
    {

      if (msg.id == 1600) // CAN ID 0x640
      {

        currRPM = msg.buf[0];
        currRPM = currRPM << 8;
        currRPM |= msg.buf[1];
        chngParamVal(10, (int)currRPM);
        // Serial.println(currRPM);
        checkRPM();
      }
      else if (msg.id == 1613) // CAN ID 0x64D
      {
        // lock code from executing other statements, return to updating gear pos

        int mask = 0x0F;
        int mask2 = 0b00000111; // 0000 0111 ; the sign is extended from bit 3
        currGearP = msg.buf[6];
        currGearP &= mask;
        currGearP &= mask2;
        chngParamVal(6, currGearP);
      }
      else if (msg.id == 1609) // CAN ID 0x649
      {
        currECT = msg.buf[0];
        currECT = ((currECT * 10) - 400) / 10; // from C125 Dash manager Multiplier, Divisor, and Adder
        // The equations applied below convert the MoTeC celcius reading to farenheit
        currECT = (currECT * 1.8) + 32;
        chngParamVal(2, currECT);

        currOilTemp = msg.buf[1];
        currOilTemp = ((currOilTemp * 10) - 400) / 10; // from C125 Dash manager Multiplier, Divisor, and Adder
        // The equations applied below convert the MoTeC celcius reading to farenheit
        currOilTemp = (currOilTemp * 1.8) + 32;
        chngParamVal(22, currOilTemp);

        if (currECT >= 220) // raise ECTO warning
        {
          chngParamVal(18, 1);
        }
        else
        {
          if (WARN_ECTO == 1)
          {
            chngParamVal(18, 0);
          }
        }

        if (currOilTemp >= 220) // raise OTEMP warning
        {
          chngParamVal(20, 1);
        }
        else
        {
          if (WARN_OTEMP == 1)
          {
            chngParamVal(20, 0);
          }
        }
      }
      else if (msg.id == 1604) // CAN ID 0x644
      {
        currOilPSR = msg.buf[6];
        currOilPSR = currOilPSR << 8;
        currOilPSR |= msg.buf[7];
        currOilPSR = (currOilPSR * 0.145038) / 10; // kPA to PSI conversion
        chngParamVal(9, currOilPSR);

        if (currRPM > 500) // raise OPRSR warning
        {
          if ((currOilPSR <= 25 && currRPM >= 3000) || (currOilPSR <= 45 && currRPM >= 6000) || (currOilPSR <= 50 && currRPM >= 7000))
          {
            chngParamVal(21, 1);
          }
          else
          {
            chngParamVal(21, 0);
          }
        }
      }
      else if (msg.id == 1601) // CAN ID 0x641
      {
        currFuelPSR = msg.buf[4];
        currFuelPSR = currFuelPSR << 8;
        currFuelPSR |= msg.buf[5];
        currFuelPSR = (currFuelPSR * 0.145038) / 10; // kPA to PSI conversion
        chngParamVal(5, currFuelPSR);

        if (currFuelPSR <= 38 && currRPM > 500) // raise FPRSR warning
        {
          chngParamVal(19, 1);
        }
        else
        {
          if (WARN_FPRSR == 1)
          {
            chngParamVal(19, 0);
          }
        }
      }
    }
    else
    {
      if (msg.id == 1600) // CAN ID 0x640
      {
        currRPM = msg.buf[0];
        currRPM = currRPM << 8;
        currRPM |= msg.buf[1];
        chngParamVal(10, (int)currRPM);

        // Serial.println(currRPM);
        checkRPM();
      }
      else if (msg.id == 1600) // CAN ID 0x640
      {
        currMAP = msg.buf[2];
        currMAP = currMAP << 8;
        currMAP |= msg.buf[3];
        currMAP *= 0.1; // Base resolution for kPA from C125
        // Serial.println(currMAP);
        chngParamVal(8, currMAP);
      }
      else if (msg.id == 1609) // CAN ID 0x649
      {
        currECT = msg.buf[0];
        currECT = ((currECT * 10) - 400) / 10; // from C125 Dash manager Multiplier, Divisor, and Adder
        // The equations applied below convert the MoTeC celcius reading to farenheit
        currECT = (currECT * 1.8) + 32;
        chngParamVal(2, currECT);

        currBatt = ((msg.buf[5]) * 10) / 100; // from C125 Dash manager Multiplier, Divisor, and Adder
        chngParamVal(0, currBatt);

        if (currECT >= 220) // raise ECTO warning
        {
          chngParamVal(18, 1);
        }
        else
        {
          if (WARN_ECTO == 1)
          {
            chngParamVal(18, 0);
          }
        }

        if (currECT >= 240) // raise ECTO warning + "Slow Down" screen
        {
          // chngScrnSlowDown();                       // can't call "chngScrn()" directly for some reason
          chngParamVal(18, WARN_ECTO);
        }
        else
        {
          returnToLastNormScrn();
        }

        currOilTemp = msg.buf[1];
        currOilTemp = ((currOilTemp * 10) - 400) / 10; // from C125 Dash manager Multiplier, Divisor, and Adder
        // The equations applied below convert the MoTeC celcius reading to farenheit
        currOilTemp = (currOilTemp * 1.8) + 32;
        chngParamVal(22, currOilTemp);

        if (currOilTemp >= 220) // raise OTEMP warning
        {
          chngParamVal(20, 1);
        }
        else
        {
          if (WARN_OTEMP == 1)
          {
            chngParamVal(20, 0);
          }
        }

        if (currOilTemp >= 240) // raise OTEMP warning + "Slow Down" screen
        {
          // chngScrnSlowDown();                       // can't call "chngScrn()" directly for some reason
          chngParamVal(18, WARN_OTEMP);
        }
        else
        {
          returnToLastNormScrn();
        }
      }
      else if (msg.id == 1604) // CAN ID 0x644
      {
        currOilPSR = msg.buf[6];
        currOilPSR = currOilPSR << 8;
        currOilPSR |= msg.buf[7];
        currOilPSR = (currOilPSR * 0.145038) / 10; // kPA to PSI conversion
        chngParamVal(9, currOilPSR);

        if (currRPM > 500) // raise OPRSR warning
        {
          if ((currOilPSR <= 25 && currRPM >= 3000) || (currOilPSR <= 45 && currRPM >= 6000) || (currOilPSR <= 50 && currRPM >= 7000))
          {
            chngParamVal(21, 1);
          }
          else
          {
            chngParamVal(21, 0);
          }

          // raise ECTO warning + "Slow Down" screen
          if ((currOilPSR <= 20 && currRPM >= 3000) || (currOilPSR <= 40 && currRPM >= 6000) || (currOilPSR <= 45 && currRPM >= 7000))
          {
            // chngScrnSlowDown();                       // can't call "chngScrn()" directly for some reason
            chngParamVal(18, WARN_OPRSR);
          }
          else
          {
            // returnToLastNormScrn();
          }
        }
      }
      else if (msg.id == 1602) // CAN ID 0x642
      {
        int temp = msg.buf[0];
        temp = temp << 8;
        temp |= msg.buf[1];
        currThrtl = temp * 0.1; // base resolution (multiplied by 100 because I need the percentage representation)
        chngParamVal(11, currThrtl);
      }
      else if (msg.id == 1601) // CAN ID 0x641
      {
        currFuelPSR = msg.buf[4];
        currFuelPSR = currFuelPSR << 8;
        currFuelPSR |= msg.buf[5];
        currFuelPSR = (currFuelPSR * 0.145038) / 10; // kPA to PSI conversion
        chngParamVal(5, currFuelPSR);

        if (currFuelPSR <= 38 && currRPM > 500) // raise FPRSR warning
        {
          chngParamVal(19, 1);
        }
        else
        {
          if (WARN_FPRSR == 1)
          {
            chngParamVal(19, 0);
          }
        }

        if (currFuelPSR <= 35) // raise FPRSR warning + "Slow Down" screen
        {
          // chngScrnSlowDown();                         // can't call "chngScrn()" directly for some reason
          chngParamVal(18, WARN_FPRSR);
        }
        else
        {
          // returnToLastNormScrn();
        }

        // using engine efficiency
        int temp = msg.buf[2];
        temp = temp << 8;
        temp |= msg.buf[3];
        currLamb = temp;
        currLamb *= 0.01; // base resolution from C125 dash manager
        chngParamVal(7, currLamb);
      }

/**
 * @deprecated maxWS statement below was bugged last time used on BM-22 at FSAE MIS
 */
#warning maxWS statement below was bugged last time used on BM-22 at FSAE MIS
      else if (msg.id == 1608) // CAN ID 0x648
      {
        int temp = 0;
        temp = msg.buf[6];
        temp = temp << 8;
        temp |= msg.buf[7];
        temp = temp * 0.1; // base resolution
        temp *= 1.609344;  // conv from km/h to mph
        if (temp > maxWSpd)
        {
          maxWSpd = temp;
        }
        chngParamVal(14, maxWSpd);

        temp /= currRPM;
        // checkShiftRatio(temp);
      }
    }
  }
  else
  {
    if (msg.id == 1613) // CAN ID 0x64D
    {
      // lock code from executing other statements, return to updating gear pos

      int mask = 0x0F;
      int mask2 = 0b00000111; // 0000 0111 ; the sign is extended from bit 3
      currGearP = msg.buf[6];
      currGearP &= mask;
      currGearP &= mask2;
      chngParamVal(6, currGearP);
    }
    else
    {
      return;
    }
  }
}

/**
 * @brief Changes the screen of the LCD
 *
 * @param page An enum that indicates the page to switch to, title accurate
 */
void chngScrn(Screen page)
{
  switch (page)
  {
  case Config1:
    Serial1.print("page Config1");
    endCommand();
    currScreen = page;

    chngParamVal(10, currRPM);

    break;

  case Config2:
    Serial1.print("page Config2");
    endCommand();
    currScreen = page;

#warning functions for changing timers are still bugged
    chngParamVal(15, (int)timer_R[0]);
    chngParamVal(16, (int)timer_R[1]);
    chngParamVal(17, (int)timer_R[2]);
    chngParamVal(12, (int)currTimerDel);
    chngParamVal(13, currTimerDelPic);
    chngParamVal(10, currRPM);

    break;

  case DragMode:
    Serial1.print("page DragMode");
    endCommand();
    currScreen = page;

    chngParamVal(12, (int)currTimerDel);
    chngParamVal(14, maxWSpd);
    chngParamVal(10, currRPM);

    break;

  case Params:
    Serial1.print("page Params");
    endCommand();
    currScreen = page;

    chngParamVal(14, maxWSpd);

    break;

  case BSPD_Trig:
    Serial1.print("page BSPD_Trig");
    endCommand();
    irregScreen = true;
    break;

  case BSPD_Trip:
    Serial1.print("page BSPD_Trip");
    endCommand();
    irregScreen = true;
    break;

  case Shift:
    Serial1.print("page Shift");
    endCommand();
    irregScreen = true;
    break;

  case SlowDown:
    Serial1.print("page SlowDown");
    endCommand();
    irregScreen = true;
    break;

  default:
    Serial1.print("page Config1");
    endCommand();
    currScreen = Config1;
    break;
  }

  /**
   * @details Sets all warnings to invisible on page change
   *
   */
  chngParamVal(18, WARN_ECTO);
  chngParamVal(19, WARN_FPRSR);
  chngParamVal(20, WARN_OTEMP);
  chngParamVal(21, WARN_OPRSR);
}

/**
 * @brief Changes all FLOATING POINT parameters on the LCD
 *
 * @param paramCode an integer that defines what parameter is being changed
 * @param val the specified parameter's new floating point value
 */
void chngParamVal(int paramCode, double val)
{
  switch (paramCode)
  {
  case 0:
    currBatt = val;
    Serial1.print("Batt.txt=\"" + String(currBatt) + "\"");
    endCommand();
    break;

  case 7:
    currLamb = val;
    Serial1.print("Lam.txt=\"" + String(currLamb) + "\"");
    endCommand();
    break;

  case 11:
    currThrtl = val;
    Serial1.print("Thrt.txt=\"" + String(currThrtl) + "\"");
    endCommand();
    break;

  case 12:
    currTimerDel = val;
    Serial1.print("timer_Delta.txt=\"" + convMSec_to_TForm(val) + "\"");
    endCommand();
    break;

  case 15:
    timer_R[0] = val;
    Serial1.print("timer_R1.txt=\"" + convMSec_to_TForm(val) + "\"");
    endCommand();
    break;

  case 16:
    timer_R[1] = val;
    Serial1.print("timer_R2.txt=\"" + convMSec_to_TForm(val) + "\"");
    endCommand();
    break;

  case 17:
    timer_R[2] = val;
    Serial1.print("timer_R3.txt=\"" + convMSec_to_TForm(val) + "\"");
    endCommand();
    break;
  }
}

/**
 * @brief Changes all INTEGER parameters on the LCD
 *
 * @param paramCode an integer that determines what parameter is being changed
 * @param val the specified parameter's new integer value INCLUDING WARNINGS
 */
void chngParamVal(int paramCode, int val)
{
  switch (paramCode)
  {

  case 2:
    if (currScreen != Params)
    {
      return;
    } // if not on proper screen, return. Changing params that are not on screen cause errors & lag w/ screen
    currECT = val;
    Serial1.print("ETC.val=" + String(currECT));
    endCommand();
    break;

  case 3:
    if (currScreen != Config1)
    {
      return;
    } // if not on proper screen, return. Changing params that are not on screen cause errors & lag w/ screen
    currFrontBP = val;
    Serial1.print("FrontBP.val=" + String(currFrontBP));
    endCommand();
    break;

  case 4:
    if (currScreen != Config1)
    {
      return;
    } // if not on proper screen, return. Changing params that are not on screen cause errors & lag w/ screen
    currRearBP = val;
    Serial1.print("RearBP.val=" + String(currRearBP));
    endCommand();
    break;

  case 5:
    if (currScreen != Params)
    {
      return;
    } // if not on proper screen, return. Changing params that are not on screen cause errors & lag w/ screen
    currFuelPSR = val;
    Serial1.print("fuelPRSR.val=" + String(currFuelPSR));
    endCommand();
    break;

  case 6:
    if (currScreen == Config1 || currScreen == Config2 || currScreen == DragMode)
    { // if not on proper screen, return. Changing params that are not on screen cause errors & lag w/ screen
      currGearP = (int)val;
      Serial1.print("gearPos.val=" + String(currGearP));
      endCommand();
    }
    else
    {
      return;
    }
    break;

  case 8:
    if (currScreen != Params)
    {
      return;
    } // if not on proper screen, return. Changing params that are not on screen cause errors & lag w/ screen
    currMAP = val;
    Serial1.print("Map.val=" + String(currMAP));
    endCommand();
    break;

  case 9:
    if (currScreen != Params)
    {
      return;
    } // if not on proper screen, return. Changing params that are not on screen cause errors & lag w/ screen
    currOilPSR = val;
    Serial1.print("oilPRSR.val=" + String(currOilPSR));
    endCommand();
    break;

  case 10:
    if (currScreen != Config1)
    {
      return;
    } // if not on proper screen, return. Changing params that are not on screen cause errors & lag w/ screen
    currRPM = val;
    Serial1.print("RPM.val=" + String(currRPM));
    endCommand();
    break;

  case 13:
    if (currScreen != Config2)
    {
      return;
    } // if not on proper screen, return. Changing params that are not on screen cause errors & lag w/ screen
    currTimerDelPic = val;
    if (currTimerDelPic == 0)
    {
      Serial1.print("pic_Delta.pic=6");
    }
    else
    {
      Serial1.print("pic_Delta.pic=7");
    }
    endCommand();
    break;

  case 14:
    if (currScreen != DragMode || currScreen != Params)
    {
      return;
    } // if not on proper screen, return. Changing params that are not on screen cause errors & lag w/ screen
    maxWSpd = val;
    Serial1.print("MaxWS.val=" + String(maxWSpd));
    endCommand();
    break;

  case 18:
    WARN_ECTO = val;
    if (WARN_ECTO == 1)
    {
      Serial1.print("WARN_ECTO.pic=1");
      endCommand();
    }
    else
    {
      Serial1.print("WARN_ECTO.pic=4");
      endCommand();
    }
    break;

  case 19:
    WARN_FPRSR = val;
    if (WARN_FPRSR == 1)
    {
      Serial1.print("WARN_FPRSR.pic=2");
      endCommand();
    }
    else
    {
      Serial1.print("WARN_FPRSR.pic=4");
      endCommand();
    }
    break;

  case 20:
    WARN_OTEMP = val;
    if (WARN_OTEMP == 1)
    {
      Serial1.print("WARN_OTEMP.pic=3");
      endCommand();
    }
    else
    {
      Serial1.print("WARN_OTEMP.pic=4");
      endCommand();
    }
    break;

  case 21:
    WARN_OPRSR = val;
    if (WARN_OPRSR == 1)
    {
      Serial1.print("WARN_OPRSR.pic=10");
      endCommand();
    }
    else
    {
      Serial1.print("WARN_OPRSR.pic=4");
      endCommand();
    }
    break;

  case 22:
    if (currScreen != Params)
    {
      return;
    } // if not on proper screen, return. Changing params that are not on screen cause errors & lag w/ screen
    currOilTemp = val;
    Serial1.print("oilTEMP.val=" + String(currOilTemp));
    endCommand();
    break;
  }
}

/* Returns time from msecs to a string in the format '00:00' */
/**
 * @brief Returns time from milliseconds to a string in the format '00:00'
 *
 * @param val time in milliseconds
 * @return String in the format "00:00"
 */
String convMSec_to_TForm(long unsigned int val)
{
  long unsigned int holder = val;
  int secs = (holder % 60000) / 1000;
  holder -= secs;
  int mins = (holder == 0) ? 0 : holder / 60000;

  String build = String(mins) + ":";
  build += (secs < 10) ? "0" + String(secs) : String(secs);

  return build;
}

/**
 * @brief Get the Time object
 *
 * @return long unsigned int, time in milliseconds
 */
long unsigned int getTime()
{
  return millis();
}

/**
 * @brief Flashy on sequence that is triggered when the vehicle is keyed on
 *
 */
void flashyOnSequence()
{
  int t1 = 90;
  int t2 = 80;
  int t3 = 1000;

  /* Turn off lights */
  digitalWrite(LED_G, OFF);
  digitalWrite(LED_O, OFF);
  digitalWrite(LED_R, OFF);
  digitalWrite(LED_W, OFF);

  delay(100);

  /* Actual light sequence */
  digitalWrite(LED_G, ON);
  delay(t1);
  digitalWrite(LED_O, ON);
  delay(t1);
  digitalWrite(LED_R, ON);
  delay(t1);
  digitalWrite(LED_W, ON);
  delay(t2);
  digitalWrite(LED_W, OFF);
  delay(t2);
  digitalWrite(LED_W, ON);
  delay(t2);
  digitalWrite(LED_W, OFF);
  delay(t1);
  digitalWrite(LED_R, OFF);
  delay(t1);
  digitalWrite(LED_O, OFF);
  delay(t1);
  digitalWrite(LED_G, OFF);
  delay(t3);
  return;
}

/**
 * @brief Turns on tachometer lights based on current RPM value and gear
 *
 */
void checkRPM()
{
  /*
  if(currRPM>=gears[currGearP] && currScreen != Params)
  {
    chngScrn(Shift);
    digitalWrite(LED_G, OFF);
    digitalWrite(LED_O, OFF);
    digitalWrite(LED_R, OFF);
    digitalWrite(LED_W, ON);
    return;
  }else{
    //digitalWrite(LED_W, OFF);
    returnToLastNormScrn();
  }
  */

  if (currRPM <= 0)
  {
    digitalWrite(LED_G, OFF);
    digitalWrite(LED_O, OFF);
    digitalWrite(LED_R, OFF);
    digitalWrite(LED_W, OFF);
  }
  else if (currRPM <= (gears[currGearP] / 4) * 1)
  {
    digitalWrite(LED_G, ON);
    digitalWrite(LED_O, OFF);
    digitalWrite(LED_R, OFF);
    digitalWrite(LED_W, OFF);
  }
  else if (currRPM <= (gears[currGearP] / 4) * 2)
  {
    digitalWrite(LED_G, ON);
    digitalWrite(LED_O, ON);
    digitalWrite(LED_R, OFF);
    digitalWrite(LED_W, OFF);
  }
  else if (currRPM <= (gears[currGearP] / 4) * 3)
  {
    digitalWrite(LED_G, ON);
    digitalWrite(LED_O, ON);
    digitalWrite(LED_R, ON);
    digitalWrite(LED_W, OFF);
  }
}

/**
 * @brief Code used to operate in conjunction with the "collective's" push-button (not rocker switches)
 *
 */
void pageBtnPressed()
{
  if (analogRead(pgBtnPin) >= 1000 && pgBtnBool == false) // Originally digitalRead was was fidgety, switched to analogRead to get a better reading
  {
    pgBtnBool = true; // Debounce, prevents button from spamming "next page" w/o transitioning to the next page

    if (currScreen == Config1)
    {
      chngScrn(Config2);
    }
    else if (currScreen == Config2)
    {
      chngScrn(DragMode);
    }
    else if (currScreen == DragMode)
    {
      chngScrn(Params);
    }
    else if (currScreen == Params)
    {
      chngScrn(Config1);
    }
    delay(300);
  }
  else if (analogRead(pgBtnPin) >= 1000 && pgBtnBool == true)
  {
    pgBtnBool = false;
    return;
  }
}

/**
 * @brief Funciton designed to return from special screens (Shift, BSPD screens, Slow Down) to normal
 * screens (Config1, Config2, DragMode, Params)
 *
 */
void returnToLastNormScrn()
{
  if (irregScreen != false)
  {
    irregScreen = false;
    chngScrn(currScreen);
  }
}

/**
 * @brief Function to call chngScrn because I could not do it in CANmsgRecieve() for some reasons
 *
 */
void chngScrnSlowDown()
{
  if (currRPM > 500)
  {
    chngScrn(SlowDown);
  }
}

/**
 * @brief All commands ran before the execution of the main loop
 *
 */
void setup()
{
  Serial.begin(112500);
  Serial1.begin(9600); // Changed from 9600, if LCD stops responding change this value back to 9600

  /* Copied from FlexCAN setup() CAN Message Recieved example */
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW); /* optional tranceiver enable pin */
  pinMode(23, INPUT);
  pinMode(LED_G, OUTPUT);            /* Tachometer LED Pin Setup*/
  pinMode(LED_O, OUTPUT);            /* Tachometer LED Pin Setup*/
  pinMode(LED_R, OUTPUT);            /* Tachometer LED Pin Setup*/
  pinMode(LED_W, OUTPUT);            /* Tachometer LED Pin Setup*/
  pinMode(13, OUTPUT);               /* internal LED */
  pinMode(pgBtnPin, INPUT_PULLDOWN); /* Collective's Page button Pin Setup*/

  digitalWrite(13, HIGH);

  flashyOnSequence();

  Can0.begin();
  Can0.setBaudRate(1000000); // MoTeC Bitrate is 1Mbps, translates to 1000000 baud
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(CANmsgRecieve);
  Can0.mailboxStatus();

  attachInterrupt(digitalPinToInterrupt(41), pageBtnPressed, HIGH); // When "Page" btn is pressed on collective, change pagest

  // Change to opening screen
  chngScrn(Params);
}

void loop()
{
  /* Redundant code to ensure flip flop of isSilentTime */
  if (getTime() - timeHolder >= silenceTime)
  {
    isSilentTime = !isSilentTime;
    timeHolder = getTime();
  }

  if (currRPM > 0)
  {
    chngParamVal(12, (double)getTime());
  }
}