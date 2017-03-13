/*
 * Copyright (c) 2016, Zolertia - http://www.zolertia.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/**
 * @file
 *   Driver for the DHT22 temperature and humidity sensor
 *
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "../cc2538-reg.h"
#include "../gpio.h"
#include "dht22.h"

#include <openthread-config.h>
#include <openthread/types.h>

#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define DHT22_OUTPUT	   cc2538GpioDirOutput(DHT22_PORT, DHT22_PIN)
#define DHT22_INPUT 	   cc2538GpioDirInput(DHT22_PORT, DHT22_PIN)
#define DHT22_ON        cc2538GpioSetPin(DHT22_PORT, DHT22_PIN)
#define DHT22_OFF       cc2538GpioClearPin(DHT22_PORT, DHT22_PIN)
#define DHT22_READ      cc2538GpioReadPin(DHT22_PORT, DHT22_PIN)
#define DHT22_SOFT_CTL  cc2538GpioSoftwareControl(DHT22_PORT, DHT22_PIN)
#define DHT22_IOC_OVER  cc2538GpioIocOver(DHT22_PORT, DHT22_PIN, 8);

static unsigned char enabled;
static unsigned char busy;
static unsigned char dht22_data[DHT22_BUFFER];

static void delay01(void)
{
    int i;
    for(i = 0; i < DHT22_READING_DELAY; ++i)
    {
      __asm("nop");
    }
}

static void delay02(unsigned int ticks)
{
    unsigned int i;
    for(i = 0; i < ticks; ++i)
    {
      __asm("nop");
    }
}

static void delayAwake(void)
{
    float i;
    for(i = 0; i < DHT22_AWAKE_TIME; ++i)
    {
      __asm("nop");
    }
}

static int dht22_read(void)
{
  unsigned char i;
  unsigned char j = 0;
  unsigned char last_state;
  unsigned char counter = 0;
  unsigned char checksum = 0;

  if(enabled) {
    /* Exit low power mode and initialize variables */
    DHT22_OUTPUT;
    DHT22_OFF;
    DHT22_ON;
    delayAwake();

    //memset(dht22_data, 0, DHT22_BUFFER);
    dht22_data[0] = \
    dht22_data[1] = \
    dht22_data[2] = \
    dht22_data[3] = \
    dht22_data[4] = 0x00;
    
    /* Initialization sequence */
    DHT22_OFF;  // GPIO_CLR_PIN
    delay02(DHT22_START_TIME);  // 20ms BUSYWAIT_UNTIL
    DHT22_ON;   // GPIO_SET_PIN
    delay02(DHT22_READY_TIME);  // 40us   clock_delay_usec
    /* Prepare to read, DHT22 should keep line low 80us, then 80us high.
     * The ready-to-send-bit condition is the line kept low for 50us, then if
     * the line is high between 24-25us the bit sent will be "0" (zero), else
     * if the line is high between 70-74us the bit sent will be "1" (one).
     */
    DHT22_INPUT;
//    __asm("nop");
    last_state = DHT22_READ;

    for(i = 0; i < DHT22_MAX_TIMMING; i++) {
      counter = 0;
      while(DHT22_READ == last_state) {
        counter++;
        delay01();     // 1us Fix Me

        /* Exit if not responsive */
        if(counter == 0xFF) {
          break;
        }
      }

      last_state = DHT22_READ;

      /* Double check for stray sensor */
      if(counter == 0xFF) {
        break;
      }

      /* Ignore the first 3 transitions (the 80us x 2 start condition plus the
       * first ready-to-send-bit state), and discard ready-to-send-bit counts
       */
      if((i >= 4) && ((i % 2) == 0)) {
        dht22_data[j / 8] <<= 1;
        if(counter > DHT22_COUNT) {
          dht22_data[j / 8] |= 1;
        }
        j++;
      }
    }

    /* If we have 5 bytes (40 bits), wrap-up and end */
    if(j >= 40) {
      /* The first 2 bytes are humidity values, the next 2 are temperature, the
       * final byte is the checksum
       */
      checksum = dht22_data[0] + dht22_data[1] + dht22_data[2] + dht22_data[3];
      checksum &= 0xFF;
      if(dht22_data[4] == checksum) {
        DHT22_INPUT;
        DHT22_ON;

        return DHT22_SUCCESS;
      }
    }
  }
  return DHT22_ERROR;
}

static unsigned int dht22_humidity(void)
{
  unsigned int res;
  res = ((unsigned int)dht22_data[0] << 8);
  res += dht22_data[1];
  busy = 0;
  return res;
}

static unsigned int dht22_temperature(void)
{
  unsigned int res;
  res = ((unsigned int)(dht22_data[2] & 0x7F) << 8);
  res += dht22_data[3];
  busy = 0;
  return res;
}

static int value(int type)
{
  if((type != DHT22_READ_HUM) && (type != DHT22_READ_TEMP) &&
     (type != DHT22_READ_ALL)) {
    return DHT22_ERROR;
  }

  if(busy) {
    return DHT22_BUSY;
  }

  busy = 1;

  if(dht22_read() != DHT22_SUCCESS) {
    DHT22_INPUT;
    DHT22_ON;

    busy = 0;
    return DHT22_ERROR;
  }

  switch(type) {
  case DHT22_READ_HUM:
    return dht22_humidity();
  case DHT22_READ_TEMP:
    return dht22_temperature();
  case DHT22_READ_ALL:
    return DHT22_SUCCESS;
  default:
    return DHT22_ERROR;
  }
}

int dht22_read_all(int *temperature, int *humidity)
{
  if((temperature == NULL) || (humidity == NULL)) {
    return DHT22_ERROR;
  }

  if(value(DHT22_READ_ALL) != DHT22_ERROR) {
    *temperature = dht22_temperature();
    *humidity = dht22_humidity();
    return DHT22_SUCCESS;
  }

  /* Already cleaned-up in the value() function */
  return DHT22_ERROR;
}

int dht22_enable(void)
{
  DHT22_SOFT_CTL;
  DHT22_INPUT;
  DHT22_IOC_OVER;
  DHT22_ON;

  /* Restart flag */
  busy = 0;

  enabled = 1;
  return DHT22_SUCCESS;
}

