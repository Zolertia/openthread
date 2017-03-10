/*
 *  Copyright (c) 2017, Zolertia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TSL2X6X_H
#define TSL2X6X_H

#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

// Light sensor specific values
#define TSL2X6X_ADDR                  0x29
#define TSL2X6X_PARTNO                0x50

#define TSL2X6X_COMMAND               0xA0

#define TSL2X6X_CONTROL               0x00
#define TSL2X6X_CONTROL_POWER_ON      0x03
#define TSL2X6X_CONTROL_POWER_OFF     0x00

#define TSL2X6X_D0LOW                 0x0C
#define TSL2X6X_D0HIGH                0x0D
#define TSL2X6X_D1LOW                 0x0E
#define TSL2X6X_D1HIGH                0x0F

// T/FN/CL package coefficients (hardcoded)
#define K1T                 0X0040
#define B1T                 0x01f2
#define M1T                 0x01b2
#define K2T                 0x0080
#define B2T                 0x0214
#define M2T                 0x02d1
#define K3T                 0x00c0
#define B3T                 0x023f
#define M3T                 0x037b
#define K4T                 0x0100
#define B4T                 0x0270
#define M4T                 0x03fe
#define K5T                 0x0138
#define B5T                 0x016f
#define M5T                 0x01fc
#define K6T                 0x019a
#define B6T                 0x00d2
#define M6T                 0x00fb
#define K7T                 0x029a
#define B7T                 0x0018
#define M7T                 0x0012
#define K8T                 0x029a
#define B8T                 0x0000
#define M8T                 0x0000

// Light sensor functions
int tsl2x6xReadLight(int *lux);

#ifdef __cplusplus
} // end extern "C"
#endif
#endif // TSL2X6X_H_
