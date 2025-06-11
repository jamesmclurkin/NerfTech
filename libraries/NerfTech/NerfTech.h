/*
 * NerfTech.h - NerfTech library for Wiring/Arduino - Version 1.0.0
 *
 * Original library        (1.0.0)   by James McLurkin
 *
 * Text goes here
 *
 */

#ifndef NERFTECH_H
#define NERFTECH_H

#include <Tachometer.h>
#include <BreakBeam.h>
#include <MotorDriver.h>
#include <BlasterDisplay.h>
#include <BlasterDisplayButtons.h>

#if defined(ESP32)
#include <analogWrite.h>
#endif
#if defined(ESP8266)
#include <analogWrite.h>
#endif


#endif // NERFTECH_H
