//Board = Arduino Nano w/ ATmega328
#define ARDUINO 103
#define __AVR_ATmega328P__
#define F_CPU 16000000L
#define __AVR__
#define __cplusplus
#define __attribute__(x)
#define __inline__
#define __asm__(x)
#define __extension__
#define __ATTR_PURE__
#define __ATTR_CONST__
#define __inline__
#define __asm__ 
#define __volatile__
#define __builtin_va_list
#define __builtin_va_start
#define __builtin_va_end
#define __DOXYGEN__
#define prog_void
#define PGM_VOID_P int
#define NOINLINE __attribute__((noinline))

typedef unsigned char byte;
extern "C" void __cxa_pure_virtual() {}

void updateRPM();
//already defined in arduno.h
void parse_from_APM();
void read_from_APM();
void write_to_APM();
void pack_msg_for_APM();
void blink_led (uint16_t led_on_time, uint16_t led_repeat);
void RPM_reattach();
void RPM_detach();
//already defined in arduno.h

#include "C:\Program Files (x86)\arduino-1.0.3\hardware\arduino\variants\eightanaloginputs\pins_arduino.h" 
#include "C:\Program Files (x86)\arduino-1.0.3\hardware\arduino\cores\arduino\arduino.h"
#include "C:\Users\SpecialK\Dropbox\Prog\APM_nano_addon\APM_nano_addon\APM_nano_addon.ino"
#include "C:\Users\SpecialK\Dropbox\Prog\APM_nano_addon\APM_nano_addon\APM_nano_addon.h"
#include "C:\Users\SpecialK\Dropbox\Prog\APM_nano_addon\APM_nano_addon\RunningMedian.h"
