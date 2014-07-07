

//#include <FastSerial.h>
#include <Servo.h>
#include "APM_nano_addon.h"
#include "RunningMedian.h"
#include <util/atomic.h>



#define PIN_OnboardLED 13
#define PIN_RPM_in		3
#define PIN_nextto_RPM  4
#define PIN_RPM_int		1 // int1
#define PIN_FrontLight  5  // PWM

#define PIN_Light  6  // PWM fähig
#define PIN_Light2 7
#define PIN_FET3   8

#define FrontLight_brightness 255  // 0-255
#define FrontLight_FlightMode 1  // (Loiter) // 1-4
#define NoLight_FlightMode 1

#define PIN_Mode_out  9
#define PIN_Gyr_in   10
#define PIN_Aux_in   11

#define PIN_SW1   A5
#define PIN_SW2   A4


//#define RPM_Divisions   1   // Unterteilungen am Abnehmer
#define RPM_lowpass_ms 14UL
#define RPM_highpass_us 300000UL

#define ALT_landing_light_On  180 // cm
#define ALT_landing_light_Off 220 // cm

#define ALT_front_light_On  20 // m
#define ALT_front_light_Off 22 // m

#define RPM_avg_Nr 20  // 0 - 255

//FastSerialPort0(Serial);
 
unsigned int ModeSigLost = 5;   // recv lost, sw to flight mode ..
unsigned int HoldAirTime = 30;  // sek bis 

unsigned long ModeIn_aux = 0;
unsigned long ModeIn_gyr = 0;
unsigned int  CurrMode = 1;
unsigned int  ModeServoOut = 0;
unsigned long lastTime50ms = 0;
unsigned long lastTime1s = 0;
unsigned long lastTimeLED = 0;
unsigned long lastTimeRPM = 0;
unsigned long curr_millis = 0;
unsigned int  cntLostSeconds = 0;
volatile unsigned long lastRPMmicros = 0;
volatile unsigned long timeRPM = 0;
boolean      failsafe = false;
boolean      apm_home_set = false;	
boolean      apm_armed = false;
boolean      SW1 = false;
boolean      SW2 = false;
uint16_t     rpm = 0;
Servo		 ModeOut;
uint8_t      cntRPM_times = 0;
uint16_t     RPM_times[RPM_avg_Nr] = {0};
uint16_t	 alt_by_sonar = 0;  // cm
uint16_t	 alt_over_home = 0;  // m
volatile boolean		 int_detached = false;
//Servo ThrottleOut;




void updateRPM()
{
	timeRPM = micros() - lastRPMmicros;
	//lastRPMmicros += timeRPM;  // 70 min overflow
	lastRPMmicros = micros();
	if (timeRPM > RPM_highpass_us) timeRPM = 0;
	if (timeRPM < 14000UL) timeRPM = 0;
	if (timeRPM != 0) 
	{
		// RPM_times[cntRPM_times++] = (uint16_t) (timeRPM / 10);
		RPM_times[cntRPM_times] = (uint16_t) (timeRPM / 10);
		cntRPM_times++;
		if ( cntRPM_times >= RPM_avg_Nr ) cntRPM_times = 0;
	}
	int_detached = true;
	lastTimeRPM = curr_millis;
	detachInterrupt(PIN_RPM_int);
}

void setup()
{
	pinMode(PIN_RPM_in, INPUT);
	pinMode(PIN_Mode_out, OUTPUT);
	pinMode(PIN_Light, OUTPUT);
	pinMode(PIN_Light2, OUTPUT);
	pinMode(PIN_FrontLight, OUTPUT);
	pinMode(PIN_Gyr_in, INPUT);
	pinMode(PIN_Aux_in, INPUT);
	pinMode(PIN_nextto_RPM, OUTPUT);
	pinMode(PIN_SW1, INPUT_PULLUP);
    pinMode(PIN_SW2, INPUT_PULLUP);

	digitalWrite(PIN_nextto_RPM, LOW);
	digitalWrite(PIN_Light, LOW); 
	digitalWrite(PIN_Light2, LOW);
	digitalWrite(PIN_FrontLight, LOW);

	ModeOut.attach(PIN_Mode_out, 1000, 2000);
	attachInterrupt(PIN_RPM_int, updateRPM, FALLING);

	Serial.begin(38400);
	//Serial.set_blocking_writes(false);

	delay(100);

}



void parse_from_APM() 
{
	byte ap_bitflags = 0;
	if (apmRXi != msg_fromAPM_size) return;
	apmRXi = 0;
	if (msg_fromAPM[0] != 0xFF) return;
	ap_bitflags = msg_fromAPM[1];
	alt_by_sonar  = (int32_t) ( msg_fromAPM[2] + (msg_fromAPM[3] << 8));
	alt_over_home = (int32_t) ( msg_fromAPM[4] + (msg_fromAPM[5] << 8));
	apm_home_set = (ap_bitflags & (1 << 0));
	apm_armed    =  (ap_bitflags & (1 << 1));
}

void read_from_APM()
{
	int      c;
	while ((Serial.available() > 0) && (apmRXi < msg_fromAPM_size))
	{
		c = Serial.read();
		msg_fromAPM[apmRXi] = (byte) c;
		apmRXi++;
	}
	while (Serial.available() > 0) c = Serial.read(); // clear buffer
}

void write_to_APM()
{
	Serial.write(msg_toAPM, msg_toAPM_size);
	//Serial.print (";  RPM: ");
    //Serial.println (rpm);
}

void pack_msg_for_APM()
{
	byte ap_bitflags = 0;

	msg_toAPM[0] = 0xFF;
	msg_toAPM[1] = (byte) (rpm & 0x00FF);
	msg_toAPM[2] = (byte) ((rpm & 0xFF00) >> 8);
//	msg_toAPM[3] = ap_bitflags;
}

void blink_led (uint16_t led_on_time, uint16_t led_repeat)
{
	if ( (curr_millis - lastTimeLED) > led_on_time) {
		digitalWrite (PIN_Light, LOW);
		digitalWrite (PIN_OnboardLED, LOW);
	}

	if ( (curr_millis - lastTimeLED) > led_repeat) {
		lastTimeLED = curr_millis;
		digitalWrite (PIN_Light, HIGH);
		digitalWrite (PIN_OnboardLED, HIGH);
	}
}

void RPM_reattach()
{
	if ( int_detached ) if  ((curr_millis - lastTimeRPM) > RPM_lowpass_ms) 
	{
		int_detached = false;
		lastTimeRPM = curr_millis;
		attachInterrupt(PIN_RPM_int, updateRPM, FALLING);
	}
}

void RPM_detach()
{
			detachInterrupt(PIN_RPM_int);
	        int_detached = false;
			//ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			lastRPMmicros = 0;
			//}
			timeRPM = 0;
			for (uint8_t i = 0; i < RPM_avg_Nr; i++)
			{
					timeRPM += ((uint32_t) RPM_times[i]);
			}
			//timeRPM *= 10;
			timeRPM = (unsigned long) (timeRPM / ((unsigned long) RPM_avg_Nr));
			if (timeRPM != 0) {
				rpm = (uint16_t) ((6000000UL / (timeRPM)) / 2);
			}
}

void loop()
{
	static uint8_t cnt = 0;

	curr_millis = millis(); 

	SW1 = !digitalRead(PIN_SW1);
	SW2 = !digitalRead(PIN_SW2);

	if ( (curr_millis - lastTime50ms) > 50 ) { // 50ms  = 20Hz
		lastTime50ms = curr_millis;
		cnt++;

		if (((cnt + 1) % 20) == 0) {
			RPM_detach();
			ModeIn_gyr = pulseIn (PIN_Gyr_in, HIGH, 20000); //read RC channel 20ms timeout
		} else if (((cnt + 0) % 20) == 0) { 
			ModeIn_aux = pulseIn (PIN_Aux_in, HIGH, 20000); //read RC channel
			int_detached = true;
			failsafe = ( ( (ModeIn_aux > 1400 ) && (ModeIn_aux < 1610)) || (ModeIn_aux < 950) ); // 1553
			if (!failsafe) {
				if ( (ModeIn_gyr < 1500) && (ModeIn_aux < 1500) )  CurrMode = 1;
				if ( (ModeIn_gyr < 1500) && (ModeIn_aux > 1500) )  CurrMode = 2;
				if ( (ModeIn_gyr > 1500) && (ModeIn_aux < 1500) )  CurrMode = 3;
				if ( (ModeIn_gyr > 1500) && (ModeIn_aux > 1500) )  CurrMode = 4;
			}
		} 



		if ( (cnt == 1) || (cnt == (1 + 10)) ) read_from_APM();
		if ( (cnt == 3) || (cnt == (3 + 10)) ) parse_from_APM();
		if ( (cnt == 5) || (cnt == (5 + 10)) ) pack_msg_for_APM();
		if ( (cnt == 7) || (cnt == (7 + 10)) ) write_to_APM();
		if (cnt >= 20) cnt = 0;
	} // 50ms = 20Hz

	RPM_reattach();

	if ( (curr_millis - lastTime1s) > 1000 ) {  // 



		//Serial.print (";  RPM: ");
		//Serial.printlm (rpm);
	    //Serial.print (";  dRot: ");
	    //Serial.print (dRot);
	    //Serial.print (";  ms: ");
	    //Serial.println ((curr_millis - lastTime1s));

		lastTime1s = curr_millis;
	}


	if (apm_armed) {
		if ((SW1) && ( (CurrMode != NoLight_FlightMode) || (SW2)) ) {                        // flying with light
			if (alt_over_home < ALT_front_light_On) {
				if ((CurrMode == FrontLight_FlightMode)) {
					analogWrite(PIN_FrontLight, FrontLight_brightness);
				} else { digitalWrite(PIN_FrontLight, LOW); }
			} 
			if (alt_over_home > ALT_front_light_Off) digitalWrite(PIN_FrontLight, LOW); 

			if (alt_by_sonar < ALT_landing_light_On) digitalWrite(PIN_Light2, HIGH);
			if (alt_by_sonar > ALT_landing_light_Off) digitalWrite(PIN_Light2, LOW);

		} else {
			digitalWrite(PIN_Light2, LOW);
			digitalWrite(PIN_FrontLight, LOW);
		}
	} else {
		digitalWrite(PIN_Light2, LOW);
		digitalWrite(PIN_FrontLight, LOW);
	}

	// navigation light
	if ( (apm_armed) && (apm_home_set) && (SW1) ) digitalWrite(PIN_Light, HIGH);
	if ( (apm_armed) && (apm_home_set) && (!SW1) ) digitalWrite(PIN_Light, LOW);
	if ( (apm_armed) && (!apm_home_set)  )  blink_led (100, 380);
	if ( (!apm_armed) && (apm_home_set)  )  blink_led (60, 1500);
	if ( (!apm_armed) && (!apm_home_set) )  blink_led (600, 2000);


	ModeServoOut = map(CurrMode, 1, 6, 1166, 1814); 
	ModeOut.writeMicroseconds(ModeServoOut);

	// delay(1);     
}




