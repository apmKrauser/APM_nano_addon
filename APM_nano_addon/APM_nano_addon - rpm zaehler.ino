

//#include <FastSerial.h>
#include <Servo.h>
#include "APM_nano_addon.h"



#define PIN_OnboardLED 13
#define PIN_RPM_in 3
#define PIN_nextto_RPM 4
#define PIN_RPM_int 1 // int1

#define PIN_Light  6  // PWM fähig
#define PIN_Light2 7
#define PIN_FET3   8

#define PIN_Mode_out  9
#define PIN_Gyr_in   10
#define PIN_Aux_in   11

#define PIN_SW1   15
#define PIN_SW2   16


#define RPM_Divisions   1   // Unterteilungen am Abnehmer
#define RPM_lowpass_ms 16

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
volatile unsigned int dRot = 0;
boolean      failsafe = false;
boolean      apm_home_set = false;	
boolean      apm_armed = false;
boolean      SW1 = false;
boolean      SW2 = false;
boolean		 int_detached = false;
uint16_t     rpm = 0;
Servo		 ModeOut;
//Servo ThrottleOut;




void updateRPM()
{
	dRot++;
	detachInterrupt(PIN_RPM_int);
	int_detached = true;
}

void setup()
{
	pinMode(PIN_RPM_in, INPUT);
	pinMode(PIN_Mode_out, OUTPUT);
	pinMode(PIN_Light, OUTPUT);
	pinMode(PIN_Gyr_in, INPUT);
	pinMode(PIN_Aux_in, INPUT);
	pinMode(PIN_nextto_RPM, OUTPUT);

	digitalWrite(PIN_nextto_RPM, LOW);
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

	apm_home_set = (ap_bitflags & (1 << 0));
	apm_armed = (ap_bitflags & (1 << 1));
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
}

void pack_msg_for_APM()
{
	byte ap_bitflags = 0;

	msg_toAPM[0] = 0xFF;
	msg_toAPM[1] = (byte) (rpm & 0x00FF);
	msg_toAPM[2] = (byte) ((rpm & 0xFF00) >> 8);
	msg_toAPM[3] = ap_bitflags;
}

void blink_led (uint16_t led_on_time, uint16_t led_repeat)
{
	if ( (curr_millis - lastTimeLED) > led_on_time) {
		digitalWrite (PIN_Light2, LOW);
		digitalWrite (PIN_OnboardLED, LOW);
	}

	if ( (curr_millis - lastTimeLED) > led_repeat) {
		lastTimeLED = curr_millis;
		digitalWrite (PIN_Light2, HIGH);
		digitalWrite (PIN_OnboardLED, HIGH);
	}
}



void loop()
{
	static uint8_t cnt = 0;

	curr_millis = millis(); 

	if ( int_detached &&  ((curr_millis - lastTimeRPM) > RPM_lowpass_ms) )
	{
		 int_detached = false;
		 lastTimeRPM = curr_millis;
		 attachInterrupt(PIN_RPM_int, updateRPM, FALLING);
	}

	SW1 = digitalRead(PIN_SW1);
	SW2 = digitalRead(PIN_SW2);

	if ( (curr_millis - lastTime50ms) > 50 ) { // 50ms  = 20Hz
		lastTime50ms = curr_millis;
		cnt++;

		if (((cnt + 0) % 2) == 0) ModeIn_gyr = pulseIn (PIN_Gyr_in, HIGH, 20000); //read RC channel 20ms timeout
		if (((cnt + 1) % 2) == 0) ModeIn_aux = pulseIn (PIN_Aux_in, HIGH, 20000); //read RC channel
		failsafe = ( (ModeIn_aux > 1490 ) && (ModeIn_aux < 1610)); // 1553

		if ( (cnt == 1) || (cnt == (1 + 10)) ) read_from_APM();
		if ( (cnt == 3) || (cnt == (3 + 10)) ) parse_from_APM();
		if ( (cnt == 5) || (cnt == (5 + 10)) ) pack_msg_for_APM();
		if ( (cnt == 7) || (cnt == (7 + 10)) ) write_to_APM();
		if (cnt >= 20) cnt = 0;
	} // 50ms = 20Hz
	
	if ( int_detached &&  ((millis() - lastTimeRPM) > RPM_lowpass_ms) ) // 2.
	{
		 int_detached = false;
		 lastTimeRPM = curr_millis;
		 attachInterrupt(PIN_RPM_int, updateRPM, FALLING);
	}

	if ( (curr_millis - lastTime1s) > 1000 ) {  // 
		rpm = dRot;
		rpm = ( rpm  ) / RPM_Divisions; // 3900 RPM overflow
		rpm = (rpm * 1000)/ (curr_millis - lastTime1s) * 60;
		
		//Serial.print (";  RPM: ");
		//Serial.print (rpm);
	    //Serial.print (";  dRot: ");
	    //Serial.print (dRot);
	    //Serial.print (";  ms: ");
	    //Serial.println ((curr_millis - lastTime1s));

		dRot = 0;
		lastTime1s = curr_millis;
	}

	if (!failsafe) {
		if ( (ModeIn_gyr < 1500) && (ModeIn_aux < 1500) )  CurrMode = 1;
		if ( (ModeIn_gyr < 1500) && (ModeIn_aux > 1500) )  CurrMode = 2;
		if ( (ModeIn_gyr > 1500) && (ModeIn_aux < 1500) )  CurrMode = 3;
		if ( (ModeIn_gyr > 1500) && (ModeIn_aux > 1500) )  CurrMode = 4;
	}

	if ( (apm_armed) && (apm_home_set) && (SW1) )  { digitalWrite(PIN_Light, HIGH); digitalWrite(PIN_Light2, HIGH); }
	if ( (apm_armed) && (apm_home_set) && (!SW1) )  { digitalWrite(PIN_Light, LOW); digitalWrite(PIN_Light2, LOW); }
	if ( (apm_armed) && (!apm_home_set)  )  blink_led (100, 380);
	if ( (!apm_armed) && (apm_home_set)  )  blink_led (80, 1000);
	if ( (!apm_armed) && (!apm_home_set)  )  blink_led (600, 2000);


	ModeServoOut = map(CurrMode, 1, 6, 1166, 1814); 
	ModeOut.writeMicroseconds(ModeServoOut);

	// delay(1);     
}




