#include <Servo.h>


#define PIN_OnboardLED 13
#define PIN_RPM_in 3
#define PIN_RPM_int 1 // int1

// #define PIN_Hold_Thr 4
#define PIN_RPM_analog_out 5

#define PIN_Light 6  // PWM fähig
// #define PIN_Thr_in 7
// #define PIN_Thr_out 8

#define PIN_Mode_out 9
#define PIN_Gyr_in 10
#define PIN_Aux_in 11

#define Thr_min 600
#define Thr_low 1200

#define RPM_Divisions 1   // Unterteilungen am Abnehmer

unsigned int ThrRampStart = 2;  // Servo us/ 50ms      us/s:20
unsigned int ThrRamp = 7;  // Servo us/ 50ms  
unsigned int ModeSigLost = 5;  // recv lost, sw to flight mode ..
unsigned int HoldAirTime = 30;  // sek bis 
boolean HoldThrOnLoss = false;  // Motor läuft weiter, wenn die verbindung abbricht

boolean failsafe = false;
unsigned long ModeIn_aux = 0;
unsigned long ModeIn_gyr = 0;
unsigned int CurrMode = 1;
unsigned int ModeServoOut = 0;
unsigned int ReadThr = 0;
unsigned int CurrThr = 0;
unsigned int LastThr = 0;
unsigned int rpm_analog = 0;
unsigned long lastTime50ms = 0;
unsigned long lastTime1s = 0;
unsigned long curr_millis = 0;
unsigned int cntLostSeconds = 0;
unsigned long rpm = 0;
volatile unsigned int dRot = 0;

Servo ModeOut;
//Servo ThrottleOut;




void updateRPM()
{
	dRot++;
}

void setup()
{
	Serial.begin(19200);
	pinMode(PIN_RPM_in, INPUT);
	//pinMode(PIN_Thr_out, OUTPUT);
	pinMode(PIN_RPM_analog_out, OUTPUT);
	pinMode(PIN_Mode_out, OUTPUT);
	pinMode(PIN_Light, OUTPUT);
	//pinMode(PIN_Hold_Thr, INPUT_PULLUP);
	//pinMode(PIN_Thr_in, INPUT);
	pinMode(PIN_Gyr_in, INPUT);
	pinMode(PIN_Aux_in, INPUT);

	ModeOut.attach(PIN_Mode_out, 1000, 2000);
//	ThrottleOut.attach(PIN_Thr_out, 1000, 2000);
	attachInterrupt(PIN_RPM_int, updateRPM, FALLING);

	delay(100);
	//if (digitalRead(PIN_Hold_Thr) == LOW) HoldThrOnLoss = true;

	Serial.println("Ready");
//	Serial.print ("HoldThrOnLoss: ");
//	Serial.print (HoldThrOnLoss);
	Serial.println("");

}


void loop()
{
	curr_millis = millis();

	if ( (curr_millis - lastTime50ms) > 50 ) { // 50ms
		lastTime50ms = curr_millis;

		ModeIn_gyr = pulseIn (PIN_Gyr_in, HIGH, 20000); //read RC channel
		ModeIn_aux = pulseIn (PIN_Aux_in, HIGH, 20000); //read RC channel
		failsafe = ( (ModeIn_aux > 1490 ) && (ModeIn_aux < 1610)); // 1553

	/*	ReadThr = pulseIn (PIN_Thr_in, HIGH, 20000);    //read RC channel

		if (( ReadThr > Thr_min ) || ( !HoldThrOnLoss ))   // signal not lost + thr not holt
		{
			if ( (ReadThr > CurrThr) && ((ReadThr - CurrThr) > ThrRamp) ) {
				if (( LastThr < Thr_min ) && ( CurrThr < Thr_min ) ) CurrThr = Thr_min;  // signal back
				if (( LastThr > Thr_min ) && ( LastThr < Thr_low )) {
					CurrThr = CurrThr + ThrRampStart;   // startup
				} else {
					CurrThr = CurrThr + ThrRamp;
				}
			} else {
				CurrThr = ReadThr;
				LastThr = ReadThr;
			}
		}
		*/
	} // 50ms
	
	if ( (curr_millis - lastTime1s) > 1000 ) {  // 
	//	Serial.print ("  dRot: ");
	//	Serial.print (dRot);
		rpm = dRot;
		rpm = rpm * 1000 * 60;
		rpm = (rpm / (curr_millis - lastTime1s)) / RPM_Divisions;
		dRot = 0;
		lastTime1s = curr_millis;
		if (ReadThr < Thr_min) cntLostSeconds++;  // no signal
		if (ReadThr > Thr_min) cntLostSeconds=0;

		Serial.print ("Mode: ");
		Serial.print (CurrMode);
		Serial.print (";  RPM: ");
		Serial.print (rpm);
		Serial.println("");
	}

/*	if (cntLostSeconds > HoldAirTime) {   // signal zu lang weg   
		cntLostSeconds = 0;
		LastThr = 0;
		CurrThr = 0;
	}
*/
	if (!failsafe) {
		if ( (ModeIn_gyr < 1500) && (ModeIn_aux < 1500) )  CurrMode = 1;
		if ( (ModeIn_gyr < 1500) && (ModeIn_aux > 1500) )  CurrMode = 2;
		if ( (ModeIn_gyr > 1500) && (ModeIn_aux < 1500) )  CurrMode = 3;
		if ( (ModeIn_gyr > 1500) && (ModeIn_aux > 1500) )  CurrMode = 4;
	}
//	if ( ReadThr < Thr_min ) CurrMode = ModeSigLost;

	// mappings
	ModeServoOut = map(CurrMode, 1, 6, 1166, 1814); 
	rpm_analog = map(rpm, 0, 3000, 0, 255);
	// Ausgaben
//	ThrottleOut.writeMicroseconds(CurrThr);
	ModeOut.writeMicroseconds(ModeServoOut);
	analogWrite(PIN_RPM_analog_out, rpm_analog);

	delay(10);     
}




/*



void updateRPM()
{
	dTime = millis()-time;
	if (dTime > 5) {
		time = time + dTime;
		rpm = ((1000 * 60) / dTime);
	}
}

void setup()
{
	Serial.begin(19200);
	pinMode(PIN_RPM_in, INPUT);
	//pinMode(PIN_Thr_out, OUTPUT);
	pinMode(PIN_RPM_analog_out, OUTPUT);
	pinMode(PIN_Mode_out, OUTPUT);
	pinMode(PIN_Light, OUTPUT);
	//pinMode(PIN_Hold_Thr, INPUT_PULLUP);
	//pinMode(PIN_Thr_in, INPUT);
	pinMode(PIN_Gyr_in, INPUT);
	pinMode(PIN_Aux_in, INPUT);

	ModeOut.attach(PIN_Mode_out, 1000, 2000);
//	ThrottleOut.attach(PIN_Thr_out, 1000, 2000);
	attachInterrupt(PIN_RPM_int, updateRPM, FALLING);

	delay(100);
	//if (digitalRead(PIN_Hold_Thr) == LOW) HoldThrOnLoss = true;

	Serial.println("Ready");
//	Serial.print ("HoldThrOnLoss: ");
//	Serial.print (HoldThrOnLoss);
	Serial.println("");

}


void loop()
{
	curr_millis = millis();

	if ( (curr_millis - lastTime50ms) > 50 ) { // 50ms
		lastTime50ms = curr_millis;

		ModeIn_gyr = pulseIn (PIN_Gyr_in, HIGH, 20000); //read RC channel
		ModeIn_aux = pulseIn (PIN_Aux_in, HIGH, 20000); //read RC channel
		failsafe = ( (ModeIn_aux > 1490 ) && (ModeIn_aux < 1610)); // 1553

		ReadThr = pulseIn (PIN_Thr_in, HIGH, 20000);    //read RC channel

		if (( ReadThr > Thr_min ) || ( !HoldThrOnLoss ))   // signal not lost + thr not holt
		{
			if ( (ReadThr > CurrThr) && ((ReadThr - CurrThr) > ThrRamp) ) {
				if (( LastThr < Thr_min ) && ( CurrThr < Thr_min ) ) CurrThr = Thr_min;  // signal back
				if (( LastThr > Thr_min ) && ( LastThr < Thr_low )) {
					CurrThr = CurrThr + ThrRampStart;   // startup
				} else {
					CurrThr = CurrThr + ThrRamp;
				}
			} else {
				CurrThr = ReadThr;
				LastThr = ReadThr;
			}
		}
		
	} // 50ms
	
	if ( (curr_millis - lastTime1s) > 1000 ) {  // 
		lastTime1s = curr_millis;
		if (ReadThr < Thr_min) cntLostSeconds++;  // no signal
		if (ReadThr > Thr_min) cntLostSeconds=0;

		Serial.print ("Mode: ");
		Serial.print (CurrMode);
		Serial.print (";  RPM: ");
		Serial.print (rpm);
		Serial.print (";  dTime: ");
		Serial.print (dTime);
		Serial.println("");
	}

	if (cntLostSeconds > HoldAirTime) {   // signal zu lang weg   
		cntLostSeconds = 0;
		LastThr = 0;
		CurrThr = 0;
	}

	if (!failsafe) {
		if ( (ModeIn_gyr < 1500) && (ModeIn_aux < 1500) )  CurrMode = 1;
		if ( (ModeIn_gyr < 1500) && (ModeIn_aux > 1500) )  CurrMode = 2;
		if ( (ModeIn_gyr > 1500) && (ModeIn_aux < 1500) )  CurrMode = 3;
		if ( (ModeIn_gyr > 1500) && (ModeIn_aux > 1500) )  CurrMode = 4;
	}
//	if ( ReadThr < Thr_min ) CurrMode = ModeSigLost;

	// mappings
	ModeServoOut = map(CurrMode, 1, 6, 1166, 1814); 
	rpm_analog = map(rpm, 0, 3000, 0, 255);
	// Ausgaben
//	ThrottleOut.writeMicroseconds(CurrThr);
	ModeOut.writeMicroseconds(ModeServoOut);
	analogWrite(PIN_RPM_analog_out, rpm_analog);

	delay(10);     
}



*/