/*
*	SMARTBUS (Proyektnaya deyatel'nost') - 181-311
*	Steering motor controller
*	__________________
*
*	© 2019, Neshumov Pavel Evgenyevich
*	(Нешумов Павел Евгеньевич)
*
*	All code was written by Neshumov Pavel Evgenyevich
*/


/* ----- SETUP ----- */
//#define DEBUG					//Uncomment if you want to recieve debug messages by Serial
#define AccelDecelRange 20		//Maximum range for Acceleration or Deceleration
#define StartDelay		20		//Maximum (starting) delay between steps, in ms
#define MinDelay		10		//Minimum (running)  delay between steps, in ms
#define	DelayPhasesDT	10		//Dead-time between phases, in us
#define MaxPWM			25		//Maximum power (8bit PWM value) 0-255
#define MinPWM			10		//Minimum power (8bit PWM value) 0-255
#define startPWM		2		//Starting power for set zero point 0-255
#define timeout			3000	//Timeout for moving, in ms
#define servoChecking	2000	//Delay between servo checks, in ms
#define minAngle		-70		//Minimum angle for Serial
#define maxAngle		70		//Maximum angle for Serial
#define freeState		1		//0 means all coils are shorted, 1 means all coils are free

/* ----- Enable / disable timers ----- */
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

/* ----- SSI absolute encoder ----- */
#define SSI_CLK_PORT PORTC		//PORT for clock (encoder)
#define SSI_DTA_PORT PINC		//PIN for data (encoder)
#define SSI_CLK_BIT		3		//Number of pin clock (encoder)
#define SSI_DTA_BIT		2		//Number of pin data (encoder)
#define CLK_ARDUINO_PIN A3		//Arduino IDE clock pin (encoder)
#define DTA_ARDUINO_PIN A2		//Arduino IDE data pin (encoder)

/* ----- System vars ----- */
long angle = 0, angleStated = 0;
unsigned long servoTimer = 0;
boolean servoMode = true;
boolean clockwise = false;
byte speed = MinPWM;
byte bldc_step = 0;

void setup() {

	DDRD |= 0b00111000;	//3 - AL, 4 - BL, 5 - CL
	DDRB |= 0b00001110;	//9 - AH, 10 - BH, 11 - CH 
	pinMode(CLK_ARDUINO_PIN, OUTPUT);
	pinMode(DTA_ARDUINO_PIN, INPUT_PULLUP);

	PORTD = 0x00;
	PORTB = 0x00;

#ifdef DEBUG
	Serial.begin(115200);
#else
	Serial.begin(9600);
#endif // DEBUG

	readAngle();
	delay(2000);
	readAngle();
	delay(200);

	/*while (true)		// Use this to calibrate encoder
	{
	readAngle();
	delay(200);
	Serial.println(angle);
	}*/

	bldc_step = 0;
	bldc_movePWM(startPWM);
	delay(250);

#if freeState == 0
	ALL_LO();
#else
	ALL_UNDEF();
#endif


#ifdef DEBUG
	Serial.println(angle);
#endif // DEBUG

	runToAngle(0);

#ifdef DEBUG
	Serial.println(map((abs(angle) % 4096), 0, 4095, 0, 47) % 6);
#endif // DEBUG
}

void loop() {
	serialAction();

	if (servoMode && millis() - servoTimer >= servoChecking) {
		readAngle();
		if (abs(angleStated - (angle / 100)) > 2) {
			if (!runToAngle(angleStated))
				servoMode = false;
		}
		servoTimer = millis();
	}
}

/* -----   Serial communication   ----- */
void serialAction() {
	if (Serial.available() > 0) {
		String incoming = Serial.readStringUntil('\n');

		if (incoming.startsWith("S1")) {

			incoming.remove(0, 2);

			if (incoming.startsWith(" A")) {				//Angle setup
				int setTo = SerialCommandConverter(incoming);

				if (setTo >= minAngle && setTo <= maxAngle) {

					if (runToAngle(setTo)) {
						angleStated = setTo;
						Serial.println("S1 S1");			//OK
					}
					else {
						readAngle();
						angleStated = angle;
						Serial.println("S1 E3");			//Timed out
					}
				}
				else
					Serial.println("S1 E0");				//Reading error
			}
			else if (incoming.startsWith(" S")) {			//Servo mode
				uint8_t command = SerialCommandConverter(incoming);

				if (command == 0)
					servoMode = false;
				else if (command == 1)
					servoMode = true;
				else
					Serial.println("S1 E0");				//Reading error

			}
			else if (incoming.startsWith(" R")) {			//Servo mode
				uint8_t command = SerialCommandConverter(incoming);

				if (command == 1)
					runBySerial(1);
				else if (command == 2)
					runBySerial(0);
				else if (command > 2)
					Serial.println("S1 E0");				//Reading error

			}
			else
				Serial.println("S1 E0");					//Reading error
		}
	}
}

/* -----   Serial G-code String to int   ----- */
int SerialCommandConverter(String incoming) { //Command converter
	incoming.remove(0, 2);
	return incoming.toInt();
}

/* -----   "map" for float   ----- */
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* -----   BLDC commutation function   ----- */
void bldc_movePWM(byte pwm) {
	//ALL_UNDEF();
	cbi(TCCR1A, COM1B1);		// Disable timers
	cbi(TCCR1A, COM1A1);
	cbi(TCCR2A, COM2A1);
	delayMicroseconds(DelayPhasesDT);
	if (!clockwise) {
		switch (bldc_step) {
		case 0:
			AH_BL_PWM(pwm);
			break;
		case 1:
			AH_CL_PWM(pwm);
			break;
		case 2:
			BH_CL_PWM(pwm);
			break;
		case 3:
			BH_AL_PWM(pwm);
			break;
		case 4:
			CH_AL_PWM(pwm);
			break;
		case 5:
			CH_BL_PWM(pwm);
			break;
		}
	}
	else {
		switch (bldc_step) {
		case 0:
			BH_AL_PWM(pwm);
			break;
		case 1:
			BH_CL_PWM(pwm);
			break;
		case 2:
			AH_CL_PWM(pwm);
			break;
		case 3:
			AH_BL_PWM(pwm);

			break;
		case 4:
			CH_BL_PWM(pwm);
			break;
		case 5:
			CH_AL_PWM(pwm);
			break;
		}
	}
}

/* -----  Start moving by serial  ----- */
void runBySerial(boolean clw) {
	clockwise = clw;
	readAngle();
	int angleShort = angle / 100;

	int sbstr;
	if (clw)
		sbstr = minAngle;
	else
		sbstr = maxAngle;

	int distance = abs(sbstr - angleShort);

	if (distance > 0) {
		int beginDistance = distance;

#ifdef DEBUG
		Serial.print("Distance: ");
		Serial.println(distance);
#endif // DEBUG

		int acceleration = (distance / 4) * 3;
		if (distance - acceleration > AccelDecelRange)
			acceleration = distance - AccelDecelRange;

		String incoming = "";
		byte Ddelay = StartDelay;
		unsigned long Timer = millis();
		boolean decel = false;
		int decelDistance = 0;

		while (distance > 2) {
			readAngle();
			angleShort = angle / 100;
			//if (!decel)
			distance = abs(sbstr - angleShort);

			if (millis() - Timer >= Ddelay) {
				bldc_step++;
				bldc_step %= 6;
				bldc_movePWM(speed);

				if (Serial.available())
					incoming = Serial.readStringUntil('\n');

				if (incoming.startsWith("S1 R0") && distance > AccelDecelRange) {
					//decel = true;
					if (clw)
						sbstr = angleShort - AccelDecelRange;
					else
						sbstr = angleShort + AccelDecelRange;
				}

				if (distance >= acceleration)
					Ddelay = map(distance, beginDistance, acceleration, StartDelay, MinDelay);

				else if (distance <= AccelDecelRange)
					Ddelay = map(distance, AccelDecelRange, 0, MinDelay, StartDelay);

				speed = map(Ddelay, MinDelay, StartDelay, MaxPWM, MinPWM);
				Timer = millis();
			}


		}
	}

#if freeState == 0
	ALL_LO();
#else
	ALL_UNDEF();
#endif

	readAngle();
	angleStated = angle / 100;
}

/* -----  Start moving to angle   ----- */
boolean runToAngle(int setAngle) {
	readAngle();

	int angleShort = angle / 100;
	int distance = abs(setAngle - angleShort);

	if (distance > 0) {					// if angleShort != setAngle
		int beginDistance = distance;

		int deceleration = distance / 4;
		if (deceleration > AccelDecelRange)
			deceleration = AccelDecelRange;

		int acceleration = (distance / 4) * 3;
		if (distance - acceleration > AccelDecelRange)
			acceleration = distance - AccelDecelRange;

		if (setAngle > angleShort)
			clockwise = false;
		else
			clockwise = true;

		byte Ddelay = StartDelay;

#ifdef DEBUG
		Serial.print("Set to: ");
		Serial.print(setAngle);
		Serial.print(" | From: ");
		Serial.print(angleShort);
		Serial.print(" | Distance: ");
		Serial.print(distance);
		Serial.print(" | Clockwise: ");
		Serial.println(clockwise);
		Serial.print("deceleration: ");
		Serial.print(deceleration);
		Serial.print(" acceleration: ");
		Serial.println(acceleration);
#endif // DEBUG

		unsigned long timeOutTimer = millis();
		unsigned long Timer = millis();

		while (millis() - timeOutTimer <= timeout &&
			((!clockwise && angleShort < setAngle)
				|| (clockwise && angleShort > setAngle))) {	//While angleShort != setAngle

			readAngle();
			angleShort = angle / 100;
			distance = abs(setAngle - angleShort);

			if (millis() - Timer >= Ddelay) {
				bldc_step++;
				bldc_step %= 6;
				bldc_movePWM(speed);
#ifdef DEBUG
				Serial.print(" Distance: ");
				Serial.print(distance);
				Serial.print(" beginDistance: ");
				Serial.print(beginDistance);
				Serial.print(" deceleration: ");
				Serial.print(deceleration);
				Serial.print(" acceleration: ");
				Serial.print(acceleration);
#endif // DEBUG

				if (distance >= acceleration)
					Ddelay = map(distance, beginDistance, acceleration, StartDelay, MinDelay);

				else if (distance <= deceleration)
					Ddelay = map(distance, deceleration, 0, MinDelay, StartDelay);

				speed = map(Ddelay, MinDelay, StartDelay, MaxPWM, MinPWM);

				Timer = millis();

#ifdef DEBUG
				Serial.print(" delay: ");
				Serial.println(Ddelay);
#endif // DEBUG
			}
		}

#if freeState == 0
		ALL_LO();
#else
		ALL_UNDEF();
#endif
		if (millis() - timeOutTimer >= timeout)
			return 0;
		else
			return 1;
	}
	else
		return 1;
}

/* -----   Phases commutation   ----- */
void AH_BL_PWM(byte pwm) {
	cbi(TCCR2A, COM2A1);
	cbi(TCCR1A, COM1B1);

	PORTD = 0b00010000; //PD3 - AL, PD4 - BL, PD5 - CL

	OCR1A = pwm;
	sbi(TCCR1A, COM1A1);
}
void AH_CL_PWM(byte pwm) {
	cbi(TCCR2A, COM2A1);
	cbi(TCCR1A, COM1B1);

	PORTD = 0b00100000; //PD3 - AL, PD4 - BL, PD5 - CL

	OCR1A = pwm;
	sbi(TCCR1A, COM1A1);
}
void BH_CL_PWM(byte pwm) {
	cbi(TCCR2A, COM2A1);
	cbi(TCCR1A, COM1A1);

	PORTD = 0b00100000; //PD3 - AL, PD4 - BL, PD5 - CL

	OCR1B = pwm;
	sbi(TCCR1A, COM1B1);
}
void BH_AL_PWM(byte pwm) {
	cbi(TCCR2A, COM2A1);
	cbi(TCCR1A, COM1A1);

	PORTD = 0b00001000; //PD3 - AL, PD4 - BL, PD5 - CL

	OCR1B = pwm;
	sbi(TCCR1A, COM1B1);
}
void CH_AL_PWM(byte pwm) {
	cbi(TCCR1A, COM1B1);
	cbi(TCCR1A, COM1A1);

	PORTD = 0b00001000; //PD3 - AL, PD4 - BL, PD5 - CL

	OCR2A = pwm;
	sbi(TCCR2A, COM2A1);
}
void CH_BL_PWM(byte pwm) {
	cbi(TCCR1A, COM1B1);
	cbi(TCCR1A, COM1A1);

	PORTD = 0b00010000; // PD3 - AL, PD4 - BL, PD5 - CL

	OCR2A = pwm;
	sbi(TCCR2A, COM2A1);
}
void ALL_LO() {			// Connect all wires to ground (brake)
	PORTD = 0b00000000; //PD3 - AL, PD4 - BL, PD5 - CL

	cbi(TCCR1A, COM1B1);
	cbi(TCCR1A, COM1A1);
	cbi(TCCR2A, COM2A1);

	delayMicroseconds(10);

	PORTB = 0b00000000; //PB1 - AH, PB2 - BH, PB3 - CH 
	PORTD = 0b00111000; //PD3 - AL, PD4 - BL, PD5 - CL

}
void ALL_UNDEF() {		// Free mooving (all wires are not connected)
	PORTD = 0b00000000; //PD3 - AL, PD4 - BL, PD5 - CL

	cbi(TCCR1A, COM1B1);
	cbi(TCCR1A, COM1A1);
	cbi(TCCR2A, COM2A1);

	delayMicroseconds(10);

	PORTB = 0b00000000; //PB1 - AH, PB2 - BH, PB3 - CH 
	PORTD = 0b00000000; //PD3 - AL, PD4 - BL, PD5 - CL

}

/* -----   Encoder reading   ----- */
void readAngle() {
	//cli();

	delayMicroseconds(10);

	angle = (GrayToBinary32(EncoderReadSSI()));

	if (angle >= 16777216 && angle < 25165823)
		angle -= 16777216;
	else
		angle -= 33554431;

	//sei();
}

/* -----   Gray code conversion   ----- */
uint32_t GrayToBinary32(uint32_t gray)
{
	gray = gray ^ (gray >> 16);
	gray = gray ^ (gray >> 8);
	gray = gray ^ (gray >> 4);
	gray = gray ^ (gray >> 2);
	gray = gray ^ (gray >> 1);
	return gray;
}

/* ----- Read Gray code from encoder  ----- */
uint32_t EncoderReadSSI()
{
	uint8_t  bit_count;
	uint32_t result = 0;
	uint8_t  data8;

	for (bit_count = 0; bit_count < 25; bit_count++)
	{
		SSI_CLK_PORT &= ~(1 << SSI_CLK_BIT);
		result = (result << 1);
		data8 = SSI_DTA_PORT;
		SSI_CLK_PORT |= (1 << SSI_CLK_BIT);
		if ((data8 & (1 << SSI_DTA_BIT)) != 0)
			result = result | 0x01;
	}
	return result;
}
