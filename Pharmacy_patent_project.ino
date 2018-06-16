

#include <PID_v1.h>
#include <Keypad.h>
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>

#define Motor_1_a 12
#define Motor_1_b 11
#define Motor_2_a 10
#define Motor_2_b 9
#define Motor_1_Enable 8
#define Motor_2_Enable 7
#define Motor_1_Encoder_A 3
#define Motor_1_Encoder_B 2
#define Motor_2_Encoder_A 19
#define Motor_2_Encoder_B 18
#define LM35 A2
#define Heater 5
#define buzzer 53

int Reading = 0;
double Temperature = 0.00;

unsigned long count = 0;

const byte ROWS = 4; //four rows
const byte COLS = 4; //three columns

unsigned int InputSpeed = 0;
double SetTemperature = 0.000;

String InputKeypad = "";
String InputKeypadTemp = "";

boolean EnableKey = false;  // Key to start the entering of the speed.
boolean Enter = false;

boolean EnableTemp = false;
boolean EnterTemp = false;

unsigned long lastMilli = 0;
unsigned long lastMilliPrint = 0;

volatile long count_1 = 0;

int speed_actual_1 = 0;

volatile long count_2 = 0;

int speed_actual_2 = 0;

int PWM_1 = 0;
int PWM_2 = 0;

float Kp = 0.2;                  // For motor 
float Kd = 0.03;
float Ki = 0.02;

boolean run = false;

char keys[ROWS][COLS] = {
{ '1','2','3', 'A' }, // A is Start
{ '4','5','6', 'B' }, // B is Stop
{ '7','8','9', 'C' }, // C is Enable Key
{ '*','0','.', 'D' }  
};
byte rowPins[ROWS] = { A8, A9, A10, A11 }; //connect to the row pinouts of the keypad
byte colPins[COLS] = { A12, A13, A14, A15 }; //connect to the column pinouts of the keypad

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

LiquidCrystal_PCF8574 lcd(0x3F);  //lcd Intialization with 0x3F Address


void setup()
{
	pinMode(Motor_1_a, OUTPUT);
	pinMode(Motor_1_b, OUTPUT);
	pinMode(Motor_2_a, OUTPUT);
	pinMode(Motor_2_b, OUTPUT);
	pinMode(Motor_1_Enable, OUTPUT);
	pinMode(Motor_2_Enable, OUTPUT);
	pinMode(Motor_1_Encoder_A, INPUT);
	pinMode(Motor_1_Encoder_B, INPUT);
	pinMode(Motor_2_Encoder_A, INPUT);
	pinMode(Motor_2_Encoder_B, INPUT);
	pinMode(Heater, OUTPUT);
	pinMode(buzzer, OUTPUT);

	digitalWrite(buzzer, LOW);

	analogReference(INTERNAL1V1);

	digitalWrite(Motor_1_Encoder_A, HIGH);
	digitalWrite(Motor_1_Encoder_B, HIGH);

	digitalWrite(Motor_2_Encoder_A, HIGH);
	digitalWrite(Motor_2_Encoder_B, HIGH);


	attachInterrupt(digitalPinToInterrupt(3), rencoder_1, FALLING);

	attachInterrupt(digitalPinToInterrupt(19), rencoder_2, FALLING);

	HeaterOff();

	Wire.begin();

	Wire.beginTransmission(0x3F);

	Serial.begin(9600);

	lcd.begin(16, 2);

	welcomeMessage();

	keypad.addEventListener(keypadEvent);
}

void loop()
{
	digitalWrite(buzzer, LOW);

	char key = keypad.getKey();

	count++;

	DataDisplay();

	ReadTemperature();

	if (EnableKey)
	{

		int Stringlength = 0;
		int i = 4;
		lcd.home();
		lcd.clear();
		lcd.print("Enter Speed");
		Enter = false;

		while (Enter == false)
		{
			key = keypad.getKey();
			if (key)
			{
				if (key != 'D')
				{
					if (key == 'A')
					{
						InputKeypad.remove(InputKeypad.length() - 1);
						i = i - 1;
						lcd.setCursor(i, 1);
						lcd.print(" ");
					}
					if (key != 'A')
					{
						InputKeypad = InputKeypad + (char)key;
						Stringlength = InputKeypad.length();
						lcd.setCursor(i, 1);
						lcd.print(key);
						i++;
					}
				}
				else
				{
					Enter = true;
				}
			}
		}

		EnableKey = false;

		InputSpeed = InputKeypad.toInt();

		//Serial.println(InputKeypad); uncomment for debugging
		//Serial.println(InputSpeed);

		InputKeypad = "";
		lcd.home();
		lcd.clear();
	}
	if (EnableTemp)
	{

		int Stringlength = 0;
		int i = 4;
		lcd.home();
		lcd.clear();
		lcd.print("Enter Temperature");
		EnterTemp = false;

		while (EnterTemp == false)
		{
			key = keypad.getKey();
			if (key)
			{
				if (key != 'D')
				{
					if (key == 'A')
					{
						InputKeypadTemp.remove(InputKeypadTemp.length() - 1);
						i = i - 1;
						lcd.setCursor(i, 1);
						lcd.print(" ");
					}
					if (key != 'A')
					{
						InputKeypadTemp = InputKeypadTemp + (char)key;
						Stringlength = InputKeypadTemp.length();
						lcd.setCursor(i, 1);
						lcd.print(key);
						i++;
					}
				}
				else
				{
					EnterTemp = true;
				}
			}
		}

		 EnableTemp = false;
		 SetTemperature = InputKeypadTemp.toDouble();

		//Serial.println(InputKeypadTemp); uncomment for debugging
		//Serial.println(SetTemperature);

		InputKeypadTemp = "";
		lcd.home();
		lcd.clear();
	}
	Motor_run();
	HeaterOn();
}
void keypadEvent(KeypadEvent key)
{
	switch (keypad.getState())
	{
	case PRESSED:
		if (key == 'C')
		{
			EnableKey = true;
			beep(0.1);
		}
		if (key == '*')
		{
			EnableTemp = true;
			beep(0.1);
		}
		break;

	case HOLD:
		if (key == 'A')
		{
			start();
			beep(0.1);
		}
		if (key == 'B')
		{
			stop();
			beep(0.1);
		}
		break;
	}
}
void start()
{
	run = true;
}
void stop()
{
	run = false;
}

void welcomeMessage()
{
	digitalWrite(buzzer, HIGH);
	lcd.setBacklight(255);
	lcd.home();
	lcd.setCursor(4, 0);
	lcd.print(" WELCOME");
	lcd.setCursor(0, 1);
	lcd.print("  HARDIQ VERMA");
	delay(2500);
	lcd.clear();
}

void rencoder_1()
{
	if (digitalRead(Motor_1_Encoder_B) == LOW)   count_1++;
	if (digitalRead(Motor_1_Encoder_B) == HIGH)   count_1--;
}
void rencoder_2()
{
	if (digitalRead(Motor_2_Encoder_B) == LOW)   count_2++;
	if (digitalRead(Motor_2_Encoder_B) == HIGH)   count_2--;
}
void getMotorData_1()
{
	static long countAnt = 0;
	speed_actual_1 = ((count_1 - countAnt)*(60 * (1000 / 100))) / (358);
	countAnt = count_1;
}
void getMotorData_2()
{
	static long countAnt = 0;
	speed_actual_2 = ((count_2 - countAnt)*(60 * (1000 / 100))) / (358);
	countAnt = count_2;
}
int updatePid_1(int command, int targetValue, int currentValue)
{
	float pidTerm = 0;
	int error = 0;
	static int last_error = 0;
	static int error_sum = 0;

	error = abs(targetValue) - abs(currentValue);

	error_sum = error_sum + (Ki* error);

	if (error_sum > 255)
	{
		error_sum = 255;
	}
	else if (error_sum < 0)
	{
		error_sum = 0;
	}

	pidTerm = (Kp * error) + (Kd * (error - last_error)) + error_sum;
	last_error = error;
	return constrain(command + int(pidTerm), 0, 255);
}
int updatePid_2(int command, int targetValue, int currentValue)
{
	float pidTerm = 0;
	int error = 0;
	static int last_error = 0;
	static int error_sum = 0;

	error = abs(targetValue) - abs(currentValue);

	error_sum = error_sum + (Ki* error);

	if (error_sum > 255)
	{
		error_sum = 255;
	}
	else if (error_sum < 0)
	{
		error_sum = 0;
	}

	pidTerm = (Kp * error) + (Kd * (error - last_error)) + error_sum;
	last_error = error;
	return constrain(command + int(pidTerm), 0, 255);
}

void printMotorInfo()
{
	if ((millis() - lastMilliPrint) >= 500)
	{
		lastMilliPrint = millis();
		/*Serial.print(InputSpeed);
		Serial.print("\t");
		Serial.print(speed_actual_1);
		Serial.print("\t");
		Serial.println(speed_actual_2);*/
	}
}

void Motor_run()
{
	if (run)
	{
		if ((millis() - lastMilli) >= 100)
		{
			lastMilli = millis();
			getMotorData_1();
			PWM_1 = updatePid_1(PWM_1, InputSpeed, speed_actual_1);

			analogWrite(Motor_1_Enable, PWM_1);
			digitalWrite(Motor_1_a, HIGH);
			digitalWrite(Motor_1_b, LOW);

			getMotorData_2();

			PWM_2 = updatePid_2(PWM_2, InputSpeed, speed_actual_2);

			analogWrite(Motor_2_Enable, PWM_2);
			digitalWrite(Motor_2_a, HIGH);
			digitalWrite(Motor_2_b, LOW);
		}
		printMotorInfo();

	}
	else
	{
		analogWrite(Motor_1_Enable, 0);
		digitalWrite(Motor_1_a, LOW);
		digitalWrite(Motor_1_b, LOW);

		analogWrite(Motor_2_Enable, 0);
		digitalWrite(Motor_2_a, LOW);
		digitalWrite(Motor_2_b, LOW);
	}
}

void ReadTemperature()
{
	for (int i = 0; i < 15; i++)
	{
		Reading = analogRead(LM35);
		Temperature = Temperature + (float)Reading / 9.31;
	}
	Temperature = Temperature / 15.0;
	//Serial.println(Temperature);
}
void DataDisplay()
{
	if (count > 500)
	{
		lcd.setBacklight(255);
		lcd.home();
		lcd.print("SPEED");
		if (speed_actual_2 < 100)
		{
			lcd.setCursor(12, 0);
			lcd.print(speed_actual_2);
			lcd.setCursor(14, 0);
			lcd.print("  ");
		}
		else
		{
			lcd.setCursor(12, 0);
			lcd.print(speed_actual_2);
		}
		lcd.setCursor(0, 1);
		lcd.print("TEMPERATURE");
		lcd.setCursor(12, 1);
		lcd.print(Temperature);
		count = 0;
	}
}
void HeaterOff()
{
	digitalWrite(Heater, LOW);
}
void HeaterOn()
{
	if (Temperature < SetTemperature)
	{
		digitalWrite(Heater, HIGH);
	}
	else
	{
		digitalWrite(Heater, LOW);
	}
}
void beep(float time)
{
	digitalWrite(buzzer, HIGH);
	delay(time*1000);
	digitalWrite(buzzer, LOW);
}



