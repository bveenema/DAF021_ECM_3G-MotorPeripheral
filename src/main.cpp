/*
 * Project DAF021_ECM_3G-Argon
 * Description:
 * Author:
 */

#include "Particle.h"
#include "pins.h"
#include "AccelStepper.h"

AccelStepper BlueMotor(AccelStepper::DRIVER, BLUE_STEP_PIN, BLUE_DIR_PIN);
AccelStepper RedMotor(AccelStepper::DRIVER, RED_STEP_PIN, RED_DIR_PIN);


void setup();
void loop();
void receiveEvent(int howMany);
void requestEvent();

SYSTEM_MODE(MANUAL);

/// Register Definition
//	Register Qualitiies:
//		- Read/Write
//		- Registers can be flow through written, read only registers are skipped. Ex if start at register 0, and 4 bytes are sent, registers 0,1,3,4 are updated
struct Register
{
	char value; // the current value of the register
	bool rw; // 0: read-only, 1: Writable
	bool updated; // 1: was updated 
};
uint32_t rp = 0; // points to most recently receieved register
const uint32_t NumRegisters = 41;
Register Registers[NumRegisters] = 
{
	{0,true,false},		// Motor Enable Register
	{0,true,false},		// Motor Direction Register
	{0,true,false},		// Motor Move Register (1: if motor is to move, 0: if motor is to stop), Gets set to 0 if corresponding motor Enable register is set to 0, Gets set to 0 when move is complete
	{0,false,false},	// Motor Error Register (1: if error, 0: if no error)
	{0,true,false}, 	// Motor 1 Speed (1) High
	{0,true,false},		// Motor 1 Speed (2)
	{0,true,false},		// Motor 1 Speed (3)
	{0,true,false},		// Motor 1 Speed (4) Low
	{0,true,false},  	// Motor 1 Acceleration (1) High
	{0,true,false},  	// Motor 1 Acceleration (2)
	{0,true,false},  	// Motor 1 Acceleration (3)
	{0,true,false},  	// Motor 1 Acceleration (4) Low
	{0,true,false},		// Motor 1 Steps (1) High
	{0,true,false},		// Motor 1 Steps (2)
	{0,true,false},		// Motor 1 Steps (3)
	{0,true,false},		// Motor 1 Steps (4) Low
	{0,true,false}, 	// Motor 2 Speed (1) High
	{0,true,false},		// Motor 2 Speed (2)
	{0,true,false},		// Motor 2 Speed (3)
	{0,true,false},		// Motor 2 Speed (4) Low
	{0,true,false},  	// Motor 2 Acceleration (1) High
	{0,true,false},  	// Motor 2 Acceleration (2)
	{0,true,false},  	// Motor 2 Acceleration (3)
	{0,true,false},  	// Motor 2 Acceleration (4) Low
	{0,true,false},		// Motor 2 Steps (1) High
	{0,true,false},		// Motor 2 Steps (2)
	{0,true,false},		// Motor 2 Steps (3)
	{0,true,false},		// Motor 2 Steps (4) Low
	{0,true,false}, 	// Motor 3 Speed (1) High
	{0,true,false},		// Motor 3 Speed (2)
	{0,true,false},		// Motor 3 Speed (3)
	{0,true,false},		// Motor 3 Speed (4) Low
	{0,true,false},  	// Motor 3 Acceleration (1) High
	{0,true,false},  	// Motor 3 Acceleration (2)
	{0,true,false},  	// Motor 3 Acceleration (3)
	{0,true,false},  	// Motor 3 Acceleration (4) Low
	{0,true,false},		// Motor 2 Steps (1) High
	{0,true,false},		// Motor 2 Steps (2)
	{0,true,false},		// Motor 2 Steps (3)
	{0,true,false},		// Motor 2 Steps (4) Low
};

/// Flags
// Registers Updated Flag: set when any register is changed during an onReceive interrupt
bool FLAG_RegistersUpdated = false;

// setup() runs once, when the device is first turned on.
void setup()
{
	// Initialize I2C
	Wire.begin(4);                // join i2c bus with address #4
	Wire.onReceive(receiveEvent); // register event
	Wire.onRequest(requestEvent); // register request handler

	// Initialize Stepper Motors
	pinMode(BLUE_STATUS_PIN, INPUT);
	pinMode(BLUE_EN_PIN, OUTPUT);
	BlueMotor.setPinsInverted(0,0,1);

	pinMode(RED_STATUS_PIN, INPUT);
	pinMode(RED_EN_PIN, OUTPUT);
	digitalWrite(RED_EN_PIN, HIGH);
	RedMotor.setPinsInverted(0, 0, 1);

	BlueMotor.setAcceleration(10000);
	BlueMotor.setMaxSpeed(3850);

	RedMotor.setAcceleration(10000);
	RedMotor.setMaxSpeed(3850);

	BlueMotor.setCurrentPosition(0);
	BlueMotor.moveTo(8000);

	RedMotor.setCurrentPosition(0);
	RedMotor.moveTo(8000);


	Serial.begin(57000);           // start serial for output
}

// loop() runs over and over again, as quickly as it can execute.
void loop()
{
	// Test Stepper Motors
	static uint WaitTime = 0;
	if(millis() - WaitTime > 2000)
	{
		BlueMotor.run();
		RedMotor.run();
	}
		
	if(FLAG_RegistersUpdated)
	{
		// Print new register definition
		Serial.println("----------------\nRegisters:");
		for(uint i=0; i<NumRegisters; i++)
		{
			if(Registers[i].updated)
			{
				if(i < 16) Serial.printlnf("0x0%x: %d", i, Registers[i].value);
				else Serial.printlnf("0x%x: %d", i, Registers[i].value);
			}
		}
		Serial.println("----------------");

		// Reset Flag
		FLAG_RegistersUpdated = false;
	}
}

void receiveEvent(int howMany)
{
	// First Byte is a Register
	uint8_t reg = Wire.read();
	Serial.printlnf("Updating Register: 0x%x", reg);

	// Update the Register Pointer
	rp = reg;

	// Exit if specified register is read-only (this is an error, a read-only register should never be written to, and any succeeding data should be ignored)
	if(Registers[reg].rw == 0) return;

	// Succeeding Bytes are new data
	while(Wire.available())
	{
		char data = Wire.read();

		// if register is read only, skip to next register
		while(Registers[reg].rw == 0)
		{
			reg++;
		}

		Registers[reg].value = data;
		Registers[reg].updated = true;
		reg += 1;
		if(reg >= NumRegisters) break;
	}

	FLAG_RegistersUpdated = true;
}

void requestEvent()
{
	// Return the register pointed to by the Register Pointer and advance the RP for subsequent reads
	Serial.printlnf("Sending Register: %x, value: %d", rp, Registers[rp].value);
	Wire.write(Registers[rp++].value);
	if(rp > NumRegisters) rp = 0;
}