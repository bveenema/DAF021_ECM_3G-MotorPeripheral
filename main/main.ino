/*
 * Project DAF021_ECM_3G-Argon
 * Description:
 * Author:
 */

#include <Wire.h>
#include "pins.h"
#include "AccelStepper.h"

AccelStepper BlueMotor(AccelStepper::DRIVER, BLUE_STEP_PIN, BLUE_DIR_PIN);
AccelStepper RedMotor(AccelStepper::DRIVER, RED_STEP_PIN, RED_DIR_PIN);
AccelStepper CoBlendMotor(AccelStepper::DRIVER, COBLED_STEP_PIN, COBLEND_DIR_PIN);


#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

// Functions
void setup();
void loop();
void receiveEvent(int howMany);
void requestEvent();
void printUpdatedRegisters();
void printRegister(uint8_t reg);
void handleRegisterUpdates();
int CalculateTargetPosition(const uint motorNum);
int CalculateAcceleration(const uint motorNum);
float CalculateMaxSpeed(const uint motorNum);

// sprintf Buffer
char buffer[255];

/// Register Definition
//	Register Qualitiies:
//		- Read/Write
//		- Registers can be flow through written, read only registers are skipped. Ex if start at register 0, and 4 bytes are sent, registers 0,1,3,4 are updated
struct Register
{
	char value; // the current value of the register
	bool rw; // 0: read-only, 1: Writable
	bool updated; // 1: was updated
	char name[16]; // the name of the register
};
uint32_t rp = 0; // points to most recently receieved register
const uint32_t NumRegisters = 41;
Register Registers[NumRegisters] = 
{
	{0,true,false, "Enable"},			// Motor Enable Register
	{0,true,false, "Direction"},		// Motor Direction Register (1: forward/pump, 0: backward/retract)
	{0,true,false, "Move"},				// Motor Move Register (1: if motor is to move, 0: if motor is to stop), Gets set to 0 if corresponding motor Enable register is set to 0, Gets set to 0 when move is complete
	{0,false,false, "Error"},			// Motor Error Register (1: if error, 0: if no error), Cleared when read
	{0,true,false, "M1 Speed 1"}, 		// Motor 1 Speed (1) High
	{0,true,false, "M1 Speed 2"},		// Motor 1 Speed (2)
	{0,true,false, "M1 Speed 3"},		// Motor 1 Speed (3)
	{0,true,false, "M1 Speed 4"},		// Motor 1 Speed (4) Low
	{0,true,false, "M1 Accel 1"},  		// Motor 1 Acceleration (1) High
	{0,true,false, "M1 Accel 2"},  		// Motor 1 Acceleration (2)
	{0,true,false, "M1 Accel 3"},  		// Motor 1 Acceleration (3)
	{0,true,false, "M1 Accel 4"},  		// Motor 1 Acceleration (4) Low
	{0,true,false, "M1 Steps 1"},		// Motor 1 Steps (1) High
	{0,true,false, "M1 Steps 2"},		// Motor 1 Steps (2)
	{0,true,false, "M1 Steps 3"},		// Motor 1 Steps (3)
	{0,true,false, "M1 Steps 4"},		// Motor 1 Steps (4) Low
	{0,true,false, "M2 Speed 1"}, 		// Motor 2 Speed (1) High
	{0,true,false, "M2 Speed 2"},		// Motor 2 Speed (2)
	{0,true,false, "M2 Speed 3"},		// Motor 2 Speed (3)
	{0,true,false, "M2 Speed 4"},		// Motor 2 Speed (4) Low
	{0,true,false, "M2 Accel 1"},  		// Motor 2 Acceleration (1) High
	{0,true,false, "M2 Accel 2"},  		// Motor 2 Acceleration (2)
	{0,true,false, "M2 Accel 3"},  		// Motor 2 Acceleration (3)
	{0,true,false, "M2 Accel 4"},  		// Motor 2 Acceleration (4) Low
	{0,true,false, "M2 Steps 1"},		// Motor 2 Steps (1) High
	{0,true,false, "M2 Steps 2"},		// Motor 2 Steps (2)
	{0,true,false, "M2 Steps 3"},		// Motor 2 Steps (3)
	{0,true,false, "M2 Steps 4"},		// Motor 2 Steps (4) Low
	{0,true,false, "M3 Speed 1"}, 		// Motor 3 Speed (1) High
	{0,true,false, "M3 Speed 2"},		// Motor 3 Speed (2)
	{0,true,false, "M3 Speed 3"},		// Motor 3 Speed (3)
	{0,true,false, "M3 Speed 4"},		// Motor 3 Speed (4) Low
	{0,true,false, "M3 Accel 1"},  		// Motor 3 Acceleration (1) High
	{0,true,false, "M3 Accel 2"},  		// Motor 3 Acceleration (2)
	{0,true,false, "M3 Accel 3"},  		// Motor 3 Acceleration (3)
	{0,true,false, "M3 Accel 4"},  		// Motor 3 Acceleration (4) Low
	{0,true,false, "M3 Steps 1"},		// Motor 3 Steps (1) High
	{0,true,false, "M3 Steps 2"},		// Motor 3 Steps (2)
	{0,true,false, "M3 Steps 3"},		// Motor 3 Steps (3)
	{0,true,false, "M3 Steps 4"},		// Motor 3 Steps (4) Low
};

enum MotorDriverRegister
{
    Motor_EN_Reg,
    Motor_DIR_Reg,
    Motor_MOVE_Reg,
    Motor_ERROR_Reg,
    Motor_1_Speed_1_Reg,
    Motor_1_Speed_2_Reg,
    Motor_1_Speed_3_Reg,
    Motor_1_Speed_4_Reg,
    Motor_1_Acceleration_1_Reg,
    Motor_1_Acceleration_2_Reg,
    Motor_1_Acceleration_3_Reg,
    Motor_1_Acceleration_4_Reg,
    Motor_1_Steps_1_Reg,
    Motor_1_Steps_2_Reg,
    Motor_1_Steps_3_Reg,
    Motor_1_Steps_4_Reg,
    Motor_2_Speed_1_Reg,
    Motor_2_Speed_2_Reg,
    Motor_2_Speed_3_Reg,
    Motor_2_Speed_4_Reg,
    Motor_2_Acceleration_1_Reg,
    Motor_2_Acceleration_2_Reg,
    Motor_2_Acceleration_3_Reg,
    Motor_2_Acceleration_4_Reg,
    Motor_2_Steps_1_Reg,
    Motor_2_Steps_2_Reg,
    Motor_2_Steps_3_Reg,
    Motor_2_Steps_4_Reg,
    Motor_3_Speed_1_Reg,
    Motor_3_Speed_2_Reg,
    Motor_3_Speed_3_Reg,
    Motor_3_Speed_4_Reg,
    Motor_3_Acceleration_1_Reg,
    Motor_3_Acceleration_2_Reg,
    Motor_3_Acceleration_3_Reg,
    Motor_3_Acceleration_4_Reg,
    Motor_3_Steps_1_Reg,
    Motor_3_Steps_2_Reg,
    Motor_3_Steps_3_Reg,
    Motor_3_Steps_4_Reg,
};

bool MotorForwardDirection[3] = {1, 0, 1}; // 1: Foward direction is clockwise, 0: Forward direction is counter-clockwise

/// Flags
// Registers Updated Flag: set when any register is changed during an onReceive interrupt
bool FLAG_RegistersUpdated = false;

// setup() runs once, when the device is first turned on.
void setup()
{
	// Initialize Serial
	Serial.begin(57600);

	// Initialize I2C
	Wire.begin(4);                // join i2c bus with address #4
	Wire.onReceive(receiveEvent); // register event
	Wire.onRequest(requestEvent); // register request handler

	// Initialize Interrupt Pin
	pinMode(MOTOR_INT_PIN, OUTPUT);
	digitalWrite(MOTOR_INT_PIN, LOW);

	// Initialize Stepper Motors
	pinMode(BLUE_STATUS_PIN, INPUT);
	pinMode(BLUE_EN_PIN, OUTPUT);
	digitalWrite(BLUE_EN_PIN, LOW); // Disable
	BlueMotor.setPinsInverted(0,0,1);

	pinMode(RED_STATUS_PIN, INPUT);
	pinMode(RED_EN_PIN, OUTPUT);
	digitalWrite(RED_EN_PIN, LOW); // Disable
	RedMotor.setPinsInverted(0, 0, 1);

	pinMode(COBLEND_STATUS_PIN, INPUT);
	pinMode(COBLEND_EN_PIN, OUTPUT);
	digitalWrite(COBLEND_EN_PIN, LOW); // Disable
	CoBlendMotor.setPinsInverted(0, 0, 1);
}

// loop() runs over and over again, as quickly as it can execute.
void loop()
{
	// Update Stepper Motors
	BlueMotor.run();
	RedMotor.run();

	// Monitor Motor
	static uint LastMotorCheck = 0;
	if(millis() - LastMotorCheck > 100)
	{
		if(BlueMotor.distanceToGo() == 0)
			bitClear(Registers[Motor_MOVE_Reg].value, 0);
		if(RedMotor.distanceToGo() == 0)
			bitClear(Registers[Motor_MOVE_Reg].value, 1);

		if(digitalRead(BLUE_STATUS_PIN))
			bitSet(Registers[Motor_ERROR_Reg].value, 0);
		if(digitalRead(RED_STATUS_PIN))
			bitSet(Registers[Motor_ERROR_Reg].value, 1);
		
		// Set the interrupt pin if move is complete or position error has occured
		if(Registers[Motor_MOVE_Reg].value == 0 || Registers[Motor_ERROR_Reg].value != 0)
			digitalWrite(MOTOR_INT_PIN, HIGH);
		else
			digitalWrite(MOTOR_INT_PIN, LOW);

		LastMotorCheck = millis();
	}
	
	// Handle I2C Communications
	if(FLAG_RegistersUpdated)
	{
		printUpdatedRegisters();
		handleRegisterUpdates();
		FLAG_RegistersUpdated = false;
	}
}

void receiveEvent(int howMany)
{
	// First Byte is a Register
	uint8_t reg = Wire.read();

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
	// Return the register pointed to by the Register Pointer
	Wire.write(Registers[rp].value);
	if(rp == Motor_ERROR_Reg) // Clear Motor ERROR register after read
		Registers[Motor_ERROR_Reg].value = 0;
	
	// Advance the register pointer
	rp += 1;
	if(rp > NumRegisters) rp = 0;
}

void printUpdatedRegisters()
{
	Serial.println("----------------\nRegisters:");
	for(uint i=0; i<NumRegisters; i++)
	{
		if(Registers[i].updated)
		{
			sprintf(buffer, "0x%s%x: %d (%s)", (i<16) ? "0":"", i, Registers[i].value, Registers[i].name);
			Serial.println(buffer);
		}
	}
	Serial.println("----------------");
}

void printRegister(uint8_t reg)
{
	Serial.println("----------------\nRegister:");

	sprintf(buffer, "0x%s%x: %d", (reg<16) ? "0":"", reg, Registers[reg].value);
	Serial.println(buffer); 

	Serial.println("----------------");
}

void handleRegisterUpdates()
{
	// Enable Register Updates get handled immediately
	if(Registers[Motor_EN_Reg].updated)
	{
		digitalWrite(BLUE_EN_PIN, Registers[Motor_EN_Reg].value & 1);
		digitalWrite(RED_EN_PIN, Registers[Motor_EN_Reg].value & 2);
		digitalWrite(COBLEND_EN_PIN, Registers[Motor_EN_Reg].value & 4);

		// Clear the updated bit
		Registers[Motor_EN_Reg].updated = false;
	}

	// Direction Register Updates are ignored until move command issued
	if(Registers[Motor_DIR_Reg].updated)
	{
		// Clear the updated bit
		Registers[Motor_DIR_Reg].updated = false;
	}

	// Move Register initiates a new move or stops a move if the bit is changed
	if(Registers[Motor_MOVE_Reg].updated)
	{
		// Clear the updated bit
		Registers[Motor_EN_Reg].updated = false;

		static Register PreviousMOVERegister = {0, true, false};
		if((Registers[Motor_MOVE_Reg].value & 1) && !(PreviousMOVERegister.value & 1)) // Motor 1 Move bit was set
		{
			sprintf(buffer, "Moving Motor 1");
			Serial.println(buffer);
			// Update Speed and Acceleration
			BlueMotor.setAcceleration(CalculateAcceleration(1));
			BlueMotor.setMaxSpeed(CalculateMaxSpeed(1));

			BlueMotor.setCurrentPosition(0); // Reset Position
			BlueMotor.moveTo(CalculateTargetPosition(1));
		}
		else if(!(Registers[Motor_MOVE_Reg].value & 1) && (PreviousMOVERegister.value & 1)) // Motor 1 Move bit was cleared
		{
			Serial.println("\nSTOP MOTOR 1");
			BlueMotor.stop();
		}

		if((Registers[Motor_MOVE_Reg].value & 2) && !(PreviousMOVERegister.value & 2)) // Motor 2 Move bit was set
		{
			sprintf(buffer, "Moving Motor 2");
			Serial.println(buffer);
			// Update Speed and Acceleration
			RedMotor.setAcceleration(CalculateAcceleration(2));
			RedMotor.setMaxSpeed(CalculateMaxSpeed(2));

			RedMotor.setCurrentPosition(0); // Reset Position
			RedMotor.moveTo(CalculateTargetPosition(2));
		}
		else if(!(Registers[Motor_MOVE_Reg].value & 2) && (PreviousMOVERegister.value & 2)) // Motor 2 Move bit was cleared
		{
			Serial.println("\nSTOP MOTOR 2");
			RedMotor.stop();
		}

		if((Registers[Motor_MOVE_Reg].value & 4) && !(PreviousMOVERegister.value & 4)) // Motor 3 Move bit was set
		{
			// Update Speed and Acceleration
			CoBlendMotor.setAcceleration(CalculateAcceleration(3));
			CoBlendMotor.setMaxSpeed(CalculateMaxSpeed(3));

			CoBlendMotor.setCurrentPosition(0); // Reset Position
			CoBlendMotor.moveTo(CalculateTargetPosition(3));
		}
		else if(!(Registers[Motor_MOVE_Reg].value & 4) && (PreviousMOVERegister.value & 4)) // Motor 3 Move bit was cleared
		{
			CoBlendMotor.stop();
		}

		// Clear the Interrupt pin if any motor is moving
		if(Registers[Motor_MOVE_Reg].value != 0)
			digitalWrite(MOTOR_INT_PIN, LOW);

		PreviousMOVERegister = Registers[Motor_MOVE_Reg];
	}

	// Step, Acceleration and Speed registers are ignored until move command is issued
	uint reg = Motor_1_Speed_1_Reg;
	while(reg <= NumRegisters)
	{
		Registers[reg++].updated = false;
	}
}

int CalculateTargetPosition(const uint motorNum)
{
	// Determine the starting register, motor must be 1,2 or 3, anything else is invalid
	uint reg = 0;
	if(motorNum == 1)
		reg = Motor_1_Steps_1_Reg;
	else if(motorNum == 2)
		reg = Motor_2_Steps_1_Reg;
	else if(motorNum == 3)
		reg = Motor_3_Steps_1_Reg;
	else
		return 0;

	// Calculate the target absolute value, next 3 registers are lower bytes
	int target = 0;
	target |= Registers[reg++].value << 24;
	target |= Registers[reg++].value << 16;
	target |= Registers[reg++].value << 8;
	target |= Registers[reg].value;

	// Get the direction bit
	bool dir = Registers[Motor_DIR_Reg].value & (1 << (motorNum-1));

	sprintf(buffer, "Direction: %s", (dir) ? "Forward" : "Backward");
	Serial.println(buffer);

	// compensate position for direction bit
	if(    (MotorForwardDirection[motorNum-1] == 1 && dir == 0)  // if forward direction is clockwise and requested direction is backward
		|| (MotorForwardDirection[motorNum-1] == 0 && dir == 1)) // if forward direction is counter-clockwise and requested direction is forward
	{
		target = -target;
	}
	
	sprintf(buffer, "New Target: %d", target);
	Serial.println(buffer);

	return target;
}

int CalculateAcceleration(const uint motorNum)
{
	// Determine the starting register, motor must be 1,2 or 3, anything else is invalid
	uint reg = 0;
	if(motorNum == 1)
		reg = Motor_1_Acceleration_1_Reg;
	else if(motorNum == 2)
		reg = Motor_2_Acceleration_1_Reg;
	else if(motorNum == 3)
		reg = Motor_3_Acceleration_1_Reg;
	else
		return 0;

	// Calculate the acceleration
	int acceleration = 0;
	acceleration |= Registers[reg++].value << 24;
	acceleration |= Registers[reg++].value << 16;
	acceleration |= Registers[reg++].value << 8;
	acceleration |= Registers[reg].value;

	sprintf(buffer, "Acceleration: %d", acceleration);			
	Serial.println(buffer);

	return acceleration;
}

float CalculateMaxSpeed(const uint motorNum)
{
	// Determine the starting register, motor must be 1,2 or 3, anything else is invalid
	uint reg = 0;
	if(motorNum == 1)
		reg = Motor_1_Speed_1_Reg;
	else if(motorNum == 2)
		reg = Motor_2_Speed_1_Reg;
	else if(motorNum == 3)
		reg = Motor_3_Speed_1_Reg;
	else
		return 0;

	// Calculate the speed
	int speed = 0;
	speed |= Registers[reg++].value << 24;
	speed |= Registers[reg++].value << 16;
	speed |= Registers[reg++].value << 8;
	speed |= Registers[reg].value;
	
	float fspeed = (float)speed/1000;

	sprintf(buffer, "Speed: %d", speed);
	Serial.println(buffer);
	Serial.println(fspeed);
	Serial.flush();

	return fspeed;
}