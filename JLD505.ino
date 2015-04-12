#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <mcp_can.h>
#include <INA226.h>
#include <EEPROMAnything.h>
#include <SoftwareSerial.h>
#include <AltSoftSerial.h>
#include <DS2480B.h>
#include <DallasTemperature.h>
#include <FrequencyTimer2.h>

SoftwareSerial BTSerial(A2, A3); // RX | TX
AltSoftSerial altSerial; //pins 8 and 9

DS2480B ds(altSerial);
DallasTemperature sensors(&ds);

//set the proper digital pins for these
#define IN0		4
#define IN1		7
#define OUT0	5
#define OUT1	6

INA226 ina;
int ADDR_AmpHours = 0;
int ADDR_KiloWattHours = 10;
int ADDR_VoltageCalibration = 20;
const unsigned long Interval = 10;
float VoltageCalibration = (100000.0*830000.0/930000.0+1000000.0)/(100275.0*830000.0/930000.0); // (Voltage Divider with (100k in parallel with 830k) and 1M )
const float CurrentCalibration = 300.0/0.075; //800A 75mV Shunt
unsigned long Time = 0; 
unsigned long PreviousMillis = 0;
unsigned long CurrentMillis = 0;
float Voltage = 0;
float Current = 0;
float Power = 0;
float KiloWattHours = 0;
float AmpHours = 0;
int Count = 0;
byte Command = 0; // "z" will reset the AmpHours and KiloWattHours counters
volatile uint8_t bStartConversion = 0;
volatile uint8_t bGetTemperature = 0;
volatile uint8_t timerIntCounter = 0;
volatile uint8_t timerFastCounter  = 0;
volatile uint8_t sensorReadPosition = 255;
uint8_t tempSensorCount = 0;
int32_t canMsgID = 0;
unsigned char canMsg[8];
unsigned char Flag_Recv = 0;

//Bunch o' chademo related stuff. 
uint8_t bStartedCharge = 0;
uint8_t bChademoMode = 0; //accessed but not modified in ISR so it should be OK non-volatile
uint8_t bChademoSendRequests = 0; //should we be sending periodic status updates?
volatile uint8_t bChademoRequest = 0;  //is it time to send one of those updates?
//target values are what we send with periodic frames and can be changed.
uint8_t askingAmps = 0; //how many amps to ask for. Trends toward targetAmperage
//Maximums are probably sent only once (in start up handshake) and set the upper limits so nothing dumb happens.
uint16_t maxVoltage = 400; //voltage to never exceed no matter what
uint8_t maxAmps = 100; //how many amps to be limited to
uint8_t packSize = 80; //how many kwh in tenths. Not used for anything other than display. Somewhat meaningless
enum CHADEMOSTATE 
{
	STARTUP,
	SEND_INITIAL_PARAMS,
	WAIT_FOR_EVSE_PARAMS,
	SET_CHARGE_BEGIN,
	WAIT_FOR_BEGIN_CONFIRMATION,
	CLOSE_CONTACTORS,
	RUNNING,
	CEASE_CURRENT,
	WAIT_FOR_ZERO_CURRENT,
	OPEN_CONTACTOR,
	FAULTED,
	STOPPED
};
CHADEMOSTATE chademoState = STOPPED;

typedef struct 
{
	uint8_t supportWeldCheck;
	uint16_t availVoltage;
	uint8_t availCurrent;
	uint16_t thresholdVoltage; //evse calculates this. It is the voltage at which it'll abort charging to save the battery pack in case we asked for something stupid
} EVSE_PARAMS;
EVSE_PARAMS evse_params;

typedef struct 
{
	uint16_t presentVoltage;
	uint8_t presentCurrent;
	uint8_t status;
	uint16_t remainingChargeSeconds;
} EVSE_STATUS;
EVSE_STATUS evse_status;

typedef struct 
{
	uint16_t targetVoltage; //what voltage we want the EVSE to put out
	uint8_t targetCurrent; //what current we'd like the EVSE to provide
	uint8_t remainingKWH; //report # of KWh in the battery pack (charge level)
	uint8_t battOverVolt : 1; //we signal that battery or a cell is too high of a voltage
	uint8_t battUnderVolt : 1; //we signal that battery is too low
	uint8_t currDeviation : 1; //we signal that measured current is not the same as EVSE is reporting
	uint8_t battOverTemp : 1; //we signal that battery is too hot
	uint8_t voltDeviation : 1; //we signal that we measure a different voltage than EVSE reports
	uint8_t chargingEnabled : 1; //ask EVSE to enable charging
	uint8_t notParked : 1; //advise EVSE that we're not in park.
	uint8_t chargingFault : 1; //signal EVSE that we found a fault
	uint8_t contactorOpen : 1; //tell EVSE whether we've closed the charging contactor 
	uint8_t stopRequest : 1; //request that the charger cease operation before we really get going
} CARSIDE_STATUS;
CARSIDE_STATUS carStatus;

//The IDs for chademo comm - both carside and EVSE side so we know what to listen for
//as well.
#define CARSIDE_BATT		0x100
#define CARSIDE_CHARGETIME	0x101
#define CARSIDE_CONTROL		0x102

#define EVSE_PARAMS			0x108
#define EVSE_STATUS			0x109

#define CARSIDE_FAULT_OVERV		1 //over voltage
#define CARSIDE_FAULT_UNDERV	2 //Under voltage
#define CARSIDE_FAULT_CURR		4 //current mismatch
#define CARSIDE_FAULT_OVERT		8 //over temperature
#define CARSIDE_FAULT_VOLTM		16 //voltage mismatch

#define CARSIDE_STATUS_CHARGE	1 //charging enabled
#define CARSIDE_STATUS_NOTPARK	2 //shifter not in safe state
#define CARSIDE_STATUS_MALFUN	4 //vehicle did something dumb
#define CARSIDE_STATUS_CONTOP	8 //main contactor open
#define CARSIDE_STATUS_CHSTOP	16 //charger stop before even charging

#define EVSE_STATUS_CHARGE		1 //charger is active
#define EVSE_STATUS_ERR		2 //something went wrong
#define EVSE_STATUS_CONNLOCK	4 //connector is currently locked
#define EVSE_STATUS_INCOMPAT	8 //parameters between vehicle and charger not compatible
#define EVSE_STATUS_BATTERR		16 //something wrong with battery?!
#define EVSE_STATUS_STOPPED		32 //charger is stopped

void MCP2515_ISR()
{
    Flag_Recv = 1;
}

void timer2Int()
{
	timerFastCounter++;
	if (timerFastCounter == 4)
	{
		if (bChademoMode  && bChademoSendRequests) bChademoRequest = 1;
		timerFastCounter = 0;
		timerIntCounter++;
		if (timerIntCounter < 10)
		{
			bGetTemperature = 1;
			sensorReadPosition++;
		}
		if (timerIntCounter == 10)
		{
			bStartConversion = 1;
			sensorReadPosition = 255;
		}
		if (timerIntCounter == 18)
		{
			timerIntCounter = 0;
		}
	}
}
  
void setup()
{ 
//first thing configure the I/O pins and set them to a sane state
	pinMode(IN0, INPUT);
	pinMode(IN1, INPUT);
	pinMode(OUT0, OUTPUT);
	pinMode(OUT1, OUTPUT);
	digitalWrite(OUT0, LOW);
	digitalWrite(OUT1, LOW);
	pinMode(A1, OUTPUT); //KEY - Must be HIGH
	pinMode(A0, INPUT); //STATE
	digitalWrite(A1, HIGH);
	pinMode(3, INPUT_PULLUP); //enable weak pull up on MCP2515 int pin connected to INT1 on MCU

	Serial.begin(115200);
	BTSerial.begin(115200);
	altSerial.begin(9600);
	
	sensors.begin();
	sensors.setWaitForConversion(false); //we're handling the time delay ourselves so no need to wait when asking for temperatures
  
	CAN.begin(CAN_500KBPS);
	attachInterrupt(1, MCP2515_ISR, FALLING);     // start interrupt

	ina.begin(69);
	ina.configure(INA226_AVERAGES_16, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
	EEPROM_readAnything(ADDR_AmpHours, AmpHours);
	EEPROM_readAnything(ADDR_KiloWattHours, KiloWattHours);
	EEPROM_readAnything(ADDR_VoltageCalibration, VoltageCalibration);
	attachInterrupt(0, Save, FALLING);
	FrequencyTimer2::setPeriod(25000); //interrupt every 25ms
	FrequencyTimer2::setOnOverflow(timer2Int);
	
	Serial.print("Found ");
	tempSensorCount = sensors.getDeviceCount(); 
	Serial.print(tempSensorCount);
	Serial.println(" temperature sensors.");

	//hard coded for now just for testing. Don't do this forever.
	carStatus.targetCurrent = 50;
	carStatus.targetVoltage = 375; 
	carStatus.contactorOpen = 1;
}

void loop()
{
	uint8_t pos;
	CurrentMillis = millis();
	uint8_t len;
 
	if(CurrentMillis - PreviousMillis >= Interval)
	{
		Time = CurrentMillis - PreviousMillis;
		PreviousMillis = CurrentMillis;   
    
		Count++;
		Voltage = ina.readBusVoltage() * VoltageCalibration;
		Current = ina.readShuntVoltage() * CurrentCalibration;
		AmpHours = AmpHours + Current * (float)Time / 1000.0 / 3600.0;
		Power = Voltage * Current / 1000.0;
		KiloWattHours = KiloWattHours + Power * (float)Time / 1000.0 / 3600.0;

		//if (!bChademoMode) 
		//{
			if (Count >= 50)
			{
				Count = 0;
				USB();				
				CANBUS();
				if (!bChademoMode) BT();
				else 
				{
					Serial.print("Chamdemo Mode: ");
					Serial.println(chademoState);
				}
				Save();
			}
		//}		
	}

	if (Flag_Recv || CAN.checkReceive() == CAN_MSGAVAIL) {
		Flag_Recv = 0;		
		CAN.readMsgBuf(&len, canMsg);            // read data,  len: data length, buf: data buf
		canMsgID = CAN.getCanId();
		if (canMsgID == EVSE_PARAMS)
		{
			if (chademoState == WAIT_FOR_EVSE_PARAMS) chademoState = SET_CHARGE_BEGIN;
			evse_params.supportWeldCheck = canMsg[0];
			evse_params.availVoltage = canMsg[1] + canMsg[2] * 256;
			evse_params.availCurrent = canMsg[3];			
			evse_params.thresholdVoltage = canMsg[4] + canMsg[5] * 256;

			//if charger cannot provide our requested voltage then GTFO
			if (evse_params.availVoltage < carStatus.targetVoltage)
			{
				Serial.println("EVSE can't provide needed voltage. Aborting.");
				Serial.println(evse_params.availVoltage);
				chademoState = CEASE_CURRENT;
			}

			//if we want more current then it can provide then revise our request to match max output
			if (evse_params.availCurrent < carStatus.targetCurrent) carStatus.targetCurrent = evse_params.availCurrent;
		}
		if (canMsgID == EVSE_STATUS)
		{
			evse_status.presentVoltage = canMsg[1] + 256 * canMsg[2];
			evse_status.presentCurrent  = canMsg[3];
			evse_status.status = canMsg[5];				
			if (canMsg[6] < 0xFF)
			{
				evse_status.remainingChargeSeconds = canMsg[6] * 10;
			}
			else 
			{
				evse_status.remainingChargeSeconds = canMsg[7] * 60;
			}

			//on fault try to turn off current immediately and cease operation
			if ((evse_status.status & 0x1A) != 0) //if bits 1, 3, or 4 are set then we have a problem.
			{
				Serial.println("EVSE reports fault. Aborting.");
				if (chademoState == RUNNING) chademoState = CEASE_CURRENT;
			}

			//if there is no remaining time then gracefully shut down
			if (evse_status.remainingChargeSeconds == 0)
			{
				Serial.println("EVSE reports time elapsed. Finishing.");
				if (chademoState == RUNNING) chademoState = CEASE_CURRENT;
			}
		}
	}
  
	if (bStartConversion == 1)
	{
		bStartConversion = 0;
		sensors.requestTemperatures();
	}
	if (bGetTemperature)
	{
		bGetTemperature = 0;
		pos = sensorReadPosition;
		if (pos < tempSensorCount)
		{		  
			Serial.print(pos);
			Serial.print(": ");
			//sensors.isConnected(pos);
			// Serial.println(sensors.getCelsius(pos));
			Serial.println(sensors.getTempCByIndex(pos));
		}
	}

	//Danger Will Robinson. There is no debouncing here. That might be naughty.
	if (!digitalRead(IN1)) //IN1 goes low if we have been plugged into the chademo port
	{
		bChademoMode = 1;
		if (chademoState == STOPPED && !bStartedCharge) {
			chademoState = STARTUP;
			Serial.println("Starting Chademo process.");
		}
	}
	else 
	{
		if (bChademoMode == 1) 
		{
			Serial.println("Stopping chademo process.");
		}
		bChademoMode = 0;
		bStartedCharge = 0;
		chademoState = STOPPED;
		//maybe it would be a good idea to try to see if EVSE is still transmitting to us and providing current
		//as it is not a good idea to open the contactors under load. But, IN1 shouldn't trigger 
		//until the EVSE is ready. Investigate options here.
		digitalWrite(OUT0, LOW);
		digitalWrite(OUT1, LOW);
	}

	if (bChademoMode)
	{
		if (bChademoSendRequests && bChademoRequest)
		{
			bChademoRequest = 0;
			sendChademoStatus();
			sendChademoBattSpecs();
			sendChademoChargingTime();
			Serial.println("Tx");
		}

		switch (chademoState)
		{
		case STARTUP: //really useful state huh?
			chademoState = SEND_INITIAL_PARAMS; 
			break;
		case SEND_INITIAL_PARAMS:
			//we could do calculations to see how long the charge should take based on SOC and 
			//also set a more realistic starting amperage. Options for the future.
			//One problem with that is that we don't yet know the EVSE parameters so we can't know
			//the max allowable amperage just yet.
			bChademoSendRequests = 1; //causes chademo frames to be sent out every 100ms
			chademoState = WAIT_FOR_EVSE_PARAMS;
			Serial.println("Sent parameters to EVSE. Waiting.");
			break;
		case WAIT_FOR_EVSE_PARAMS:
			//for now do nothing while we wait. Might want to try to resend start up messages periodically if no reply
			break;
		case SET_CHARGE_BEGIN:
			Serial.println("Setting begin charge request.");
			digitalWrite(OUT1, HIGH); //signal that we're ready to charge
			//carStatus.chargingEnabled = 1; //should this be enabled here???
			chademoState = WAIT_FOR_BEGIN_CONFIRMATION;
			break;
		case WAIT_FOR_BEGIN_CONFIRMATION:
			if (digitalRead(IN0)) //inverse logic from how IN1 works. Be careful!
			{
				chademoState = CLOSE_CONTACTORS;
			}
			break;
		case CLOSE_CONTACTORS:
			Serial.println("Closing contactor");
			digitalWrite(OUT0, HIGH);
			chademoState = RUNNING;
			carStatus.contactorOpen = 0; //its closed now
			carStatus.chargingEnabled = 1; //please sir, I'd like some charge
			bStartedCharge = 1;
			break;
		case RUNNING:
			//do processing here by taking our measured voltage, amperage, and SOC to see if we should be commanding something
			//different to the EVSE. Also monitor temperatures to make sure we're not incinerating the pack.
			break;
		case CEASE_CURRENT:
			Serial.println("Setting current request to zero.");
			carStatus.targetCurrent = 0;
			chademoState = WAIT_FOR_ZERO_CURRENT;
			break;
		case WAIT_FOR_ZERO_CURRENT:
			if (evse_status.presentCurrent == 0)
			{
				chademoState = OPEN_CONTACTOR;
			}
			break;
		case OPEN_CONTACTOR:
			Serial.println("Opening contactor");
			digitalWrite(OUT0, LOW);
			carStatus.contactorOpen = 1;
			carStatus.chargingEnabled = 0;
			sendChademoStatus(); //we probably need to force this right now
			chademoState = STOPPED;
			break;
		case FAULTED:
			Serial.println("Detected fault!");
			chademoState = CEASE_CURRENT;
			//digitalWrite(OUT0, LOW);
			//digitalWrite(OUT1, LOW);
			break;
		case STOPPED:
			digitalWrite(OUT0, LOW);
			digitalWrite(OUT1, LOW);
			bChademoSendRequests = 0; //don't need to keep sending anymore.
			break;
		}
	}
}

void Save()
{
	EEPROM_writeAnything(ADDR_AmpHours, AmpHours);
	EEPROM_writeAnything(ADDR_KiloWattHours, KiloWattHours);
	EEPROM_writeAnything(ADDR_VoltageCalibration, VoltageCalibration);
}  

void USB()
{
  Serial.print (Voltage, 3);   
  Serial.print ("V ");
  Serial.print (Current, 2);    
  Serial.print ("A ");
  Serial.print (AmpHours, 1);    
  Serial.print ("Ah ");
  Serial.print (Power, 1);        
  Serial.print ("kW ");
  Serial.print (KiloWattHours, 1);    
  Serial.println ("kWh");
  if (Serial.available() > 0)
  {Command = Serial.read();
   if (Command == 'z')
     {
      Serial.println("Reset Ah & Wh");
      AmpHours = 0;
      KiloWattHours = 0;
      Serial.println("Done!!!"); 
    }
     if (Command == '+')
     {
      VoltageCalibration +=0.004;
      Serial.println (VoltageCalibration, 5);   
    }
     if (Command == '-')
     {
      VoltageCalibration -=0.004;
      Serial.println (VoltageCalibration, 5);   
    }
   while(Serial.available()>0) Serial.read();}
}

void BT()
{
	
  BTSerial.print (Voltage, 2);   
  BTSerial.print ("V ");
  BTSerial.print (Current, 2);    
  BTSerial.print ("A ");
  BTSerial.print (AmpHours, 1);    
  BTSerial.print ("Ah ");
  BTSerial.print (Power, 1);        
  BTSerial.print ("kW ");
  BTSerial.print (KiloWattHours, 1);    
  BTSerial.println ("kWh");
	
  /*
  BTSerial.write(02);
  BTSerial.write(highByte((int)(Voltage*10)));  
  BTSerial.write(lowByte((int)(Voltage*10)));
  BTSerial.write(highByte((int)(Current*10)));    
  BTSerial.write(lowByte((int)(Current*10)));
  BTSerial.write(highByte((int)(AmpHours*10)));    
  BTSerial.write(lowByte((int)(AmpHours*10)));
  BTSerial.write(highByte((int)(Power*10)));        
  BTSerial.write(lowByte((int)(Power*10)));
  BTSerial.write(highByte((int)(KiloWattHours*10)));    
  BTSerial.write(lowByte((int)(KiloWattHours*10)));
  BTSerial.write(03);*/
  
  if (BTSerial.available() > 0)
  {Command = BTSerial.read();
   if (Command == 'z')
   {AmpHours = 0;
    KiloWattHours = 0;}
   while(BTSerial.available()>0) BTSerial.read();}
   
}

void CANBUS()
{
	canMsgID = 0x404;
	canMsg[0] = highByte((int)(Voltage*10)); // Voltage High Byte
	canMsg[1] = lowByte((int)(Voltage*10)); // Voltage Low Byte
	canMsg[2] = highByte((int)(Current*10)); // Current High Byte
	canMsg[3] = lowByte((int)(Current*10)); // Current Low Byte
	canMsg[4] = highByte((int)(AmpHours*10)); // AmpHours High Byte
	canMsg[5] = lowByte((int)(AmpHours*10)); // AmpHours Low Byte
	canMsg[6] = 0x00; // Not Used
	canMsg[7] = 0x00; // Not Used
	CAN.sendMsgBuf(canMsgID, 0, 6, canMsg);
	
  
	canMsgID = 0x505;
	canMsg[0] = highByte((int)(Power*10)); // Power High Byte
	canMsg[1] = lowByte((int)(Power*10)); // Power Low Byte
	canMsg[2] = highByte((int)(KiloWattHours*10)); // KiloWattHours High Byte
	canMsg[3] = lowByte((int)(KiloWattHours*10)); // KiloWattHours Low Byte
	canMsg[4] = 0x00; // Not Used
	canMsg[5] = 0x00; // Not Used
	canMsg[6] = 0x00; // Not Used
	canMsg[7] = 0x00; // Not Used
	CAN.sendMsgBuf(canMsgID, 0, 4, canMsg);
 }

void sendChademoBattSpecs()
{
	
	canMsgID = CARSIDE_BATT;
	canMsg[0] = 0x00; // Not Used
	canMsg[1] = 0x00; // Not Used
	canMsg[2] = 0x00; // Not Used
	canMsg[3] = 0x00; // Not Used
	canMsg[4] = lowByte(maxVoltage);
	canMsg[5] = highByte(maxVoltage); 
	canMsg[6] = packSize;
	canMsg[7] = 0; //not used
	CAN.sendMsgBuf(canMsgID, 0, 8, canMsg);
}

void sendChademoChargingTime()
{
	
	canMsgID = CARSIDE_CHARGETIME;
	canMsg[0] = 0x00; // Not Used
	canMsg[1] = 0xFF; //not using 10 second increment mode
	canMsg[2] = 10; //ask for a 10 minute charge - for safety since this is still a test
	canMsg[3] = 10; //how long we think the charge will take. Also 10 minutes for safety
	canMsg[4] = 0; //not used
	canMsg[5] = 0; //not used
	canMsg[6] = 0; //not used
	canMsg[7] = 0; //not used
	CAN.sendMsgBuf(canMsgID, 0, 8, canMsg);
}

void sendChademoStatus()
{
	uint8_t faults = 0;
	uint8_t status = 0;

	if (carStatus.battOverTemp) faults |= CARSIDE_FAULT_OVERT;
	if (carStatus.battOverVolt) faults |= CARSIDE_FAULT_OVERV;
	if (carStatus.battUnderVolt) faults |= CARSIDE_FAULT_UNDERV;
	if (carStatus.currDeviation) faults |= CARSIDE_FAULT_CURR;
	if (carStatus.voltDeviation) faults |= CARSIDE_FAULT_VOLTM;

	if (carStatus.chargingEnabled) status |= CARSIDE_STATUS_CHARGE;
	if (carStatus.notParked) status |= CARSIDE_STATUS_NOTPARK;
	if (carStatus.chargingFault) status |= CARSIDE_STATUS_MALFUN;
	if (carStatus.contactorOpen) status |= CARSIDE_STATUS_CONTOP;
	if (carStatus.stopRequest) status |= CARSIDE_STATUS_CHSTOP;

	canMsgID = CARSIDE_CONTROL;
	canMsg[0] = 1; //claim to only support the chademo 0.9 protocol. It's safer/easier that way
	canMsg[1] = lowByte(carStatus.targetVoltage);
	canMsg[2] = highByte(carStatus.targetVoltage);
	canMsg[3] = askingAmps;
	canMsg[4] = faults;
	canMsg[5] = status;
	canMsg[6] = packSize / 2; //always claim that the battery is at 50% charge. Also not particularly bright but probably OK for early testing
	canMsg[7] = 0; //not used
	CAN.sendMsgBuf(canMsgID, 0, 8, canMsg);
	if (chademoState == RUNNING &&  askingAmps < carStatus.targetCurrent) askingAmps++;
	//not a typo. We're allowed to change requested amps by +/- 20A per second. We send the above frame every 100ms so a single
	//increment means we can ramp up 10A per second. But, we want to ramp down quickly if there is a problem so do two which
	//gives us -20A per second.
	if (chademoState != RUNNING && askingAmps > 0) askingAmps--;
	if (askingAmps > carStatus.targetCurrent) askingAmps--;
	if (askingAmps > carStatus.targetCurrent) askingAmps--;
}