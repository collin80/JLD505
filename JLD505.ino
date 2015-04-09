#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <MCP2515.h>
#include <INA226.h>
#include <EEPROMAnything.h>
#include <SoftwareSerial.h>
#include <AltSoftSerial.h>
#include <DS2480B.h>
#include <DallasTemperature.h>
#include <FrequencyTimer2.h>

SoftwareSerial BTSerial(A2, A1); // RX | TX
AltSoftSerial altSerial; //pins 8 and 9

           //CS, RESET, INT
MCP2515 CAN(10, 9, 3); //there is no controllable reset pin but I set it to 9 because I believe D9 to be unused.

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
int canMsgID  = 0x000;
unsigned char canMsg[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte Command = 0; // "z" will reset the AmpHours and KiloWattHours counters
volatile uint8_t bStartConversion = 0;
volatile uint8_t bGetTemperature = 0;
volatile uint8_t timerIntCounter = 0;
volatile uint8_t timerFastCounter  = 0;
volatile uint8_t sensorReadPosition = 255;
uint8_t tempSensorCount = 0;

//Bunch o' chademo related stuff. 
uint8_t bChademoMode = 0; //accessed but not modified in ISR so it should be OK non-volatile
uint8_t bChademoSendRequests = 0; //should we be sending periodic status updates?
volatile uint8_t bChademoRequest = 0;  //is it time to send one of those updates?
//target values are what we send with periodic frames and can be changed.
uint16_t targetVoltage = 375; //in 1V increments so literally the voltage we want to target
uint8_t targetAmperage = 50; //amperage to ask for
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
	FAULTED,
	STOPPED
};
CHADEMOSTATE chademoState = STOPPED;

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

void CANHandler() {
	CAN.intHandler();
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

	//set up SPI so we can use the MCP2515 module
  	SPI.setClockDivider(SPI_CLOCK_DIV2);
	SPI.setDataMode(SPI_MODE0);
	SPI.setBitOrder(MSBFIRST);
	SPI.begin();

	Serial.begin(115200);
	BTSerial.begin(38400);
	altSerial.begin(9600);
	
	sensors.begin();
	sensors.setWaitForConversion(false); //we're handling the time delay ourselves so no need to wait when asking for temperatures
  
	CAN.Init(250, 16);
	CAN.InitFilters(false);
	CAN.SetRXMask(MASK0, 0x7F0, 0); //match all but bottom four bits
	CAN.SetRXFilter(FILTER0, 0x100, 0); //allows 0x100 - 0x10F which is perfect for CHADEMO
	attachInterrupt(1, CANHandler, FALLING); //interrupt 1 on this chip is pin 3 which is properly hooked up

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
}

void loop()
{
	uint8_t pos;
	CurrentMillis = millis();
 
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

		if (!bChademoMode) 
		{
			if (Count >= 50)
			{
				Count = 0;
				USB();
				BT();
				CANBUS();
				Save();
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
		if (chademoState = STOPPED) chademoState = STARTUP;
	}
	else 
	{
		bChademoMode = 0;
		chademoState = STOPPED;
	}

	if (bChademoMode)
	{
		switch (chademoState)
		{
		case STARTUP: //really useful state huh?
			chademoState = SEND_INITIAL_PARAMS; 
			break;
		case SEND_INITIAL_PARAMS:
			sendChademoBattSpecs();
			sendChademoChargingTime();
			chademoState = WAIT_FOR_EVSE_PARAMS;
			break;
		case WAIT_FOR_EVSE_PARAMS:
			//for now do nothing while we wait. Might want to try to resend start up messages periodically if no reply
			break;
		case SET_CHARGE_BEGIN:
			digitalWrite(OUT1, HIGH); //signal that we're ready to charge
			chademoState = WAIT_FOR_BEGIN_CONFIRMATION;
			break;
		case WAIT_FOR_BEGIN_CONFIRMATION:
			if (digitalRead(IN0)) //inverse logic from how IN1 works. Be careful!
			{
				chademoState = CLOSE_CONTACTORS;
			}
			break;
		case CLOSE_CONTACTORS:
			digitalWrite(OUT0, HIGH);
			chademoState = RUNNING;
			bChademoSendRequests = 1; //superfluous likely... could just use chademoState == RUNNING in other code
			sendChademoStatus(); //send it right away to be sure we're in good shape
			break;
		case RUNNING:
			if (bChademoSendRequests && bChademoRequest)
			{
				bChademoRequest = 0;
				sendChademoStatus();
			}
			break;
		case FAULTED:
			digitalWrite(OUT0, LOW);
			digitalWrite(OUT1, LOW);
			break;
		case STOPPED:
			digitalWrite(OUT0, LOW);
			digitalWrite(OUT1, LOW);
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
	Frame outputFrame;
	outputFrame.id = 0x404;
	outputFrame.dlc = 6;
	outputFrame.ide = 0;
	outputFrame.data[0] = highByte((int)(Voltage*10)); // Voltage High Byte
	outputFrame.data[1] = lowByte((int)(Voltage*10)); // Voltage Low Byte
	outputFrame.data[2] = highByte((int)(Current*10)); // Current High Byte
	outputFrame.data[3] = lowByte((int)(Current*10)); // Current Low Byte
	outputFrame.data[4] = highByte((int)(AmpHours*10)); // AmpHours High Byte
	outputFrame.data[5] = lowByte((int)(AmpHours*10)); // AmpHours Low Byte
	outputFrame.data[6] = 0x00; // Not Used
	outputFrame.data[7] = 0x00; // Not Used
	CAN.EnqueueTX(outputFrame);
	
  
	outputFrame.id = 0x505;
	outputFrame.dlc = 4;
	outputFrame.data[0] = highByte((int)(Power*10)); // Power High Byte
	outputFrame.data[1] = lowByte((int)(Power*10)); // Power Low Byte
	outputFrame.data[2] = highByte((int)(KiloWattHours*10)); // KiloWattHours High Byte
	outputFrame.data[3] = lowByte((int)(KiloWattHours*10)); // KiloWattHours Low Byte
	outputFrame.data[4] = 0x00; // Not Used
	outputFrame.data[5] = 0x00; // Not Used
	outputFrame.data[6] = 0x00; // Not Used
	outputFrame.data[7] = 0x00; // Not Used
	CAN.EnqueueTX(outputFrame);
 }

void sendChademoBattSpecs()
{
	Frame outputFrame;
	outputFrame.id = CARSIDE_BATT;
	outputFrame.dlc = 8;
	outputFrame.ide = 0;
	outputFrame.data[0] = 0x00; // Not Used
	outputFrame.data[1] = 0x00; // Not Used
	outputFrame.data[2] = 0x00; // Not Used
	outputFrame.data[3] = 0x00; // Not Used
	outputFrame.data[4] = lowByte(maxVoltage);
	outputFrame.data[5] = highByte(maxVoltage); 
	outputFrame.data[6] = packSize;
	outputFrame.data[7] = 0; //not used
	CAN.EnqueueTX(outputFrame);
}

void sendChademoChargingTime()
{
	Frame outputFrame;
	outputFrame.id = CARSIDE_CHARGETIME;
	outputFrame.dlc = 8;
	outputFrame.ide = 0;
	outputFrame.data[0] = 0x00; // Not Used
	outputFrame.data[1] = 0xFF; //not using 10 second increment mode
	outputFrame.data[2] = 10; //ask for a 10 minute charge - for safety since this is still a test
	outputFrame.data[3] = 10; //how long we think the charge will take. Also 10 minutes for safety
	outputFrame.data[4] = 0; //not used
	outputFrame.data[5] = 0; //not used
	outputFrame.data[6] = 0; //not used
	outputFrame.data[7] = 0; //not used
	CAN.EnqueueTX(outputFrame);
}

void sendChademoStatus()
{
	Frame outputFrame;
	outputFrame.id = CARSIDE_CONTROL;
	outputFrame.dlc = 8;
	outputFrame.ide = 0;
	outputFrame.data[0] = 0; //claim to only support the chademo 0.9 protocol. It's safer/easier that way
	outputFrame.data[1] = lowByte(targetVoltage);
	outputFrame.data[2] = highByte(targetVoltage);
	outputFrame.data[3] = targetAmperage;
	outputFrame.data[4] = 0; //hard code claim to have no faults. Is that smart? Probably not.
	outputFrame.data[5] = 1; //enable charging, say we're in park, we have no faults, the contactor is shut, and we want to charge
	outputFrame.data[6] = packSize / 2; //always claim that the battery is at 50% charge. Also not particularly bright but probably OK for early testing
	outputFrame.data[7] = 0; //not used
	CAN.EnqueueTX(outputFrame);
}