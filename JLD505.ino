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
#include "globals.h"
#include "chademo.h"
/*
Notes on what needs to be done:
- Timing analysis showed that the USB, CANBUS, and BT routines take up entirely too much time. They can delay processing by
   almost 100ms!
*/

//#define DEBUG_TIMING	//if this is defined you'll get time related debugging messages

template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; } //Sets up serial streaming Serial<<someshit;

SoftwareSerial BTSerial(A2, A3); // RX | TX
AltSoftSerial altSerial; //pins 8 and 9

DS2480B ds(altSerial);
DallasTemperature sensors(&ds);

//These have been moved to eeprom. After initial compile the values will be read from EEPROM.
//These thus set the default value to write to eeprom upon first start up
#define MAX_CHARGE_V	180
#define MAX_CHARGE_A	120
#define TARGET_CHARGE_V	170
#define MIN_CHARGE_A	10
#define INITIAL_SOC 100
#define CAPACITY 180



INA226 ina;
const unsigned long Interval = 10;
unsigned long Time = 0; 
unsigned long PreviousMillis = 0;
unsigned long CurrentMillis = 0;
float Voltage = 0;
float Current = 0;
float Power = 0;
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
volatile uint8_t debugTick = 0;
<<<<<<< HEAD
typedef struct
{
	uint8_t valid; //a token to store EEPROM version and validity. If it matches expected value then EEPROM is not reset to defaults //0
	float ampHours; //floats are 4 bytes //1
	float kiloWattHours; //5
	float packSizeKWH; //9
	float voltageCalibration; //13
	float currentCalibration; //17
	uint16_t maxChargeVoltage; //21
	uint16_t targetChargeVoltage; //23
	uint8_t maxChargeAmperage; //25
	uint8_t minChargeAmperage; //26
        uint8_t capacity;
        uint8_t SOC;
} EESettings;
=======

>>>>>>> origin/debug
EESettings settings;
#define EEPROM_VALID	0xDE

void MCP2515_ISR()
{
    Flag_Recv = 1;
}

void timer2Int()
{
	timerFastCounter++;
	if (timerFastCounter == 8)
	{
		debugTick = 1;
		if (chademo.bChademoMode  && chademo.bChademoSendRequests) chademo.bChademoRequest = 1;
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

	EEPROM_readAnything(256, settings);
	if (settings.valid != EEPROM_VALID) //not proper version so reset to defaults
	{
		settings.valid = EEPROM_VALID;
		settings.ampHours = 0.0;
		settings.kiloWattHours = 0.0;
		settings.currentCalibration = 300.0/0.075; //800A 75mv shunt
		settings.voltageCalibration = (100000.0*830000.0/930000.0+1000000.0)/(100275.0*830000.0/930000.0); // (Voltage Divider with (100k in parallel with 830k) and 1M )
		settings.packSizeKWH = 15.0; //just a random guess. Maybe it should default to zero though?
		settings.maxChargeAmperage = MAX_CHARGE_A;
		settings.maxChargeVoltage = MAX_CHARGE_V;
		settings.targetChargeVoltage = TARGET_CHARGE_V;
		settings.minChargeAmperage = MIN_CHARGE_A;
<<<<<<< HEAD
                settings.SOC=INITIAL_SOC;
                settings.capacity=CAPACITY;
=======
        settings.SOC=INITIAL_SOC;
        settings.capacity=CAPACITY;
		settings.debuggingLevel = 2;
>>>>>>> origin/debug
                
     		EEPROM_writeAnything(256, settings);
	}

	settings.debuggingLevel = 2; //locked in to max debugging for now.

	attachInterrupt(0, Save, FALLING);
	FrequencyTimer2::setPeriod(25000); //interrupt every 25ms
	FrequencyTimer2::setOnOverflow(timer2Int);
	
	if (settings.debuggingLevel > 0)
	{
		Serial.print(F("Found "));
		tempSensorCount = sensors.getDeviceCount(); 
		Serial.print(tempSensorCount);
		Serial.println(F(" temperature sensors."));
	}

	chademo.setTargetAmperage(settings.maxChargeAmperage);
	chademo.setTargetVoltage(settings.targetChargeVoltage);
}

void loop()
{
	uint8_t pos;
	CurrentMillis = millis();
	uint8_t len;

#ifdef DEBUG_TIMING
	if (debugTick == 1)
	{
		debugTick = 0;
		Serial.println(millis());
	}
#endif 

	chademo.loop();
 
	if(CurrentMillis - PreviousMillis >= Interval)
	{
		Time = CurrentMillis - PreviousMillis;
		PreviousMillis = CurrentMillis;   
    
		Count++;
		Voltage = ina.readBusVoltage() * settings.voltageCalibration;
		Current = ina.readShuntVoltage() * settings.currentCalibration;
		settings.ampHours += Current * (float)Time / 1000.0 / 3600.0;
		Power = Voltage * Current / 1000.0;
		settings.kiloWattHours += Power * (float)Time / 1000.0 / 3600.0;
                settings.SOC=((settings.capacity-settings.ampHours)/settings.capacity)*100;

		chademo.doProcessing();

		if (Count >= 50)
		{
			Count = 0;
			USB();												
			CANBUS();							
			if (!chademo.bChademoMode) //save some processor time by not doing these in chademo mode
			{					
				BT();
			}
<<<<<<< HEAD
                         //Constant Current/Constant Voltage Taper checks.  If minimum current is set to zero, we terminate once target voltage is reached.
                        //If not zero, we will adjust current up or down as needed to maintain voltage until current decreases to the minimum entered
                
                        if(Count==20)  //To allow batteries time to react, we only do this once in 50 counts
                          {
                            if (Voltage > settings.targetChargeVoltage-1) //All initializations complete and we're running.We've reached charging target
                              {
                                settings.SOC=100;
                                settings.ampHours=0;
                                settings.kiloWattHours=0;
                                if (settings.minChargeAmperage == 0 || carStatus.targetCurrent < settings.minChargeAmperage) chademoState = CEASE_CURRENT;  //Terminate charging
                                   else carStatus.targetCurrent--;  //Taper. Actual decrease occurs in sendChademoStatus                                   
                              }
                              else //Only adjust upward if we have previous adjusted downward and do not exceed max amps
                                {
                                 if (carStatus.targetCurrent < settings.maxChargeAmperage)carStatus.targetCurrent++;  
                                }
                          }
 		}

			if (Count >= 50)
			{
				Count = 0;
				USB();	
				CANBUS();							
				if (!bChademoMode) //save some processor time by not doing these in chademo mode
				{
					
					BT();
				}
				else 
				{
					Serial.print(F("Chademo Mode: "));
					Serial.println(chademoState);
				}
				Save();
=======
			else if (settings.debuggingLevel > 0) 
			{		
				Serial.print(F("Chademo Mode: "));
				Serial.println(chademo.getState());
>>>>>>> origin/debug
			}
			Save();
		}
	}

	if (Flag_Recv || CAN.checkReceive() == CAN_MSGAVAIL) {
		Flag_Recv = 0;		
		CAN.readMsgBuf(&len, canMsg);            // read data,  len: data length, buf: data buf
		canMsgID = CAN.getCanId();
<<<<<<< HEAD
		if (canMsgID == EVSE_PARAMS)
		{
			if (chademoState == WAIT_FOR_EVSE_PARAMS) chademoDelayedState(SET_CHARGE_BEGIN, 100);
			evse_params.supportWeldCheck = canMsg[0];
			evse_params.availVoltage = canMsg[1] + canMsg[2] * 256;
			evse_params.availCurrent = canMsg[3];			
			evse_params.thresholdVoltage = canMsg[4] + canMsg[5] * 256;
                        Serial.print(F("EVSE: MaxVoltage: "));
                        Serial.print(evse_params.availVoltage);
                        Serial.print(F(" Max Current:"));
                        Serial.print(evse_params.availCurrent);
                        Serial.print(F(" Threshold Voltage:"));
                        Serial.print(evse_params.thresholdVoltage);
                        timestamp();
			//if charger cannot provide our requested voltage then GTFO
			if (evse_params.availVoltage < carStatus.targetVoltage)
			{
				Serial.print(F("EVSE can't provide needed voltage. Aborting."));
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

                        Serial.print(F("EVSE: Measured Voltage: "));
                        Serial.print(evse_status.presentVoltage);
                        Serial.print(F(" Current: "));
                        Serial.print(evse_status.presentCurrent);
                        Serial.print(F(" Time remaining: "));
                        Serial.print(evse_status.remainingChargeSeconds);
                        Serial.print(F(" Status: "));
                        Serial.print(evse_status.status,BIN);
                         timestamp();
                   

			//on fault try to turn off current immediately and cease operation
			if ((evse_status.status & 0x1A) != 0) //if bits 1, 3, or 4 are set then we have a problem.
			{
				Serial.println(F("EVSE:fault! Abort."));
				if (chademoState == RUNNING) chademoState = CEASE_CURRENT;
			}
			
			if (chademoState == RUNNING)
			{
				if (bListenEVSEStatus)
				{
					if ((evse_status.status & EVSE_STATUS_STOPPED) != 0)
					{
					Serial.println(F("EVSE:stop charging."));
						chademoState = CEASE_CURRENT;
					}

					//if there is no remaining time then gracefully shut down
					if (evse_status.remainingChargeSeconds == 0)
					{
					Serial.println(F("EVSE:time elapsed..Ending"));
						chademoState = CEASE_CURRENT;
					}
				}
				else
				{
					//if charger is not reporting being stopped and is reporting remaining time then enable the checks.
					if ((evse_status.status & EVSE_STATUS_STOPPED) == 0 && evse_status.remainingChargeSeconds > 0) bListenEVSEStatus = 1;
				}
			}
		}
=======
		chademo.handleCANFrame(canMsgID, canMsg);
>>>>>>> origin/debug
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
			if (settings.debuggingLevel > 0)
			{
				Serial.print(pos);
				Serial.print(": ");
				//sensors.isConnected(pos);
				// Serial.println(sensors.getCelsius(pos));
				Serial.println(sensors.getTempCByIndex(pos));
			}
		}
	}
}

void Save()
{
	EEPROM_writeAnything(256, settings);
}  

void USB()
{
  Serial.print (Voltage, 3);   
  Serial.print ("V ");
  Serial.print (Current, 2);    
  Serial.print ("A ");
  Serial.print (settings.ampHours, 1);    
  Serial.print ("Ah ");
  Serial.print (Power, 1);        
  Serial.print ("kW ");
  Serial.print (settings.kiloWattHours, 1);    
  Serial.println ("kWh");
  if (Serial.available() > 0)
  {Command = Serial.read();
   if (Command == 'z')
     {
      Serial.println("Reset Ah & Wh");
	  settings.ampHours = 0.0;
	  settings.kiloWattHours = 0.0;
      Serial.println("Done!!!"); 
    }
     if (Command == '+')
     {
		settings.voltageCalibration +=0.004;
		Serial.println (settings.voltageCalibration, 5);   
    }
     if (Command == '-')
     {
		settings.voltageCalibration -=0.004;
		Serial.println (settings.voltageCalibration, 5);   
    }
   while(Serial.available()>0) Serial.read();}
}

void BT()
{
	
  BTSerial.print (Voltage, 2);   
  BTSerial.print ("V ");
  BTSerial.print (Current, 2);    
  BTSerial.print ("A ");
  BTSerial.print (settings.ampHours, 1);    
  BTSerial.print ("Ah ");
  BTSerial.print (Power, 1);        
  BTSerial.print ("kW ");
  BTSerial.print (settings.kiloWattHours, 1);    
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
	{
		Command = BTSerial.read();
		if (Command == 'z')
		{
			settings.ampHours = 0.0;
			settings.kiloWattHours = 0.0;
		}
		while(BTSerial.available()>0) BTSerial.read();
	}
}

void CANBUS()
{
	canMsgID = 0x404;
        uint16_t currINT = abs(Current*10);
	canMsg[0] = highByte((int)(Voltage * 10)); // Voltage High Byte
	canMsg[1] = lowByte((int)(Voltage * 10)); // Voltage Low Byte
	canMsg[2] = highByte(currINT); // Current High Byte
	canMsg[3] = lowByte(currINT); // Current Low Byte
	canMsg[4] = highByte((int)(settings.ampHours * 10)); // AmpHours High Byte
	canMsg[5] = lowByte((int)(settings.ampHours * 10)); // AmpHours Low Byte
	canMsg[6] = settings.capacity; // Not Used
	canMsg[7] = settings.SOC; // Not Used
	CAN.sendMsgBuf(canMsgID, 0, 8, canMsg);
	
  
	canMsgID = 0x505;
        uint16_t Pwr=abs(Power*10);
        uint16_t KWH=abs(settings.kiloWattHours *10);
	canMsg[0] = highByte(Pwr); // Power High Byte
	canMsg[1] = lowByte(Pwr); // Power Low Byte
	canMsg[2] = highByte(KWH); // KiloWattHours High Byte
	canMsg[3] = lowByte(KWH); // KiloWattHours Low Byte
	canMsg[4] = 0x00; // Not Used
	canMsg[5] = 0x00; // Not Used
	canMsg[6] = 0x00; // Not Used
	canMsg[7] = 0x00; // Not Used
	CAN.sendMsgBuf(canMsgID, 0, 4, canMsg);
 }

<<<<<<< HEAD
void sendChademoBattSpecs()
{
	
	canMsgID = CARSIDE_BATT;
	canMsg[0] = 0x00; // Not Used
	canMsg[1] = 0x00; // Not Used
	canMsg[2] = 0x00; // Not Used
	canMsg[3] = 0x00; // Not Used
	canMsg[4] = lowByte(settings.maxChargeVoltage);
	canMsg[5] = highByte(settings.maxChargeVoltage); 
	canMsg[6] = (uint8_t)settings.packSizeKWH;
	canMsg[7] = 0; //not used
	CAN.sendMsgBuf(canMsgID, 0, 8, canMsg);

             Serial.print(F("CAR: Absolute MAX Voltage:"));
             Serial.print(settings.maxChargeVoltage);
             Serial.print(F(" Pack size: "));
             Serial.print(settings.packSizeKWH);
             timestamp();
                 
}

void sendChademoChargingTime()
{
	
	canMsgID = CARSIDE_CHARGETIME;
	canMsg[0] = 0x00; // Not Used
	canMsg[1] = 0xFF; //not using 10 second increment mode
	canMsg[2] = 90; //ask for how long of a charge? It will be forceably stopped if we hit this time
	canMsg[3] = 60; //how long we think the charge will actually take
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
	canMsg[0] = 2; //tell EVSE we are talking 1.0 protocol
	canMsg[1] = lowByte(carStatus.targetVoltage);
	canMsg[2] = highByte(carStatus.targetVoltage);
	canMsg[3] = askingAmps;
	canMsg[4] = faults;
	canMsg[5] = status;
	canMsg[6] = (uint8_t)settings.kiloWattHours;
	canMsg[7] = 0; //not used
	CAN.sendMsgBuf(canMsgID, 0, 8, canMsg);
             Serial.print(F("CAR: Protocol:"));
             Serial.print(canMsg[0]);
             Serial.print(F(" Target Voltage: "));
             Serial.print(carStatus.targetVoltage);
             Serial.print(F(" Current Command: "));
             Serial.print(askingAmps);
             Serial.print(F(" Faults: "));
             Serial.print(faults,BIN);
             Serial.print(F(" Status: "));
             Serial.print(status,BIN);
             Serial.print(F(" kWh: "));
             Serial.print(settings.kiloWattHours);
             timestamp();
                 
	if (chademoState == RUNNING &&  askingAmps < carStatus.targetCurrent) askingAmps++;
	//not a typo. We're allowed to change requested amps by +/- 20A per second. We send the above frame every 100ms so a single
	//increment means we can ramp up 10A per second. But, we want to ramp down quickly if there is a problem so do two which
	//gives us -20A per second.
	if (chademoState != RUNNING && askingAmps > 0) askingAmps--;
	if (askingAmps > carStatus.targetCurrent) askingAmps--;
	if (askingAmps > carStatus.targetCurrent) askingAmps--;
}

=======
>>>>>>> origin/debug
void timestamp()
{
	int milliseconds = (int) (millis()/1) %1000 ;
	int seconds = (int) (millis() / 1000) % 60 ;
	int minutes = (int) ((millis() / (1000*60)) % 60);
	int hours   = (int) ((millis() / (1000*60*60)) % 24);
            
	Serial.print(F(" Time:"));
	Serial.print(hours);
	Serial.print(F(":"));
	Serial.print(minutes);
	Serial.print(F(":"));
	Serial.print(seconds);
	Serial.print(F("."));
	Serial.println(milliseconds);    
}
