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
/*
Notes on what needs to be done:
- Timing analysis showed that the USB, CANBUS, and BT routines take up entirely too much time. They can delay processing by
   almost 100ms!
*/

//#define DEBUG_TIMING	//if this is defined you'll get time related debugging messages
#define DEBUGLEVEL	2 //0 = no debugging, 1 = mild debugging, 2 = behold, the avalanche

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


//set the proper digital pins for these
#define IN0		4
#define IN1		7
#define OUT0	5
#define OUT1	6

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
    uint8_t capacity; //27
    uint8_t SOC; //28
} EESettings;
EESettings settings;
#define EEPROM_VALID	0xDE

//Bunch o' chademo related stuff. 
uint8_t bStartedCharge = 0; //we have started a charge since the plug was inserted. Prevents attempts to restart charging if it stopped previously
uint8_t bChademoMode = 0; //accessed but not modified in ISR so it should be OK non-volatile
uint8_t bChademoSendRequests = 0; //should we be sending periodic status updates?
volatile uint8_t bChademoRequest = 0;  //is it time to send one of those updates?
uint8_t bChademo10Protocol = 0; //can we use 1.0 protocol?
//target values are what we send with periodic frames and can be changed.
uint8_t askingAmps = 0; //how many amps to ask for. Trends toward targetAmperage
uint8_t bListenEVSEStatus = 0; //should we pay attention to stop requests and such yet?
uint8_t bDoMismatchChecks = 0; //should we be checking for voltage and current mismatches?
uint32_t mismatchStart;
uint32_t stateMilli;
uint32_t insertionTime = 0;
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
	STOPPED,
	LIMBO
};
CHADEMOSTATE chademoState = STOPPED;
CHADEMOSTATE stateHolder = STOPPED;

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
#define EVSE_STATUS_ERR			2 //something went wrong
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
	if (timerFastCounter == 8)
	{
		debugTick = 1;
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
  
//will wait delayTime milliseconds and then transition to new state. Sets state to LIMBO in the meantime
void chademoDelayedState(int newstate, uint16_t delayTime)
{
	chademoState = LIMBO;
	stateHolder = (CHADEMOSTATE)newstate;
	stateMilli = millis() + delayTime;
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
        settings.SOC=INITIAL_SOC;
        settings.capacity=CAPACITY;
                
		EEPROM_writeAnything(256, settings);
	}

	attachInterrupt(0, Save, FALLING);
	FrequencyTimer2::setPeriod(25000); //interrupt every 25ms
	FrequencyTimer2::setOnOverflow(timer2Int);
	
#if DEBUGLEVEL > 0
	Serial.print(F("Found "));
	tempSensorCount = sensors.getDeviceCount(); 
	Serial.print(tempSensorCount);
	Serial.println(F(" temperature sensors."));
#endif

	//carStatus.targetCurrent = settings.maxChargeAmperage;
	carStatus.targetCurrent = 90; //hard coded target amperage
	carStatus.targetVoltage = settings.targetChargeVoltage;
	carStatus.contactorOpen = 1;
        carStatus.battOverTemp = 0;
}

void loop()
{
	uint8_t pos;
	CurrentMillis = millis();
	uint8_t len;
	uint8_t tempCurrVal;

#ifdef DEBUG_TIMING
	if (debugTick == 1)
	{
		debugTick = 0;
		Serial.println(millis());
	}
#endif 
 
	if(CurrentMillis - PreviousMillis >= Interval)
	{
		Time = CurrentMillis - PreviousMillis;
		PreviousMillis = CurrentMillis;   

		if (!bDoMismatchChecks && chademoState == RUNNING)
		{
			if (CurrentMillis > mismatchStart) bDoMismatchChecks = 1;
		}

		if (chademoState == LIMBO && CurrentMillis > stateMilli)
		{
			chademoState = stateHolder;
		}
    
		Count++;
		Voltage = ina.readBusVoltage() * settings.voltageCalibration;
		Current = ina.readShuntVoltage() * settings.currentCalibration;
		settings.ampHours += Current * (float)Time / 1000.0 / 3600.0;
		Power = Voltage * Current / 1000.0;
		settings.kiloWattHours += Power * (float)Time / 1000.0 / 3600.0;
        settings.SOC=((settings.capacity-settings.ampHours)/settings.capacity)*100;

		if (chademoState == RUNNING && bDoMismatchChecks)
		{
			if (abs(Voltage - evse_status.presentVoltage) > (evse_status.presentVoltage >> 3) && !carStatus.voltDeviation)
			{
				Serial.println(F("Voltage mismatch! Aborting!"));
				carStatus.voltDeviation = 1;
				chademoState = CEASE_CURRENT;
			}

			tempCurrVal = evse_status.presentCurrent >> 3;
			if (tempCurrVal < 3) tempCurrVal = 3;
			if (abs((Current * -1.0) - evse_status.presentCurrent) > tempCurrVal && !carStatus.currDeviation)
			{
				Serial.println(F("Current mismatch! Aborting!"));
				carStatus.currDeviation = 1;
				chademoState = CEASE_CURRENT;
			}

			if (Voltage > settings.maxChargeVoltage)
			{
				Serial.println(F("Over voltage fault!"));
				carStatus.battOverVolt = 1;
				chademoState = CEASE_CURRENT;
			}
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
								if (carStatus.targetCurrent < settings.maxChargeAmperage) carStatus.targetCurrent++;  
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
#if DEBUGLEVEL > 0
					Serial.print(F("Chademo Mode: "));
					Serial.println(chademoState);
#endif
				}
				Save();
			}
	}

	if (Flag_Recv || CAN.checkReceive() == CAN_MSGAVAIL) {
		Flag_Recv = 0;		
		CAN.readMsgBuf(&len, canMsg);            // read data,  len: data length, buf: data buf
		canMsgID = CAN.getCanId();
		if (canMsgID == EVSE_PARAMS)
		{
			if (chademoState == WAIT_FOR_EVSE_PARAMS) chademoDelayedState(SET_CHARGE_BEGIN, 100);
			evse_params.supportWeldCheck = canMsg[0];
			evse_params.availVoltage = canMsg[1] + canMsg[2] * 256;
			evse_params.availCurrent = canMsg[3];			
			evse_params.thresholdVoltage = canMsg[4] + canMsg[5] * 256;
#if DEBUGLEVEL > 1
			Serial.print(F("EVSE: MaxVoltage: "));
			Serial.print(evse_params.availVoltage);
			Serial.print(F(" Max Current:"));
			Serial.print(evse_params.availCurrent);
			Serial.print(F(" Threshold Voltage:"));
			Serial.print(evse_params.thresholdVoltage);
			timestamp();
#endif
			//if charger cannot provide our requested voltage then GTFO
			if (evse_params.availVoltage < carStatus.targetVoltage)
			{
				Serial.print(F("EVSE can't provide needed voltage. Aborting."));
				Serial.println(evse_params.availVoltage);
				chademoState = CEASE_CURRENT;
			}

			//if we want more current then it can provide then revise our request to match max output
			if (evse_params.availCurrent < carStatus.targetCurrent) carStatus.targetCurrent = evse_params.availCurrent;

			//If not in running then also change our target current up to the minimum between the 
			//available current reported and the max charge amperage. This should fix an issue where
			//the target current got wacked for some reason and left at zero.
			if (chademoState != RUNNING && evse_params.availCurrent > carStatus.targetCurrent)
			{
				carStatus.targetCurrent = min(evse_params.availCurrent, settings.maxChargeAmperage);
			}
		}
		if (canMsgID == EVSE_STATUS)
		{
			if (canMsg[0] > 1) bChademo10Protocol = 1;
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
#if DEBUGLEVEL > 1
			Serial.print(F("EVSE: Measured Voltage: "));
			Serial.print(evse_status.presentVoltage);
			Serial.print(F(" Current: "));
			Serial.print(evse_status.presentCurrent);
			Serial.print(F(" Time remaining: "));
			Serial.print(evse_status.remainingChargeSeconds);
			Serial.print(F(" Status: "));
			Serial.print(evse_status.status,BIN);
			timestamp();
#endif                   

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
#if DEBUGLEVEL > 0
			Serial.print(pos);
			Serial.print(": ");
			//sensors.isConnected(pos);
			// Serial.println(sensors.getCelsius(pos));
			Serial.println(sensors.getTempCByIndex(pos));
#endif
		}
	}

	if (!digitalRead(IN1)) //IN1 goes low if we have been plugged into the chademo port
	{
		if (insertionTime == 0)
		{
			insertionTime = millis();
		}
		else if (millis() > insertionTime + 500)
		{
			if (bChademoMode == 0)
			{
				bChademoMode = 1;
				if (chademoState == STOPPED && !bStartedCharge) {
					chademoState = STARTUP;
					Serial.println(F("Starting Chademo process."));
					carStatus.battOverTemp = 0;
					carStatus.battOverVolt = 0;
					carStatus.battUnderVolt = 0;
					carStatus.chargingFault = 0;
					carStatus.chargingEnabled = 0;
					carStatus.contactorOpen = 1;
					carStatus.currDeviation = 0;
					carStatus.notParked = 0;
					carStatus.stopRequest = 0;
					carStatus.voltDeviation = 0;
					bChademo10Protocol = 0;
				}
			}
		}
	}
	else
	{
		insertionTime = 0;
		if (bChademoMode == 1)
		{
			Serial.println(F("Stopping chademo process."));
			bChademoMode = 0;
			bStartedCharge = 0;
			chademoState = STOPPED;
			//maybe it would be a good idea to try to see if EVSE is still transmitting to us and providing current
			//as it is not a good idea to open the contactors under load. But, IN1 shouldn't trigger 
			//until the EVSE is ready. Also, the EVSE should have us locked so the only way the plug should come out under
			//load is if the idiot driver took off in the car. Bad move moron.
			digitalWrite(OUT0, LOW);
			digitalWrite(OUT1, LOW);
#if DEBUGLEVEL > 0
			Serial<<"CAR: Contactor open\n";
			Serial<<"CAR: Charge Enable OFF\n";
#endif
		}
	}

	if (bChademoMode)
	{
		if (bChademoSendRequests && bChademoRequest)
		{
			bChademoRequest = 0;
			sendChademoStatus();
			sendChademoBattSpecs();
			sendChademoChargingTime();
#if DEBUGLEVEL > 1
			Serial.println("Tx");
#endif
		}

		switch (chademoState)
		{
		case STARTUP: 
			bDoMismatchChecks = 0; //reset it for now
			chademoDelayedState(SEND_INITIAL_PARAMS, 50);
			break;
		case SEND_INITIAL_PARAMS:
			//we could do calculations to see how long the charge should take based on SOC and 
			//also set a more realistic starting amperage. Options for the future.
			//One problem with that is that we don't yet know the EVSE parameters so we can't know
			//the max allowable amperage just yet.
			bChademoSendRequests = 1; //causes chademo frames to be sent out every 100ms
			chademoDelayedState(WAIT_FOR_EVSE_PARAMS, 50);
#if DEBUGLEVEL > 0			
			Serial.println(F("Sent params to EVSE. Waiting."));
#endif
			break;
		case WAIT_FOR_EVSE_PARAMS:
			//for now do nothing while we wait. Might want to try to resend start up messages periodically if no reply
			break;
		case SET_CHARGE_BEGIN:
#if DEBUGLEVEL > 0
			Serial.println(F("CAR:Charge enable ON"));
#endif
			digitalWrite(OUT1, HIGH); //signal that we're ready to charge
			carStatus.chargingEnabled = 1; //should this be enabled here???
			chademoDelayedState(WAIT_FOR_BEGIN_CONFIRMATION, 50);
			break;
		case WAIT_FOR_BEGIN_CONFIRMATION:
			if (digitalRead(IN0)) //inverse logic from how IN1 works. Be careful!
			{
				chademoDelayedState(CLOSE_CONTACTORS, 100);
			}
			break;
		case CLOSE_CONTACTORS:
#if DEBUGLEVEL > 0
			Serial.println(F("CAR:Contactor close."));
#endif
			digitalWrite(OUT0, HIGH);
			chademoDelayedState(RUNNING, 50);
			carStatus.contactorOpen = 0; //its closed now
			carStatus.chargingEnabled = 1; //please sir, I'd like some charge
			bStartedCharge = 1;
			mismatchStart = millis() + 10000; //start mismatch checks 10 seconds after we start the charge			
			break;
		case RUNNING:
			//do processing here by taking our measured voltage, amperage, and SOC to see if we should be commanding something
			//different to the EVSE. Also monitor temperatures to make sure we're not incinerating the pack.
			break;
		case CEASE_CURRENT:
#if DEBUGLEVEL > 0
			Serial.println(F("CAR:Current req to 0"));
#endif
			carStatus.targetCurrent = 0;
			chademoState = WAIT_FOR_ZERO_CURRENT;
			break;
		case WAIT_FOR_ZERO_CURRENT:
			if (evse_status.presentCurrent == 0)
			{
				chademoDelayedState(OPEN_CONTACTOR, 150);
			}
			break;
		case OPEN_CONTACTOR:
#if DEBUGLEVEL > 0
			Serial.println(F("CAR:Contactor OPEN"));
#endif
			digitalWrite(OUT0, LOW);
			carStatus.contactorOpen = 1;
			carStatus.chargingEnabled = 0;
			sendChademoStatus(); //we probably need to force this right now
			chademoDelayedState(STOPPED, 100);
			break;
		case FAULTED:
#if DEBUGLEVEL > 0
			Serial.println(F("CAR: fault!"));
#endif
			chademoState = CEASE_CURRENT;
			//digitalWrite(OUT0, LOW);
			//digitalWrite(OUT1, LOW);
			break;
		case STOPPED:
			if (bChademoSendRequests == 1)
			{
			digitalWrite(OUT0, LOW);
			digitalWrite(OUT1, LOW);
#if DEBUGLEVEL > 0
			Serial.println(F("CAR:Contactor OPEN"));
            Serial.println(F("CAR:Charge Enable OFF"));
#endif
			bChademoSendRequests = 0; //don't need to keep sending anymore.
			bListenEVSEStatus = 0; //don't want to pay attention to EVSE status when we're stopped
			}
			break;
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
	canMsg[0] = highByte((int)(Voltage * 10)); // Voltage High Byte
	canMsg[1] = lowByte((int)(Voltage * 10)); // Voltage Low Byte
	canMsg[2] = highByte((int)(Current * 10)); // Current High Byte
	canMsg[3] = lowByte((int)(Current * 10)); // Current Low Byte
	canMsg[4] = highByte((int)(settings.ampHours * 10)); // AmpHours High Byte
	canMsg[5] = lowByte((int)(settings.ampHours * 10)); // AmpHours Low Byte
	canMsg[6] = settings.capacity; // Not Used
	canMsg[7] = settings.SOC; // Not Used
	CAN.sendMsgBuf(canMsgID, 0, 8, canMsg);
	
  
	canMsgID = 0x505;
	canMsg[0] = highByte((int)(Power * 10)); // Power High Byte
	canMsg[1] = lowByte((int)(Power * 10)); // Power Low Byte
	canMsg[2] = highByte((int)(settings.kiloWattHours * 10)); // KiloWattHours High Byte
	canMsg[3] = lowByte((int)(settings.kiloWattHours * 10)); // KiloWattHours Low Byte
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
	canMsg[4] = lowByte(settings.maxChargeVoltage);
	canMsg[5] = highByte(settings.maxChargeVoltage); 
	canMsg[6] = (uint8_t)settings.packSizeKWH;
	canMsg[7] = 0; //not used
	CAN.sendMsgBuf(canMsgID, 0, 8, canMsg);
#if DEBUGLEVEL > 1
	Serial.print(F("CAR: Absolute MAX Voltage:"));
	Serial.print(settings.maxChargeVoltage);
	Serial.print(F(" Pack size: "));
	Serial.print(settings.packSizeKWH);
	timestamp();
#endif
                 
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
	if (bChademo10Protocol)
	{
		if (carStatus.contactorOpen) status |= CARSIDE_STATUS_CONTOP;
		if (carStatus.stopRequest) status |= CARSIDE_STATUS_CHSTOP;
	}

	canMsgID = CARSIDE_CONTROL;
	if (bChademo10Protocol)	canMsg[0] = 2; //tell EVSE we are talking 1.0 protocol
	else canMsg[0] = 1; //talking 0.9 protocol
	canMsg[1] = lowByte(carStatus.targetVoltage);
	canMsg[2] = highByte(carStatus.targetVoltage);
	canMsg[3] = askingAmps;
	canMsg[4] = faults;
	canMsg[5] = status;
	canMsg[6] = (uint8_t)settings.kiloWattHours;
	canMsg[7] = 0; //not used
	CAN.sendMsgBuf(canMsgID, 0, 8, canMsg);

#if DEBUGLEVEL > 1
	Serial.print(F("CAR: Protocol:"));
	Serial.print(canMsg[0]);
	Serial.print(F(" Target Voltage: "));
	Serial.print(carStatus.targetVoltage);
	Serial.print(F(" Current Command: "));
	Serial.print(askingAmps);
	Serial.print(F(" Target Amps: "));
	Serial.print(carStatus.targetCurrent);
	Serial.print(F(" Faults: "));
	Serial.print(faults,BIN);
	Serial.print(F(" Status: "));
	Serial.print(status,BIN);
	Serial.print(F(" kWh: "));
	Serial.print(settings.kiloWattHours);
    timestamp();
#endif

	if (chademoState == RUNNING &&  askingAmps < carStatus.targetCurrent) askingAmps++;
	//not a typo. We're allowed to change requested amps by +/- 20A per second. We send the above frame every 100ms so a single
	//increment means we can ramp up 10A per second. But, we want to ramp down quickly if there is a problem so do two which
	//gives us -20A per second.
	if (chademoState != RUNNING && askingAmps > 0) askingAmps--;
	if (askingAmps > carStatus.targetCurrent) askingAmps--;
	if (askingAmps > carStatus.targetCurrent) askingAmps--;
}

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
