/*
chademo.cpp - Houses all chademo related functionality
*/

#include "chademo.h"

template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; } //Sets up serial streaming Serial<<someshit;
void timestamp();

CHADEMO::CHADEMO()
{
	bStartedCharge = 0;
	bChademoMode = 0;
	bChademoSendRequests = 0;
	bChademoRequest = 0;
	bChademo10Protocol = 0;
	askingAmps = 0;
	bListenEVSEStatus = 0;
	bDoMismatchChecks = 0;
	insertionTime = 0;
	chademoState = STOPPED;
	stateHolder = STOPPED;
	carStatus.contactorOpen = 1;
    carStatus.battOverTemp = 0;

}

//will wait delayTime milliseconds and then transition to new state. Sets state to LIMBO in the meantime
void CHADEMO::setDelayedState(int newstate, uint16_t delayTime)
{
	chademoState = LIMBO;
	stateHolder = (CHADEMOSTATE)newstate;
	stateMilli = millis() + delayTime;
}

CHADEMOSTATE CHADEMO::getState()
{
	return chademoState;
}

void CHADEMO::setTargetAmperage(uint8_t t_amp)
{
	carStatus.targetCurrent = t_amp;
}

void CHADEMO::setTargetVoltage(uint16_t t_volt)
{
	carStatus.targetVoltage = t_volt;
}

//stuff that should be frequently run (as fast as possible)
void CHADEMO::loop()
{
	if (!bDoMismatchChecks && chademoState == RUNNING)
	{
		if (CurrentMillis > mismatchStart) bDoMismatchChecks = 1;
	}

	if (chademoState == LIMBO && CurrentMillis > stateMilli)
	{
		chademoState = stateHolder;
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
			if (settings.debuggingLevel > 0)
			{
				Serial<<"CAR: Contactor open\n";
				Serial<<"CAR: Charge Enable OFF\n";
			}
		}
	}

	if (bChademoMode)
	{
		if (bChademoSendRequests && bChademoRequest)
		{
			bChademoRequest = 0;
			sendCANStatus();
			sendCANBattSpecs();
			sendCANChargingTime();
			if (settings.debuggingLevel > 1) Serial.println("Tx");
		}

		switch (chademoState)
		{
		case STARTUP: 
			bDoMismatchChecks = 0; //reset it for now
			setDelayedState(SEND_INITIAL_PARAMS, 50);
			break;
		case SEND_INITIAL_PARAMS:
			//we could do calculations to see how long the charge should take based on SOC and 
			//also set a more realistic starting amperage. Options for the future.
			//One problem with that is that we don't yet know the EVSE parameters so we can't know
			//the max allowable amperage just yet.
			bChademoSendRequests = 1; //causes chademo frames to be sent out every 100ms
			setDelayedState(WAIT_FOR_EVSE_PARAMS, 50);
			if (settings.debuggingLevel > 0) Serial.println(F("Sent params to EVSE. Waiting."));
			break;
		case WAIT_FOR_EVSE_PARAMS:
			//for now do nothing while we wait. Might want to try to resend start up messages periodically if no reply
			break;
		case SET_CHARGE_BEGIN:
			if (settings.debuggingLevel > 0) Serial.println(F("CAR:Charge enable ON"));
			digitalWrite(OUT1, HIGH); //signal that we're ready to charge
			carStatus.chargingEnabled = 1; //should this be enabled here???
			setDelayedState(WAIT_FOR_BEGIN_CONFIRMATION, 50);
			break;
		case WAIT_FOR_BEGIN_CONFIRMATION:
			if (digitalRead(IN0)) //inverse logic from how IN1 works. Be careful!
			{
				setDelayedState(CLOSE_CONTACTORS, 100);
			}
			break;
		case CLOSE_CONTACTORS:
			if (settings.debuggingLevel > 0) Serial.println(F("CAR:Contactor close."));
			digitalWrite(OUT0, HIGH);
			setDelayedState(RUNNING, 50);
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
			if (settings.debuggingLevel > 0) Serial.println(F("CAR:Current req to 0"));
			carStatus.targetCurrent = 0;
			chademoState = WAIT_FOR_ZERO_CURRENT;
			break;
		case WAIT_FOR_ZERO_CURRENT:
			if (evse_status.presentCurrent == 0)
			{
				setDelayedState(OPEN_CONTACTOR, 150);
			}
			break;
		case OPEN_CONTACTOR:
			if (settings.debuggingLevel > 0) Serial.println(F("CAR:Contactor OPEN"));
			digitalWrite(OUT0, LOW);
			carStatus.contactorOpen = 1;
			carStatus.chargingEnabled = 0;
			sendCANStatus(); //we probably need to force this right now
			setDelayedState(STOPPED, 100);
			break;
		case FAULTED:
			Serial.println(F("CAR: fault!"));
			chademoState = CEASE_CURRENT;
			//digitalWrite(OUT0, LOW);
			//digitalWrite(OUT1, LOW);
			break;
		case STOPPED:
			if (bChademoSendRequests == 1)
			{
			digitalWrite(OUT0, LOW);
			digitalWrite(OUT1, LOW);
			if (settings.debuggingLevel > 0) 
			{
				Serial.println(F("CAR:Contactor OPEN"));
				Serial.println(F("CAR:Charge Enable OFF"));
			}
			bChademoSendRequests = 0; //don't need to keep sending anymore.
			bListenEVSEStatus = 0; //don't want to pay attention to EVSE status when we're stopped
			}
			break;
		}
	}
}

//things that are less frequently run - run on a set schedule
void CHADEMO::doProcessing()
{
	uint8_t tempCurrVal;
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
                
		if(Count==20)
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
}

void CHADEMO::handleCANFrame(uint32_t ID, unsigned char *data)
{
	if (ID == EVSE_PARAMS_ID)
	{
		if (chademoState == WAIT_FOR_EVSE_PARAMS) setDelayedState(SET_CHARGE_BEGIN, 100);
		evse_params.supportWeldCheck = data[0];
		evse_params.availVoltage = data[1] + data[2] * 256;
		evse_params.availCurrent = data[3];			
		evse_params.thresholdVoltage = data[4] + data[5] * 256;
		if (settings.debuggingLevel > 1) 
		{
			Serial.print(F("EVSE: MaxVoltage: "));
			Serial.print(evse_params.availVoltage);
			Serial.print(F(" Max Current:"));
			Serial.print(evse_params.availCurrent);
			Serial.print(F(" Threshold Voltage:"));
			Serial.print(evse_params.thresholdVoltage);
			timestamp();
		}
		
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

	if (ID == EVSE_STATUS_ID)
	{
		if (data[0] > 1) bChademo10Protocol = 1;
		evse_status.presentVoltage = data[1] + 256 * data[2];
		evse_status.presentCurrent  = data[3];
		evse_status.status = data[5];				
		if (data[6] < 0xFF)
		{
			evse_status.remainingChargeSeconds = data[6] * 10;
		}
		else 
		{
			evse_status.remainingChargeSeconds = data[7] * 60;
		}
		if (settings.debuggingLevel > 1) 
		{
			Serial.print(F("EVSE: Measured Voltage: "));
			Serial.print(evse_status.presentVoltage);
			Serial.print(F(" Current: "));
			Serial.print(evse_status.presentCurrent);
			Serial.print(F(" Time remaining: "));
			Serial.print(evse_status.remainingChargeSeconds);
			Serial.print(F(" Status: "));
			Serial.print(evse_status.status,BIN);
			timestamp();
		}                

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

void CHADEMO::sendCANBattSpecs()
{	
	uint32_t canMsgID;
	unsigned char canMsg[8];

	canMsgID = CARSIDE_BATT_ID;
	canMsg[0] = 0x00; // Not Used
	canMsg[1] = 0x00; // Not Used
	canMsg[2] = 0x00; // Not Used
	canMsg[3] = 0x00; // Not Used
	canMsg[4] = lowByte(settings.maxChargeVoltage);
	canMsg[5] = highByte(settings.maxChargeVoltage); 
	canMsg[6] = (uint8_t)settings.packSizeKWH;
	canMsg[7] = 0; //not used
	CAN.sendMsgBuf(canMsgID, 0, 8, canMsg);
	if (settings.debuggingLevel > 1)
	{
		Serial.print(F("CAR: Absolute MAX Voltage:"));
		Serial.print(settings.maxChargeVoltage);
		Serial.print(F(" Pack size: "));
		Serial.print(settings.packSizeKWH);
		timestamp();
	}            
}

void CHADEMO::sendCANChargingTime()
{
	uint32_t canMsgID;
	unsigned char canMsg[8];

	canMsgID = CARSIDE_CHARGETIME_ID;
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

void CHADEMO::sendCANStatus()
{
	uint8_t faults = 0;
	uint8_t status = 0;
	uint32_t canMsgID;
	unsigned char canMsg[8];

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

	canMsgID = CARSIDE_CONTROL_ID;
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

	if (settings.debuggingLevel > 1)
	{
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
	}

	if (chademoState == RUNNING &&  askingAmps < carStatus.targetCurrent) askingAmps++;
	//not a typo. We're allowed to change requested amps by +/- 20A per second. We send the above frame every 100ms so a single
	//increment means we can ramp up 10A per second. But, we want to ramp down quickly if there is a problem so do two which
	//gives us -20A per second.
	if (chademoState != RUNNING && askingAmps > 0) askingAmps--;
	if (askingAmps > carStatus.targetCurrent) askingAmps--;
	if (askingAmps > carStatus.targetCurrent) askingAmps--;
}

CHADEMO chademo;