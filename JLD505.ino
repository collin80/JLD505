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

SoftwareSerial BTSerial(A2, A1); // RX | TX
AltSoftSerial altSerial; //pins 8 and 9

DS2480B ds(altSerial);
DallasTemperature sensors(&ds);

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
volatile uint8_t sensorReadPosition = 255;

void oneWireInt()
{
	timerIntCounter++;
	if (timerIntCounter < 10) 
	{
		bGetTemperature = 1;
		sensorReadPosition++;
	}
	if (timerIntCounter == 10)
	{
		bStartConversion = 1;
	}
	if (timerIntCounter == 18)
	{
		timerIntCounter = 0;
	}
}
  
void setup()
{  
  Serial.begin(115200);
  BTSerial.begin(38400);
  altSerial.begin(9600);
  sensors.begin();
  sensors.setWaitForConversion(false); //we're handling the time delay ourselves so no need to wait when asking for temperatures
  CAN.begin(CAN_500KBPS);
  ina.begin(69);
  ina.configure(INA226_AVERAGES_16, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
  EEPROM_readAnything(ADDR_AmpHours, AmpHours);
  EEPROM_readAnything(ADDR_KiloWattHours, KiloWattHours);
  EEPROM_readAnything(ADDR_VoltageCalibration, VoltageCalibration);
  attachInterrupt(0, Save, FALLING);
  FrequencyTimer2::setPeriod(100); //interrupt every 100ms
  FrequencyTimer2::setOnOverflow(oneWireInt);
}
void loop()
{
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

    if (Count >= 50)
    {Count = 0;
     USB();
     BT();
     CANBUS();
     Save();
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
	  if (sensorReadPosition < sensors.getDeviceCount())
	  {
		  //sensors.isConnected(sensorReadPosition);
		  //Serial.println(sensors.getCelsius(sensorReadPosition));
		  Serial.println(sensors.getTempCByIndex(sensorReadPosition));
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
  canMsgID  = 0x404;
  canMsg[0] = highByte((int)(Voltage*10)); // Voltage High Byte
  canMsg[1] = lowByte((int)(Voltage*10)); // Voltage Low Byte
  canMsg[2] = highByte((int)(Current*10)); // Current High Byte
  canMsg[3] = lowByte((int)(Current*10)); // Current Low Byte
  canMsg[4] = highByte((int)(AmpHours*10)); // AmpHours High Byte
  canMsg[5] = lowByte((int)(AmpHours*10)); // AmpHours Low Byte
  canMsg[6] = 0x00; // Not Used
  canMsg[7] = 0x00; // Not Used
  CAN.sendMsgBuf(canMsgID, 0, 6, canMsg);
  
  canMsgID  = 0x505;
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
