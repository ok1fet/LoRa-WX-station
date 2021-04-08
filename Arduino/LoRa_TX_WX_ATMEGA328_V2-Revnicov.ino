/*
 * uP ATMEGA328 ja pouzivam bootloader FIO
  * 
 * Pin
 * RFM96  UNO   TTGO
 * SCK    D13   5
 * MISO   D12   19
 * MOSI   D11   27
 * NSS    D4    18    lora_PNSS
 * RST    D5    23    lora_PReset
 * 
 * bme280
 * SCL    A5    
 * SDA    A4
 * 
 */
///////////////////////////////////////////////////////////
#include <LowPower.h>
unsigned int PeriodeCounterSleep = 0; //3 kdyz je stejne jako "timer" spusti mereni v prvni cyklu
unsigned int timer = 3;               //3 timer * 8 (SLEEP_8S) = cas buzeni ATMEGA328 sekundy 3 x 8=24sekund
int pocetmereniAgregace = 11;         //11 kolik mereni nez se udela agragace dat cca 5minut
unsigned int cyklus =0;               //11 kdyz je stejne jako "pocetmereniAgregace" spusti agregace v prvni cyklu

//LoRa
const byte lora_PNSS   = 4;  // 18 pin number where the NSS line for the LoRa device is connected.
const byte lora_PReset = 5;  // 23 pin where LoRa device reset line is connected
const byte PLED1 = 6;        // pin number for LED on Tracker
#include <SPI.h>
#include "LoRaTX.h"
#include <Wire.h>
String Outstring=""; 
   

// BME280
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013) 
Adafruit_BME280 bme;
float SLpressure_hPa;
float wind, temp, tempf, humi, alti, pres;

// wind speed aneometer
float DataSpeed[30];  // pole pro data
float WindSpeedMax;
float WindSpeedAvg;
int anopin = 3;       // PIN for anemometer input
int InterruptCounter;
float WindSpeed;
int WindSpeedAvgA;
int WindSpeedMaxA;
float  wsm,wsa;

//wind direction smerova ruzice
#include "Yamartino.h"
Yamartino yamartino(2);   // pocet mereni za sekundu
const int RecordTime = 3; // Define Measuring wind speed Time (Seconds)
const int SensorPin = 2;  // Define Interrupt Pin (2 or 3 @ Arduino Uno)
int i=0;

#include "AverageAngle.h"
float DataSmer[30]; // pole pro data
float wdn;  //promena pro prepocet z ADC na uhly
int wd;   // hodnota AD prevodniku
int heading; // promena pro Yamartino knihovnu
float wda;    // WindDirectoryArchiv
float WindDirectoryAvg; // vystupni hodnota z AverageAngle
int pinWindDir = A0;

//mereni baterie
int voltPin = A1;
float vcc;
float predradnik;
float adc;
float resistor1 = 90.92;  // 66K pro 8,5V vstupni svorka 92k -0,2V Cukrak 90.94/9.889
float resistor2 = 10.08;  // 10Kohm na GND
float napetiCPU = 1.1;    // hodnoto napajeciho napeti uProcesoru analogReference(INTERNAL)1.1V

char sentence[150];

/////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  analogReference(INTERNAL);
  predradnik = resistor2 / (resistor1 + resistor2);
  
  Serial.begin(9600);  
  Serial.println("start");// 
  delay(20);
  
  
  bool bme_status;
  bme_status = bme.begin(0x76);  //address either 0x76 or 0x77
  if (!bme_status) {
  Serial.print("No valid BME280 found"); }

  pinMode(lora_PReset, OUTPUT);     // RFM98 reset line
  digitalWrite(lora_PReset, LOW);   // Reset RFM98
  pinMode (lora_PNSS, OUTPUT);      // set the slaveSelectPin as an output:
  digitalWrite(lora_PNSS, HIGH);
  pinMode(PLED1, OUTPUT);                                                          // for shield LED neni na pcb sciti pri TX
  
  SPI.begin();                                                                     // initialize SPI:
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
   
   lora_ResetDev();                                                                 // Reset the device
   lora_Setup();                                                                    // Do the initial LoRa Setup

// LoRa frequency calculation (sample for 434.4 MHz): 
// ------------------------------------
// 434400000/61.03515625 = 71172096
// 71172096 (DEC) = 6C 99 99 (HEX)  
// 6C 99 99 (HEX) = 108 153 153 (DEC)  

   lora_SetFreq(108, 113, 153);  //433.775 MHz                                       // Set the LoRa frequency, 433.775 Mhz

}

///////////////////////////////////////////////////////////////////////////////////////
void loop(){
  
  while(PeriodeCounterSleep < timer){
  Serial.print("PeriodeCounterSleep++ ");
  Serial.println(PeriodeCounterSleep);
  delay(100);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
    PeriodeCounterSleep++;
   }
PeriodeCounterSleep = 0; 
/*****************************************************************
                     5 minut cyklus Agregace dat
*****************************************************************/
if (cyklus >= pocetmereniAgregace){
Serial.println("5 minut cyklus Agregace dat + odeslani dat LoRa"); 
/*****************************************************************/
  for (i=0;i<cyklus;i++){
  //Serial.println(DataSpeed[i]); 
   WindSpeedAvg = (DataSpeed[i])+ WindSpeedAvg; // secte vsechny hodnoty v poly DataSpeed
  
  if ((DataSpeed[i]) > WindSpeedMax){     // pri listovani pole DataSpeed hleda maximalni hodnotu
  //Serial.println("Vyhodnoceni WindSpeedMax") ;
    WindSpeedMax=(DataSpeed[i]) ;     // zapise maximalni hodnotu do promene WindSpeedMax
  }
}
WindSpeedAvg = WindSpeedAvg / cyklus; // vypocita prumernou hodnotu WindSpeedAvg

/*****************************************************************/  
AverageAngle Smer(AverageAngle::DEGREES);
  Smer.reset();
  for (i=0;i<cyklus;i++){     //cyklus precte pole DataSmer a pomoci knihovny vypocitá průmerný úhel
      Smer.add(DataSmer[i]);
  }
  WindDirectoryAvg = Smer.getAverage();

/**************************************************************/  
// zmeri napeti 
  vcc = analogRead(voltPin);
// Convert to actual voltage (0 - 1.1 V internal)
  vcc = (vcc / 1024) * napetiCPU;
  vcc = vcc / predradnik;


  bme.takeForcedMeasurement();
  temp = bme.readTemperature();
  humi = bme.readHumidity();
  alti = bme.readAltitude(SEALEVELPRESSURE_HPA);
  pres = (bme.readPressure()/100.0F)+52;  // upravit + vyska v metrech deleno 10
  wda = Smer.getAverage();
  wsa = WindSpeedAvg;
  wsm = WindSpeedMax;

  
   byte i;
   byte ltemp;
   tempf=(temp*1.8)+32; // celsius to fahrenheit

    
// prezentace dat
  Serial.println("") ;
  Serial.print("Average Direction = ");
  Serial.print(wda,0); 
  Serial.println("°") ;

  Serial.print("Wind Speed Avg    = ");
  Serial.print(wsa,1);       //Speed in mph
  Serial.println(" mph"); 
    
  Serial.print("Wind Speed Max    = ");
  Serial.print(wsm,1);       //Speed in mph
  Serial.println(" mph");

  Serial.print("Napeti            = ");
  Serial.print(vcc);
  Serial.println("V");
  Serial.print("Temperature       = ");
  Serial.print(temp,1);
  Serial.println("°C");
  Serial.print("Humidiy           = ");
  Serial.print(humi,0);
  Serial.println("%");
  Serial.print("Pressure          = ");
  Serial.print(pres,0);
  Serial.println("hPa");
  Serial.print("Altitude          = ");
  Serial.print(alti,0);
  Serial.println("m");

// odeslani dat pres LoRa 
   Outstring = "OK1FET-14>APRS:!5011.21N/01347.39E_";  // upravit
   if(wda<99) { Outstring += "0"; }
   if(wda<9)  { Outstring += "0"; }
   Outstring += String(wda,0);
   Outstring += "/";
   if(wsa<99) { Outstring += "0"; }
   if(wsa<9)  { Outstring += "0"; }
   Outstring += String(wsa,0);
   Outstring += ("g");
   if(wsm<99) { Outstring += "0"; }
   if(wsm<9)  { Outstring += "0"; }
   Outstring += String(wsm,0);
   Outstring += ("t");
   if(tempf<99) { Outstring += "0"; }
   if(tempf<9) { Outstring += "0"; }
   Outstring += String(tempf,0);
   Outstring += ("h");
   Outstring += String(humi,0);
   Outstring += ("b");
   if(pres<999) { Outstring += "0"; }
   Outstring += String(pres,0);
   Outstring += ("0");
   Outstring += ("_BATT=");
   Outstring += String(vcc,2);
   Outstring += ("V");

   Serial.print("sending: ");
   //Serial.println(Outstring);
   Outstring.replace(" ","");// wsa a wda pri zaukrohlovani obcas vkladji mezeru pred hodnotu!
   Serial.println(Outstring);
 

//LoRa
    ltemp = Outstring.length();
    lora_SetModem(lora_BW125, lora_SF12, lora_CR4_5, lora_Explicit, lora_LowDoptON);    // Setup the LoRa modem parameters
    lora_PrintModem();                                                                  // Print the modem parameters
    lora_TXStart = 0;
    lora_TXEnd = 0;
    for (i = 0; i <= ltemp; i++)
    {
    lora_TXBUFF[i] = Outstring.charAt(i);
    }
    i--;
    lora_TXEnd = i;    
    digitalWrite(PLED1, HIGH);  // LED ON on during sending
    
    Serial.print("ON AIR");

    lora_Send(lora_TXStart, lora_TXEnd, 60, 255, 1, 10, 17);  
    digitalWrite(PLED1, LOW);   // LED OFF after sending
    lora_TXBuffPrint(0);
//konec vysilani dat LoRa     
    

    Outstring = "";
    cyklus = 0;
    WindSpeedAvg = 0;
    WindSpeedMax = 0;
/**************konec 5 minutoveho i cyklu************************************************/  
}
/*****************************************************************
                 standardni mereni kazdych 20 sekund
*****************************************************************/
Serial.print(cyklus);
Serial.println(" - standardni mereni kazdych 20 sekund");
    mereniWindSpeed();
    DataSpeed[cyklus]=(WindSpeed);// zapise hodnotu do pole DataSpeed

    heading = getWindDirection();
    yamartino.add(heading);
    DataSmer[cyklus] = (yamartino.averageHeading());// zapise hodnotu do pole DataSmer

  Serial.print(WindSpeed,1); //Speed in m/s
  Serial.println(" mph");
  Serial.print(yamartino.averageHeading(),0);
  Serial.println("°") ;
  Serial.println("ulozeno do datoveho pole goto Sleeping...");


cyklus++;
}// endloop
/*****************************************************************
                 mereni rychlosti vetru
*****************************************************************/
void mereniWindSpeed() {
  InterruptCounter = 0;
  attachInterrupt(digitalPinToInterrupt(SensorPin), countup, RISING);
  delay(1000 * RecordTime);
  detachInterrupt(digitalPinToInterrupt(SensorPin));
  WindSpeed = (float)InterruptCounter / (float)RecordTime * 0.4973;
//WindSpeed = ((float)InterruptCounter / (float)3 * 2.4) / 2; //Convert counts & time to km/h
}

void countup() {
  InterruptCounter++;
}
/*****************************************************************
                 mereni smeru vetru
*****************************************************************/
float getWindDirection() {

   wd =  analogRead(pinWindDir);
delay(10);
// Serial.print("WindDir");// pouzit pro kalibraci smerove ruzice
// Serial.println(wd);

      if      (wd < 150) wdn =  90; 
      else if (wd < 270) wdn = 135; 
      else if (wd < 400) wdn = 180; 
      else if (wd < 580) wdn =  45; 
      else if (wd < 760) wdn = 225; 
      else if (wd < 890) wdn =   0; 
      else if (wd < 960) wdn = 315; 
      else if (wd < 1000)wdn = 270;       
  
  return wdn;  
}
/* REMARKS
zapise bootloader nastavi pojistky 8MHz vnitrni hodiny na pin19 D13 blika 1S programovat Arduino FIO
avrdude -c arduino -p m328p -b 19200 -P /dev/ttyACM0 -U flash:w:ATmegaBOOT_168_atmega328_pro_8MHz.hex -Uefuse:w:0xFD:m -Uhfuse:w:0xDA:m -Ulfuse:w:0xE2:m

https://weather.gladstonefamily.net/aprswxnet.html
 
Field  Meaning
CW0003  Your CW number
>APRS,TCPIP*: Boilerplate
/241505z  The ddhhmm in UTC of the time that you generate the report. However, the timestamp is pretty much ignored by everybody as it is assumed that your clock is not set correctly! If you want to omit this field, then just send an exclamation mark '!' instead.
4220.45N/07128.59W  Your location. This is ddmm.hh -- i.e. degrees, minutes and hundreths of minutes. The Longitude has three digits of degrees and leading zero digits cannot be omitted.
_032  The direction of the wind from true north (in degrees).
/005  The average windspeed in mph
g008  The maximum gust windspeed in mph (over the last five minutes)
t054  The temperature in degrees Farenheit -- if not available, then use '...' Temperatures below zero are expressed as -01 to -99.
r001  The rain in the last 1 hour (in hundreths of an inch) -- this can be omitted
p078  Rain in the last 24 hours (in hundreths of an inch) -- this can be omitted
P044  The rain since the local midnight (in hundreths of an inch) -- this can be omitted
h50 The humidity in percent. '00' => 100%. -- this can be omitted.
b10245  The barometric pressure in tenths of millbars -- this can be omitted. This is a corrected pressure and not the actual (station) pressure as measured at your weatherstation. The pressure is adjusted according to altimeter rules -- i.e. the adjustment is purely based on station elevation and does not include temperature compensation.
 
OK1FET-5>APN100,TCPIP*:=5020.55N/01419.98E_045/002g...t045r...p...P000h55b.....eESP8266
 
ANT 433MHz
https://quadmeup.com/3d-printed-433mhz-moxon-antenna-with-arm-and-snap-mount/
https://www.thingiverse.com/thing:2068392/files
 
lf33
http://robodoupe.cz/2012/vezmu-lf33-a-zadny-problem/
https://ok1kvk.cz/clanek/2017/LF33-aneb-ctete-datasheety/

Lithium Battery charging tp4056
https://www.buildcircuit.com/how-to-use-micro-usb-5v-1a-lithium-battery-charging-board-charger-module/
http://jimlaurwilliams.org/wordpress/?p=4731

inspirace
https://www.instructables.com/Solar-Powered-WiFi-Weather-Station-V30/?utm_source=newsletter&utm_medium=email

prevest WGS84  N 50°10.84313', E 13°53.28152'zapsat jako 5010.84N/01353.28E
Cukrak    = 4956.28N/01420.12E 
Stankovka = 4958.11N/01419.50E
Malesice  = 5004.91N/01431.53E
Msec      = 5010.84N/01353.28E
Revnicov  = 5011.21N/01347.39E

chyba
2021-03-28 20:03:24 CEST: OK1FET-14>APRS,qAS,OK0HCS-1:!4958.11N/01419.50E_090/000g000t044h47b10010_BATT=3.97V SNR=-18dB RSSI=-83db
2021-03-28 20:08:34 CEST: OK1FET-14>APRS,qAS,OK0HCS-1:!4958.11N/01419.50E_0100/000g000t043h47b10010_BATT=3.97V SNR=-18dB RSSI=-83db
2021-03-28 20:24:05 CEST: OK1FET-14>APRS,qAS,OK0HCS-1:!4958.11N/01419.50E_180/000g000t043h48b10010_BATT=3.97V SNR=-18dB RSSI=-83db

 */
