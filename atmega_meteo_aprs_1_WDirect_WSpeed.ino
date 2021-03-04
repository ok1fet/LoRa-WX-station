/* 
 * 2021/02/09 Software serial pro debag aplikace
 * 2021/01/10 Meteostanice bezi na ATMEGA328 spolupracuje s ESP8266 ktery slouzi jen jako WIFI modem.
 * Mereni probíha kazdych 20 vterin data se agreguji kazdych 5 minut vypocitaji se prumery smeru rychlosti a 
 * narazy vetru. Mereni rychlosti je Mph. Pripravi APRS vetu vzbudi se ESP8266 a pomoci UART ji preda
 * upraveno na Meteostanice WH1080
 * 1Hz 1.492MPh - 2.4KMh / 3.6 = m/s
 * V = P(1.492/3) = P * 0,4973333   
 * OK1FET-5>APN100,TCPIP*:=5020.55N/01419.98E_045/002g...t...r...p...P...h..b.....eESP8266r
 *
 * usb nabijec jak
 * https://www.best-microcontroller-projects.com/tp4056.html
 * 
 * www.aprs.fi/fw4671 
 * 
 * https://www.hadex.cz/_15meteocidla-meteostanice-hodiny-kalkul-vnejsi-cidla-k-meteostanicim/
 * 
 * knihovna AverageAngle pro výpočet průměru úhlů.
 * https://github.com/RobTillaart/AverageAngle
*/
#include <SoftwareSerial.h>
SoftwareSerial SoftSerial(11, 12);//Rx-Tx pro debug aplikace

#include <LowPower.h>
unsigned int PeriodeCounterSleep = 3;//3 kdyz je stejne jako "timer" spusti mereni v prvni cyklu
unsigned int timer = 3;         //3 timer * 8 (SLEEP_8S) = cas buzeni ATMEGA328 sekundy 3 x 8=24sekund
int pocetmereniAgregace = 12;   //12 kolik mereni nez se udela agragace dat cca 5minut
unsigned int cyklus =12;   //12 kdyz je stejne jako "pocetmereniAgregace" spusti agregace v prvni cyklu
int casOdeslani;
float DataSpeed[30];// pole pro data
float WindSpeedMax;
float WindSpeedAvg;
boolean pocetmereniset = true;
int countresetESP;
int cekej;


int pinESPreset=7;//7
int ESPready;
int pinESPready=8;
char cti;
int i=0;
//const char*     navratovykod = "Arduino";
char sentence[150];
long setupcas = 0;
int counterr;


#include "AverageAngle.h"
float DataSmer[30]; // pole pro data
float wdn;  //promena pro prepocet z ADC na uhly
int wd;   // hodnota AD prevodniku
int heading; // promena pro Yamartino knihovnu
int wda;    // WindDirectoryArchiv
float WindDirectoryAvg; // vystupni hodnota z AverageAngle
int pinWindDir = A0;


#include "Yamartino.h"
Yamartino yamartino(2);//pocet mereni za sekundu
const int RecordTime = 3; //Define Measuring wind speed Time (Seconds)
const int SensorPin = 2;  //Define Interrupt Pin (2 or 3 @ Arduino Uno)
int InterruptCounter;
float WindSpeed;
int WindSpeedAvgA;
int WindSpeedMaxA;
int wsa;
int wsm;


int voltPin = A1;
float napeti;
static char  vcc[6]; //= napeti konverze sprintf %f nefunguje na atmega328
int  vca; //
float predradnik;
float adc;
float resistor1 = 90.94;  // 66K pro 8,5V vstupni svorka 92k -0,2V
float resistor2 = 9.889;  // 10Kohm na GND
float napetiCPU = 1.1;    // hodnoto napajeciho napeti uProcesoru analogReference(INTERNAL)1.1V


void setup() {
  Serial.begin(9600);
  SoftSerial.begin(9600);
  pinMode(pinESPreset, OUTPUT);
  pinMode(pinESPready, INPUT);
 
  analogReference(INTERNAL);
  predradnik = resistor2 / (resistor1 + resistor2);
  digitalWrite(pinESPreset,HIGH);// aby nastartovolo ESP

}

void loop() {

while ((ESPready=digitalRead(pinESPready))==0){
SoftSerial.println("ESP pin ready = 0");
 PeriodeCounterSleep = timer; // kdyz je na pinu ESPready 0 tak preskoci sleep rezim (while(PeriodeCounterSleep < timer)

      if (Serial.available()>0){
      cti = Serial.read();
      
      
      if (cti == '*'){
        SoftSerial.println("ESP poslalo *");
sprintf(sentence,"%03d/%03dg%03dt...dr...p...P000h..b....._FIO_%02d_%sV_---", wda, wsa, wsm, counterr, vca);
Serial.println(sentence); 
SoftSerial.println("Atmega328 poslal WX vetu: "); 
SoftSerial.println(sentence); 
SoftSerial.println("data WDA / WSA / WSM / VCA"); 
SoftSerial.println(wda); 
SoftSerial.println(wsa); 
SoftSerial.println(wsm); 
SoftSerial.println(vca); 

countresetESP =0;

//navratovykod = "Arduino";
 setupcas = millis(); 
  while ( millis() <= setupcas + 6000){}
}
    if (cti == '1'){
      SoftSerial.println("ESP poslalo 1 volam reset ESP ");
 if (countresetESP < 5 ){
  resetESP();
 }
 counterr++;
//navratovykod = "Xrduino";
countresetESP++;
 setupcas = millis(); 
  while ( millis() <= setupcas + 6000){}

    }

      if (cti == '2'){
        // if ((cti == '2')&& ( millis() <= setupcas + 5000)){
SoftSerial.println("ESP poslalo 2 volam reset ESP");        
if (countresetESP < 5 ){
  resetESP();
 } 
//navratovykod = "Yrduino";
counterr++;
countresetESP++;
 setupcas = millis(); 
  while ( millis() <= setupcas + 6000){}
}



      }
delay(50);  
}
  while(PeriodeCounterSleep < timer){
//   Serial.print("PeriodeCounterSleep++ ");
//  Serial.println(PeriodeCounterSleep);
//   delay(100);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
    PeriodeCounterSleep++;
   }

   
  PeriodeCounterSleep = 0;

/*****************************************************************
                     5 minut cyklus Agregace dat
*****************************************************************/
if (cyklus >= pocetmereniAgregace){
 
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
//  Serial.print("DataSmer aktual") ;
    Smer.add(DataSmer[i]);
  }
  WindDirectoryAvg = Smer.getAverage();
//  Serial.print("AVG:\t");
//  Serial.println(AverageAngle, 0);// vrací float pocet desetinych mist 0 - 6

/**************************************************************/  
// zmeri napeti 
  napeti = analogRead(voltPin);
// Convert to actual voltage (0 - 1.1 V internal)
  napeti = (napeti / 1024) * napetiCPU;
  napeti = napeti / predradnik;
   dtostrf(napeti,3, 1, vcc);      //3 celkovy pocet znaku vcetne tecky 1 pocet znaku za teckou

/**************************************************************/  
//  tf = ((bme.readTemperature()) * 9.0) / 5.0 + 32.0; // prepocet *C na *F 
//  pr = (bme.readPressure()/10)+vnm;

/**************************************************************/  
// prezentace dat
//  Serial.print("Average Direction ");
//  Serial.print(WindDirectoryAvg, 0); 
//  Serial.println("°") ;
//
//  Serial.print("Wind Speed Avg: ");
//  Serial.print(WindSpeedAvg);       //Speed in mph
//  Serial.println(" mph"); 
//    
//  Serial.print("Wind Speed Max: ");
//  Serial.print(WindSpeedMax);       //Speed in mph
//  Serial.println(" mph");
//
//  Serial.print("Napeti ");
//  Serial.print(napeti);
//  Serial.println("V");
//  Serial.print(vcc);   
//
//  Serial.print("casOdeslani  ");
//  Serial.println(casOdeslani);

  SoftSerial.print("Average Direction ");
  SoftSerial.print(WindDirectoryAvg, 0); 
  SoftSerial.println("°") ;

  SoftSerial.print("Wind Speed Avg: ");
  SoftSerial.print(WindSpeedAvg);       //Speed in mph
  SoftSerial.println(" mph"); 
    
  SoftSerial.print("Wind Speed Max: ");
  SoftSerial.print(WindSpeedMax);       //Speed in mph
  SoftSerial.println(" mph");

  SoftSerial.print("Napeti ");
  SoftSerial.print(napeti);
  SoftSerial.print("V ");
  SoftSerial.println(vcc);   

  SoftSerial.print("casOdeslani  ");
  SoftSerial.println(casOdeslani);
 

/**************************************************************/     
//Serial.println("Agregace dat do archivu");
 SoftSerial.println("Agregace dat do archivu");  
      wda = Smer.getAverage();
      wsa = WindSpeedAvg;
      wsm = WindSpeedMax;
      vca = vcc;
//       SoftSerial.println(wda);
//        SoftSerial.println(wsa);
//         SoftSerial.println(wsm);
//          SoftSerial.println(vca);
sprintf(sentence,"%03d/%03dg%03dt...dr...p...P000h..b....._FIO_%02d_%sV_---", wda, wsa, wsm, counterr, vca);
 SoftSerial.println(sentence); 


  countresetESP = 0;//pocet zasebou pokusu o pripojeni ESP
  resetESP();
      cyklus = 0;
      WindSpeedAvg = 0;
      WindSpeedMax = 0;

      if (counterr > 99 ) counterr = 0;
       


////

if ( casOdeslani < 1 ){
pocetmereniset = true;


if (pocetmereniset == true){
    SoftSerial.print("cas cyklu zmena ");
    SoftSerial.println(casOdeslani);
          if  (napeti < 3.1 )                       casOdeslani = 120;// 6 hodin
          if ((napeti > 3.0 ) &&  (napeti < 3.5 ))  casOdeslani =   9;//45 minut
          if ((napeti > 3.4 ) &&  (napeti < 3.8 ))  casOdeslani =   2;//10 minut
          if  (napeti > 3.7 )                       casOdeslani =   0;// 5 minut
          
         pocetmereniset = false;

                            } 


}
////
casOdeslani--;

/**************konec 5 minutoveho i cyklu************************************************/  
}

/*****************************************************************
                 standardni mereni kazdych 20 sekund
*****************************************************************/
////Serial.print F("*********start 20 sekundoveho******cyklu= ");
//Serial.println(cyklus);
SoftSerial.println(cyklus);
/************ standardni mereni kazdych 20 sekund **************************************************/  
    mereniWindSpeed();
    DataSpeed[cyklus]=(WindSpeed);// zapise hodnotu do pole DataSpeed

    heading = getWindDirection();
    yamartino.add(heading);
    DataSmer[cyklus] = (yamartino.averageHeading());// zapise hodnotu do pole DataSmer


//  Serial.print(WindSpeed); //Speed in m/s
//  Serial.println(" mph");
//  Serial.print(yamartino.averageHeading());
//  Serial.println("°") ;
//  Serial.println("ulozeno do datoveho pole goto Sleeping...");

  SoftSerial.print(WindSpeed); //Speed in m/s
  SoftSerial.println(" mph");
  SoftSerial.print(yamartino.averageHeading());
  SoftSerial.println("°") ;
  SoftSerial.println(" ") ;
//  SoftSerial.println("ulozeno do datoveho pole goto Sleeping...");
/**************************************************************/  
cyklus++;
 
}
/////////////////////////////////////////////////////////////////////////////////////////////
void resetESP(){
 // tady se provede reset esp
//  Serial.print("pozadavek na reset ESP za ");
//  Serial.println(casOdeslani);
if ( casOdeslani < 1){
 
//  Serial.println("!!!!!RESET ESP!!!!!");
SoftSerial.println("!!!!!RESET ESP!!!!!");
  digitalWrite(pinESPreset,LOW);
  delay(450);  
  digitalWrite(pinESPreset,HIGH);
  setupcas = millis(); 
  while ( millis() <= setupcas + 500){}
}                        
 
}

void mereniWindSpeed() {
  InterruptCounter = 0;
  attachInterrupt(digitalPinToInterrupt(SensorPin), countup, RISING);
  delay(1000 * RecordTime);
  detachInterrupt(digitalPinToInterrupt(SensorPin));
  WindSpeed = (float)InterruptCounter / (float)RecordTime * 0.4973;
}

void countup() {
  InterruptCounter++;
}
float getWindDirection() {

   wd =  analogRead(pinWindDir);
delay(10);
// Serial.print("AD A0 ");// pouzit pro kalibraci smerove ruzice
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
/*
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
 
 */
