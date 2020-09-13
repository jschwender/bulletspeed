/* Frequenz/Geschwindigkeitsmessung mit der Input Capture Unit (getestet mit arduino mini pro)
 J. Schwender
 Funktion: Geschwindigkeitsmessung von Geschossen mit einer Laser-Lichtschranke.
 Der Laserstrahl wird zwei mal umgelenkt, so dass das Geschoss den Strahl zwei mal unterbricht.
 Dies ergibt jeweils einen kurzen LOW Impuls. Die Geschwindigkeit wird mit der bekannten Distanz
 und der Zeit zwischen den beiden Fallflanken ermittelt. 

 Timer 1 (16bit) läuft im CTC modus von 0…65536, mit einem Vorteiler von 1, also mit 16 MHz. 
 Bei dessen Überlauf wird ein Interrupt erzeugt mit dem ein Software-Zähler hochgezählt wird.
 Ein Fallflankensignal an Pin 9 erzeugt einen Interrupt, der die Zählerwerte ausliest und ausgibt.
 Das Erste Signal mrkiert den Startwert, das zweite Signal den Endwert.
 Da beide die gleiche Latenz (ca 2µs) für den ISR Eintritt haben ist die ausgewertete Differenz
 frei von diesem Latenzfehler.
 
 Bei 16 MHz Takt ergibt sich eine Grundauflösung von 62,5 ns.
 Bei einer Messdistanz von 0,10 m wäre die obere Messgrenze vmax=1600000 m/s.
 Bei v=1000 m/s ergeben sich 2 Impulse im Abstand von 100 µs oder ein Zählerstand von 1600.
 Also bei einer Zählunsicherheit von +-1 ist die Zählgenauigkeit immernoch 0,000625 oder 0,0625%
 Dazu kommt die wesentlich grössere Unsicherheit bei der Feststellung der Messtrecke die 
 bei ca 1% liegt.

 Input: Pin D8, also ICP1
*/
#define SER
//
#include <Wire.h>
#include <LiquidCrystal_I2C.h>  // Vorher hinzugefügte LiquidCrystal_I2C Bibliothek hochladen
LiquidCrystal_I2C lcd(0x27, 16, 2);   // also 16 Zeichen × 2 Zeilen
volatile boolean ReadingStarted;
volatile boolean ReadingCompleted;
volatile unsigned long overflowCount;
volatile unsigned long startTime;
volatile unsigned long finishTime;
const float MessDistanz = 0.1000;   // in m
const float DistanzFehler = 0.001;  // absoluter Fehler der Messdistanz in m (Schätzung)
float Geschwindigkeit, FehlerMax; //, FehlerMin; wird nicht wirklich benötigt da fast identisch mit Fehlermax
byte plusminus[8] = {  B00100,  B00100,  B11111,  B00100,  B00100,  B00000,  B11111,  B00000};
char S1[]="000000.00";
char S2[]="0000.00";

ISR (TIMER1_OVF_vect)  {    // Interrup Service Routine  for timer overflows (every 65536 counts)
  overflowCount++;        // we just count the overflows
}

ISR (TIMER1_CAPT_vect) {
  // grab counter value before it changes once again
  unsigned int timer1CounterValue;
  timer1CounterValue = ICR1;  // Input Capture Register, see datasheet, page 117 (accessing 16-bit registers)
  // ICES1=1 also Steigflankentriggerung, ICES1=0 also Fallflankentriggerung zum Auslesen des TCNT1 --> ICR1
  unsigned long overflowCopy = overflowCount;
  
  if ((TIFR1 & bit (TOV1)) && timer1CounterValue < 0x7FFF) {         // if just missed an overflow, TOV1 is set in the cycle when overrun to 0 occurs
    overflowCopy++;
  }
  if (ReadingCompleted)  {        // do nothing further if the reading is alredy completed
    return;
  }
  
  if (ReadingStarted)    {            // the first time we capture the 
    startTime = (overflowCopy << 16) + timer1CounterValue;
    ReadingStarted = false;          // second time this part will be skipped
    return;  
    }
    
  finishTime = (overflowCopy << 16) + timer1CounterValue;
  ReadingCompleted = true;
  TIMSK1 = 0;    // no more interrupts for now
}  
//*********************************************************** end of TIMER1_CAPT_vect
void prepareForInterrupts () {
  noInterrupts ();  // protected code
  ReadingStarted = true;
  ReadingCompleted = false;  // re-arm for next time

  TCCR1A = 0;    TCCR1B = 0;    // reset Timer 1
  
  bitSet(TIFR1,ICF1);
  bitSet(TIFR1,TOV1);  // clear flags so we don't get a bogus interrupt
  TCNT1 = 0;     overflowCount = 0;  // Counter to zero, no overflows yet
  
  bitSet(TIMSK1,TOIE1);  // Timer 1 - counts clock pulses
  bitSet(TIMSK1,ICIE1);  // interrupt on Timer 1 overflow and input capture
  bitSet(TCCR1B,CS10);   // start Timer 1, prescaler factor  1
  bitSet(TCCR1B,ICES1);  // Input Capture Edge Select (rising edge on D8)
  interrupts ();
#ifdef SER
//  Serial.println("Interrupts set");
#endif
}  // end of prepareForInterrupts
//======================================================
void setup () {
  lcd.init(); //Im Setup wird der LCD gestartet (anders als beim einfachen LCD Modul ohne 16,2 in den Klammern denn das wurde vorher festgelegt
  Wire.setClock(400000);   // fast mode auf I2C bus
  lcd.createChar(2, plusminus);
  lcd.clear();
  lcd.backlight(); // Hintergrundlicht einschalten
  lcd.setCursor(0,0);    lcd.print(" Laser-");  // erste Zeile, Anfang
  lcd.setCursor(0,1);    lcd.print(" Lichtschranke, 2020 "); // erste Zeile, Anfang

#ifdef SER
  Serial.begin(230400);       
  Serial.println("Zeitmessung mit Arduino Timer1 mit InputCapture am externen Eingang D8");
#endif
  Wire.setClock(400000);
  prepareForInterrupts ();     // set up for interrupts
} // end of setup
//---------------
void float2String( char sdr[], int Size, int nks, float X) {
  dtostrf(X,Size-1,nks,sdr);   // \0 am Ende zählt nicht mit, deshalb -1 für die Stringlänge!
  char* p  = strchr(sdr, '.');       // . mit pointer lokalisieren
  *p = ',';                         // ersetze . durch  ,
}

//=======================================================
void loop () 
  {
  if (!ReadingCompleted)      // wait till reading is completed
    return;
 
  int Stellen;
  unsigned long elapsedTime = finishTime - startTime;
  // frequency is inverse of period, adjusted for clock period
  Geschwindigkeit = (float) F_CPU * MessDistanz / (float) (elapsedTime);  // each tick is 62.5 ns at 16 MHz
  FehlerMax = (float) F_CPU * MessDistanz *(1.0+DistanzFehler/MessDistanz) / (float) (elapsedTime - 1) - Geschwindigkeit; 
  if (Geschwindigkeit < 0) { Stellen = 4; }
  if (Geschwindigkeit >= 1) { Stellen = 5; }
  if (Geschwindigkeit >= 10) { Stellen = 3; }
  if (Geschwindigkeit >= 100) { Stellen = 2; }
  float2String(S1,8,Stellen,Geschwindigkeit);
  if (FehlerMax < 1)   { Stellen=2; }
  if (FehlerMax >= 1)  { Stellen=1; }
  if (FehlerMax >= 10) { Stellen=0; }
  float2String(S2,5,Stellen,FehlerMax);
#ifdef SER
  Serial.println (String(Geschwindigkeit,3)+" ±"+String(FehlerMax)+" m/s (n="+String(elapsedTime)+")");
#endif
  lcd.clear();
  lcd.setCursor(0,0);   lcd.print(String(S1)+"\002"+String(S2)+" m/s"); // 002=plusminus-Zeichen im RAM des Display
  lcd.setCursor(0,1);   lcd.print("("+String(elapsedTime)+" \344s)");  // 344="µ"-Zeichen im Display
  delay (1000);      // so we can read it  
  prepareForInterrupts ();   
}   // end of loop
