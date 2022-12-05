#include "Arduino.h"
#include "time.h"
#include "WiFi.h"
#include "DHTesp.h"
#include <SPIFFS.h>
//#include "rtc.h"
void UpdateDisplay();
void DisplayTest();
void TouchControl();

/* Pinout WEMOS D1 mini
               ---------- [USB] ---------------- 
        [IO06] xx IO00 (SD0)           (SD3)  IO10 xx IO11 (CMD)
        [IO08] x? IO15 (TD0) RTC? (TCK) RTC?  IO13 ?x IO09 (SD2)
  LEDTCK  IO02 0o VCC                          3V3 ox NC 
  RTC?    IO00 ?o GND                 LEDSEC  IO05 00 IO14 TOUCH6(TMS)
  DHT     IO04 00 IO16 LED_7C         LED_M1  IO23 0x IO34 (IN)
  RTC     IO12 00 IO17 LED_7D         LED_M10 IO19 00 IO33 TOUCH8 
  LED_7E  IO32 00 IO21 LED_7G         LED_H1  IO18 0x IO35 (IN)
  LED_7F  IO25 00 IO22 LED_7A         LED_H10 IO26 0x IO39 (IN) (SVN)
  LED_7B  IO27 0o IO03 (RXD)        (SVP)(IN) IO36 xx NC
           GND oo IO01 (TX)                    RST xo GND
 Use LED_RH also as LEDSEC free IO05 or IO12
 RTC 3 Wire POSSIBLE GPIO Also IO05 or IO12
 CLK OUT    IO00    
 DAT I/O    IO13
 RST OUT    IO15
*/
// WLAN Struktur
const char *ntp = "ptbtime1.ptb.de";
// Replace with your network credentials
String StrSSID = "";
String StrPWD = "";

// Support Struktur
//#define ADC_PIN 34 //ADC2-7 an GPIO 14
#define LED_CTL 2 // Ticker LED
#define LED_SEC 5 // Sekunden-Anzeige ehem. 33 verschoben wegen TouchPin
//#define LED_RH 12  // PAD zu Anzeige der aktiven T/Rh Anzeige
//#define delTime 3000 // Künstliche PWM für Loop

// H10=26, H1=18, M10=19, M1=23 : GPIO Ports der 7Segmenter per Fritzing
// 7Segment Ansteuerung Struktur
#define LED_H10 26 // Akivierung eines 7Segmentblocks Stunden 10er
#define LED_H1 18  // Aktivierung des 7Segment-Blocks Stunden 1er
#define LED_M10 19 // Aktivierung des 7Segment-Blocks Minuten 10er
#define LED_M1 23  // Aktivierung des 7Segment-Blocks Minuten 1er

// a=22, b=27, c=16, d=17, e=32, f=25, g=21 : Ports jedes Segments per Fritzing
#define LED_7_A 22 // Pad zu Seg A  ebenso I2C0:SCL, Ersatzweise IO35 (Also unused ADC1)
#define LED_7_B 27 // Pad zu Seg B
#define LED_7_C 16 // Pad zu Seg C
#define LED_7_D 17 // Pad zu Seg D
#define LED_7_E 32 // Pad zu Seg E
#define LED_7_F 25 // Pad zu Seg F
#define LED_7_G 21 // Pad zu Seg G ebenso I2C0:SDA, Esatzweise IO39 (Also unused ADC3)


/* PWM Kanal, Dimmer Struktur, jeder PWM Kanal steuert ein Segment,
which is later connected to a GPIO*/
#define LEDC_CH_0 0 // PWM Kanal 0, 0 von 15 : Port H10
#define LEDC_CH_1 1 // PWM Kanae 1, 1 von 15 : Port H1
#define LEDC_CH_2 2 // PWM Kanal 2, 2 von 15 : Port M10
#define LEDC_CH_3 3 // PWM Kanal 3, 3 von 15 : Port M10

// Setup Hum/Temp-Messung
#define DHTPIN 4
#define DHTTYPE DHT11 // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
// DHT dht(DHTPIN, DHTTYPE);
DHTesp dht;
TempAndHumidity dhtData;

// Auflösung der PWM Pulsbreiten-Quantisierung

#define LEDC_RES 12
// Der ADC liefert 12 Bit Tiefe, vllt. ist das zu viel
#define LEDC_FRQ 2000 // Repetition Rate der Pulsfolge
/*See
  LEDC_CH_N defines the PWM Channel
  LEDC_RES defines the resolution to 12 bit
  LEDC_FRQ ist the Freqnuency whoch is output through tha channel
  LED_H10 is an example and in this case defines GPIO26
  ledcSetup(LEDC_CH_0, LEDC_FRQ, LEDC_RES);
  ledcAttachPin(LED_H10, LEDC_CH_0);
*/

// Zeitmessung
unsigned long ulTime = 0; // Timer für Sekundenblink
unsigned long ulLstTime = 0;
unsigned long ulTRH = 0;
unsigned long ulLstTRH = 0;
unsigned long ulStart;
unsigned long ulStop;

const long gmtOffs_Sec = 3600;           //  Deutschland ist 1h plus
unsigned int DayLightSavingTime_Sec = 0; // Sommerzeit 1h = 3600 sec, wenn keine Sommerzeit dann 0
// Touch Sens
// DayLight Saving Time
unsigned int uiTouch14 = 0;
#define TOUCH14 T6 // Touch Channel 6
#define TPIN14 14  // TouchPad GPIO Pin Saving DayLight
// Helligkeit LED
#define TOUCH33 T8 // Touch Channel 8
#define TPIN33 33  // Touchpad GPIO Pin Luminescence
unsigned int uiTouch33 = 0;
unsigned int uiLum = 600; // Speicher für aktuelle Helligkeit
bool bLumDir = true;      // Wirkrichtung abwärts FALSE oder aufwärts TRUE
bool bFirstLoop = false;

// Definition der Segment-Ports für Test-Durchlauf
int iarSegs[7] = {LED_7_A, LED_7_B, LED_7_C, LED_7_D, LED_7_E, LED_7_F, LED_7_G};

/*Definition der aktiven Segmente abhängig von dem numerischen Wert einer Stelle
 * Wie verpackt Mensch sinnvoll die Info Port & ON/OFF, so dass mit möglichst wenigen
 * Befehlen alle Ports durchlaufen und ON/OFF gesetzt werden.
 * da Byte-Weises Schreiben eines Registers erst mal nicht auf dem Programm steht
 * Das geht, ist aber programtechnisch aufwendiger, muss ich erst mal lernen
 */
int iarNum2Segs[10][7] = {{1, 1, 1, 1, 1, 1, 0}, {0, 1, 1, 0, 0, 0, 0}, {1, 1, 0, 1, 1, 0, 1}, {1, 1, 1, 1, 0, 0, 1}, {0, 1, 1, 0, 0, 1, 1}, {1, 0, 1, 1, 0, 1, 1}, {1, 0, 1, 1, 1, 1, 1}, {1, 1, 1, 0, 0, 0, 0}, {1, 1, 1, 1, 1, 1, 1}, {1, 1, 1, 1, 0, 1, 1}};
int iChClk = 0;               // Counter für die Position der Stelle/Digit
uint uiClk[4] = {2, 4, 5, 9}; // Speicher f.d. 4 Digits
// speicher für T/Rh
int iHrTmp = 0;
int iMinRH = 0;
/* The following bRead booleans control asynchronous read/write access
to the display array by the timer interrupt and the Time and RH data buffer
by the CORE0 process Officially this should prevent access by both functions
at the same tme to the same variables */
bool bReadTM = false;
bool bReadRH = false;
// Zeit-Strukur-Speicher
struct tm timeinfo;

void printLocalTime()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

// Task Code: reads THR or Time  every 500 msecond
void ReadTHR(void *pvParameter)
{
  unsigned long C0Time = 0;

  for (;;)
  {
    //  DT11 T/RH read data
    //  Reading temperature or humidity takes about 22 milliseconds!
    if (bReadRH && !bReadTM)
    {
      Serial.print("Reading THR running on core ");
      Serial.println(xPortGetCoreID());
      //ulStart = micros();
      dhtData = dht.getTempAndHumidity();
      iHrTmp = int(dhtData.temperature);
      iMinRH = int(dhtData.humidity);
      /*ulStop = micros() - ulStart;
        Serial.print(ulStop);
        Serial.println(" µs");
      */
      bReadRH = false;
    }
    if (bReadTM && !bReadRH)
    {
      Serial.print("Reading Time running on core ");
      Serial.println(xPortGetCoreID());

      if (!getLocalTime(&timeinfo))
      {
        Serial.println("Failed to obtain time");
        while (true)
        {
        } // Loop forever with no function when time could not be loaded
      }

      bReadTM = false;
    }
    //}
    delay(5); // Break down the loop to give other processes on core0 time
  }
}

// Setup Timer and Variables as Flags/Semaphore
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
int iTick = 0;

// Ende Setup Timer

// IRQ Routine mit kritische Sektion, kann nicht unterbrochen werden
void IRAM_ATTR onTime()
{
  portENTER_CRITICAL_ISR(&timerMux);

  if (iTick < 7)
  {
    digitalWrite(LED_SEC, not(digitalRead(LED_SEC)));
  }
  else
  {
    digitalWrite(LED_SEC, HIGH);
  }

  switch (iTick)
  {
  case 0:

    // GetTime, wenn 0 <= iTick < 6 wird die Zeit angezeigt
    
    if (!bReadTM)
    {
      // Semaphore denies access to data as long as CORE0 Process accesses variables
      uiClk[0] = timeinfo.tm_hour / 10;
      uiClk[1] = timeinfo.tm_hour % 10;
      uiClk[2] = timeinfo.tm_min / 10;
      uiClk[3] = timeinfo.tm_min % 10;
      bReadRH = true; // While showing time Rh can be read
    }
    break;

  case 7:
    // Write T/Rh Data into Diplay-Array
    if (!bReadRH)
    {
      // Semaphore denies data access as long as CORE0 Process has accesss
      uiClk[0] = iHrTmp / 10;
      uiClk[1] = iHrTmp % 10;
      uiClk[2] = iMinRH / 10;
      uiClk[3] = iMinRH % 10;
      bReadTM = true; // While showing RH Time can be updated
    }
    break;
  default:
    break;
  } // End Switch

  // iTick is reset after 10 seconds to 0
  if (iTick <= 9)
  {
    iTick++;
  }
  else
  {
    iTick = 0;
  }

  portEXIT_CRITICAL_ISR(&timerMux);
} // ENd if Timer IRQ Routine

TaskHandle_t Task1;

void GetTime()
{
  // Connect to Wi-Fi für Zeit Server
  // Initialize SPIFFS
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    while (true)
    {
    }
  }
  else
  {
    Serial.println("SPIFFS mounted");
  }
  //If WiFi fails make it switch to the next entry in WiFi.txt until no further entry
  File file = SPIFFS.open("/wifi.txt");
  StrSSID = file.readStringUntil('\n');
  StrPWD = file.readStringUntil('\n');
  //Serial.printf("FromSPIFFS: %s, %s \n", StrSSID.c_str(), StrPWD.c_str());
  
  WiFi.begin(StrSSID.c_str(), StrPWD.c_str());
  Serial.print("Connecting to WiFi..");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");

  Serial.println("WiFi connected.");
  
  // GetTime from Server
  configTime(gmtOffs_Sec, DayLightSavingTime_Sec, ntp);
  printLocalTime();
  Serial.println("Time was read from PTB...");
  // disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println("WiFi disconnected.");
  file.close();
  SPIFFS.end();
}
/************************************
 * Setup Begin
 *************************************/

void setup()
{
  int i;
  Serial.begin(115200);
  // Starte DHT zugriff
  dht.setup(DHTPIN, DHTesp::AUTO_DETECT);
  dhtData = dht.getTempAndHumidity();

  // Die Segmente werden positiv angesteuert,
  //  aber mit PWM Steuerung
  ledcSetup(LEDC_CH_0, LEDC_FRQ, LEDC_RES);
  ledcSetup(LEDC_CH_1, LEDC_FRQ, LEDC_RES);
  ledcSetup(LEDC_CH_2, LEDC_FRQ, LEDC_RES);
  ledcSetup(LEDC_CH_3, LEDC_FRQ, LEDC_RES);
  ledcAttachPin(LED_H10, LEDC_CH_0);
  ledcAttachPin(LED_H1, LEDC_CH_1);
  ledcAttachPin(LED_M10, LEDC_CH_2);
  ledcAttachPin(LED_M1, LEDC_CH_3);

  // Setup ports
  pinMode(LED_CTL, OUTPUT);
  pinMode(LED_SEC, OUTPUT);
  //  pinMode (DHTPIN, INPUT);
  pinMode(LED_7_A, OUTPUT);
  pinMode(LED_7_B, OUTPUT);
  pinMode(LED_7_C, OUTPUT);
  pinMode(LED_7_D, OUTPUT);
  pinMode(LED_7_E, OUTPUT);
  pinMode(LED_7_F, OUTPUT);
  pinMode(LED_7_G, OUTPUT);

  DisplayTest();

  // Konfiguriere die Uhrzeit
  GetTime();

  // Setze Sekunden ticker
  ulTime = millis();
  ulLstTime = millis();

  // Configure the Prescaler at 80 the quarter of the ESP32 is cadence at 80Mhz
  iTick = 0;
  timer = timerBegin(0, 80, true);            // Prescaler 80000000 / 80 = 1000000 ticks / sekunde
  timerAttachInterrupt(timer, &onTime, true); // Timer an Interrupt anhängen
  timerAlarmWrite(timer, 1000000, true);      // Alarm jede Sekunde

  // Allow Time Reading first by setting bReadTM and bReadRH
  bReadTM = true;
  bReadRH = false;
  Serial.println("Setup fertig...");
  // create a task that will be executed in the Task1code() function,
  //  with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
      ReadTHR,   /* Task function. */
      "ReadTHR", /* name of task. */
      10000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      2,         /* priority of the task */
      &Task1,    /* Task handle to keep track of created task */
      0);        /* pin task to core 0 */

  timerAlarmEnable(timer);
  digitalWrite(LED_CTL, HIGH);
  delay(500); // Let Timer and CORE0 Function do their job before activating the display
} // End SETUP
/*
******************************************************
* Hier beginnt die Loop
******************************************************
*/

void loop()
{
  if (!bFirstLoop)
  {
    bFirstLoop = true;
    Serial.println("Erster Eintritt in Loop()");
  }

  TouchControl();

  UpdateDisplay();

  /*
   * Das nachfolgende Delay ist experimentell eingestellt worden
   * und sorgt für ein Gleichgewicht von ausreichend schnellem Scan der Digits (Einzelne Anzeige-Blöcke)
   * zum Zweck der Flackerfreien Darstellung und ausreichend langer Dauer der Ansteuerung der Digits, so dass
   * diese Zeit haben zu leuchten. Kürzere Zeiten lassen die Digits zu dunkel werden.
   * Längere Zeiten lassen die Gesamtanzeige zwar heller werden, dafür flackert sie aber auch deutlicher
   *
   * Die Delay-Zeit ist abhängig vom Zeitbedarf der Ansteuerung in Void Loop, da hier noch keine Bitmaskenansteuerung
   * eingesetzt wird, was mit der Arduino Sprache direkt nicht geht. Man muss einen Umweg über die nativen Elemente
   * des ESP32 gehen und Aufgaben der Kommandos "digitalWrite" und "pinmode" auflösen und selber ausführen.
   * Aufgrund der komplexen Struktur innerhalb des ESP32 von zuweisbaren Funtkionen und verfügbare Pads, die sowohl
   * direkt als auch über einen I/O Multiplexer miteineder dynamisch verbunden werden können, ist das Ansprechen von Registern
   * (Wie in der alten 8Bit microController-Welt der 90iger Jahre) nicht mehr so einfach möglich.
   * (Quelle: https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf)
   */

  delay(5);

} // End LOOP

void UpdateDisplay()
{
  int i;
  int iSeg;
  int iNum;
  bool bOut;

  // Letzte aktive Anzeige ausschalten / löschen, jetzt sind alle Anzeigen aus
  ledcWrite(iChClk, 0); // Setz die Pulsweite für Digit i auf 0, das Digit i ist aus.

  iChClk++;
  if(iChClk>3){iChClk=0;}
  iNum = uiClk[iChClk]; // Wert der aktuellen Stelle im DisplayBuffer gemäß iChClk abrufen

  // die 7 Segment Ports 0 - 6 gemäß aktuellem Wert setzen
  for (i = 0; i <= 6; i++)
  {
    iSeg = iarSegs[i];
    /* Diese Funktion führt zu einem sporadischen CORE1 Panic
    digitalWrite(iarSegs[i], (iarNum2Segs[uiClk[iChClk]][i] == 1));
    */
    bOut = (iarNum2Segs[iNum][i] == 1);
    digitalWrite(iSeg, bOut);
    ledcWrite(iChClk, uiLum); // Brightness is set to value of uiLUM
  }
} // End UpdateDisplay

void DisplayTest()
  {
    int i;
    // Start testlauf
    Serial.println("Testlauf");

    // Kontroll-LED, die blaue LED dient während der Entwicklung
    // der Visualisierung des Testlaufs.

    // Kontroll-LED 10 mal blinken
    for (i = 0; i < 9; ++i)
    {
      digitalWrite(LED_CTL, HIGH);
      delay(200);
      digitalWrite(LED_CTL, LOW);
      delay(200);
    }

    // Alle 7 Segmente aus
    for (i = 0; i < 7; ++i)
    {
      digitalWrite(iarSegs[i], LOW);
    }

    // Alle 7Segment-Digits über Common Cath aus.
    for (i = 0; i <= 3; ++i)
    {
      ledcWrite(i, 0); // Setz die Pulsweite für jeden Kanal auf 0, die Digits sind aus
    }

    // Jetzt alle 7 Segmente eines Digits EIN, Common Cath. ist noch aus
    for (i = 0; i < 7; ++i)
    {
      digitalWrite(iarSegs[i], HIGH); //
    }

    // Testlauf: jedes Digit einmal ein mit allen Segmenten angesteuert
    for (i = 0; i <= 3; ++i)
    {
      digitalWrite(LED_CTL, HIGH); // Kontroll-LED an
      ledcWrite(i, 1000);          // Setz die Pulsweite für Kanal/Digit i auf ca maximum, jeweils ein Digit i ist an
      delay(200);                  // Warte 300ms
      ledcWrite(i, 0);             // Setz die Pulsweite für Kanal/Digit i auf 0, das Digit i ist wieder aus.
      digitalWrite(LED_CTL, LOW);  // Kontroll-LED aus
      delay(200);                  // Warte 300 ms
    }
  } // End DisplayTest

  void TouchControl()
  {
    // Touch sensor lesen, auswerten und ggf Sommerzeit setzen
    ulTime = millis();
    if ((ulTime - ulLstTime) > 100)
    { // alle 100 ms den TouchSensor abfragen
      ulLstTime = ulTime;
      /*
      if ((uiTouch - touchRead(14)) > 50){ //Mit Pad Max 73 und Min ~10 Counts
      //Werten wenn Count < 15 ist anstatt die Differenz zu nutzen??
       GetTime();
      }
      */
      // if (touchRead(TPIN14) < 35)
      if (touchRead(TOUCH14) < 35)
      {
        // Alle100ms wird uiTouch14 + 1 gezählt
        // nach 3 Durchläufen (TouchRead(TPIN) < 35) = 300ms wird die Routine aktiv
        uiTouch14++;
        if (uiTouch14 > 3)
        {
          uiTouch14 = 0;
          if (DayLightSavingTime_Sec == 0)
          {
            DayLightSavingTime_Sec = 3600;
          }
          else
          {
            DayLightSavingTime_Sec = 0;
          }
          GetTime();
          Serial.printf("Touch Value %i\n", uiTouch14);
          Serial.printf("Sommerzeit : %i\n", DayLightSavingTime_Sec);
        }
      }
      else
      {
        // TouchRead(TPIN) > 35 setzt uiTouch14 immer auf null
        uiTouch14 = 0;
      }

      // uiTouch33 : Helligkeit
      // if (touchRead(TPIN33) < 35)
      if (touchRead(TOUCH33) < 35)
      {
        // Alle100ms wird uiTouch33 + 1 gezählt
        // nach 5 Durchläufen (TouchRead(TPIN) < 35) = 500ms wird die Routine aktiv
        uiTouch33++;
        if (uiTouch33 > 3)
        {
          uiTouch33 = 0;

          if (bLumDir)
          {
            uiLum = uiLum + 100;
            if (uiLum > 2000)
            {
              uiLum = 2000;    // Maximale Leuchtstärke
              bLumDir = false; // Zählrichtung abwärts
            }
          }
          else
          {
            uiLum = uiLum - 100;
            if (uiLum < 200)
            {
              uiLum = 200;    // Minimale Leuchtstärke
              bLumDir = true; // Zählrichtung abwärts
            }
          }
          Serial.printf("Touch Value LUM %i\n", uiTouch33);
          Serial.printf("Luminescence : %i\n", uiLum);
          digitalWrite(LED_CTL, LOW);
        }
      }
      else
      {
        // TouchRead(TPIN33) >35 setzt uiTouch33 immer auf null
        uiTouch33 = 0;
      }
    }
  } //End TouchControl