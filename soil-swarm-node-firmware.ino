// Singel read: gatttool -b E8:62:A2:78:17:55 -t random --char-read --handle=0x0011
// Cont. read:  gatttool -b E8:62:A2:78:17:55 -t random --char-write-req --handle=0x0012 --value=0100 --listen

#include <OneWire.h>
#include <SimbleeBLE.h>

#define UPDATE_INTERVAL SECONDS(5)

#define BATT_EN_PIN 17 // Pin 12 ; P0.17 ; I/O ; GPIO 17
#define BATT_PIN 5 // Pin 21 ; P0.15 ; GPIO 5 / ANALOG 6

#define TEMP_PIN_EN 13  // Port 28 ; P0.13 ; GPIO13
#define TEMP_PIN 14  // Port 29 ; P0.14 ; GPIO14

#define SENSE_EN_PIN   7  // Port 26 / P0.7 / GPIO7
#define SENSE_LOW_PIN  2  // Port 24 / P0.2 / GPIO2
#define SENSE_HIGH_PIN 3  // Port 22 / P0.3 / GPIO3

bool bConnected = false;

//Temperature chip i/o
byte addr[8];
OneWire ds(TEMP_PIN); // Pin 26 / GPIO 7

typedef struct {
  float temp;
  float battery;
  int16_t moisture;
} sense_t;

uint16_t moisture_l;
uint16_t moisture_h;

extern "C" {
  extern uint32_t *PWM_Channels_Value;
  
  void start_500kHz_clock(uint32_t ulPin)
  {
    digitalWrite(ulPin, LOW);
    pinMode(ulPin, OUTPUT);
    
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
    }
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    
    NRF_GPIOTE->CONFIG[0] = GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos |
                            GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos |
                            ulPin << GPIOTE_CONFIG_PSEL_Pos | 
                            GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos;
                            
    NRF_TIMER1->PRESCALER = 0;
    // Adjust the output frequency by adjusting the CC. 
    // Due to PAN item 33, you can't have this at 1 for first revision chips, and will hence be limited to 4 MHz. 
    NRF_TIMER1->CC[0] = 16;
    NRF_TIMER1->CC[0] = 1;
    NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;
    //NRF_TIMER1->TASKS_START = 1;
    
    NRF_PPI->CH[0].EEP = (uint32_t) &NRF_TIMER1->EVENTS_COMPARE[0];
    NRF_PPI->CH[0].TEP = (uint32_t) &NRF_GPIOTE->TASKS_OUT[0];
    
    NRF_PPI->CHENSET = PPI_CHENSET_CH0_Enabled << PPI_CHENSET_CH0_Pos;
  }

  void start_timer1(void) {
    NRF_TIMER1->TASKS_START = 1;
  }
  
  void stop_timer1(void) {
    NRF_TIMER1->TASKS_STOP = 1;
  }
}

/* --- ADC ------------------------------------------------------ */
uint8_t batt_read_value(void)
{
  digitalWrite(BATT_EN_PIN, HIGH);
  int temp = analogRead(BATT_PIN);
  digitalWrite(BATT_EN_PIN, LOW);
  return map(temp, 0, 1023, 0, 255);
}

/* --- LED ------------------------------------------------------ */

void led_on(void)
{
  digitalWrite(4, LOW);
  digitalWrite(6, HIGH);
}

void led_off(void)
{
  digitalWrite(4, LOW);
  digitalWrite(6, LOW);
}

void setup()
{
  Serial.begin(9600);

  /* --- ADC ------------------------ */
  pinMode(BATT_EN_PIN, OUTPUT);
  digitalWrite(BATT_EN_PIN, LOW);
  
  pinMode(SENSE_EN_PIN, OUTPUT);
  digitalWrite(SENSE_EN_PIN, LOW); // LOW == off / HIGH == on

  pinMode(TEMP_PIN_EN, OUTPUT);
  digitalWrite(TEMP_PIN_EN, HIGH);

  /* --- LED ------------------------ */
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);
  //start_500kHz_clock(6);
  
  /* --- CSENSOR -------------------- */
  start_500kHz_clock(10);
  //pinMode(10, OUTPUT);
  //digitalWrite(10, LOW);    // disable
  
  SimbleeBLE.deviceName = "SoilSensor";
  SimbleeBLE.advertisementInterval = 1000;
  SimbleeBLE.txPowerLevel = 4;
  SimbleeBLE.connectable = true;

  // start the BLE stack
  SimbleeBLE.begin();

  ds.reset_search();
  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      addr[0] = -1;    
  }
}

void loop()
{
  char buf[80];  
  //if(!bConnected)
  //  return;

  led_on();
  //start_timer1();
  Simblee_ULPDelay( 1 );
  //stop_timer1();
  led_off();
  
  const uint16_t dry_val = 880;
  const uint16_t wet_val = 440;

  sense_t value;
  // get value from 0.0 (dry) to 1.0 (wet)
  //value.moisture = (float)analogRead(SOIL_OUT);
  //value.moisture = ((float)analogRead(SOIL_OUT) - (float)wet_val) / ((float) dry_val - (float) wet_val);
  //value.moisture = constrain(1.0 - value.moisture, 0.0, 1.0);
  measure_moisture();
  
  value.moisture   = moisture_h - moisture_l;
  value.temp       = read_temperature();
  value.battery    = (float)batt_read_value();

  SimbleeBLE.send((char *)&value, sizeof(value));

  if(true)
  {
  //return;

    sprintf(buf, "CSense: Low: %u ; High: %u ; diff: %d",
      moisture_l, moisture_h, value.moisture);
    Serial.println(buf);
  
    Serial.print(value.temp);
    Serial.print(" ; ");
    Serial.print(value.battery);
    Serial.println();
  }
  
  Simblee_ULPDelay( UPDATE_INTERVAL - 100 );
}

void SimbleeBLE_onAdvertisement(bool start) {
  if (start) {
    Serial.println("Advertising started");
  } else {
    Serial.println("Advertising ended");
  }
}

void SimbleeBLE_onConnect() {
  Serial.println("Connected!");
  bConnected = true;
}

void SimbleeBLE_onDisconnect() {
  Serial.println("Disconnected");
  bConnected = false;
}

float read_temperature()
{
  //returns the temperature from one DS18S20 in DEG Celsius
 
  byte data[12];
 
  if (addr[0] == -1) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1001;
  }
 
  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1002;
  }
 
  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1003;
  }
 
  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end
  Simblee_ULPDelay( 900 );
 
  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad
 
  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  
  //ds.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];
 
  uint16_t tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = (float)tempRead / 16.0;
  
  return TemperatureSum;
}

void measure_moisture()
{
  uint16_t value = 0.0f;

  start_timer1();
  digitalWrite(SENSE_EN_PIN, HIGH);         // enable
  Simblee_ULPDelay( 1 );
  
  moisture_l = analogRead(SENSE_LOW_PIN);   // read
  moisture_h = analogRead(SENSE_HIGH_PIN);  // read
  
  digitalWrite(SENSE_EN_PIN, LOW);          // disable
  stop_timer1();
}

