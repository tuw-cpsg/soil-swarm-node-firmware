// Singel read: gatttool -b E8:62:A2:78:17:55 -t random --char-read --handle=0x0011
// Cont. read:  gatttool -b E8:62:A2:78:17:55 -t random --char-write-req --handle=0x0012 --value=0100 --listen
/*
 * BEACON UUID: ec60de83-4b7e-4c75-96c9-2f4e76617a7e
 * MAJOR: 102
 * MINOR: 101
 */
 
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
  unsigned long timestamp;
  int16_t temp;
  int16_t moisture;
  uint8_t battery;
} sense_t;

uint16_t moisture_l;
uint16_t moisture_h;

extern "C" {
  
  extern uint32_t *PWM_Channels_Value;
  
  void start_500kHz_clock(uint32_t ulPin)
  {
    
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
    }
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
                            
    NRF_TIMER1->PRESCALER = 0;
    // Adjust the output frequency by adjusting the CC. 
    // Due to PAN item 33, you can't have this at 1 for first revision chips, and will hence be limited to 4 MHz. 
    //NRF_TIMER1->CC[0] = 16;
    NRF_TIMER1->CC[0] = 1;
    NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;
  }

  void start_timer1(uint32_t ulPin) {
    NRF_GPIOTE->CONFIG[0] = GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos |
                            GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos |
                            ulPin << GPIOTE_CONFIG_PSEL_Pos | 
                            GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos;
                            
    NRF_PPI->CH[0].EEP = (uint32_t) &NRF_TIMER1->EVENTS_COMPARE[0];
    NRF_PPI->CH[0].TEP = (uint32_t) &NRF_GPIOTE->TASKS_OUT[0];
    
    NRF_PPI->CHENSET = PPI_CHENSET_CH0_Enabled << PPI_CHENSET_CH0_Pos;
    
    NRF_TIMER1->TASKS_START = 1;
  }
  
  void stop_timer1(uint32_t ulPin) {
    NRF_TIMER1->TASKS_STOP = 1;
    
    turn_Off_GPIOTE_PPI_from_GPIO(ulPin);
    nrf_gpiote_unconfig(0);
    simblee_ppi_channel_unassign(0);
    NRF_GPIO->OUTCLR = (1 << ulPin);
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

unsigned long wakeup_time = 0;
unsigned long past = 0, now = 0, seconds = 0, diff = 0;

void setup()
{
#ifdef DEBUG
  Serial.begin(9600);
#endif

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
  digitalWrite(4, LOW);
  digitalWrite(6, LOW);
  
  /* --- CSENSOR -------------------- */
  start_500kHz_clock(10);
  stop_timer1(10);
  //pinMode(10, OUTPUT);
  //digitalWrite(10, LOW);    // disable

  digitalWrite(TEMP_PIN_EN, HIGH);
  Simblee_ULPDelay( 50 );
  ds.reset_search();
  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      addr[0] = -1;    
  }

  digitalWrite(TEMP_PIN_EN, LOW);

  /* --- BLE ------------------------ */
  // do iBeacon advertising
  SimbleeBLE.iBeacon = true;
  
  // override the default iBeacon settings
  uint8_t uuid[16] = {0xEC, 0x60, 0xde, 0x83, 0x4b, 0x7e, 0x4c, 0x75, 0x96, 0xc9, 0x2f, 0x4e, 0x76, 0x61, 0x7a, 0x7e};
  memcpy(SimbleeBLE.iBeaconUUID, uuid, sizeof(SimbleeBLE.iBeaconUUID));
  SimbleeBLE.iBeaconMajor = 102;
  SimbleeBLE.iBeaconMinor = 101;
  SimbleeBLE.iBeaconMeasuredPower = 0xC6;
  
  SimbleeBLE.txPowerLevel = 4;
  SimbleeBLE.advertisementInterval = 900;
  
  SimbleeBLE.begin(); // start the BLE stack
}

void loop()
{
  
  char buf[80];  
  //if(!bConnected)
  //  return;

  past = now;
  now = millis();
  diff = now - past;
  
  if(now < past)
  {
    diff = 4294967295 - past + now;
  }
  seconds += (diff / 1000);
  
  wakeup_time = now + UPDATE_INTERVAL - 100;
  Simblee_ULPDelay( UPDATE_INTERVAL - 100 );

  led_on();
  Simblee_ULPDelay( 1 );
  led_off();
  
  const uint16_t dry_val = 880;
  const uint16_t wet_val = 440;

  sense_t value;
  measure_moisture();
  
  value.timestamp = seconds;
  value.moisture  = moisture_h - moisture_l;
  value.temp      = read_temperature();
  value.battery   = batt_read_value();

  SimbleeBLE.send((char *)&value, sizeof(value));

#ifdef DEBUG
  sprintf(buf, "CSense: Low: %u ; High: %u ; diff: %d ; ",
    moisture_l, moisture_h, value.moisture);
  Serial.print(buf);
  
  Serial.print(value.temp);
  Serial.print(" ; ");
  Serial.print(value.battery);
  Serial.println();
#endif
}

void SimbleeBLE_onAdvertisement(bool start) {
  if (start) {
    Serial.println("Advertising started");
  } else {
    Serial.println("Advertising ended");
  }
}

void SimbleeBLE_onConnect() {
#ifdef DEBUG
  Serial.println("Connected!");
#endif
  bConnected = true;
}

void SimbleeBLE_onDisconnect() {
#ifdef DEBUG
  Serial.println("Disconnected");
#endif
  bConnected = false;
}

int16_t read_temperature()
{
  //returns the temperature from one DS18S20 in DEG Celsius

  digitalWrite(TEMP_PIN_EN, HIGH);
  
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

  digitalWrite(TEMP_PIN_EN, LOW);
  
  byte MSB = data[1];
  byte LSB = data[0];

  return (int16_t) ((MSB << 8) | LSB);  //using two's compliment ; needs to be divided / 16
}

void measure_moisture()
{
  uint16_t value = 0.0f;

  start_timer1(10);
  digitalWrite(SENSE_EN_PIN, HIGH);         // enable
  Simblee_ULPDelay( 1 );
  
  moisture_l = analogRead(SENSE_LOW_PIN);   // read
  moisture_h = analogRead(SENSE_HIGH_PIN);  // read
  
  digitalWrite(SENSE_EN_PIN, LOW);          // disable
  stop_timer1(10);
}

