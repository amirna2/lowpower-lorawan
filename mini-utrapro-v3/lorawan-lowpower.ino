#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <RTCZero.h>
#include <SerialFlash.h>

// for the BME280
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//for BH1750
#include <BH1750.h>

// Create the Lightsensor object
BH1750 lightSensor(0x23);
// Create bme280 I2C sensor object
Adafruit_BME280 bme;

/* for capacitive moisture sensor */
int MOISTURE_SENSOR_PIN = 13;
int MIN_ADC = 1720;
int MAX_ADC = 3800;

#define EUI64_CHIP_ADDRESS 0x50
#define EUI64_MAC_ADDRESS 0xF8
#define EUI64_MAC_LENGTH 0x08

RTCZero rtc;

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
u1_t PROGMEM DEVEUI[8];
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x10, 0x09, 0x10, 0x50, 0x00, 0x5d, 0x30, 0x06, 0x10, 0x09, 0x10, 0x50, 0x00, 0x5d, 0x30, 0x06 };
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

// sensor data: variable length
// [header]
// [battery]      2 bytes  - Volt ( e.g 355 -> 3.55v)
// [temp]         1 byte  - Farenheit
// [humidity]     1 byte  - percent
// [pressure]     2 bytes - hPa
// [moisture]     1 byte  - percent
// [ground temp]  1 byte  - Farenheit
// [light]        2 bytes - Lux (max 65535)

#define BAT       0x01
#define ATEMP     0x02
#define HUMID     0x04
#define PRESS     0x08
#define MOIST     0x10
#define GTEMP     0x20
#define LIGHT     0x40


//const uint8_t PACKET_SIZE = 10;
//uint8_t fullPayload[PACKET_SIZE];
uint8_t *fullPayload;
uint8_t data_length = 1; // counting the header

typedef struct sensor_data {
  uint16_t light;
  uint16_t pressure;
  uint16_t battery;

  int8_t air_temp;
  int8_t humidity;
  int8_t moisture;
  int8_t ground_temp;
} sensor_data;


sensor_data sensorData;

// save the session info in RTC memory during deep sleep
u4_t netid = 0;
devaddr_t devaddr = 0;
u1_t nwkKey[16];
u1_t artKey[16];
u4_t sequence_up = 0;
u4_t sequence_dn = 0;

bool isShutdown = false;
bool deep_sleep_enabled = true;
bool send_once = false;
bool always_send = false;

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 30;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 5,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 3,
  .dio = {2, 6, LMIC_UNUSED_PIN},
};

/**
   void initLoraRadio()
   Performs the LoRa radio initialization using LMIC library APIs
   If the radio was previous shutdown for low power mode, we re-establish the session from the saved parameters instead
   of joining the network again
*/
void initLoraRadio() {
  // LMIC init
  SerialUSB.println("LMIC: Initializing...");
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.

  SerialUSB.println("LMIC: Reseting...");
  LMIC_reset();
  LMIC_setClockError(5 * MAX_CLOCK_ERROR / 100);
  //For CFG_us915
  LMIC_selectSubBand(1);
  //LMIC_setDrTxpow(DR_SF7,14);
  if (isShutdown) {
    isShutdown = false;
    
    SerialUSB.println("Was shutdown..set session");
    printSessionInfo();
    LMIC_setSession(netid, devaddr, nwkKey, artKey);
    LMIC_setSeqnoUp(sequence_up);
    LMIC.seqnoDn = sequence_dn;
  }
}

/**
   void shutDownRadio()
   Powers down the Lora Radio (RFM95W) using the LMIC shutdown API
   The downlink and uplink sequence numbers must be saved as they will be needed when
   we restore the radio context when re-initializing the radio without needing to Join again.
   This method assumes the device EUI, the app session and the network session have also been saved when first made available
*/
void shutDownRadio() {
  Serial.println(F("Shutting LMIC Down"));
  // save sequence numbers so we can restore/resume when we wake up
  sequence_up = LMIC.seqnoUp;
  sequence_dn = LMIC.seqnoDn;
  LMIC_shutdown();
  
  digitalWrite(24, LOW); // SCK
  pinMode(24, OUTPUT);
  digitalWrite(23, LOW); // MOSI
  pinMode(23, OUTPUT);

  pinMode(22, INPUT_PULLUP); // MISO

  digitalWrite(5, LOW); // NSS
  pinMode(5, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP); // RST

  // DIO Inputs
  pinMode(2, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);

  isShutdown = true;
}

void goToSleepNow() {
  shutDownRadio();

  SerialUSB.println(F("Going to standby mode"));
  // Ensure all debugging messages are sent before sleep
  SerialUSB.flush();
  // Set sleep period of TX_INTERVAL using single shot alarm
  rtc.setAlarmEpoch(rtc.getEpoch() + TX_INTERVAL);
  rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
  rtc.attachInterrupt(alarmMatch);
  
  // USB port consumes extra current
  USBDevice.detach();
  
  // Enter sleep mode
  rtc.standbyMode();
  
  // Waking up and resuming code execution
  // Reinitialize USB for debugging
  USBDevice.init();
  USBDevice.attach();

  initLoraRadio();

  // Schedule next transmission to be immediately after this
  // because we send data as part of the sleep/wake cycle
  // if the device was always awake the callback interval would be TX_INTERVAL
  os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(1), do_send);  
}

void onEvent (ev_t ev)
{
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      SerialUSB.println(F("SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      SerialUSB.println(F("BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      SerialUSB.println(F("BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      SerialUSB.println(F("BEACON_TRACKED"));
      break;
    case EV_JOINING:
      SerialUSB.println(F("JOINING"));
      break;
    case EV_JOINED:
      SerialUSB.println(F("JOINED"));
      LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
      printSessionInfo();
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      SerialUSB.println(F("RFU1"));
      break;
    case EV_JOIN_FAILED:
      SerialUSB.println(F("JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      SerialUSB.println(F("REJOIN_FAILED"));
      break;
      break;
    case EV_TXCOMPLETE:
      digitalWrite(LED_BUILTIN, LOW);

      if (!deep_sleep_enabled) {      
        SerialUSB.println(F("TXCOMPLETE ( + waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK) {
            SerialUSB.println(F("Received ack"));
        }
        if (LMIC.dataLen) {
            Serial.print(F("Received "));
            Serial.print(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }
      }
      if (deep_sleep_enabled) {
        goToSleepNow();
      } else {
        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      }
      break;
    case EV_LOST_TSYNC:
      SerialUSB.println(F("LOST_TSYNC"));
      break;
    case EV_RESET:
      SerialUSB.println(F("RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      SerialUSB.println(F("RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      SerialUSB.println(F("LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      SerialUSB.println(F("LINK_ALIVE"));
      break;
    case EV_SCAN_FOUND:
      SerialUSB.println(F("SCAN_FOUND"));
      break;
    case EV_TXSTART:
      SerialUSB.println(F("TXSTART"));
      break;
    default:
      SerialUSB.print(F("Unknown event:"));
      SerialUSB.println(ev);
      break;
  }
}

void printDevEUI() {
  SerialUSB.print(F("DEVEUI: "));
  for (int count = 8; count > 0; count--) {
    SerialUSB.print("0x");
    if (DEVEUI[count - 1] <= 0x0F) SerialUSB.print("0");
    SerialUSB.print(DEVEUI[count - 1], HEX);
    SerialUSB.print(" ");
  }
  SerialUSB.println();
}

void printSessionInfo() {
  SerialUSB.print("netid: ");
  SerialUSB.println(netid, DEC);
  SerialUSB.print("devaddr: ");
  SerialUSB.println(devaddr, HEX);
  SerialUSB.print("appSessionKey: ");
  for (int i = 0; i < sizeof(artKey); ++i) {
    SerialUSB.print(artKey[i], HEX);
  }
  SerialUSB.println("");
  SerialUSB.print("netSessiomKey: ");
  for (int i = 0; i < sizeof(nwkKey); ++i) {
    SerialUSB.print(nwkKey[i], HEX);
  }
  SerialUSB.println("");
  SerialUSB.print("sequence up: ");
  SerialUSB.println(sequence_up, DEC);
  SerialUSB.print("sequence dn: ");
  SerialUSB.println(sequence_dn, DEC);
  SerialUSB.println("");

}

uint16_t getBatteryVoltage() {
  unsigned char counter;
  float batteryVoltage;
  int adcReading;
  uint16_t voltage;

  adcReading = analogRead(A5);
  // Discard inaccurate 1st reading
  adcReading = 0;
  // Perform averaging
  for (counter = 10; counter > 0; counter--)
  {
    adcReading += analogRead(A5);
  }
  adcReading = adcReading / 10;
  // Convert to volts
  batteryVoltage = adcReading * (4.3 / 1023.0);

  SerialUSB.print(F("Battery: "));
  SerialUSB.print(batteryVoltage);
  SerialUSB.println(F(" V"));

  // Pack float into int with 2 decimal point resolution
  voltage = batteryVoltage * 100;
  return voltage;
}

void do_send(osjob_t* j)
{
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    SerialUSB.println(F("OP_TXRXPEND, not sending"));
  }
  else {
    SerialUSB.println(F("Getting sensor data and sending..."));

    // check if we have any sensor data updates and send them
    // otherwise we go right back to sleep
    if (get_sensor_data()) {
      digitalWrite(LED_BUILTIN, HIGH);
      // Prepare upstream data transmission at the next possible time.
      int err = LMIC_setTxData2(1, fullPayload, data_length, 0);
      if (err != 0) {
        SerialUSB.print("LMIC_setTxData2 returned an error: ");SerialUSB.println(err);
      }
      SerialUSB.println("[do_send] Packet queued");
    } else {
      if (deep_sleep_enabled) {
        // going right back to sleep. This will also schedule the next send job (1 second after waking up);
        goToSleepNow(); 
      } else {
        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      }
    }
  }
}

void configureUnusedPins() {

  unsigned char pinNumber;

  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);

  // D7-D13, A0(D14)-A5(D19), SDA(D20), SCL(D21), MISO(D22)
  for (pinNumber = 7; pinNumber <= 22; pinNumber++) {
    pinMode(pinNumber, INPUT_PULLUP);
  }

  // RX_LED (D25) & TX_LED (D26) (both LED not mounted on Mini Ultra Pro)
  pinMode(25, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
  // D30 (RX) & D31 (TX) of SerialUSB
  pinMode(30, INPUT_PULLUP);
  pinMode(31, INPUT_PULLUP);
  // D34-D38 (EBDG Interface)
  for (pinNumber = 34; pinNumber <= 38; pinNumber++) {
    pinMode(pinNumber, INPUT_PULLUP);
  }
}

float RawToLux(int raw) {
  float rawRange = 4096.0; // 3.3v
  float logRange = 5.0; // 3.3v = 10^5 lux
  float logLux = raw * logRange / rawRange;
  return pow(10, logLux);
}

float dewPoint( float temp, float hum, bool celcius)
{
  float dewPoint = NAN;

  if (celcius) {
    dewPoint = 243.04 * (log(hum / 100.0) + ((17.625 * temp) / (243.04 + temp)))
               / (17.625 - log(hum / 100.0) - ((17.625 * temp) / (243.04 + temp)));
  }
  else { // Farenheit
    float ctemp = (temp - 32.0) * 5.0 / 9.0;

    dewPoint = 243.04 * (log(hum / 100.0) + ((17.625 * ctemp) / (243.04 + ctemp)))
               / (17.625 - log(hum / 100.0) - ((17.625 * ctemp) / (243.04 + ctemp)));

    dewPoint = dewPoint * 9.0 / 5.0 + 32.0;
  }
  return dewPoint;
}

void init_bme280() {
  unsigned status = bme.begin();
  if (!status) {
    SerialUSB.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    SerialUSB.print("SensorID was: 0x"); SerialUSB.println(bme.sensorID(), 16);
    while (1);
  }
  SerialUSB.println();
}

void init_bh1750() {
  SerialUSB.println("BH1750: Initializing...");

  if (lightSensor.begin(BH1750::ONE_TIME_HIGH_RES_MODE_2)) {
    SerialUSB.println(F("BH1750 Advanced begin"));
  }
  else {
    SerialUSB.println(F("Error initialising BH1750"));
  }
}

void setDevEui(unsigned char* buf) {
  Wire.begin();
  Wire.beginTransmission(EUI64_CHIP_ADDRESS);
  Wire.write(EUI64_MAC_ADDRESS);
  Wire.endTransmission();
  Wire.requestFrom(EUI64_CHIP_ADDRESS, EUI64_MAC_LENGTH);

  // Format needs to be little endian (LSB...MSB)
  while (Wire.available()) {
    *buf-- = Wire.read();
  }
}

void setup()
{

  configureUnusedPins();

  pinMode(LED_BUILTIN, OUTPUT);

  while (!SerialUSB && millis() < 10000);
  SerialUSB.begin(115200);
  SerialUSB.println(F("Starting"));

  setDevEui(&DEVEUI[EUI64_MAC_LENGTH - 1]);
  printDevEUI();

  // Initialize serial flash
  SerialFlash.begin(4);
  // Put serial flash in sleep
  SerialFlash.sleep();

  // Initialize RTC
  rtc.begin();
  // Use RTC as a second timer instead of calendar
  rtc.setEpoch(0);

  SerialUSB.println("HUB-001 STARTING...");
  SerialUSB.println("BME280: Initializing...");
  init_bme280();
  init_bh1750();
  
  memset(&sensorData, 0, sizeof(sensor_data));

  initLoraRadio();

  // Start job (sending automatically starts OTAA too)
  SerialUSB.println("LMIC: Starting OTAA...");
  do_send(&sendjob);
}

bool get_sensor_data() {
  uint16_t l, p, b;
  int8_t t, h, m, g = 20;

  if ( fullPayload != NULL ) {
    data_length = 1;
    free(fullPayload);
  }

  // ambient air temp
  t = bme.readTemperature();
  // barometric pressure
  p = bme.readPressure() / 100.0F;
  // air humidity
  h = bme.readHumidity(); // Percentage
  // moisture
  int val = analogRead(MOISTURE_SENSOR_PIN);
  if (val <= MIN_ADC) {
    val = MIN_ADC;
  } else if ( val >= MAX_ADC) {
    val = MAX_ADC;
  }
  m  = ((MAX_ADC - val) / (float)(MAX_ADC - MIN_ADC) ) * 100.0;

  // light intensity
  l = lightSensor.readLightLevel();

  // read battery voltage
  b = getBatteryVoltage();

  SerialUSB.println("============================");

  uint8_t header = 0x00;

  if (b != sensorData.battery || always_send) {
    header += BAT;
    data_length += 2;
    sensorData.battery = b;
    SerialUSB.print("B = "); SerialUSB.print((float)b / 100.00); SerialUSB.println("v");
  }

  if (t != sensorData.air_temp || always_send) {
    data_length += 1;
    header += ATEMP;
    sensorData.air_temp = t;
    SerialUSB.print("T = "); SerialUSB.print(t); SerialUSB.println(" *C");
  }

  if (h != sensorData.humidity || always_send) {
    data_length += 1;
    header += HUMID;
    sensorData.humidity = h;
    SerialUSB.print("H = "); SerialUSB.print(h); SerialUSB.println(" %");
  }

  if (p != sensorData.pressure || always_send) {
    data_length += 2;
    header += PRESS;
    sensorData.pressure = p;
    SerialUSB.print("P = "); SerialUSB.print(p); SerialUSB.println(" hPa");
  }

  if (m != sensorData.moisture || always_send) {
    data_length += 1;
    header += MOIST;
    sensorData.moisture = m;
    SerialUSB.print("M = "); SerialUSB.print(m); SerialUSB.println(" %");
  }
  if (g != sensorData.ground_temp || always_send) {
    data_length += 1;
    header += GTEMP;
    sensorData.ground_temp = g;
    SerialUSB.print("G = "); SerialUSB.print(g); SerialUSB.println(" *C");
  }

  if (l != sensorData.light || always_send) {
    data_length += 2;
    header += LIGHT;
    sensorData.light = l;
    SerialUSB.print("L = "); SerialUSB.print(l); SerialUSB.println(" lux");
  }

  // don't send data if nothing's changed
  if (header == 0) {
    return false;
  }
  
  // [header]       1 byte
  // [battery]      2 bytes - Volt
  // [temp]         1 byte  - Celicius
  // [humidity]     1 byte  - Percent
  // [pressure]     2 bytes - HPa
  // [moisture]     1 byte  - Percent
  // [ground temp]  1 byte  - Celcius
  // [light]        2 bytes - Lux

  fullPayload = (uint8_t *) malloc(sizeof(uint8_t) * data_length);

  int pos = 0;
  fullPayload[pos] = header;

  if ((header & BAT) == BAT) {
    fullPayload[++pos] = lowByte(sensorData.battery);
    fullPayload[++pos] = highByte(sensorData.battery);
  }
  if ((header & ATEMP) == ATEMP) {
    fullPayload[++pos] = sensorData.air_temp;
  }
  if ((header & HUMID) == HUMID) {
    fullPayload[++pos] = sensorData.humidity;
  }
  if ((header & PRESS) == PRESS) {
    fullPayload[++pos] = lowByte(sensorData.pressure);
    fullPayload[++pos] = highByte(sensorData.pressure);
  }
  if ((header & MOIST) == MOIST) {
    fullPayload[++pos] = sensorData.moisture;
  }
  if ((header & GTEMP) == GTEMP) {
    fullPayload[++pos] = sensorData.ground_temp;
  }
  if ((header & LIGHT) == LIGHT) {
    fullPayload[++pos] = lowByte(sensorData.light);
    fullPayload[++pos] = highByte(sensorData.light);
  }

  SerialUSB.print(" length:"); SerialUSB.println(data_length);
  for (int i = 0; i < data_length; ++i) {
    SerialUSB.print("0x");
    SerialUSB.print(fullPayload[i], HEX);
    SerialUSB.print(" ");
  }

  SerialUSB.println("");
  SerialUSB.println("============================");

  return true;
}

void loop() {
  os_runloop_once();
}

void alarmMatch() {

}