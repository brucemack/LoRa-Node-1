// REMOTE
// 80mA idle, 160mA transmit with normal clock
// 30mA at 10 MHz clock rate

// Install 
// * RadioHead
// * SimpleSerialShell

#include <SPI.h>
#include <RH_RF95.h>
#include <RHMesh.h>
#include "WiFi.h"
#include <esp_task_wdt.h>
#include <Preferences.h>
#include <SimpleSerialShell.h>

#define VERSION 2

// Define the pins used by the LoRa transceiver module
// ESP32
#define ss 5
#define rst 14
#define dio0 4
#define LED_PIN 2
#define BATTERY_LEVEL_PIN 33

#define US_TO_S_FACTOR 1000000

// Channel configurations
//float frequency = 434;
float frequency = 916;
// Transmit power in dBm
int txPower = 20;
// Higher spreading factor for longer distance
int spreadingFactor = 9;
// Setup BandWidth, option: 7800,10400,15600,20800,31250,41700,62500,125000,250000,500000
// Lower BandWidth for longer distance.
//long signalBandwidth = 125000;
long signalBandwidth = 31250;
// Setup Coding Rate:5(4/5),6(4/6),7(4/7),8(4/8) 
int codingRate = 5;
// Address
int address = 0xf3;

// Deep sleep duration when low battery is detected
#define DEEP_SLEEP_SECONDS 60 * 15

// ===== NODE CONFIGURATION ================================================================
// Battery reading scale
float batteryScale = (3.3 / 4096.0) * 2.2;
// Battery limit
uint16_t lowBatteryLimitMv = 3600;
//uint16_t lowBatteryLimitMv = 0;

//#define MY_NODE_ADDR 3
#define MY_NODE_ADDR 4
//#define MY_NODE_ADDR 1
// ==========================================================================================

static char CALL[9] = "KC1FSZ  ";

int rec_count = 0;
unsigned long tick_count = 0;

// Watchdog timeout in seconds (NOTE: I think this time might be off because
// we are changing the CPU clock frequency)
#define WDT_TIMEOUT 15

RH_RF95 rf95(ss, dio0);
RHMesh mesh_manager(rf95, MY_NODE_ADDR);

Preferences preferences;

// Shell Stuff

static auto msg_arg_error = F("Argument error");

int helloWorld(int argc, char **argv) { 
  shell.println("Hello!");
}

int sendReset(int argc, char **argv) { 

  if (argc == 2) {

    uint8_t targetNodeAddr = atoi(argv[1]);
    
    uint8_t data[32];
    // VERSION
    data[0] = VERSION;
    // COMMAND
    data[1] = 4;

    shell.print("Resetting node ");
    shell.println(targetNodeAddr);
  
    uint8_t rc = mesh_manager.sendtoWait(data, 26, targetNodeAddr);
    if (rc != RH_ROUTER_ERROR_NONE) {
      shell.println("Failed to send");
    }
        
  } else {
    shell.println(msg_arg_error);
  }
}

int sendPing(int argc, char **argv) { 

  static int counter = 0;
  
  if (argc == 2) {

    uint8_t targetNodeAddr;
    
    if (argv[1][0] == '*') 
      targetNodeAddr = RH_BROADCAST_ADDRESS;
    else 
      targetNodeAddr = atoi(argv[1]);
    
    uint8_t data[32];
    // VERSION
    data[0] = 1;
    // COMMAND
    data[1] = 1;
    // Callsign
    for (int i = 0; i < 8; i++)
      data[2 + i] = CALL[i];
    // Payload
    for (int i = 0; i < 16; i++)
      data[10 + i] = 0;
    sprintf((char*)(data + 10), "KC1FSZ %d", counter++);

    shell.print("Pinging node ");
    shell.println(targetNodeAddr);
  
    uint8_t rc = mesh_manager.sendtoWait(data, 26, targetNodeAddr);
    if (rc != RH_ROUTER_ERROR_NONE) {
      shell.println("Failed to send");
    }
        
  } else {
    shell.println(msg_arg_error);
  }
}

void configRadio(RH_RF95& radio) {
  //radio.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr48Sf4096);
  radio.setFrequency(frequency);
  radio.setTxPower(txPower);
  radio.setSpreadingFactor(spreadingFactor);
  // Adjust over-current protection
  radio.spiWrite(RH_RF95_REG_0B_OCP, 0x31);
  //radio.setSignalBandwidth(signalBandwidth);
  //radio.setCodingRate4(codingRate);
  //radio.setThisAddress(address);
}

// Returns the battery level in mV
uint16_t checkBattery() {
    float batteryLevel = (float)analogRead(BATTERY_LEVEL_PIN) * batteryScale;
    return batteryLevel * 1000.0;
}

void setup() {

  delay(2000);
  
  Serial.begin(115200);
  Serial.println(F("KC1FSZ LoRa Mesh System"));

  // LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Slow down ESP32 to 10 MHz in order to reduce battery consumption
  setCpuFrequencyMhz(10);

  Serial.print(F("Version "));
  Serial.println(VERSION);
  Serial.print(F("This is node "));
  Serial.println(MY_NODE_ADDR);
  Serial.print(F("Crystal frequency "));
  Serial.println(getXtalFrequencyMhz());
  Serial.print(F("Battery level: "));
  Serial.println(checkBattery());
  Serial.print(F("Low battery limit: "));
  Serial.println(lowBatteryLimitMv);

  preferences.begin("my-app", false); 

  uint16_t bootCount = preferences.getUShort("bootcount", 0);  
  Serial.print(F("Boot count: "));
  Serial.println(bootCount);
  preferences.putUShort("bootcount", bootCount+1);

  uint16_t sleepCount = preferences.getUShort("sleepcount", 0);  
  Serial.print(F("Sleep count: "));
  Serial.println(sleepCount);

  shell.attach(Serial); 
  shell.addCommand(F("hello"), helloWorld);
  shell.addCommand(F("ping"), sendPing);
  shell.addCommand(F("reset"), sendReset);
  
  // Reset the radio 
  pinMode(rst, OUTPUT);
  digitalWrite(rst, HIGH);
  digitalWrite(rst, LOW);
  digitalWrite(rst, HIGH);
  // Float the reset pin
  pinMode(rst, INPUT);
  // Per datasheet, wait 5ms
  delay(5);

  if (!mesh_manager.init()) {
    Serial.println("LoRa init failed");
  } else {
    configRadio(rf95);

    // Diagnostics
    Serial.println("LoRa Initializing OK!");
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
  }

  // Turn off WIFI and BlueTooth to reduce power 
  WiFi.mode(WIFI_OFF);
  btStop();

  // Enable the watchdog timer
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  esp_task_wdt_reset();
}

void loop() {

  // Check the battery
  uint16_t battery = checkBattery();
  // If the battery is low then deep sleep
  if (battery < lowBatteryLimitMv) {
    Serial.println("Low battery detected, going to sleep ...");
    // Keep track of how many times this has happened
    uint16_t sleepCount = preferences.getUShort("sleepcount", 0);  
    preferences.putUShort("sleepcount", sleepCount+1);   
    // Put the system into a deep sleep that will be awakened using the timer
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_SECONDS * US_TO_S_FACTOR);
    esp_deep_sleep_start();
  }

  // Service the shell
  shell.executeIfInput();
  
  // Wait to receive a message
  uint8_t rec_buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t rec_len = RH_RF95_MAX_MESSAGE_LEN;
  uint8_t rec_source = 0;
  // This is how long we sleep waiting for receive.  This will impact the 
  // interactiviy of the shell and anything else that is interactive in
  // the main loop.
  uint16_t timeout = 250;

  if (mesh_manager.recvfromAckTimeout(rec_buf, &rec_len, timeout, &rec_source)) {

    rec_count++;

    int16_t last_rssi = (int)rf95.lastRssi();
    
    // ---------------------------------------------------------------------------------------
    // Ping request
    //
    if (rec_buf[1] == 1) {

        // Measure uptime
        int32_t uptime_seconds = esp_timer_get_time() / 1000000L;
        uint16_t battery = checkBattery();
        
        // Build ping response
        uint8_t snd_buf[40];
        uint8_t snd_len = 40;
        
        snd_buf[0] = VERSION;
        snd_buf[1] = 2;
        // Call sign
        for (int i = 0; i < 8; i++)
          snd_buf[2 + i] = CALL[i];
        // Message receive count
        snd_buf[10] = (rec_count >> 8) & 0xff;
        snd_buf[11] = rec_count & 0xff;
        // RSSI of last received message
        snd_buf[12] = (last_rssi >> 8) & 0xff;
        snd_buf[13] = last_rssi & 0xff;
        // Battery reading in mv (16 bits)
        snd_buf[14] = (battery >> 8) & 0xff;
        snd_buf[15] = battery & 0xff;
        // Uptime 
        snd_buf[16] = (uptime_seconds >> 24) & 0xff;
        snd_buf[17] = (uptime_seconds >> 16) & 0xff;
        snd_buf[18] = (uptime_seconds >>  8) & 0xff;
        snd_buf[19] =  uptime_seconds        & 0xff;
        // Boot count
        uint16_t bootCount = preferences.getUShort("bootcount", 0);  
        snd_buf[20] = (bootCount >>  8) & 0xff;
        snd_buf[21] =  bootCount        & 0xff;
        // Sleep count
        uint16_t sleepCount = preferences.getUShort("sleepcount", 0);  
        snd_buf[22] = (sleepCount >>  8) & 0xff;
        snd_buf[23] =  sleepCount        & 0xff;

        // Copy the 16 byte payload of the ping request
        for (int i = 0; i < 16; i++) {
          snd_buf[24 + i] = rec_buf[10 + i];
        }
  
        uint8_t rc = mesh_manager.sendtoWait(snd_buf, snd_len, rec_source);
        if (rc == RH_ROUTER_ERROR_NONE) {
        } else if (rc == RH_ROUTER_ERROR_NO_ROUTE) {
          Serial.println("NR");
        } else if (rc == RH_ROUTER_ERROR_UNABLE_TO_DELIVER) {
          Serial.println("UTD");
        }    
    } 
    
    // ---------------------------------------------------------------------------------------
    // Ping Response
    //
    else if (rec_buf[1] == 2) {

      if (rec_len >= 40) {

        // TODO: CREATE A STRUCT
        int16_t counter = rec_buf[10] << 8;
        counter |= rec_buf[11];
        int16_t rssi = rec_buf[12] << 8;
        rssi |= rec_buf[13];
        uint16_t battery = rec_buf[14] << 8;
        battery |= rec_buf[15];
        uint32_t uptime_seconds = rec_buf[16] << 24;
        uptime_seconds |= rec_buf[17] << 16;
        uptime_seconds |= rec_buf[18] << 8;
        uptime_seconds |= rec_buf[19];
        uint16_t bootCount = rec_buf[20] << 8;
        bootCount |= rec_buf[21];
        uint16_t sleepCount = rec_buf[22] << 8;
        sleepCount |= rec_buf[23];

        Serial.print("{ \"count\": ");
        Serial.print(counter, DEC);
        Serial.print(", \"source\": ");
        Serial.print(rec_source, DEC);
        Serial.print(", \"lrsi\": ");
        Serial.print(last_rssi, DEC);
        Serial.print(", \"version\": ");
        Serial.print(rec_buf[0], DEC);
        Serial.print(", \"rrssi\": ");
        Serial.print(rssi, DEC);
        Serial.print(", \"battery\": ");
        Serial.print(battery, DEC);
        Serial.print(", \"uptime\": ");
        Serial.print(uptime_seconds, DEC);
        Serial.print(", \"boots\": ");
        Serial.print(bootCount, DEC);
        Serial.print(", \"sleeps\": ");
        Serial.print(sleepCount, DEC);
        Serial.print(", \"msg\": \"");
        Serial.print((const char*) & (rec_buf[24]));
        Serial.println("\" }");

        digitalWrite(21, HIGH);
        delay(300);
        digitalWrite(21, LOW);
      } else {
        Serial.println(F("Length error"));
        Serial.println(rec_len);
      }
    }
    
    // ---------------------------------------------------------------------------------------
    // Console Message request (No ACK)
    //
    else if (rec_buf[1] == 3) {

      char rec_call[9];
      for (int i = 0; i < 8; i++)
        rec_call[i] = rec_buf[2 + i];
      rec_call[8] = 0;

      uint16_t msg_num = rec_buf[10] << 8;
      msg_num |= rec_buf[11];

      char rec_msg[64];
      uint8_t rec_msg_len = rec_len - 12;
      if (rec_msg_len > 63) {
        // ERROR CONDITION
      } 
      else {
        // Pull the message body out 
        for (int i = 0; i < rec_msg_len; i++)
          rec_msg[i] = rec_buf[12 + i];
        rec_msg[rec_msg_len] = 0;
  
        Serial.println();
        Serial.print("Message from node: ");
        Serial.print(rec_source, DEC);
        Serial.print(", call: ");
        Serial.print(rec_call);
        Serial.print(", num: ");
        Serial.println(msg_num, DEC);
        Serial.print(">>> ");
        Serial.println(rec_msg);
      }
    }
    
    // ---------------------------------------------------------------------------------------
    // Reboot request)
    //
    else if (rec_buf[1] == 4) {
      shell.println("Asked to reboot ...");
      ESP.restart();
    }
    
    else {
        Serial.print("Unrecognized command ");
        Serial.println(rec_buf[1], DEC);
    }
  }

  // Keep the watchdog alive
  esp_task_wdt_reset();
}
