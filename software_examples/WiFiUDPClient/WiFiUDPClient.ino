#include <Arduino.h>

#include <WiFi.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>

#include <Wire.h>

/* SENSOR SPECIFIC INCLUDES */
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include "esp32s2/rom/rtc.h"

/* Pins where UART pin header is routed */
#define RXD0              40
#define TXD0              39

#define DEBUG_PIN         40

/* Conversion factors for time related stuff */ 
#define SECONDS_PER_MIN   60
#define MIN_PER_HOUR      60
#define uS_TO_S_FACTOR    1000000

/* Constants which influence wakup and sleep behaviour */
#define MIN_SLEEP_TIME    SECONDS_PER_MIN*10              // Time ESP32 will go to sleep (in seconds)
#define MAX_SLEEP_TIME    SECONDS_PER_MIN*MIN_PER_HOUR*8  // sleep third a day at max
#define DEBUG_SLEEP_TIME  SECONDS_PER_MIN                 // for quickly debugging
#define WIFI_WAIT_MS      1                               // task delay in polling loop
#define MAX_WAKEUP_MS     1000                            // total amount of milli seconds device operation can be done

/* Supercap related */
#define CAP_FULL_LVL      3.4                             // if this or higher is read, then supercap is definetly full
#define CAP_NORMAL_LVL    3.3                             // voltage threshold above which normal operation can take place
#define PIN_R_DIV         2                               // pin which powers voltage divider for ADC
#define PIN_ADC           1                               // where midpoint of divider is connected

// only need to use (and waste) ebergy when debugging
#define custom_println(s) if(use_debug_mode){customSerial.println(s);}

/* values for statemachine */
enum
{
  STATE_SENSOR_INIT,      // optional: in case sensor feature sleeping state as well, init can be skipped most of times
  STATE_SENSOR_READ,      // optional: do sensor readings before starting wifi transceiver!
  STATE_WIFI_INIT,        // setting up wifi
  STATE_WIFI_WAIT,        // waiting for intial connect
  STATE_SEND,             // transmitting data
  STATE_WAIT_ACK,         // optional: receive data back
  STATE_SLEEP,            // prepare sleeping
};

/* Network related constants */
const char * networkName    = "your_ssid"; // name of wifi
const char * networkPswd    = "your_password";          // password of wifi

// set these up in router, so that ít conflicts with nothing else
const char * device_ip      = "192.168.178.4";
const char * device_subnet  = "255.255.255.0";
const char * device_gateway = "192.168.178.1";

const char * udpAddress     = "192.168.178.33";               // IP address to send UDP data to
const int udpListenPort     = 1337;                           // on which port to listen
const int udpSendPort       = 1880;                           // to which port to send to

/* Network variables */
WiFiUDP Udp; //The Udp library class

/* State machine */
uint32_t millis_start, active_time;
volatile uint8_t curr_state = STATE_SENSOR_READ; // default state, init only set if 
bool use_debug_mode = false; // flag for easily debugging code via prints :)

float supercap_voltage; // supercap voltage in V
RTC_DATA_ATTR uint32_t timeToSleepSeconds = MIN_SLEEP_TIME; // sleep time, stored in RTC RAM to also take past operation into account


/* ADD SENSOR SPECIFIC CODE HERE */
#define BME_WAIT_MS       10
#define I2C_SDA           15
#define I2C_SCL           16

RTC_DATA_ATTR Adafruit_BME280 bme; // I2C
float temp, humi, press;


HardwareSerial customSerial(0);

/* Wifi callbacks */
void connectToWiFi(const char * ssid, const char * pwd);
void WiFiEvent(WiFiEvent_t event);



// wrapper enter deepsleep
static void enter_deepsleep(void)
{
  custom_println("Entering sleep");
  custom_println(active_time);

  esp_sleep_enable_timer_wakeup(timeToSleepSeconds*uS_TO_S_FACTOR);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_deep_sleep_start();
}

// determines next sleep period and stores it in RTC RAM
static void calc_next_sleep(void)
{
  pinMode(PIN_R_DIV, OUTPUT); // use pin to briefly drive voltage divider
  digitalWrite(PIN_R_DIV, HIGH);

  analogSetAttenuation(ADC_11db);
  supercap_voltage = (analogReadMilliVolts(PIN_ADC)/1000.0)*2; // read voltage, is double that what we read

  digitalWrite(PIN_R_DIV, LOW); // reset output so no current is drawn anymore

  /* Logic to determine next wakeup, depending on supercap charge */
  if (use_debug_mode)
  {
    timeToSleepSeconds = DEBUG_SLEEP_TIME;
  }
  else if (supercap_voltage >= CAP_FULL_LVL)
  {
    // cap is full, no matter what previously happend, reset sleep period to min
    timeToSleepSeconds = MIN_SLEEP_TIME;
  }
  else if (supercap_voltage >= CAP_NORMAL_LVL)
  {
    // cap charge is recovering, half the used period
    timeToSleepSeconds /= 2;
    if (timeToSleepSeconds < MIN_SLEEP_TIME) // watch out to not make it too small
    {
      timeToSleepSeconds = MIN_SLEEP_TIME;
    }
  }
  else if (timeToSleepSeconds < MAX_SLEEP_TIME)
  {
    // double sleep period to save more energy
    timeToSleepSeconds *= 2;
  }
  else
  {
    // we are already waiting with the maximum possible period
  }
  custom_println("Chose sleep period:");
  custom_println(timeToSleepSeconds);
}

void setup()
{
  RESET_REASON reason = rtc_get_reset_reason(0); // check why we woke up
  pinMode(DEBUG_PIN, INPUT);
  use_debug_mode = digitalRead(DEBUG_PIN);

  // Initialize hardware serial
  customSerial.begin(115200, SERIAL_8N1, RXD0, TXD0);
  custom_println("SunsparkESP32 start, reset reason:");

  custom_println(reason); // output reset reason
  switch (reason) // depending on 
  {
    case RTCWDT_BROWN_OUT_RESET: // when brownout was active: don't continue, save energy
      enter_deepsleep();
      break;

    case POWERON_RESET:
      curr_state = STATE_SENSOR_INIT;

    default: // will most ofen land here, rtc wakeup 
      break;
  }

  calc_next_sleep();
  millis_start = millis(); // remember start of statemachine

  /* DO SENSOR SPECIFIC STUFF HERE */
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  bme.begin(0x76);
}

void loop()
{
  do
  {
    active_time = millis() - millis_start;       // calculate wifi active time

    switch(curr_state)
    {
      case STATE_SENSOR_INIT:
        if (bme.init()) // success
        {
          curr_state = STATE_SENSOR_READ;

          custom_println("BME280 found");
          custom_println(active_time);
          break;
        }
        else
        {
          custom_println("Retrying BME init...");
          vTaskDelay(BME_WAIT_MS);
        }
        break;

      case STATE_SENSOR_READ:
        bme.setSampling(Adafruit_BME280::MODE_FORCED,
              Adafruit_BME280::SAMPLING_X1, // temperature
              Adafruit_BME280::SAMPLING_X1, // pressure
              Adafruit_BME280::SAMPLING_X1, // humidity
              Adafruit_BME280::FILTER_OFF);

        if (bme.takeForcedMeasurement())
        {
          curr_state = STATE_WIFI_INIT;

          temp  = bme.readTemperature();
          press = bme.readPressure() / 100.0F;
          humi  = bme.readHumidity();

          custom_println("Sensor read done");
          custom_println(active_time);
          break;
        }
        else
        {
          custom_println("Retrying measurement...");
          vTaskDelay(BME_WAIT_MS);
        }
        break;

      case STATE_WIFI_INIT: //Connect to the WiFi network
        curr_state = STATE_WIFI_WAIT;
        connectToWiFi(networkName, networkPswd);
        break;

      case STATE_WIFI_WAIT:
        vTaskDelay(WIFI_WAIT_MS);
        break;

      case STATE_SEND: //Send a packet
        curr_state = STATE_WAIT_ACK;

        Udp.begin(udpListenPort);
        Udp.beginPacket(udpAddress, udpSendPort);
        Udp.printf("{\"T\":%.2f,\"P\":%.2f,\"H\":%.2f,\"L\":%lu,\"B\":%.2f,\"D\":%lu}",
                    temp,
                    press,
                    humi,
                    active_time,
                    supercap_voltage,
                    timeToSleepSeconds);
        Udp.endPacket();

        custom_println("Transmitted");
        custom_println(active_time);
        break;

      case STATE_WAIT_ACK:
        if (Udp.parsePacket()) // received something
        {
          char packetBuffer[32]; // use some task stack to buffer message
          int len = Udp.read(packetBuffer, sizeof(packetBuffer));

          if (len > 0) // set null terminator
          {
            packetBuffer[len] = 0;
          }

          /* Parse received contents of server here, if desired */
          custom_println("Received");
          custom_println(active_time);
          custom_println("Contents:");
          custom_println(packetBuffer);

          curr_state = STATE_SLEEP;
        }
        else
        {
          vTaskDelay(WIFI_WAIT_MS);
        }
        break;

      default:
        curr_state = STATE_SLEEP;
        break;
    }

    if (curr_state == STATE_SLEEP)
    {
      break;
    }

  } while(active_time < MAX_WAKEUP_MS);

  /* Come here if either success or if timed out */
  enter_deepsleep();
}

void connectToWiFi(const char * ssid, const char * pwd){
  custom_println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);

  // setup static config
  IPAddress localIP, gateway, subnet;
  localIP.fromString(device_ip);
  gateway.fromString(device_gateway);
  subnet.fromString(device_subnet);

  WiFi.config(localIP, gateway, subnet);
  
  //Initiate connection
  WiFi.begin(ssid, pwd);

  custom_println("Waiting for WIFI connection...");
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event)
    {
      case ARDUINO_EVENT_WIFI_STA_GOT_IP: //When connected set           
        custom_println("WiFi connected! IP address: ");
        custom_println(WiFi.localIP());  
        //initializes the UDP state -> This initializes the transfer buffer
        Udp.begin(WiFi.localIP(),udpListenPort);
        curr_state = STATE_SEND;
        break;

      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        custom_println("WiFi lost connection");
        curr_state = STATE_WIFI_WAIT;
        break;
      default:
        break;
    }
}
