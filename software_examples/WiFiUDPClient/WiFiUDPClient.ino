#include <Arduino.h>

#include <WiFi.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>

#include <Wire.h>

/* SENSOR SPECIFIC INCLUDES */
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include "esp32s2/rom/rtc.h"  // for reset reason etc.
#include "esp32/clk.h"

/* Pins where UART pin header is routed */
#define RXD0              40
#define TXD0              39

#define DEBUG_PIN         40

/* Conversion factors for time related stuff */ 
#define SECONDS_PER_MIN   60
#define MIN_PER_HOUR      60
#define uS_TO_S_FACTOR    1000000
#define mS_TO_S_FACTOR    1000

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
#define custom_println(...)         if(use_debug_mode)                                            \
                                    {                                                             \
                                      snprintf(print_buffer, sizeof(print_buffer), __VA_ARGS__);  \
                                      customSerial.println(print_buffer);                         \
                                    }

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
RTC_DATA_ATTR uint32_t last_active;               // to measure entire cycle, can be outputted upon next connect
RTC_DATA_ATTR float last_delta_v;                 // to measure entire cycle, can be outputted upon next connect
volatile uint8_t curr_state = STATE_SENSOR_READ;  // default state, init only set if 
bool use_debug_mode = false;                      // flag for easily debugging code via prints :)

float supercap_voltage; // supercap voltage in V
RTC_DATA_ATTR uint32_t timeToSleepSeconds = MIN_SLEEP_TIME; // sleep time, stored in RTC RAM to also take past operation into account


/* ADD SENSOR SPECIFIC CODE HERE */
#define BME_WAIT_MS       10
#define I2C_SDA           15
#define I2C_SCL           16

RTC_DATA_ATTR Adafruit_BME280 bme; // I2C
float temp, humi, press;

/* Print buffer for easily formatting */
char print_buffer[256];

HardwareSerial customSerial(0);

/* Wifi callbacks */
void connectToWiFi(const char * ssid, const char * pwd);
void WiFiEvent(WiFiEvent_t event);


// wrapper enter deepsleep
static void enter_deepsleep(void)
{
  custom_println("%s", "Entering sleep");
  last_active = active_time;
  uint64_t sleepUs = timeToSleepSeconds*uS_TO_S_FACTOR - active_time*mS_TO_S_FACTOR;

  // take delay due to active time into account, to compensate for runtime
  esp_sleep_enable_timer_wakeup(sleepUs);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_deep_sleep_start();
}

static float measure_adc(void)
{
  float value;

  pinMode(PIN_R_DIV, OUTPUT); // use pin to briefly drive voltage divider
  digitalWrite(PIN_R_DIV, HIGH);

  analogSetAttenuation(ADC_11db);
  value = (analogReadMilliVolts(PIN_ADC)/1000.0)*2; // read voltage, is double that what we read

  digitalWrite(PIN_R_DIV, LOW); // reset output so no current is drawn anymore

  return value;
}

// determines next sleep period and stores it in RTC RAM
static void calc_next_sleep(void)
{
  supercap_voltage = measure_adc();

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
  else // we are already waiting with the maximum possible period
  {
    timeToSleepSeconds = MAX_SLEEP_TIME; // make sure to reset it to a proper value
  }
  custom_println("Chose sleep period: %ld", timeToSleepSeconds);
}

void setup()
{
  RESET_REASON reason = rtc_get_reset_reason(0); // check why we woke up
  pinMode(DEBUG_PIN, INPUT);
  use_debug_mode = digitalRead(DEBUG_PIN);

  // Initialize hardware serial
  customSerial.begin(115200, SERIAL_8N1, RXD0, TXD0);
  custom_println("SunsparkESP32 start, reset reason: %d", reason); // output reset reason

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
    bool state_changed = false;

    switch(curr_state)
    {
      case STATE_SENSOR_INIT:
        if (bme.init()) // success
        {
          curr_state = STATE_SENSOR_READ;
          state_changed = true;

          custom_println("BME280 found");
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
          state_changed = true;

          temp  = bme.readTemperature();
          press = bme.readPressure() / 100.0F;
          humi  = bme.readHumidity();

          custom_println("Sensor read done");
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
        state_changed = true;

        connectToWiFi(networkName, networkPswd);
        break;

      case STATE_WIFI_WAIT:
        vTaskDelay(WIFI_WAIT_MS);
        break;

      case STATE_SEND: //Send a packet
      {
        /* Approximate length in worst case:
         * 6 keys with quotation marks, colon         = 6 * 4   = 24
         * 5 comma separators                         = 5 * 1   =  5
         * 2 floats with 3 digits + point + 2 digits  = 2 * 6   = 12
         * 1 float with 2 digits + point + 2 digits   = 1 * 5   =  5
         * 1 float with 1 digits + point + 2 digits   = 1 * 4   =  4
         * 1 float with 4 digits + point + 0 digits   = 1 * 5   =  5
         * 2 integers with 10 digits max              = 2 * 10  = 20
         * terminator                                 = 1 * 1   =  1
         *
         * sum                                                  = 76
         * 
         * some padding :-) for furture use -> 256
         */
        char txBuffer[256];

        curr_state = STATE_WAIT_ACK;
        state_changed = true;

        // move data into buffer for sending and possible later evaluation
        snprintf(txBuffer, sizeof(txBuffer),
                "{\"T\":%2.2f,\"P\":%3.2f,\"H\":%3.2f,\"L\":%lu,\"B\":%1.2f,\"D\":%lu,\"V\":%4.0f}",
                temp,
                press,
                humi,
                last_active,
                supercap_voltage,
                timeToSleepSeconds,
                last_delta_v
        );

        /* hand over to network stack */
        Udp.begin(udpListenPort);
        Udp.beginPacket(udpAddress, udpSendPort);
        Udp.printf("%s",txBuffer);
        Udp.endPacket();

        /* Print message contents for simple debugging */
        custom_println("Sent: %s", txBuffer);
        break;
      }
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
          custom_println("Received contents: %s", packetBuffer);

          curr_state = STATE_SLEEP;
          state_changed = true;
        }
        else
        {
          vTaskDelay(WIFI_WAIT_MS);
        }
        break;

      default:
        curr_state = STATE_SLEEP;
        state_changed = true;
        break;
    }

    if (state_changed)
    {
      custom_println("Active time: %lu, CPU freq: %lu", active_time, esp_clk_cpu_freq());
    }

    if (curr_state == STATE_SLEEP)
    {
      break;
    }

  } while(active_time < MAX_WAKEUP_MS);

  /* Come here if either success or if timed out */
  last_delta_v = (supercap_voltage - measure_adc())*1000; // calculate how much voltage dropped during operation
  custom_println("mV consumed: %f", last_delta_v);

  /* Prepare to sleep again */
  enter_deepsleep();
}

void connectToWiFi(const char * ssid, const char * pwd){
  custom_println("Connecting to WiFi network: %s", ssid);

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
        custom_println("WiFi connected! IP address: %s", WiFi.localIP().toString().c_str());
        
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
