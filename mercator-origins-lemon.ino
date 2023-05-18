// https://www.hackster.io/pradeeplogu0/real-time-gps-monitoring-with-qubitro-and-m5stickc-a2bc7c
// https://github.com/mikalhart/TinyGPSPlus/blob/master/README.md
// http://arduiniana.org/libraries/tinygpsplus/

//possible fix to deepSleep with timer #31 - https://github.com/m5stack/M5StickC-Plus/pull/31
//Sleep causing unresponsive device #13 https://github.com/m5stack/M5StickC-Plus/issues/13
//AXP192.cpp SetSleep() is different than the one for M5StickC #1 https://github.com/m5stack/M5StickC-Plus/issues/1

// compilation switches

//#define ENABLE_TWITTER_AT_COMPILE_TIME
//#define ENABLE_SMTP_AT_COMPILE_TIME
#define ENABLE_QUBITRO_AT_COMPILE_TIME
#define ENABLE_ELEGANT_OTA_AT_COMPILE_TIME

#include <M5StickCPlus.h>

+// rename the git file "mercator_secrets_template.c" to the filename below, filling in your wifi credentials etc.
#include "mercator_secrets.c"

#include <TinyGPS++.h>

#define SCREEN_LENGTH 240
#define SCREEN_WIDTH 135

#define GPS_BAUD_RATE 9600
#define UPLINK_BAUD_RATE 9600

#define USB_SERIAL Serial
#define GOPRO_SERIAL Serial1

const bool enableIMUSensor=true;
const bool enableReadUplinkComms=true;
const bool enableAllUplinkMessageIntegrityChecks = false;
const bool enableOTAServer = true;          // over the air updates
const bool enableConnectToQubitro = true;
const bool enableUploadToQubitro=true;
const bool enableConnectToTwitter = false;
const bool enableConnectToSMTP = false;

uint32_t consoleDownlinkMsgCount = 0;
const uint32_t consoleDownlinkMsgDutyCycle = 4;

#ifdef ENABLE_TWITTER_AT_COMPILE_TIME
  // see mercator_secrets.c for Twitter login credentials
  #include <WiFiClientSecure.h>   // Twitter
  #include <TweESP32.h>          // Install from Github - https://github.com/witnessmenow/TweESP32
  #include <TwitterServerCert.h> // included with above
  #include <UrlEncode.h> //Install from library manager
  #include <ArduinoJson.h> //Install from library manager
  bool connectToTwitter=false;
  WiFiClientSecure secureTwitterClient;
  TweESP32 twitter(secureTwitterClient, twitterConsumerKey, twitterConsumerSecret, twitterAccessToken, twitterAccessTokenSecret, twitterBearerToken);
  uint8_t countdownToSendStartupLocationTweet = 20;   // wait 10 seconds or 20 fix messages to send location tweet
  char tweet[512];
#endif

#ifdef ENABLE_SMTP_AT_COMPILE_TIME
  // see mercator_secrets.c for SMTP login credentials
  #include <ESP_Mail_Client.h>
  SMTPSession smtp;
#endif

#ifdef ENABLE_QUBITRO_AT_COMPILE_TIME
  // see mercator_secrets.c for Qubitro login credentials
  #include <WiFi.h>
  #include <QubitroMqttClient.h>
  float qubitro_upload_duty_ms = 0; // disable throttling to qubitro
  uint32_t last_qubitro_upload = 0;
  char uplinkMessage[4096];
  char qubitro_payload[8196];
  WiFiClient wifiClient;
  QubitroMqttClient qubitro_mqttClient(wifiClient);
  bool qubitro_connect();
#endif

#ifdef ENABLE_ELEGANT_OTA_AT_COMPILE_TIME
  // see mercator_secrets.c for wifi login globals
  #include <WiFi.h>
  #include <AsyncTCP.h>
  #include <ESPAsyncWebServer.h>
  #include <AsyncElegantOTA.h>
  bool otaActiveListening=true;   // OTA updates toggle
  AsyncWebServer asyncWebServer(80);
#endif

bool imuAvailable=true;

const char* leakAlarmMsg = "    Float\n\n    Leak!";

uint32_t fixCount=0;
uint32_t passedChecksumCount=0;
bool newFixReceived=false;

uint8_t journey_activity_count=0;
char journey_activity_indicator[]="\\|/-";

uint32_t currentQubitroUploadAt=0,lastQubitroUploadAt=0;
uint32_t qubitroUploadDutyCycle=0;


#define RED_LED_GPIO 10
#define ORANGE_LED_GPIO 0
#define IR_LED_GPIO 9

const bool writeLogToSerial=false;

char uplinkTestMessages[][7]={"MSG0! ","MSG1@ ","MSG2@ ","MSG3% "};

TinyGPSPlus gps;
int uart_number_gps = 2;
HardwareSerial gps_serial(uart_number_gps);

int uart_number_gopro = 1;
HardwareSerial ss_to_gopro(uart_number_gopro); 

double Lat, Lng;
String  lat_str , lng_str;
uint32_t satellites=0;
//double b, c = 0;
int nofix_byte_loop_count = 0;

template <typename T> struct vector
{
  T x, y, z;
};

double heading_to_target=0, distance_to_target=0;
double journey_lat = 0,journey_lng = 0, journey_course = 0, journey_distance = 0;
float magnetic_heading = 0,magnetic_heading_compensated=-0;

uint16_t mako_screen_display=0,mako_seconds_on=0, mako_user_action=0;
float mako_AXP192_temp=0, mako_usb_voltage=0,mako_usb_current=0,mako_bat_voltage=0,mako_bat_charge_current=0;
float mako_lsm_mag_x=0,mako_lsm_mag_y=0,mako_lsm_mag_z=0, mako_lsm_acc_x=0,mako_lsm_acc_y=0,mako_lsm_acc_z=0;

float mako_imu_gyro_x=0,mako_imu_gyro_y=0,mako_imu_gyro_z=0, mako_imu_lin_acc_x=0,mako_imu_lin_acc_y=0,mako_imu_lin_acc_z=0, mako_imu_rot_acc_x=0,mako_imu_rot_acc_y=0,mako_imu_rot_acc_z=0;
float mako_imu_temperature=0;

float lemon_imu_gyro_x=0,lemon_imu_gyro_y=0,lemon_imu_gyro_z=0, lemon_imu_lin_acc_x=0,lemon_imu_lin_acc_y=0,lemon_imu_lin_acc_z=0, lemon_imu_rot_acc_x=0,lemon_imu_rot_acc_y=0,lemon_imu_rot_acc_z=0;
float lemon_imu_temperature=0;

void getM5ImuSensorData(float* gyro_x, float* gyro_y, float* gyro_z,
                        float* lin_acc_x, float* lin_acc_y, float* lin_acc_z,
                        float* rot_acc_x, float* rot_acc_y, float* rot_acc_z,
                        float* IMU_temperature)
{
  if (enableIMUSensor)
  {
    M5.IMU.getGyroData(gyro_x, gyro_y, gyro_z);
    M5.IMU.getAccelData(lin_acc_x, lin_acc_y, lin_acc_z);
    M5.IMU.getAhrsData(rot_acc_x, rot_acc_y, rot_acc_z);
    M5.IMU.getTempData(IMU_temperature);
  }
  else
  {
    *gyro_x=*gyro_y=*gyro_z=*lin_acc_x=*lin_acc_y=*lin_acc_z=*rot_acc_x=*rot_acc_y=*rot_acc_z=*IMU_temperature=0.1;
  }
}

const char* preamble_pattern="MBJAEJ";


uint16_t mako_way_marker_enum;
char mako_way_marker_label[3];
char mako_direction_metric[2];

uint16_t sideCount=0,topCount=0;
vector<float> magnetometer_vector, accelerometer_vector;

float depth=0,water_pressure=0,water_temperature=0,enclosure_temperature=0,enclosure_humidity=0,enclosure_air_pressure=0;
uint16_t console_flags=0;

bool console_requests_send_tweet = false;
bool console_requests_emergency_tweet = false;

uint32_t goodUplinkMessageCount=0;
uint32_t badUplinkMessageCount=0;
uint32_t lastGoodUplinkMessage=0;
uint16_t uplinkMessageLength=0;
uint32_t qubitroUploadCount=0;
uint16_t qubitroMessageLength=0;
float KB_to_Qubitro=0.0;
float KB_from_mako=0.0;

#define GROVE_GPS_RX_PIN 33
#define GROVE_GPS_TX_PIN 32

#define HAT_GPS_TX_PIN 26
#define HAT_GPS_RX_PIN 36

#define M5_POWER_SWITCH_PIN 255

Button* p_primaryButton = NULL;
Button* p_secondButton = NULL;
void updateButtonsAndBuzzer();


const float minimumUSBVoltage=2.0;
long USBVoltageDropTime=0;
long milliSecondsToWaitForShutDown=3000;


void shutdownIfUSBPowerOff();
void toggleOTAActive();
void toggleWiFiActive();

void checkForLeak(const char* msg, const uint8_t pin);

void sendFakeGPSData1();
void sendFakeGPSData2();

bool setupOTAWebServer(const char* _ssid, const char* _password, const char* label, uint32_t timeout);
bool connectWiFiNoOTA(const char* _ssid, const char* _password, const char* label, uint32_t timeout);

void updateButtonsAndBuzzer()
{
  p_primaryButton->read();
  p_secondButton->read();
  M5.Beep.update();
}

void setup()
{
  M5.begin();

  if (enableIMUSensor)
  {
    M5.Imu.Init();
  }
  else
  {
    USB_SERIAL.println("IMU Sensor Off");
    M5.Lcd.println("IMU Sensor Off");
    imuAvailable=false;        
  }
  
  pinMode(RED_LED_GPIO, OUTPUT); // Red LED
  digitalWrite(RED_LED_GPIO, HIGH); // switch off

  pinMode(ORANGE_LED_GPIO, OUTPUT); // Red LED
  digitalWrite(ORANGE_LED_GPIO, LOW); // switch off

  pinMode(IR_LED_GPIO, OUTPUT); // Red LED
  digitalWrite(IR_LED_GPIO, HIGH); // switch off

  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(2);
  M5.Axp.ScreenBreath(14);

  p_primaryButton = &M5.BtnA;
  p_secondButton = &M5.BtnB;

  // if there is an infinite restart loop after connecting to wifi this will allow
  // for shutdown once the power is pulled. Otherwise it will continue on the battery power.
  if (enableOTAServer)
  {
    shutdownIfUSBPowerOff();
    if (!setupOTAWebServer(ssid_1, password_1, label_1, timeout_1))
    {
      shutdownIfUSBPowerOff();
      if (!setupOTAWebServer(ssid_2, password_2, label_2, timeout_2))
      {
        shutdownIfUSBPowerOff();
        setupOTAWebServer(ssid_3, password_3, label_3, timeout_3);
      }
    }
  }
    
  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html
//  uart_set_mode(uart_number, UART_MODE_RS485_HALF_DUPLEX);

  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.setCursor(0, 0);

  gps_serial.begin(GPS_BAUD_RATE, SERIAL_8N1, GROVE_GPS_RX_PIN, GROVE_GPS_TX_PIN);   // pin 33=rx (white M5), pin 32=tx (yellow M5), specifies the grove SCL/SDA pins for Rx/Tx
  
  // setup second serial port for sending/receiving data to/from GoPro
  GOPRO_SERIAL.begin(UPLINK_BAUD_RATE, SERIAL_8N1, HAT_GPS_RX_PIN, HAT_GPS_TX_PIN);
  GOPRO_SERIAL.setRxBufferSize(256);

        // cannot use Pin 0 for receive of GPS (resets on startup), can use Pin 36, can use 26
        // cannot use Pin 0 for transmit of GPS (resets on startup), only Pin 26 can be used for transmit.

        /*
         * Hi, I would like to use the GPIO 25 and 26 pins in the header for I2C, so I am setting them up as follows in MicroPython:

from machine import Pin, I2C
i2c = I2C(1, scl=Pin(26), sda=Pin(25), freq=400000)

However, the description says that "G36/G25 share the same port, when one of the pins is used, the other pin should be set as a floating input" and 
provides the following example
]
For example, to use the G36 pin as the ADC input, Configuration the G25 pin as FLOATING
setup()
{
M5.begin();
pinMode(36, INPUT);
gpio_pulldown_dis(GPIO_NUM_25);
gpio_pullup_dis(GPIO_NUM_25);
}

It seems that I should set GPIO36 to Floating.
  */
  // see https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/HardwareUSB_SERIAL.h
  // see https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/HardwareUSB_SERIAL.cpp


    // https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/

/*
    timerSemaphore = xSemaphoreCreateBinary();
    timer = timerBegin(0, 80, true);      // pre-scaler of 80, dividing 80MHz by 80 to trigger every 1uS
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 50000, true);  // interupt generated every 50,000 uS = 5s
    timerAlarmEnable(timer);
*/
//    SendCASICNavXQuery();
//    waitForCASICNavXResponse();
//    waitForCASICACKResponse();


#ifdef ENABLE_TWITTER_AT_COMPILE_TIME
//  sendOriginsTweet("Startup Test Tweet From Mercator Origins - Bluepad Labs. The Scuba Hacker");
#endif

#ifdef ENABLE_SMTP_AT_COMPILE_TIME
  if (enableConnectToSMTP && WiFi.status() == WL_CONNECTED)
  {
   sendTestByEmail();
  }
#endif

#ifdef ENABLE_QUBITRO_AT_COMPILE_TIME
  if (enableConnectToQubitro && WiFi.status() == WL_CONNECTED)
  {
     qubitro_connect();
  }
#endif
}
    
bool sendOriginsTweet(char* tweet)
{
  bool success=false;
#ifdef ENABLE_TWITTER_AT_COMPILE_TIME
  if (enableConnectToTwitter && WiFi.status() == WL_CONNECTED)
  {
    //Required for Oauth for sending tweets
    twitter.timeConfig();
    // Checking the cert is the best way on an ESP32i
    // This will verify the server is trusted.
    secureTwitterClient.setCACert(twitter_server_cert);

    success = twitter.sendTweet(tweet);

    if (success)
      USB_SERIAL.printf("Twitter send tweet successful: %s",tweet);
    else
      USB_SERIAL.printf("Twitter send tweet failed: %s",tweet);
  }
  #endif
  return success;
}

void loop() 
{  
  shutdownIfUSBPowerOff();

  if (p_primaryButton->pressedFor(100)) // toggle ota only
  {
    toggleOTAActive();
  }

  if (p_secondButton->pressedFor(100)) // toggle wifi only
  {
    toggleWiFiActive();
  }

  while (gps_serial.available() > 0)
  {      
    checkForLeak(leakAlarmMsg,M5_POWER_SWITCH_PIN);

    char nextByte=gps_serial.read();

    // these serial write outputs all bytes received
//    if (writeLogToSerial)
  //    USB_SERIAL.write(nextByte);

    if (gps.encode(nextByte))
    {
      if (gps.location.isValid() && gps.location.isUpdated())
      {
        if (writeLogToSerial)
        {
            // only writes GNGGA and GNRMC messages          
            USB_SERIAL.printf("Received from GPS: %s",gps.getSentence());
        }
                   
        // only enter here on GPRMC and GPGGA msgs with M5 GPS unit, 0.5 sec between each message.
        // 1 second between updates on the same message type.

        //////////////////////////////////////////////////////////
        // send message to outgoing serial connection to gopro
//        if (consoleDownlinkMsgCount % consoleDownlinkMsgDutyCycle == 0)
        {
          GOPRO_SERIAL.write(gps.getSentence());
          consoleDownlinkMsgCount++;
          if (writeLogToSerial)
          {
            USB_SERIAL.printf("\nSend to Mako: %s",gps.getSentence());
          }
        }
        //////////////////////////////////////////////////////////

         getM5ImuSensorData(&lemon_imu_gyro_x, &lemon_imu_gyro_y, &lemon_imu_gyro_z, 
                            &lemon_imu_lin_acc_x,&lemon_imu_lin_acc_y,&lemon_imu_lin_acc_z,
                            &lemon_imu_rot_acc_x,&lemon_imu_rot_acc_y,&lemon_imu_rot_acc_z,
                            &lemon_imu_temperature);

        newFixReceived=true;
        
        uint32_t newFixCount = gps.sentencesWithFix();
        uint32_t newPassedChecksum = gps.passedChecksum();
        if (newFixCount > fixCount)
        {
          fixCount=newFixCount;
          if (writeLogToSerial)
          {
            digitalWrite(RED_LED_GPIO, fixCount%2);
            digitalWrite(ORANGE_LED_GPIO, fixCount%2); // switch off

           USB_SERIAL.printf("\nFix: %lu Good Msg: %lu Bad Msg: %lu",fixCount,newPassedChecksum,gps.failedChecksum());
          }
        }

        if (nofix_byte_loop_count > -1)
        {
          // clear the onscreen counter that increments whilst attempting to get first valid location
          nofix_byte_loop_count=-1;
          M5.Lcd.fillScreen(TFT_BLACK);
        } 

        updateButtonsAndBuzzer();

        if (newPassedChecksum <= passedChecksumCount)
        {
          // incomplete message received, continue reading bytes, don't update display.
          return;
        }
        else
        {
          passedChecksumCount=newPassedChecksum;
        }

        M5.Lcd.setCursor(0, 0);
        
        Lat = gps.location.lat();
        lat_str = String(Lat , 7);
        Lng = gps.location.lng();
        lng_str = String(Lng , 7);
        satellites = gps.satellites.value();

#ifdef ENABLE_QUBITRO_AT_COMPILE_TIME
        if (enableConnectToQubitro && enableUploadToQubitro)
          uploadTelemetryToQubitro();
#endif

#ifdef ENABLE_TWITTER_AT_COMPILE_TIME
        
        if (countdownToSendStartupLocationTweet)
        {
          countdownToSendStartupLocationTweet--;
          if (countdownToSendStartupLocationTweet == 0)
          {
            // send first location fix automatically by twitter
            buildTwitterTelemetryTweet(tweet,false);  // not SOS
            sendOriginsTweet(tweet);
          }
        }
        
        if (console_requests_send_tweet)
        {
          if (console_requests_emergency_tweet)
          {
            console_requests_emergency_tweet = false;
          }

          console_requests_send_tweet = false;
          buildTwitterTelemetryTweet(tweet,true); // this is an SOS
          sendOriginsTweet(tweet); 
        }
#endif

      }
      else
      {
        // get location invalid if there is no new fix to read before 1 second is up.
        if (nofix_byte_loop_count > -1)
        {
          // Bytes are being received but no valid location fix has been seen since startup
          // Increment byte count shown until first fix received.
          M5.Lcd.setCursor(50, 90);
          M5.Lcd.printf("%d",nofix_byte_loop_count++);
        }
      }
    }
    else
    {
      // no byte received.
    }
  }
    
  if (nofix_byte_loop_count > 0)
  {
    // No fix only shown on first acquisition. 
    M5.Lcd.setCursor(55, 5);
    M5.Lcd.setTextSize(4);
    M5.Lcd.printf("No Fix %lu %lu\n  Lemon\n",goodUplinkMessageCount, badUplinkMessageCount);
//    M5.Lcd.printf("No Fix\n");
    M5.Lcd.setCursor(110, 45);
    M5.Lcd.printf("%c",journey_activity_indicator[(++journey_activity_count)%4]);

    // tells gopro M5 that gps is alive but no fix yet.
    // gopro M5 can choose to show this data for test purposes, otherwise in 
    // swimming pool like new malden or putney there may be no gps signal so
    // won't be able to test the rest, eg compass, temperature, humidity, buttons, reed switches
    // note the leak sensor is active at all times in the gopro M5.
    sendFakeGPSData1();
  
    delay(250); // no fix wait
  }
  else if (nofix_byte_loop_count != -1)
  {
    // No GPS is reported when no bytes have ever been received on the UART.
    // Once messages start being received, this is blocked as it is normal
    // to have gaps in the stream. There is no indication if GPS stream hangs
    // after first byte received, eg no bytes within 10 seconds.
    M5.Lcd.setCursor(55, 5);
    M5.Lcd.setTextSize(4);
    M5.Lcd.printf("No GPS %lu %lu\n  Lemon\n",goodUplinkMessageCount, badUplinkMessageCount);
//    M5.Lcd.printf("No GPS\n");
    M5.Lcd.setCursor(110, 45);
    M5.Lcd.printf("%c",journey_activity_indicator[(++journey_activity_count)%4]);
    
    // tells gopro M5 that gps is alive but no fix yet.
    // gopro M5 can choose to show this data for test purposes, otherwise in 
    // swimming pool like new malden or putney there may be no gps signal so
    // won't be able to test the rest, eg compass, temperature, humidity, buttons, reed switches
    // note the leak sensor is active at all times in the gopro M5.
    sendFakeGPSData2(); 

    delay(250); // no fix wait
  }
  else
  {
    if (newFixReceived)
    {
      M5.Lcd.setCursor(5, 5);
      M5.Lcd.setTextColor(TFT_RED,TFT_BLACK);
      M5.Lcd.setTextSize(3);
      M5.Lcd.printf("Fix %lu\nUpOk %lu\nUpBad %lu\nQOk %lu\n",fixCount,goodUplinkMessageCount, badUplinkMessageCount,qubitroUploadCount);
      M5.Lcd.setTextSize(2);
      M5.Lcd.printf("IP: %s",(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "No WiFi"));

      if (enableReadUplinkComms)
      {
        M5.Lcd.setTextSize(2);
        GOPRO_SERIAL.flush(); // wait until all output data sent

        // search for lead-in pattern from Tracker - MBJAEJ
        const char* nextByteToFind=preamble_pattern;
        
        while(GOPRO_SERIAL.available() && *nextByteToFind != 0)
        {
          char next = GOPRO_SERIAL.read();          // throw away trash bytes from half-duplex clash
          if (next == *nextByteToFind)
            nextByteToFind++;
        }
        
       if (*nextByteToFind == 0)
       {
          // message pre-amble found - read the rest of the received message.
          if (writeLogToSerial)
            USB_SERIAL.print("Pre-Amble Found\n");

          char* nextByte=uplinkMessage;

          while(true)
          {
            // must only listen for data when not sending gps data.
            // after send of gps must flush rx buffer
            if (GOPRO_SERIAL.available())
            {
              *(nextByte++)=GOPRO_SERIAL.read();
            }
            else
            {
              break;
            }
          }

          if (writeLogToSerial)
            USB_SERIAL.printf("Calling decodeUplink %d bytes\n",nextByte-uplinkMessage);

          const int maxLengthOfTestUplinkMessage=20;

          if (nextByte-uplinkMessage <= maxLengthOfTestUplinkMessage)
          {
              // This is a test message - just output to the display - no decode
              M5.Lcd.setTextSize(2);
              M5.Lcd.setCursor(0,100);
              M5.Lcd.printf("Uplink Test Msgs:");
              M5.Lcd.setCursor(0,116);
              M5.Lcd.printf("                        ");
              M5.Lcd.setCursor(0,116);
             
              for (int i=0; i<nextByte-uplinkMessage; i++)
                M5.Lcd.printf("%c",uplinkMessage[i]);
                
              M5.Lcd.setTextSize(2);
          }
          else
          {
            decodeUplinkMessageV5(uplinkMessage,(nextByte-uplinkMessage));
          }
        }
      }
      else
      {
        M5.Lcd.setCursor(110, 70);
        M5.Lcd.printf("%c",journey_activity_indicator[(++journey_activity_count)%4]);
        M5.Lcd.setCursor(25, 100);
        M5.Lcd.setTextSize(3);
        M5.Lcd.printf(" {%lu}",fixCount);
      }

      M5.Lcd.setTextSize(4);
      
      newFixReceived=false;
    }
  }

  checkForLeak(leakAlarmMsg,M5_POWER_SWITCH_PIN);
}

bool decodeUplinkMessageV1(char* uplinkMsg,const uint16_t length)
{
  bool result=false;
  
  uint16_t uplink_length = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);  
  uint16_t uplink_msgtype = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_depth = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_water_pressure = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_water_temperature = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_enclosure_temperature = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_enclosure_humidity = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_enclosure_air_pressure = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_compensated_magnetic_heading = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_flags = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_checksum = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);

  // must have msg length of 22, type 0 and a correct checksum
  if (!(uplink_length == 22 && 
      uplink_msgtype == 0/* && 
      uplink_checksum == calcUplinkChecksum(uplinkMsg,length-2)*/))
  {
  //  USB_SERIAL.printf("decodeUplink bad msg: %d msg type, checksum calc bad\n",uplink_msgtype);
    badUplinkMessageCount++;
    return result;
  }

//  USB_SERIAL.printf("decodeUplink good msg: %d msg type\n",uplink_msgtype);

  // all ok, assign the data to globals

  depth = ((float)uplink_depth) / 10.0;
  water_pressure = (float)uplink_water_pressure / 10.0;
  water_temperature = (float)uplink_water_temperature / 10.0;
  enclosure_temperature = (float)uplink_enclosure_temperature / 10.0;
  enclosure_humidity = (float)uplink_enclosure_humidity / 10.0;
  enclosure_air_pressure = (float)uplink_enclosure_air_pressure / 10.0;
  magnetic_heading_compensated = (float)uplink_compensated_magnetic_heading / 10.0;
  console_requests_send_tweet = uplink_flags & 0x01;
  console_requests_emergency_tweet = uplink_flags & 0x02;
  console_flags = uplink_flags;

  goodUplinkMessageCount++;
  lastGoodUplinkMessage=millis();

//  dumpUplinkMessageToSerial();

  return result;
}

bool decodeUplinkMessageV2(char* uplinkMsg,const uint16_t length)
{
  bool result=false;
  
  uint16_t uplink_length = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);  
  uint16_t uplink_msgtype = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_depth = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_water_pressure = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_water_temperature = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_enclosure_temperature = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_enclosure_humidity = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_enclosure_air_pressure = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_compensated_magnetic_heading = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);

  uint16_t uplink_heading_to_target = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_distance_to_target = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_journey_course = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_journey_distance = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_screen_display = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_seconds_on = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_user_action = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_AXP192_temp = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_usb_voltage = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_usb_current = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_bat_voltage = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_bat_charge_current = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  
  
  uint16_t uplink_flags = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_checksum = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);

  // must have msg length of 46, type 0 and a correct checksum
  if (!(uplink_length == 46 && 
      uplink_msgtype == 0/* && 
      uplink_checksum == calcUplinkChecksum(uplinkMsg,length-2)*/))
  {
  //  USB_SERIAL.printf("decodeUplink bad msg: %d msg type, checksum calc bad\n",uplink_msgtype);
    badUplinkMessageCount++;
    return result;
  }

//  USB_SERIAL.printf("decodeUplink good msg: %d msg type\n",uplink_msgtype);

  // all ok, assign the data to globals

  depth = (float)uplink_depth / 10.0;
  water_pressure = (float)uplink_water_pressure / 10.0;
  water_temperature = (float)uplink_water_temperature / 10.0;
  enclosure_temperature = (float)uplink_enclosure_temperature / 10.0;
  enclosure_humidity = (float)uplink_enclosure_humidity / 10.0;
  enclosure_air_pressure = (float)uplink_enclosure_air_pressure / 10.0;
  magnetic_heading_compensated = (float)uplink_compensated_magnetic_heading / 10.0;

  heading_to_target = (float)uplink_heading_to_target / 10.0;
  distance_to_target = (float)uplink_distance_to_target / 10.0;
  journey_course = (float)uplink_journey_course / 10.0;
  journey_distance = (float)uplink_journey_distance / 100.0;

  mako_screen_display = 0xFFFF; // MBJ LATER
  mako_seconds_on = uplink_mako_seconds_on;
  mako_user_action = 0xFFFF;  // MBJ LATER
  mako_AXP192_temp = (float)uplink_mako_AXP192_temp / 10.0;
  mako_usb_voltage = (float)uplink_mako_usb_voltage / 1000.0;
  mako_usb_current =(float)uplink_mako_usb_current / 100.0;
  mako_bat_voltage = (float)uplink_mako_bat_voltage / 1000.0;
  mako_bat_charge_current =  (float)uplink_mako_bat_charge_current / 100.0;
    
  console_requests_send_tweet = uplink_flags & 0x01;
  console_requests_emergency_tweet = uplink_flags & 0x02;
  console_flags = uplink_flags;

  goodUplinkMessageCount++;
  lastGoodUplinkMessage=millis();

//  dumpUplinkMessageToSerial();

  return result;
}


bool decodeUplinkMessageV3(char* uplinkMsg,const uint16_t length)
{
  bool result=false;
  
  uint16_t uplink_length = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);  
  uint16_t uplink_msgtype = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_depth = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_water_pressure = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_water_temperature = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_enclosure_temperature = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_enclosure_humidity = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_enclosure_air_pressure = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_compensated_magnetic_heading = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);

  uint16_t uplink_heading_to_target = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_distance_to_target = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_journey_course = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_journey_distance = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_screen_display = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_seconds_on = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_user_action = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_AXP192_temp = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_usb_voltage = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_usb_current = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_bat_voltage = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_bat_charge_current = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);

  float uplink_mako_lsm_mag_x=0.0; char* p=(char*)&uplink_mako_lsm_mag_x;
  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);

  float uplink_mako_lsm_mag_y=0.0; p=(char*)&uplink_mako_lsm_mag_y;
  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);

  float uplink_mako_lsm_mag_z=0.0; p=(char*)&uplink_mako_lsm_mag_z;
  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);

  float uplink_mako_lsm_acc_x=0.0; p=(char*)&uplink_mako_lsm_acc_x;
  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);

  float uplink_mako_lsm_acc_y=0.0; p=(char*)&uplink_mako_lsm_acc_y;
  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);

  float uplink_mako_lsm_acc_z=0.0; p=(char*)&uplink_mako_lsm_acc_z;
  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);

  uint16_t uplink_flags = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_checksum = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);

/*
  if (!(uplink_length == 70 && 
      uplink_msgtype == 0 && 
      uplink_checksum == calcUplinkChecksum(uplinkMsg,length-2)))
  {
  //  USB_SERIAL.printf("decodeUplink bad msg: %d msg type, checksum calc bad\n",uplink_msgtype);
    badUplinkMessageCount++;
    return result;
  }
*/

//  USB_SERIAL.printf("decodeUplink good msg: %d msg type\n",uplink_msgtype);

  // all ok, assign the data to globals

  depth = ((float)uplink_depth) / 10.0;
  water_pressure = (float)uplink_water_pressure / 10.0;
  water_temperature = (float)uplink_water_temperature / 10.0;
  enclosure_temperature = (float)uplink_enclosure_temperature / 10.0;
  enclosure_humidity = (float)uplink_enclosure_humidity / 10.0;
  enclosure_air_pressure = (float)uplink_enclosure_air_pressure / 10.0;
  magnetic_heading_compensated = (float)uplink_compensated_magnetic_heading / 10.0;

  heading_to_target = (float)uplink_heading_to_target / 10.0;
  distance_to_target = (float)uplink_distance_to_target / 10.0;
  journey_course = (float)uplink_journey_course / 10.0;
  journey_distance = (float)uplink_journey_distance / 100.0;

  mako_screen_display = 0xFFFF; // MBJ LATER
  mako_seconds_on = uplink_mako_seconds_on;
  mako_user_action = 0xFFFF;  // MBJ LATER
  mako_AXP192_temp = (float)uplink_mako_AXP192_temp / 10.0;
  mako_usb_voltage = (float)uplink_mako_usb_voltage / 1000.0;
  mako_usb_current =(float)uplink_mako_usb_current / 100.0;
  mako_bat_voltage = (float)uplink_mako_bat_voltage / 1000.0;
  mako_bat_charge_current =  (float)uplink_mako_bat_charge_current / 100.0;

  mako_lsm_mag_x = uplink_mako_lsm_mag_x;
  mako_lsm_mag_y = uplink_mako_lsm_mag_y;
  mako_lsm_mag_z = uplink_mako_lsm_mag_z;

  mako_lsm_acc_x = uplink_mako_lsm_acc_x;
  mako_lsm_acc_y = uplink_mako_lsm_acc_y;
  mako_lsm_acc_z = uplink_mako_lsm_acc_z;

  console_requests_send_tweet = uplink_flags & 0x01;
  console_requests_emergency_tweet = uplink_flags & 0x02;
  console_flags = uplink_flags;

  goodUplinkMessageCount++;
  lastGoodUplinkMessage=millis();

//  dumpUplinkMessageToSerial();

  return result;
}


bool decodeUplinkMessageV4(char* uplinkMsg,const uint16_t length)
{
  bool result=false;
  
  uint16_t uplink_length = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);  
  uint16_t uplink_msgtype = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_depth = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_water_pressure = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_water_temperature = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_enclosure_temperature = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_enclosure_humidity = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_enclosure_air_pressure = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_compensated_magnetic_heading = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);

  uint16_t uplink_heading_to_target = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_distance_to_target = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_journey_course = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_journey_distance = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_screen_display = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_seconds_on = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_user_action = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_AXP192_temp = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_usb_voltage = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_usb_current = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_bat_voltage = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_bat_charge_current = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);

  float uplink_mako_lsm_mag_x=0.0; char* p=(char*)&uplink_mako_lsm_mag_x;
  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);

  float uplink_mako_lsm_mag_y=0.0; p=(char*)&uplink_mako_lsm_mag_y;
  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);

  float uplink_mako_lsm_mag_z=0.0; p=(char*)&uplink_mako_lsm_mag_z;
  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);

  float uplink_mako_lsm_acc_x=0.0; p=(char*)&uplink_mako_lsm_acc_x;
  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);

  float uplink_mako_lsm_acc_y=0.0; p=(char*)&uplink_mako_lsm_acc_y;
  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);

  float uplink_mako_lsm_acc_z=0.0; p=(char*)&uplink_mako_lsm_acc_z;
  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);

  uint16_t uplink_mako_way_marker_enum = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  
  uint16_t uplink_mako_way_marker_label = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);

  uint16_t uplink_mako_direction_metric = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);

  uint16_t uplink_flags = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_checksum = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);

/*
  if (!(uplink_length == 70 && 
      uplink_msgtype == 0 && 
      uplink_checksum == calcUplinkChecksum(uplinkMsg,length-2)))
  {
  //  USB_SERIAL.printf("decodeUplink bad msg: %d msg type, checksum calc bad\n",uplink_msgtype);
    badUplinkMessageCount++;
    return result;
  }
*/

//  USB_SERIAL.printf("decodeUplink good msg: %d msg type\n",uplink_msgtype);

  // all ok, assign the data to globals

  depth = ((float)uplink_depth) / 10.0;
  water_pressure = (float)uplink_water_pressure / 10.0;
  water_temperature = (float)uplink_water_temperature / 10.0;
  enclosure_temperature = (float)uplink_enclosure_temperature / 10.0;
  enclosure_humidity = (float)uplink_enclosure_humidity / 10.0;
  enclosure_air_pressure = (float)uplink_enclosure_air_pressure / 10.0;
  magnetic_heading_compensated = (float)uplink_compensated_magnetic_heading / 10.0;

  heading_to_target = (float)uplink_heading_to_target / 10.0;
  distance_to_target = (float)uplink_distance_to_target / 10.0;
  journey_course = (float)uplink_journey_course / 10.0;
  journey_distance = (float)uplink_journey_distance / 100.0;

  mako_screen_display = 0xFFFF; // MBJ LATER
  mako_seconds_on = uplink_mako_seconds_on;
  mako_user_action = 0xFFFF;  // MBJ  LATER
  mako_AXP192_temp = (float)uplink_mako_AXP192_temp / 10.0;
  mako_usb_voltage = (float)uplink_mako_usb_voltage / 1000.0;
  mako_usb_current =(float)uplink_mako_usb_current / 100.0;
  mako_bat_voltage = (float)uplink_mako_bat_voltage / 1000.0;
  mako_bat_charge_current =  (float)uplink_mako_bat_charge_current / 100.0;

  mako_lsm_mag_x = uplink_mako_lsm_mag_x;
  mako_lsm_mag_y = uplink_mako_lsm_mag_y;
  mako_lsm_mag_z = uplink_mako_lsm_mag_z;

  mako_lsm_acc_x = uplink_mako_lsm_acc_x;
  mako_lsm_acc_y = uplink_mako_lsm_acc_y;
  mako_lsm_acc_z = uplink_mako_lsm_acc_z;

  mako_way_marker_enum = uplink_mako_way_marker_enum;
  mako_way_marker_label[0] = uplink_mako_way_marker_label & 0xFF;
  mako_way_marker_label[1] = (uplink_mako_way_marker_label & 0xFF00)>>8;
  mako_way_marker_label[2] = NULL;

  mako_direction_metric[0] = (uplink_mako_direction_metric == 0 ? 'C' : 'M');
  mako_direction_metric[1] = '\0';
  
  console_requests_send_tweet = uplink_flags & 0x01;
  console_requests_emergency_tweet = uplink_flags & 0x02;
  console_flags = uplink_flags;

  goodUplinkMessageCount++;
  lastGoodUplinkMessage=millis();

//  dumpUplinkMessageToSerial();

  return result;
}


bool decodeUplinkMessageV5(char* uplinkMsg,const uint16_t length)
{
  bool result=false;
  
  uint16_t uplink_length = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);  
  uint16_t uplink_msgtype = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_depth = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_water_pressure = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_water_temperature = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_enclosure_temperature = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_enclosure_humidity = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_enclosure_air_pressure = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_compensated_magnetic_heading = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);

  uint16_t uplink_heading_to_target = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_distance_to_target = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_journey_course = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_journey_distance = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_screen_display = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_seconds_on = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_user_action = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_AXP192_temp = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_usb_voltage = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_usb_current = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_bat_voltage = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_bat_charge_current = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);

  char* p=NULL;
  
  float uplink_mako_lsm_mag_x=0.0; p=(char*)&uplink_mako_lsm_mag_x; *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);
  float uplink_mako_lsm_mag_y=0.0; p=(char*)&uplink_mako_lsm_mag_y;       *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);
  float uplink_mako_lsm_mag_z=0.0; p=(char*)&uplink_mako_lsm_mag_z;       *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);
  float uplink_mako_lsm_acc_x=0.0; p=(char*)&uplink_mako_lsm_acc_x;       *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);
  float uplink_mako_lsm_acc_y=0.0; p=(char*)&uplink_mako_lsm_acc_y;       *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);
  float uplink_mako_lsm_acc_z=0.0; p=(char*)&uplink_mako_lsm_acc_z;       *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);

// -- new 

  float uplink_mako_imu_gyro_x=0.0;    p=(char*)&uplink_mako_imu_gyro_x;     *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);
  float uplink_mako_imu_gyro_y=0.0;    p=(char*)&uplink_mako_imu_gyro_y;     *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);
  float uplink_mako_imu_gyro_z=0.0;    p=(char*)&uplink_mako_imu_gyro_z;     *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);
  float uplink_mako_imu_lin_acc_x=0.0; p=(char*)&uplink_mako_imu_lin_acc_x;  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);
  float uplink_mako_imu_lin_acc_y=0.0; p=(char*)&uplink_mako_imu_lin_acc_y;  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);
  float uplink_mako_imu_lin_acc_z=0.0; p=(char*)&uplink_mako_imu_lin_acc_z;  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);
  float uplink_mako_imu_rot_acc_x=0.0; p=(char*)&uplink_mako_imu_rot_acc_x;  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);
  float uplink_mako_imu_rot_acc_y=0.0; p=(char*)&uplink_mako_imu_rot_acc_y;  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);
  float uplink_mako_imu_rot_acc_z=0.0; p=(char*)&uplink_mako_imu_rot_acc_z;  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);  *(p++)=*(uplinkMsg++);

  uint16_t uplink_mako_imu_temperature = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_way_marker_enum = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_way_marker_label = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_mako_direction_metric = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_flags = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);
  uint16_t uplink_checksum = *(uplinkMsg++) + ((*(uplinkMsg++))<<8);

  if (enableAllUplinkMessageIntegrityChecks)
  {
    if (length != 114)// ||  uplink_checksum != calcUplinkChecksum(uplinkMsg,length-2))
    {
      USB_SERIAL.printf("decodeUplink bad msg: %d msg type, %d length\n",uplink_msgtype,length);
      badUplinkMessageCount++;
      return result;
    }    
  }

//  USB_SERIAL.printf("decodeUplink good msg: %d msg type\n",uplink_msgtype);

  // all ok, assign the data to globals

  depth = ((float)uplink_depth) / 10.0;
  water_pressure = (float)uplink_water_pressure / 10.0;
  water_temperature = (float)uplink_water_temperature / 10.0;
  enclosure_temperature = (float)uplink_enclosure_temperature / 10.0;
  enclosure_humidity = (float)uplink_enclosure_humidity / 10.0;
  enclosure_air_pressure = (float)uplink_enclosure_air_pressure / 10.0;
  magnetic_heading_compensated = (float)uplink_compensated_magnetic_heading / 10.0;

  heading_to_target = (float)uplink_heading_to_target / 10.0;
  distance_to_target = (float)uplink_distance_to_target / 10.0;
  journey_course = (float)uplink_journey_course / 10.0;
  journey_distance = (float)uplink_journey_distance / 100.0;

  mako_screen_display = 0xFFFF; // MBJ LATER
  mako_seconds_on = uplink_mako_seconds_on;
  mako_user_action = 0xFFFF;  // MBJ LATER
  mako_AXP192_temp = (float)uplink_mako_AXP192_temp / 10.0;
  mako_usb_voltage = (float)uplink_mako_usb_voltage / 1000.0;
  mako_usb_current =(float)uplink_mako_usb_current / 100.0;
  mako_bat_voltage = (float)uplink_mako_bat_voltage / 1000.0;
  mako_bat_charge_current =  (float)uplink_mako_bat_charge_current / 100.0;

  mako_lsm_mag_x = uplink_mako_lsm_mag_x;
  mako_lsm_mag_y = uplink_mako_lsm_mag_y;
  mako_lsm_mag_z = uplink_mako_lsm_mag_z;

  mako_lsm_acc_x = uplink_mako_lsm_acc_x;
  mako_lsm_acc_y = uplink_mako_lsm_acc_y;
  mako_lsm_acc_z = uplink_mako_lsm_acc_z;


  mako_imu_gyro_x = uplink_mako_imu_gyro_x;
  mako_imu_gyro_y = uplink_mako_imu_gyro_y;
  mako_imu_gyro_z = uplink_mako_imu_gyro_z;

  mako_imu_lin_acc_x = uplink_mako_imu_lin_acc_x;
  mako_imu_lin_acc_y = uplink_mako_imu_lin_acc_y;
  mako_imu_lin_acc_z = uplink_mako_imu_lin_acc_z;

  mako_imu_rot_acc_x = uplink_mako_imu_rot_acc_x;
  mako_imu_rot_acc_y = uplink_mako_imu_rot_acc_y;
  mako_imu_rot_acc_z = uplink_mako_imu_rot_acc_z;

  mako_imu_temperature = uplink_mako_imu_temperature / 10.0;

  mako_way_marker_enum = uplink_mako_way_marker_enum;
  mako_way_marker_label[0] = uplink_mako_way_marker_label & 0xFF;
  mako_way_marker_label[1] = (uplink_mako_way_marker_label & 0xFF00)>>8;
  mako_way_marker_label[2] = '\0';

  mako_direction_metric[0] = (uplink_mako_direction_metric == 0 ? 'C' : 'M');
  mako_direction_metric[1] = '\0';
    
  console_requests_send_tweet = uplink_flags & 0x01;
  console_requests_emergency_tweet = uplink_flags & 0x02;
  console_flags = uplink_flags;

  goodUplinkMessageCount++;
  lastGoodUplinkMessage=millis();

//  dumpUplinkMessageToSerial();

  uplinkMessageLength=uplink_length;
  
  KB_from_mako=KB_from_mako+(((float)uplink_length)/1024.0);
  return result;
}




void dumpUplinkMessageToSerial()
{
  USB_SERIAL.printf("d=%f,wp=%f,wt=%f,et=%f,eh=%f,eap=%f,mhc=%f\n,flags=%d",
      depth,water_pressure,water_temperature,enclosure_temperature, 
      enclosure_humidity, enclosure_air_pressure, magnetic_heading_compensated,console_flags);
}


uint16_t calcUplinkChecksum(char* buffer,uint16_t length)
{ 
  uint16_t* word_buffer=(uint16_t*)buffer;
  uint16_t word_length = length/2;
  
  uint16_t checksum=0;
  
  while (word_length--)
    checksum = checksum ^ *(word_buffer++);

  return checksum;
}

void sendFakeGPSData1()
{
   char fake1[]="$GPRMC,235316.000,A,4003.9040,N,10512.5792,W,0.09,144.75,141112,,*19\n";
   GOPRO_SERIAL.write(fake1);
   delay(100);
}

void sendFakeGPSData2()
{
   char fake1[]="$GPRMC,092204.999,A,4250.5589,S,14718.5084,E,0.00,89.68,211200,,*25\n";
   GOPRO_SERIAL.write(fake1);
   delay(100);
}


///// GPS MESSAGE QUERY/SET

void sendCAS06QueryMessages()
{
/*
 * 
  Query the information type of the product. Refer to 1.5.8 for information content. 
  0=Query firmware version number
  1=Query the hardware model and serial number
  2=Query the working mode of the multimode receiver MO=GB means dual mode of GPS+BDS
  3=Query the customer number of the product 
  5=Query upgrade code information
 */
  
    // these are sent CAS026 messages and they work using grove port and HAT pins
    gps_serial.write("$PCAS06,0*1B\r\n");   // response: $GPTXT,01,01,02,SW=URANUS5,V5.3.0.0*1D
//    gps_serial.write("$PCAS06,1*1A\r\n");   // response: $GPTXT,01,01,02,HW=ATGM336H,0001010379462*1F  (chip model ATGM336H)
//    gps_serial.write("$PCAS06,2*19\r\n");   // response: $GPTXT,01,01,02,MO=GB*77
//    gps_serial.write("$PCAS06,3*18\r\n");   // response: $GPTXT,01,01,02,CI=01B94154*04
//    gps_serial.write("$PCAS06,5*1E\r\n");   // response: $GPTXT,01,01,02,BS=SOC_BootLoader,V6.2.0.2*34
}


void sendCAS02LocationUpdateRateMessages()
{
/*
 * 1000 = update rate 1Hz, output per second 1 
 * 500 = update rate 2Hz, output per second 2
 * 250 = update rate 4Hz, output per second 4
 * 200 = update rate 5Hz, output per second 5
 * 100 = update rate 10Hz, output per second 10
 * 
 */
    gps_serial.write("$PCAS02,1000*2E\r\n");   // set to 1Hz response: 
//    gps_serial.write("$PCAS02,500*1A\r\n");   // set to 2Hz response: 
//    gps_serial.write("$PCAS02,100*1E\r\n");   // set to 10Hz response: 
}

void SendCASICNavXQuery()
{
  unsigned char NavXQuery[]={0xBA,0xCE,0x00,0x00,0x06,0x07,0x00,0x00,0x06,0x07};

  gps_serial.write(NavXQuery,sizeof(NavXQuery));
}

void SendCASICNavXWalkingDynamicModel(const bool setWalking)
{
  uint8_t d=(setWalking ? 2 : 0); // walking mode dynModel is 2, default is 0 (portable)

  uint8_t dynModelOffset=4;
  uint8_t preamble[]={0xBA,0xCE,0x2c,0x00,0x06,0x07};
  uint8_t payload[]={0,0,0,1,255,0,0,0,0,0,
                     0,0,0,0,0,0,0,0,0,0,
                     0,0,0,0,0,0,0,0,0,0,
                     0,0,0,0,0,0,0,0,0,0,
                     0,0,0,0};    // 44 bytes of payload

  payload[dynModelOffset]=d;
                     
  uint32_t checksum = (preamble[2] << 24) + (preamble[3] << 16) + sizeof(payload);
  for (int i=0; i<sizeof(payload);i=i+4)
    checksum = checksum + *((uint32_t*)(payload+i));

  gps_serial.write(preamble,sizeof(preamble));
  gps_serial.write(payload,sizeof(payload));
  gps_serial.write((uint8_t)(checksum & 0xFF));
  gps_serial.write((uint8_t)(checksum >> 8 & 0xFF));
  gps_serial.write((uint8_t)(checksum >> 16 & 0xFF));
  gps_serial.write((uint8_t)(checksum >> 24 & 0xFF));

// TOMORROW: need to find out if the checksum calculation is correct comparing to the Nav response and the ACK-ACK msg.
// BEFORE SENDING ANY SET COMMANDS
}

void waitForCASICNavXResponse()
{
  USB_SERIAL.println("Waiting for Navx Response Ok");

  uint8_t payloadLength=44;
  uint8_t payload[payloadLength];

  uint8_t checksumLength=4;
  uint8_t checksum[checksumLength];

  // length 44 == 0x002C

  uint8_t searchCriteria[]={0xBA,0xCE,0x2C,0x00,0x06,0x07};
  uint8_t criteriaCount=sizeof(searchCriteria);
  uint8_t next = 0;
  
  while (next < criteriaCount)
  {
    if (gps_serial.available())
      next = ((gps_serial.read() == searchCriteria[next]) ? next+1 : 0);
  }
  USB_SERIAL.println("found all Navx Response preamble bytes");

  uint8_t nextPayload = 0;
  while (nextPayload < payloadLength)
  {
    if (gps_serial.available())
    {
      payload[nextPayload] = gps_serial.read();
      nextPayload++;
    }
  }      
  USB_SERIAL.printf("read all payload bytes\n");

  uint8_t nextChecksumByte=0;
  while (nextChecksumByte < checksumLength)
  {
    if (gps_serial.available())
      checksum[nextChecksumByte++] = gps_serial.read();
  }
  USB_SERIAL.printf("read all checksum bytes\n");

  // Verify checksum
  // LATER  

  // What is in the payload?
  uint8_t mask0 = payload[3]; // MSB
  uint8_t mask1 = payload[2];
  uint8_t mask2 = payload[1];
  uint8_t mask3 = payload[0]; // LSB
  uint8_t dynamicModel = payload[4];
  uint8_t fixMode = payload[5];
  uint8_t minimumNumberofSatellites = payload[6];
  uint8_t maximumNumberofSatellites = payload[7];
  uint8_t minCNO = payload[8];
  uint8_t iniFix3D = payload[10];
  int8_t minElev = payload[11];

  USB_SERIAL.printf("mask0: %x mask1: %x mask2: %x mask3: %x\n",mask0,mask1,mask2,mask3);
  USB_SERIAL.printf("dyModel: %i  fixMode: %i  minSat: %i  maxSat: %i\n",dynamicModel,fixMode,minimumNumberofSatellites,maximumNumberofSatellites);
  USB_SERIAL.printf("minCNO: %i  iniFix3D: %i  minElev: %i\n",minCNO,iniFix3D,minElev);
}


void waitForCASICACKResponse()
{
  while (true)
  {
    if (gps_serial.available())
    {
      char nextByte=gps_serial.read();
      if (nextByte == 0x05)
      {
        nextByte=gps_serial.read();
        if (nextByte == 0x01)
        {
          USB_SERIAL.println("ACK-ACK Received");
          // format: 0xBA 0xCE 0x00 0x04 0x05 0x01 <class> <msgid> <reserve0> <reserve1> <4 checksum bytes>  
          // class = type of information received correctly - unsigned char
          // msgid = number of correctly received message - unsigned char
          // reserve 0 and reserve1 = two bytes containing an unsigned short int

          // throw away next 8 bytes
          int i=8;
          while (i)
          {
            if (gps_serial.available())
            {
              gps_serial.read();
              i--;
            }
          }
        }
        else if (nextByte == 0x00)
        {
          USB_SERIAL.println("ACK-NACK Received");
          // format: 0xBA 0xCE 0x00 0x04 0x05 0x00 <class> <msgid> <reserve0> <reserve1> <4 checksum bytes>
          // class = Type of information not received correctly - unsigned char
          // msgid = Number of incorrectly received messages - unsigned char
          // reserve0 and reserve1 = two bytes containing an unsigned short int       
          // throw away next 8 bytes
          int i=8;
          while (i)
          {
            if (gps_serial.available())
            {
              gps_serial.read();
              i--;
            }
          }
        }
        
        break;
      }
    }
  }
}

void toggleOTAActive()
{
   M5.Lcd.fillScreen(TFT_ORANGE);
   M5.Lcd.setCursor(0,0);

   if (otaActiveListening)
   {
     asyncWebServer.end();
     M5.Lcd.println("OTA Disabled");
     otaActiveListening=false;
     delay (2000);
   }
   else
   {
     if (WiFi.status() == WL_CONNECTED)
     {
       asyncWebServer.begin();
       M5.Lcd.printf("OTA Enabled");
       otaActiveListening=true;
     }
     else
     {
       M5.Lcd.println("Error: Enable Wifi First");
     }
     delay (2000);
   }  

   M5.Lcd.fillScreen(TFT_BLACK);
}

void toggleWiFiActive()
{
   M5.Lcd.fillScreen(TFT_ORANGE);
   M5.Lcd.setCursor(0,0);

   if (WiFi.status() == WL_CONNECTED)
   {
      if (otaActiveListening)
      {
         asyncWebServer.end();
         M5.Lcd.println("OTA Disabled");
         otaActiveListening=false;
      }

       WiFi.disconnect();
       M5.Lcd.printf("Wifi Disabled");
       delay (2000);
   }
   else
   {
     M5.Lcd.printf("Wifi Connecting");

    // startup Wifi only
    shutdownIfUSBPowerOff();
    if (!connectWiFiNoOTA(ssid_1, password_1, label_1, timeout_1))
    {
      shutdownIfUSBPowerOff();
      if (!connectWiFiNoOTA(ssid_2, password_2, label_2, timeout_2))
      {
        shutdownIfUSBPowerOff();
        connectWiFiNoOTA(ssid_3, password_3, label_3, timeout_3);
      }
    }
   }

   M5.Lcd.fillScreen(TFT_BLACK);
}

void shutdownIfUSBPowerOff()
{
  if (M5.Axp.GetVBusVoltage() < minimumUSBVoltage)
  {
    if (USBVoltageDropTime == 0)
      USBVoltageDropTime=millis();
    else 
    {
      if (millis() > USBVoltageDropTime + milliSecondsToWaitForShutDown)
      {
        // initiate shutdown after 3 seconds.
        delay(1000);
        fadeToBlackAndShutdown();
      }
    }
  }
  else
  {
    if (USBVoltageDropTime != 0)
      USBVoltageDropTime = 0;
  }
}

void fadeToBlackAndShutdown()
{
  for (int i=14; i>6; i--)
  {
    M5.Axp.ScreenBreath(i);             // 7-14 fade to black
    delay(100);
  }

  M5.Axp.PowerOff(); 
}

void checkForLeak(const char* msg, const uint8_t pin)
{
  bool leakStatus = false;
  
  if (pin == M5_POWER_SWITCH_PIN)
  {
    leakStatus = (M5.Axp.GetBtnPress());
  }
  else
  {
    leakStatus = !(digitalRead(pin));
  }

  if (leakStatus)
  {
    M5.Lcd.fillScreen(TFT_RED);
    M5.Lcd.setTextSize(3);
    M5.Lcd.setCursor(5, 10);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_RED);
    M5.Lcd.print(msg);
    M5.Beep.setBeep(1200, 100);
    M5.Beep.beep();
    delay(100);
    updateButtonsAndBuzzer();

    M5.Lcd.fillScreen(TFT_ORANGE);
    M5.Lcd.setCursor(5, 10);
    M5.Lcd.setTextColor(TFT_YELLOW, TFT_ORANGE);
    M5.Lcd.print(msg);
    M5.Beep.setBeep(1500, 100);
    M5.Beep.beep();
    delay(100);

    updateButtonsAndBuzzer();
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Beep.mute();
  }
}

bool connectWiFiNoOTA(const char* _ssid, const char* _password, const char* label, uint32_t timeout)
{
  bool forcedCancellation = false;
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextSize(2);
  bool connected = false;
  WiFi.mode(WIFI_STA);
  WiFi.begin(_ssid, _password);

  // Wait for connection for max of timeout/1000 seconds
  M5.Lcd.printf("%s Wifi", label);
  int count = timeout / 500;
  while (WiFi.status() != WL_CONNECTED && --count > 0)
  {
    // check for cancellation button - top button.
    updateButtonsAndBuzzer();

    if (p_primaryButton->isPressed()) // cancel connection attempts
    {
      forcedCancellation = true;
      break;
    }

    M5.Lcd.print(".");
    delay(500);
  }
  M5.Lcd.print("\n\n");

  if (WiFi.status() == WL_CONNECTED)
  {
    M5.Lcd.setRotation(0);
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setCursor(0,155);
    M5.Lcd.setTextSize(2);
    M5.Lcd.printf("%s\n\n",WiFi.localIP().toString());
    M5.Lcd.println(WiFi.macAddress());
    connected = true;

    M5.Lcd.qrcode("http://"+WiFi.localIP().toString()+"/update",0,0,135);

    updateButtonsAndBuzzer();

    if (p_secondButton->isPressed())
    {
      M5.Lcd.print("\n20 second pause");
      delay(20000);
    }
  }
  else
  {
    if (forcedCancellation)
      M5.Lcd.print("\n     Cancelled\n Connection Attempts");
    else
      M5.Lcd.print("No Connection");
  }

  delay(1000);

  M5.Lcd.fillScreen(TFT_BLACK);

  return connected;
}

bool setupOTAWebServer(const char* _ssid, const char* _password, const char* label, uint32_t timeout)
{
#ifdef ENABLE_ELEGANT_OTA_AT_COMPILE_TIME
  bool forcedCancellation = false;
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextSize(2);
  bool connected = false;
  WiFi.mode(WIFI_STA);
  WiFi.begin(_ssid, _password);

  // Wait for connection for max of timeout/1000 seconds
  M5.Lcd.printf("%s Wifi", label);
  int count = timeout / 500;
  while (WiFi.status() != WL_CONNECTED && --count > 0)
  {
    // check for cancellation button - top button.
    updateButtonsAndBuzzer();

    if (p_primaryButton->isPressed()) // cancel connection attempts
    {
      forcedCancellation = true;
      break;
    }

    M5.Lcd.print(".");
    delay(500);
  }
  M5.Lcd.print("\n\n");

  if (WiFi.status() == WL_CONNECTED)
  {
    asyncWebServer.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(200, "text/plain", "To upload firmware use /update");
    });

    AsyncElegantOTA.begin(&asyncWebServer);    // Start AsyncElegantOTA
    asyncWebServer.begin();

    M5.Lcd.setRotation(0);
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setCursor(0,155);
    M5.Lcd.setTextSize(2);
    M5.Lcd.printf("%s\n\n",WiFi.localIP().toString());
    M5.Lcd.println(WiFi.macAddress());
    connected = true;

    M5.Lcd.qrcode("http://"+WiFi.localIP().toString()+"/update",0,0,135);

    updateButtonsAndBuzzer();

    if (p_secondButton->isPressed())
    {
      M5.Lcd.print("\n20 second pause");
      delay(20000);
    }
  }
  else
  {
    if (forcedCancellation)
      M5.Lcd.print("\n     Cancelled\n Connection Attempts");
    else
      M5.Lcd.print("No Connection");
  }

  delay(1000);

  M5.Lcd.fillScreen(TFT_BLACK);

  return connected;
#else
  return false;
#endif
}

#ifdef ENABLE_QUBITRO_AT_COMPILE_TIME
bool qubitro_connect() 
{
  
  bool success=true;
  
  if (enableConnectToQubitro && WiFi.status() == WL_CONNECTED)
  {
    qubitro_mqttClient.setId(qubitro_device_id);
    qubitro_mqttClient.setDeviceIdToken(qubitro_device_id, qubitro_device_token);
    
    USB_SERIAL.println("Connecting to Qubitro...");
  
    if (!qubitro_mqttClient.connect(qubitro_host, qubitro_port))
    {
      USB_SERIAL.print("Connection failed. Error code: ");
      USB_SERIAL.println(qubitro_mqttClient.connectError());
      USB_SERIAL.println("Visit docs.qubitro.com or create a new issue on github.com/qubitro");
      success=false;
    }
    else
    {
      USB_SERIAL.println("Connected to Qubitro.");
    }
  
    qubitro_mqttClient.subscribe(qubitro_device_id);
  }
  else
  {
    success=false;
  }
  
  return success;
}

void buildTwitterTelemetryTweet(char* payload, bool SOS)
{  
  if (SOS)
  {
    sprintf(payload,"Ignore. This is a test: SOS Live Dive Log UTC: %02d:%02d:%02d: https://www.google.co.uk/maps/@%f,%f,14z Depth %.1f, water_temp %.1f, heading %.0f, console_temp %.1f, console_humidity %.1f, console_mB %.0f",
              gps.time.hour(), gps.time.minute(),gps.time.second(),
              gps.location.lat(),
              gps.location.lng(),
              depth,
              water_temperature,
              magnetic_heading_compensated,
              enclosure_temperature,
              enclosure_humidity,
              enclosure_air_pressure); 
  }
  else
  {
    sprintf(payload,"Scuba Hacker's Mercator Origins Live Dive Log UTC: %02d:%02d:%02d: https://www.google.co.uk/maps/@%f,%f,14z Depth %.1f, water_temp %.1f, heading %.0f, console_temp %.1f, console_humidity %.1f, console_mB %.0f",
            gps.time.hour(), gps.time.minute(),gps.time.second(),
            gps.location.lat(),
            gps.location.lng(),
            depth,
            water_temperature,
            magnetic_heading_compensated,
            enclosure_temperature,
            enclosure_humidity,
            enclosure_air_pressure);
  }
}

void buildUplinkTelemetryMessageV1(char* payload)
{
    sprintf(payload,"{\"UTC time\":\"%02d:%02d:%02d\",\"UTC date\":\"%02d:%02d:%02d\",\"on_seconds\":%lu,\"coordinates\":[%f,%f],\"depth\":%f,\"water_pressure\":%f,\"water_temperature\":%f,\"enclosure_temperature\":%f,\"enclosure_humidity\":%f,\"enclosure_air_pressure\":%f,\"magnetic_heading_compensated\":%f}",
           gps.time.hour(), gps.time.minute(),gps.time.second(),
            gps.date.day(), gps.date.month(),gps.date.year(),
            millis()/1000,
            gps.location.lat(),
            gps.location.lng(),
            depth,
            water_pressure,
            water_temperature,
            enclosure_temperature,
            enclosure_humidity,
            enclosure_air_pressure,
            magnetic_heading_compensated);
}

void buildUplinkTelemetryMessageV2(char* payload)
{
    sprintf(payload,
          "{\"UTC time\":\"%02d:%02d:%02d\",\"UTC date\":\"%02d:%02d:%02d\",\"lemon_on_seconds\":%lu,\"coordinates\":[%f,%f],\"depth\":%f,"
          "\"water_pressure\":%f,\"water_temperature\":%f,\"enclosure_temperature\":%f,\"enclosure_humidity\":%f,\"enclosure_air_pressure\":%f,"
          "\"magnetic_heading_compensated\":%f,\"heading_to_target\":%f,\"distance_to_target\":%f,\"journey_course\":%f,\"journey_distance\":%f,"
          "\"mako_screen_display\":%d,\"mako_seconds_on\":%lu,\"mako_user_action\":%d,\"mako_AXP192_temp\":%f,"
          "\"mako_usb_voltage\":%f,\"mako_usb_current\":%f,\"mako_bat_voltage\":%f,"
          "\"mako_bat_charge_current\":%f"
          "}",
           gps.time.hour(), gps.time.minute(),gps.time.second(),
            gps.date.day(), gps.date.month(),gps.date.year(),
            millis()/1000/60,
            gps.location.lat(),
            gps.location.lng(),
            depth,
            water_pressure,
            water_temperature,
            enclosure_temperature,
            enclosure_humidity,
            enclosure_air_pressure,
            magnetic_heading_compensated,            
            heading_to_target,
            distance_to_target,
            journey_course,
            journey_distance,
            mako_screen_display,
            mako_seconds_on,
            mako_user_action,
            mako_AXP192_temp,
            mako_usb_voltage,
            mako_usb_current,
            mako_bat_voltage,
            mako_bat_charge_current
            );

}

void buildUplinkTelemetryMessageV3(char* payload)
{
    sprintf(payload,
          "{\"UTC time\":\"%02d:%02d:%02d\",\"UTC date\":\"%02d:%02d:%02d\",\"lemon_on_seconds\":%lu,\"coordinates\":[%f,%f],\"depth\":%f,"
          "\"water_pressure\":%f,\"water_temperature\":%f,\"enclosure_temperature\":%f,\"enclosure_humidity\":%f,\"enclosure_air_pressure\":%f,"
          "\"magnetic_heading_compensated\":%f,\"heading_to_target\":%f,\"distance_to_target\":%f,\"journey_course\":%f,\"journey_distance\":%f,"
          "\"mako_screen_display\":%d,\"mako_seconds_on\":%lu,\"mako_user_action\":%d,\"mako_AXP192_temp\":%f,"
          "\"mako_usb_voltage\":%f,\"mako_usb_current\":%f,\"mako_bat_voltage\":%f,\"mako_bat_charge_current\":%f,"
          
          "\"fix_count\":%lu,\"lemon_usb_voltage\":%f,\"lemon_usb_current\":%f,\"lemon_bat_voltage\":%f,\"lemon_bat_current\":%f,"
          
          "\"sats\":%lu,\"hdop\":%f,\"gps_course\":%f,\"gps_speed_knots\":%f"
          
          "}",
           gps.time.hour(), gps.time.minute(),gps.time.second(),
            gps.date.day(), gps.date.month(),gps.date.year(),
            millis()/1000,
            gps.location.lat(),
            gps.location.lng(),
            depth,
            water_pressure,
            water_temperature,
            enclosure_temperature,
            enclosure_humidity,
            enclosure_air_pressure,
            magnetic_heading_compensated,            
            heading_to_target,
            distance_to_target,
            journey_course,
            journey_distance,
            mako_screen_display,
            mako_seconds_on,
            mako_user_action,
            mako_AXP192_temp,
            mako_usb_voltage,
            mako_usb_current,
            mako_bat_voltage,
            mako_bat_charge_current,

            fixCount,
            M5.Axp.GetVBusVoltage(),
            M5.Axp.GetVBusCurrent(),
            M5.Axp.GetBatVoltage(),
            M5.Axp.GetBatCurrent(),

            satellites,
            gps.hdop.hdop(),
            gps.course.deg(),
            gps.speed.knots()
            );

}


void buildUplinkTelemetryMessageV4(char* payload)
{
    sprintf(payload,
          "{\"UTC time\":\"%02d:%02d:%02d\",\"UTC date\":\"%02d:%02d:%02d\",\"lemon_on_seconds\":%lu,\"coordinates\":[%f,%f],\"depth\":%f,"
          "\"water_pressure\":%f,\"water_temperature\":%f,\"enclosure_temperature\":%f,\"enclosure_humidity\":%f,\"enclosure_air_pressure\":%f,"
          "\"magnetic_heading_compensated\":%f,\"heading_to_target\":%f,\"distance_to_target\":%f,\"journey_course\":%f,\"journey_distance\":%f,"
          "\"mako_screen_display\":%d,\"mako_seconds_on\":%lu,\"mako_user_action\":%d,\"mako_AXP192_temp\":%f,"
          "\"mako_usb_voltage\":%f,\"mako_usb_current\":%f,\"mako_bat_voltage\":%f,\"mako_bat_charge_current\":%f,"
          "\"fix_count\":%lu,\"lemon_usb_voltage\":%f,\"lemon_usb_current\":%f,\"lemon_bat_voltage\":%f,\"lemon_bat_current\":%f,"
          "\"sats\":%lu,\"hdop\":%f,\"gps_course\":%f,\"gps_speed_knots\":%f,"

          "\"mako_lsm_mag_x\":%f,\"mako_lsm_mag_y\":%f,\"mako_lsm_mag_z\":%f,\"mako_lsm_acc_x\":%f,\"mako_lsm_acc_y\":%f,\"mako_lsm_acc_z\":%f,"
          "\"mako_waymarker_e\":%d,\"mako_waymarker_label\":\"%s\",\"mako_direction_metric\":\"-\""
          
          "}",
           gps.time.hour(), gps.time.minute(),gps.time.second(),
            gps.date.day(), gps.date.month(),gps.date.year(),
            millis()/1000,
            gps.location.lat(),
            gps.location.lng(),
            depth,
            water_pressure,
            water_temperature,
            enclosure_temperature,
            enclosure_humidity,
            enclosure_air_pressure,
            magnetic_heading_compensated,            
            heading_to_target,
            distance_to_target,
            journey_course,
            journey_distance,
            mako_screen_display,
            mako_seconds_on,
            mako_user_action,
            mako_AXP192_temp,
            mako_usb_voltage,
            mako_usb_current,
            mako_bat_voltage,
            mako_bat_charge_current,

            fixCount,
            M5.Axp.GetVBusVoltage(),
            M5.Axp.GetVBusCurrent(),
            M5.Axp.GetBatVoltage(),
            M5.Axp.GetBatCurrent(),

            satellites,
            gps.hdop.hdop(),
            gps.course.deg(),
            gps.speed.knots(),

            mako_lsm_mag_x,
            mako_lsm_mag_y,
            mako_lsm_mag_z,
            mako_lsm_acc_x,
            mako_lsm_acc_y,
            mako_lsm_acc_z,

            mako_way_marker_enum,
            mako_way_marker_label,
            mako_direction_metric
            );

}



void buildUplinkTelemetryMessageV5(char* payload)
{
    // 66 metrics as of 8 May 13:40
    sprintf(payload,
          "{\"UTC time\":\"%02d:%02d:%02d\",\"UTC date\":\"%02d:%02d:%02d\",\"lemon_on_seconds\":%lu,\"coordinates\":[%f,%f],\"depth\":%f,"
          "\"water_pressure\":%f,\"water_temperature\":%f,\"enclosure_temperature\":%f,\"enclosure_humidity\":%f,\"enclosure_air_pressure\":%f,"
          "\"magnetic_heading_compensated\":%f,\"heading_to_target\":%f,\"distance_to_target\":%f,\"journey_course\":%f,\"journey_distance\":%f,"
          "\"mako_screen_display\":%d,\"mako_seconds_on\":%lu,\"mako_user_action\":%d,\"mako_AXP192_temp\":%f,"
          "\"mako_usb_voltage\":%f,\"mako_usb_current\":%f,\"mako_bat_voltage\":%f,\"mako_bat_charge_current\":%f,"
          "\"fix_count\":%lu,\"lemon_usb_voltage\":%f,\"lemon_usb_current\":%f,\"lemon_bat_voltage\":%f,\"lemon_bat_current\":%f,"
          "\"sats\":%lu,\"hdop\":%f,\"gps_course\":%f,\"gps_speed_knots\":%f,"

          "\"mako_lsm_mag_x\":%f,\"mako_lsm_mag_y\":%f,\"mako_lsm_mag_z\":%f,"
          "\"mako_lsm_acc_x\":%f,\"mako_lsm_acc_y\":%f,\"mako_lsm_acc_z\":%f,"
          
          "\"mako_imu_gyro_x\":%f,\"mako_imu_gyro_y\":%f,\"mako_imu_gyro_z\":%f,"
          "\"mako_imu_lin_acc_x\":%f,\"mako_imu_lin_acc_y\":%f,\"mako_imu_lin_acc_z\":%f,"
          "\"mako_imu_rot_acc_x\":%f,\"mako_imu_rot_acc_y\":%f,\"mako_imu_rot_acc_z\":%f,"
          "\"mako_imu_temperature\":%f,"

          "\"lemon_imu_gyro_x\":%f,\"lemon_imu_gyro_y\":%f,\"lemon_imu_gyro_z\":%f,"
          "\"lemon_imu_lin_acc_x\":%f,\"lemon_imu_lin_acc_y\":%f,\"lemon_imu_lin_acc_z\":%f,"
          "\"lemon_imu_rot_acc_x\":%f,\"lemon_imu_rot_acc_y\":%f,\"lemon_imu_rot_acc_z\":%f,"
          "\"lemon_imu_temperature\":%f,"
                    
          "\"mako_waymarker_e\":%d,\"mako_waymarker_label\":\"-\",\"mako_direction_metric\":\"-\""
          
          "}",
          
           gps.time.hour(), gps.time.minute(),gps.time.second(),
            gps.date.day(), gps.date.month(),gps.date.year(),
            millis()/1000/60,
            gps.location.lat(),
            gps.location.lng(),
            depth,
            water_pressure,
            water_temperature,
            enclosure_temperature,
            enclosure_humidity,
            enclosure_air_pressure,
            magnetic_heading_compensated,            
            heading_to_target,
            distance_to_target,
            journey_course,
            journey_distance,
            mako_screen_display,
            mako_seconds_on,
            mako_user_action,
            mako_AXP192_temp,
            mako_usb_voltage,
            mako_usb_current,
            mako_bat_voltage,
            mako_bat_charge_current,

            fixCount,
            M5.Axp.GetVBusVoltage(),
            M5.Axp.GetVBusCurrent(),
            M5.Axp.GetBatVoltage(),
            M5.Axp.GetBatCurrent(),

            satellites,
            gps.hdop.hdop(),
            gps.course.deg(),
            gps.speed.knots(),

            mako_lsm_mag_x,
            mako_lsm_mag_y,
            mako_lsm_mag_z,
            mako_lsm_acc_x,
            mako_lsm_acc_y,
            mako_lsm_acc_z,


            mako_imu_gyro_x,
            mako_imu_gyro_y,
            mako_imu_gyro_z,
            mako_imu_lin_acc_x,
            mako_imu_lin_acc_y,
            mako_imu_lin_acc_z,
            mako_imu_rot_acc_x,
            mako_imu_rot_acc_y,
            mako_imu_rot_acc_z,
            mako_imu_temperature,

            lemon_imu_gyro_x,
            lemon_imu_gyro_y,
            lemon_imu_gyro_z,
            lemon_imu_lin_acc_x,
            lemon_imu_lin_acc_y,
            lemon_imu_lin_acc_z,
            lemon_imu_rot_acc_x,
            lemon_imu_rot_acc_y,
            lemon_imu_rot_acc_z,
            lemon_imu_temperature,

            mako_way_marker_enum,
            mako_way_marker_label,
            mako_direction_metric
            );

}

void buildUplinkTelemetryMessageV6(char* payload)
{
    currentQubitroUploadAt=millis();
    qubitroUploadDutyCycle=currentQubitroUploadAt-lastQubitroUploadAt;
    uint32_t live_metrics_count = 75; // as of 9 May 20:16
    sprintf(payload,
          "{\"UTC_time\":\"%02d:%02d:%02d\",\"UTC_date\":\"%02d:%02d:%02d\",\"lemon_on_seconds\":%lu,\"coordinates\":[%f,%f],\"depth\":%f,"
          "\"water_pressure\":%f,\"water_temperature\":%f,\"enclosure_temperature\":%f,\"enclosure_humidity\":%f,\"enclosure_air_pressure\":%f,"
          "\"magnetic_heading_compensated\":%f,\"heading_to_target\":%f,\"distance_to_target\":%f,\"journey_course\":%f,\"journey_distance\":%f,"
          "\"mako_screen_display\":%d,\"mako_seconds_on\":%lu,\"mako_user_action\":%d,\"mako_AXP192_temp\":%f,"
          "\"mako_usb_voltage\":%f,\"mako_usb_current\":%f,\"mako_bat_voltage\":%f,\"mako_bat_charge_current\":%f,"
          "\"fix_count\":%lu,\"lemon_usb_voltage\":%f,\"lemon_usb_current\":%f,\"lemon_bat_voltage\":%f,\"lemon_bat_current\":%f,"
          "\"sats\":%lu,\"hdop\":%f,\"gps_course\":%f,\"gps_speed_knots\":%f,"

          "\"mako_lsm_mag_x\":%f,\"mako_lsm_mag_y\":%f,\"mako_lsm_mag_z\":%f,"
          "\"mako_lsm_acc_x\":%f,\"mako_lsm_acc_y\":%f,\"mako_lsm_acc_z\":%f,"
          
          "\"mako_imu_gyro_x\":%f,\"mako_imu_gyro_y\":%f,\"mako_imu_gyro_z\":%f,"
          "\"mako_imu_lin_acc_x\":%f,\"mako_imu_lin_acc_y\":%f,\"mako_imu_lin_acc_z\":%f,"
          "\"mako_imu_rot_acc_x\":%f,\"mako_imu_rot_acc_y\":%f,\"mako_imu_rot_acc_z\":%f,"
          "\"mako_imu_temperature\":%f,"

          "\"lemon_imu_gyro_x\":%f,\"lemon_imu_gyro_y\":%f,\"lemon_imu_gyro_z\":%f,"
          "\"lemon_imu_lin_acc_x\":%f,\"lemon_imu_lin_acc_y\":%f,\"lemon_imu_lin_acc_z\":%f,"
          "\"lemon_imu_rot_acc_x\":%f,\"lemon_imu_rot_acc_y\":%f,\"lemon_imu_rot_acc_z\":%f,"
          "\"lemon_imu_temperature\":%f,"
                    
          "\"mako_waymarker_e\":%d,\"mako_waymarker_label\":\"-\",\"mako_direction_metric\":\"-\","

          "\"uplink_msgs_from_mako\":%lu,\"uplink_msg_length\":%hu,\"msgs_to_qubitro\":%d,\"qubitro_msg_length\":%hu,\"KB_to_qubitro\":%.1f,\"KB_uplinked_from_mako\":%.1f,"
          "\"live_metrics\":%lu,\"qubitro_upload_duty_cycle\":%lu,\"console_downlink_msg\":%lu,\"geo_location\":\"Gozo, Malta\""
          "}",         
           gps.time.hour(), gps.time.minute(),gps.time.second(),
            gps.date.day(), gps.date.month(),gps.date.year(),
            millis()/1000/60,
            gps.location.lat(),
            gps.location.lng(),
            depth,
            water_pressure,
            water_temperature,
            enclosure_temperature,
            enclosure_humidity,
            enclosure_air_pressure,
            magnetic_heading_compensated,            
            heading_to_target,
            distance_to_target,
            journey_course,
            journey_distance,
            mako_screen_display,
            mako_seconds_on,
            mako_user_action,
            mako_AXP192_temp,
            mako_usb_voltage,
            mako_usb_current,
            mako_bat_voltage,
            mako_bat_charge_current,

            fixCount,
            M5.Axp.GetVBusVoltage(),
            M5.Axp.GetVBusCurrent(),
            M5.Axp.GetBatVoltage(),
            M5.Axp.GetBatCurrent(),

            satellites,
            gps.hdop.hdop(),
            gps.course.deg(),
            gps.speed.knots(),

            mako_lsm_mag_x,
            mako_lsm_mag_y,
            mako_lsm_mag_z,
            mako_lsm_acc_x,
            mako_lsm_acc_y,
            mako_lsm_acc_z,


            mako_imu_gyro_x,
            mako_imu_gyro_y,
            mako_imu_gyro_z,
            mako_imu_lin_acc_x,
            mako_imu_lin_acc_y,
            mako_imu_lin_acc_z,
            mako_imu_rot_acc_x,
            mako_imu_rot_acc_y,
            mako_imu_rot_acc_z,
            mako_imu_temperature,

            lemon_imu_gyro_x,
            lemon_imu_gyro_y,
            lemon_imu_gyro_z,
            lemon_imu_lin_acc_x,
            lemon_imu_lin_acc_y,
            lemon_imu_lin_acc_z,
            lemon_imu_rot_acc_x,
            lemon_imu_rot_acc_y,
            lemon_imu_rot_acc_z,
            lemon_imu_temperature,

            mako_way_marker_enum,
// DO NOT POPULATE (HARDCODED IN SPRINTF STRING)           mako_way_marker_label,
// DO NOT POPULATE (HARDCODED IN SPRINTF STRING)          mako_direction_metric,

            goodUplinkMessageCount,
            uplinkMessageLength,
            qubitroUploadCount,
            qubitroMessageLength,
            KB_to_Qubitro,
            KB_from_mako,
            live_metrics_count,
            qubitroUploadDutyCycle,
            consoleDownlinkMsgCount
 // DO NOT POPULATE (HARDCODED IN SPRINTF STRING) geo_location
            );

   qubitroMessageLength=strlen(payload);
   KB_to_Qubitro=KB_from_mako+(((float)(qubitroMessageLength))/1024.0);

   lastQubitroUploadAt=millis();
}

void buildUplinkTelemetryMessageV7(char* payload)
{
    currentQubitroUploadAt=millis();
    qubitroUploadDutyCycle=currentQubitroUploadAt-lastQubitroUploadAt;
    uint32_t live_metrics_count = 75; // as of 9 May 20:16
    sprintf(payload,
          "{\"UTC time\":\"%02d:%02d:%02d\",\"UTC date\":\"%02d:%02d:%02d\",\"lemon_on_seconds\":%lu,\"coordinates\":[%f,%f],\"depth\":%f,"
          "\"water_pressure\":%f,\"water_temperature\":%f,\"enclosure_temperature\":%f,\"enclosure_humidity\":%f,\"enclosure_air_pressure\":%f,"
          "\"magnetic_heading_compensated\":%f,\"heading_to_target\":%f,\"distance_to_target\":%f,\"journey_course\":%f,\"journey_distance\":%f,"
          "\"mako_screen_display\":%d,\"mako_seconds_on\":%lu,\"mako_user_action\":%d,\"mako_AXP192_temp\":%f,"
          "\"mako_usb_voltage\":%f,\"mako_usb_current\":%f,\"mako_bat_voltage\":%f,\"mako_bat_charge_current\":%f,"
          "\"fix_count\":%lu,\"lemon_usb_voltage\":%f,\"lemon_usb_current\":%f,\"lemon_bat_voltage\":%f,\"lemon_bat_current\":%f,"
          "\"sats\":%lu,\"hdop\":%f,\"gps_course\":%f,\"gps_speed_knots\":%f,"

          "\"mako_lsm_mag_x\":%f,\"mako_lsm_mag_y\":%f,\"mako_lsm_mag_z\":%f,"
          "\"mako_lsm_acc_x\":%f,\"mako_lsm_acc_y\":%f,\"mako_lsm_acc_z\":%f,"
          
          "\"mako_imu_gyro_x\":%f,\"mako_imu_gyro_y\":%f,\"mako_imu_gyro_z\":%f,"
          "\"mako_imu_lin_acc_x\":%f,\"mako_imu_lin_acc_y\":%f,\"mako_imu_lin_acc_z\":%f,"
          "\"mako_imu_rot_acc_x\":%f,\"mako_imu_rot_acc_y\":%f,\"mako_imu_rot_acc_z\":%f,"
          "\"mako_imu_temperature\":%f,"

          "\"lemon_imu_gyro_x\":%f,\"lemon_imu_gyro_y\":%f,\"lemon_imu_gyro_z\":%f,"
          "\"lemon_imu_lin_acc_x\":%f,\"lemon_imu_lin_acc_y\":%f,\"lemon_imu_lin_acc_z\":%f,"
          "\"lemon_imu_rot_acc_x\":%f,\"lemon_imu_rot_acc_y\":%f,\"lemon_imu_rot_acc_z\":%f,"
          "\"lemon_imu_temperature\":%f,"
                    
          "\"mako_waymarker_e\":%d,\"mako_waymarker_label\":\"-\",\"mako_direction_metric\":\"-\",",
          
           gps.time.hour(), gps.time.minute(),gps.time.second(),
            gps.date.day(), gps.date.month(),gps.date.year(),
            millis()/1000/60,
            gps.location.lat(),
            gps.location.lng(),
            depth,
            water_pressure,
            water_temperature,
            enclosure_temperature,
            enclosure_humidity,
            enclosure_air_pressure,
            magnetic_heading_compensated,            
            heading_to_target,
            distance_to_target,
            journey_course,
            journey_distance,
            mako_screen_display,
            mako_seconds_on,
            mako_user_action,
            mako_AXP192_temp,
            mako_usb_voltage,
            mako_usb_current,
            mako_bat_voltage,
            mako_bat_charge_current,

            fixCount,
            M5.Axp.GetVBusVoltage(),
            M5.Axp.GetVBusCurrent(),
            M5.Axp.GetBatVoltage(),
            M5.Axp.GetBatCurrent(),

            satellites,
            gps.hdop.hdop(),
            gps.course.deg(),
            gps.speed.knots(),

            mako_lsm_mag_x,
            mako_lsm_mag_y,
            mako_lsm_mag_z,
            mako_lsm_acc_x,
            mako_lsm_acc_y,
            mako_lsm_acc_z,


            mako_imu_gyro_x,
            mako_imu_gyro_y,
            mako_imu_gyro_z,
            mako_imu_lin_acc_x,
            mako_imu_lin_acc_y,
            mako_imu_lin_acc_z,
            mako_imu_rot_acc_x,
            mako_imu_rot_acc_y,
            mako_imu_rot_acc_z,
            mako_imu_temperature,

            lemon_imu_gyro_x,
            lemon_imu_gyro_y,
            lemon_imu_gyro_z,
            lemon_imu_lin_acc_x,
            lemon_imu_lin_acc_y,
            lemon_imu_lin_acc_z,
            lemon_imu_rot_acc_x,
            lemon_imu_rot_acc_y,
            lemon_imu_rot_acc_z,
            lemon_imu_temperature,

            mako_way_marker_enum,
            mako_way_marker_label,
            mako_direction_metric);

 sprintf(payload+strlen(payload),
          "\"uplink_msgs_from_mako\":%lu,\"uplink_msg_length\":%hu,\"msgs_to_qubitro\":%lu,\"qubitro_msg_length\":%hu,\"KB_to_qubitro\":%.1f,\"KB_uplinked_from_mako\":%.1f,"
          "\"live_metrics\":%lu,\"qubitro_upload_duty_cycle\":%lu,\"geo_location\":\"Gozo, Malta\""
          "}",
           goodUplinkMessageCount,
            uplinkMessageLength,
            qubitroUploadCount,
            qubitroMessageLength,
            KB_to_Qubitro,
            KB_from_mako,
            live_metrics_count,
            qubitroUploadDutyCycle
            );

   qubitroMessageLength=strlen(payload);
   KB_to_Qubitro=KB_from_mako+(((float)(qubitroMessageLength))/1024.0);

   lastQubitroUploadAt=millis();

   USB_SERIAL.println(payload);
}
void buildBasicTelemetryMessage(char* payload)
{
  sprintf(payload, "{\"lat\":%f,\"lng\":%f}",  gps.location.lat(), gps.location.lng());                  
}

bool uploadTelemetryToQubitro()
{
  bool success = false;

  if (millis() < last_qubitro_upload + qubitro_upload_duty_ms)
  {
    return true;
  }
  else
  {
    last_qubitro_upload = millis();
  }

  if (enableConnectToQubitro)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      if (qubitro_mqttClient.connectError() == SUCCESS)
      {
//        buildUplinkTelemetryMessageV4(qubitro_payload);
        buildUplinkTelemetryMessageV6(qubitro_payload);
//        USB_SERIAL.println(qubitro_payload);
    
        qubitro_mqttClient.poll();
        qubitro_mqttClient.beginMessage(qubitro_device_id);
        qubitro_mqttClient.print(qubitro_payload);
        int endMessageResult = qubitro_mqttClient.endMessage();
        if (endMessageResult == 1)
        {
          success = true;       
//          USB_SERIAL.printf("Qubitro Client sent message %s\n", qubitro_payload);
          qubitroUploadCount++;
        }
        else
        {
  //        USB_SERIAL.printf("Qubitro Client failed to send message, EndMessage error: %d\n", endMessageResult);
        }
      }
      else
      {
        USB_SERIAL.printf("Qubitro Client error status %d\n", qubitro_mqttClient.connectError());
      }
    }
    else
    {
       USB_SERIAL.println("Q No Wifi\n"); 
    }
  }
  else
  {  
    USB_SERIAL.println("Q Not On\n");
  }

  return success;
}

#endif


#ifdef ENABLE_SMTP_AT_COMPILE_TIME
void sendTestByEmail()
{
  ESP_Mail_Session session;

  session.server.host_name = SMTP_SERVER ;
  session.server.port = SMTP_PORT;
  session.login.email = SENDER_EMAIL;
  session.login.password = SENDER_PASSWORD;
  session.login.user_domain = "";

  if (!smtp.connect(&session))
  {
    USB_SERIAL.println("Error connecting to SMTP, " + smtp.errorReason());  
    return;
  }

  SMTP_Message emailMessage;

  emailMessage.sender.name = "Mercator Origins";
  emailMessage.sender.email = SENDER_EMAIL;
  emailMessage.subject = "Mercator Origins Test Email";
  emailMessage.addRecipient("BluepadLabs",RECIPIENT_EMAIL);

   //Send HTML message
  String htmlMsg = "<div style=\"color:#FF0000;\"><h1>Hello Bluepad Labs!</h1><p>This is a test email from Mercator Origins.</p></div>";
  emailMessage.html.content = htmlMsg.c_str();
  emailMessage.html.content = htmlMsg.c_str();
  emailMessage.text.charSet = "us-ascii";
  emailMessage.html.transfer_encoding = Content_Transfer_Encoding::enc_7bit; 

  if (!MailClient.sendMail(&smtp, &emailMessage))
    USB_SERIAL.println("Error sending Email, " + smtp.errorReason());  
}

void sendLocationByEmail()
{
  ESP_Mail_Session session;

  session.server.host_name = SMTP_SERVER ;
  session.server.port = SMTP_PORT;
  session.login.email = SENDER_EMAIL;
  session.login.password = SENDER_PASSWORD;
  session.login.user_domain = "";

  if (!smtp.connect(&session))
  {
    USB_SERIAL.println("Error connecting to SMTP, " + smtp.errorReason());  
    return;
  }
  else
  {
    USB_SERIAL.println("Connected to SMTP Ok");  
  }
  SMTP_Message emailMessage;

  emailMessage.sender.name = "Mercator Origins";
  emailMessage.sender.email = SENDER_EMAIL;
  emailMessage.subject = "Mercator Origins Location Fix";
  emailMessage.addRecipient("BluepadLabs",RECIPIENT_EMAIL);
  
   //Send HTML message
  String htmlMsg = "<div style=\"color:#FF0000;\"><h1>Hello BluePad Labs!</h1><p>This is a location email sent from Mercator Origins</p></div>";
  emailMessage.html.content = htmlMsg.c_str();
  emailMessage.html.content = htmlMsg.c_str();
  emailMessage.text.charSet = "us-ascii";
  emailMessage.html.transfer_encoding = Content_Transfer_Encoding::enc_7bit; 

  if (!MailClient.sendMail(&smtp, &emailMessage))
    USB_SERIAL.println("Error sending Email, " + smtp.errorReason());  
  else
    USB_SERIAL.println("Error sending Email, " + smtp.errorReason());  
  
}
#endif
