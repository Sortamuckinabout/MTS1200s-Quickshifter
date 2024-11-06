

#include <FS.h>
#include <WiFi.h>                                     // needed to connect to WiFi
#include <ESPAsyncWebSrv.h>                        // needed to create a simple webserver (make sure tools -> board is set to ESP32, otherwise you will get a "WebServer.h: No such file or directory" error)
#include <WebSocketsServer.h>                         // needed for instant communication between client and server through Websockets
#include <ArduinoJson.h>                              // needed for JSON encapsulation (send multiple variables with one string)
#include <SPIFFS.h>
// SSID and password of Wifi connection:
const char* ssid = "whateveryoucallit";
const char* password = "yourpassword";

//configure IP adresses of the local access point
IPAddress local_IP(192,168,1,22);
IPAddress gateway(192,168,1,5);
IPAddress subnet(255,255,255,0);
int interval = 500;                                  // send data to the client every 1000ms -> 1s changed to 500ms ok with speed
unsigned long previousMillis = 0;                    
const int samples = 1000;
AsyncWebServer server(80);                                 // the server uses port 80 (standard port for websites
WebSocketsServer webSocket = WebSocketsServer(81);    // the websocket uses port 81 (standard port for websockets
//------------------------------------------------------------------------------------------

#include "driver/twai.h"
#define RX_PIN 22
#define TX_PIN 21
#define TWAI_STD_ID_MASK 
#define POLLING_RATE_MS 10//0//00 changed from 1000ms data transmitted at 20Hz (every 50ms) for TPS,RPM,GEAR

//secondary var for JSONstring
  uint16_t rpm;
  //uint16_t rearwheelspeed;
  uint8_t Tps;
  int8_t gear;
//----------------------------------------------------------------------------
   unsigned long RPM ;
    unsigned int Gear;
    unsigned int Killtime ;
    unsigned int TPS ;
    unsigned long Speed ;
    unsigned long sensorV;
    unsigned long TriggerV ;
    unsigned long DownV ;
uint16_t SparkInterupt;
float calibValue;
float measurements;
float sensorValue;
float Time;
String CAN_ALERT;
String CAN_alert;
float DownsensorSetpointValue;//shift down raises mV value
float UpsensorSetpointValue;//Shift Up lower mV value (current only)
float UpsensorSetpointValueF;//decrease sensitivity at high throttle opening by 25%
unsigned long QSsensormV;
int recalinterval = 30000;
float RunTime;
float previousRunTime;
 uint16_t KillTime ;  
 uint16_t MaxKillTime= 240;//works for all at low speed
 uint16_t MinKillTime= 60; 
//added test below for 1st gear to allow full time range of 4 cycles.
//int KDelay =0 ;      
uint32_t sensitivity =800;
///int NCycles =4 ;  removed as not needed in calcs fixed at 4 engine revolutions works well
int Killpin=2;
int Shiftsensepin=32;
//--------------------------------------------------------------------------
static bool driver_installed = false;

void setup() {
  Serial.begin(115200);
Serial.println("Driver installed");
  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);//switch to normal mode to blip or testing on bench otherwise listen only
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  //Look in the api-reference for other speed sets.
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
   
    Serial.println("DQSv4CANBUS_07022024_4cycle_final_copy_20240722.ino");
  } else {
    Serial.println("Failed to install driver");
    return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("Driver started");
  } else {
    Serial.println("Failed to start driver");
    return;
  }

  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    Serial.println("CAN Alerts reconfigured");

      } else {
        Serial.println("Failed to reconfigure alerts");
      
        return;
      }

      // TWAI driver is now successfully installed and started
      driver_installed = true;

      //------------------------------------------------------holds http page-----
    if(! SPIFFS.begin()){
          Serial.println("SPIFFS could not initialize");
          }
          Serial.print("Setting up Access Point ... ");
          Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");

          Serial.print("Starting Access Point ... ");
          Serial.println(WiFi.softAP(ssid, password) ? "Ready" : "Failed!");

          Serial.print("IP address = ");
          Serial.println(WiFi.softAPIP());                   // show IP address that the ESP32 has received from router
                    
      
          server.on("/",HTTP_GET,[](AsyncWebServerRequest *request) {    // define here wat the webserver needs to do
          request->send(SPIFFS, "/index.html","text/html");           //    -> it needs to send out the HTML string "webpage" to the client
          });
                              
      server.serveStatic("/",SPIFFS,"/");
      webSocket.begin();// start websocket
      server.begin();
    
        pinMode(Killpin, OUTPUT); // onboard led and output to kill solenoid/IGBT's
        digitalWrite(Killpin, HIGH); //initialise with QS inactive during startup
        pinMode(Shiftsensepin,INPUT);//initialise the gear lever sensor 
    //gear lever sensor calibration - average of 1000 samples to constant level at rest mV
    for (int i = 0; i < samples; i++)
      {
          measurements = measurements + float(analogRead(Shiftsensepin));
      }
      measurements = (measurements / samples);     

        delay(5000);

        UpsensorSetpointValue =measurements-sensitivity; //shiftpoint limits determined after calibration
        DownsensorSetpointValue=measurements+sensitivity;
      
    }


    void loop() {
      //--------------------------------------------------------------------------------------------
        KillTime = 65;  //default value at the begining of the loop which will be modified by rpm and Gear IF reading make sense (test routines)

      if (!driver_installed) { //driver failure will stop program
        delay(1000);
        return;
        }

      // Check if alert happened
      uint32_t alerts_triggered;
      twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
      twai_status_info_t twaistatus;
      twai_get_status_info(&twaistatus);

      // Handle alerts
      if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
        Serial.println("Alert: TWAI controller has become error passive.");
      CAN_ALERT="Alert: error passive.";
        }
      if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
        Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
        Serial.printf("Bus error count: %d\n", twaistatus.bus_error_count);
        CAN_ALERT="Alert: error has occurred on the bus.";
        }
      if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
      Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
        Serial.printf("RX buffered: %d\t", twaistatus.msgs_to_rx);
      Serial.printf("RX missed: %d\t", twaistatus.rx_missed_count);
        Serial.printf("RX overrun %d\n", twaistatus.rx_overrun_count);
      CAN_ALERT="Alert queue is full ";
        }
        

        // Check if message is received
      if (alerts_triggered & TWAI_ALERT_RX_DATA) {
          twai_message_t message;
          
      while (twai_receive(&message, 0) == ESP_OK) {
          handle_rx_message(message);   //Subroutine looks for the two messages and runs the KillTime calc at the same time as monitoring the shift sensor
          
          }
        } 
    }
    //-----------------------------------------------------------------------------------------------------------------------------
    static void handle_rx_message(twai_message_t& message) {
    switch (message.identifier) {
      case 0x18:
        gear = ((message.data[4]));// /16))/2;//  removed the -1 as showing one gear less !//gear = ((message.data[4] /16)-1)/2;
          //Serial.print("Gear   ");
         // Serial.print(gear);
       // Serial.print( "  ");
          Gear = gear;
          // rearwheelspeed =  ((((message.data[4]%16)*256) + message.data[5])*0.15)/0.625;//kmperhr
          // Serial.print( " RearWheel SPeed ");
          // Serial.println  (rearwheelspeed);
       // Speed =rearwheelspeed;
    break;
  case 0x80:
    Tps = message.data[7];
        // Serial.print("TPS    ");
       // Serial.print(Tps);
       //  Serial.print( "  ");
         TPS =Tps;
      rpm = (message.data[5])*256 +(message.data[6]);
     // Serial.print("RPM    ");
     // Serial.println(rpm);
      RPM = rpm;
    break;
  default:  //essentially the main loop runs when any other CANBUS message is in the buffer
  //--------------------------------------------------------------------------------------------------
KillTime= ((1000/(rpm/60)*8)) ;
if (Tps>=51){UpsensorSetpointValueF=UpsensorSetpointValue-200;}
if(TPS<=50){UpsensorSetpointValueF=UpsensorSetpointValue;}

// if  (Tps>60){KillTime= ((1000/(rpm/60)*8)+(gear)) ;} // and rpm is used to set 4 cycles (revolutions of the engine)
 //if (Tps<60){KillTime= ((1000/(rpm/60)*8)) ;}

 // test if max and min limits for killtime have been reached
  if (KillTime < MinKillTime ) { KillTime =MinKillTime; }
  if (KillTime > MaxKillTime ) { KillTime =MaxKillTime; }
//*****************************************************************************
 //print to console for debug
      //Serial.print ("KillTime   ");
     // Serial.println(KillTime);
       SparkInterupt = KillTime;
       Killtime = SparkInterupt;
        //read the strain gauge sensor
       sensorValue = float(analogRead(Shiftsensepin)); 
      // Serial.print ("Sensormv  ");
     //  Serial.println(sensorValue);  
        QSsensormV=sensorValue;
//************place the interupt here******************************

    if (sensorValue < UpsensorSetpointValueF)  //UPSHIFT trigger initiated by low reading from sensor  
     {
        digitalWrite(Killpin,LOW);  // turn the LED on and interupt ignition.
        delay(KillTime);                      
        digitalWrite(Killpin, HIGH);    //resume ignition
        CAN_ALERT = "UPshift detected";
        delay (1500);
       }
 //-------- blip actions here------------------------------------------------------------------------------

       if (sensorValue > DownsensorSetpointValue)  //DOWNSHIFT trigger initiated by low reading from sensor  
         {

        CAN_ALERT = "DOWNshift detected";
        delay(1000); 
         }

    else {
      digitalWrite(Killpin,HIGH);
      }  //no iterupt

  //------------------------------------------recalibrate loop---------------------- 
          RunTime = millis();
  if ((RunTime - previousRunTime) > recalinterval)
    {
   CAN_ALERT="recalibrated";

   measurements =1;
      for (int i = 0; i < samples; i++)
    {
      measurements = measurements + float(analogRead(32));
    }
   measurements = (measurements / samples);     
    UpsensorSetpointValue =measurements-sensitivity; //shiftpoint limits determined after calibration
    DownsensorSetpointValue=measurements+sensitivity;
      
     previousRunTime  = RunTime;
    }


  //---------------------------------------------------------------------------------
webSocket.loop(); 
   TriggerV = UpsensorSetpointValue;
   DownV = DownsensorSetpointValue;
   sensorV =  QSsensormV;

    Time = millis();  //appears on the screen as a visual the program has not stopped or hung up
    CAN_alert =CAN_ALERT;  //either error message or upshift detection send to screen
    unsigned long now = millis();// read out the current "time" ("millis()" gives the time in ms since the Arduino started)                       
   if ((unsigned long)(now - previousMillis) > interval)
    { // check if "interval" ms has passed since last time the clients were updated
 
    String jsonString = "";                           // create a JSON string for sending data to the client                
    StaticJsonDocument<500> doc;                      // create a JSON container
    JsonObject object = doc.to<JsonObject>();         // create a JSON Object
    object["RPM"] = RPM; // write data into the JSON object -> 
    object["Gear"] = Gear;
    object["TPS"] = TPS;
    object["CAN_alert"] = CAN_alert;     //send to web page if a can failure has occurred and what it is. 
    object["Time"] = Time;
    object["sensorV"] = sensorV;
    object["TriggerV"] = TriggerV;
    object["DownV"] = DownV;
    object["Killtime"] = Killtime;
   
 
    serializeJson(doc, jsonString);                   // convert JSON object to string
     Serial.println(jsonString);                   
    webSocket.broadcastTXT(jsonString);  
     previousMillis = now;    
   } // end of websocket loop
   CAN_ALERT=" ";
    break;
 }
    
}
