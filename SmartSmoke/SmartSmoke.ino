#include <FS.h> //this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h> //https://github.com/esp8266/Arduino

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson
#include <PubSubClient.h>

#define IOPIN 12
#define TESTPIN 13
#define RESETPIN 5

//define your default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40] = "192.168.0.118";
char mqtt_port[6] = "1883";
char pub_topic[34] = "/feeds/smoke";

//flag for saving data
bool shouldSaveConfig = false;

bool smokealarm = false;
bool testalarm = false;
bool reset = false;

//mqtt
WiFiClient espClient;
PubSubClient client(espClient);
char msg[50];

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(pub_topic, "alive");
    }
    else
    {
      Serial.print("failed to connect to MQTT broker, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// Deep-sleep for specified amount of hours, one hour at a time.
// If powered on (not a deep-sleep reset), nothing will happen.
// Call this twice: in the beginning of setup (end_of_setup == false)
// and at the end of setup (end_of_setup == true).
void deepSleepCycle(uint32_t hours, bool end_of_setup = false)
{

  uint32_t reset_counter = 0;
  bool waking_from_sleep = ESP.getResetReason() == "Deep-Sleep Wake";

  if (smokealarm)
    waking_from_sleep = false;

  if (!end_of_setup)
  {
    if (waking_from_sleep)
    {
      Serial.print("Waking up from deep-sleep via reset pin. Reset counter: ");
      ESP.rtcUserMemoryRead(0, &reset_counter, sizeof(reset_counter));
      reset_counter++;
      ESP.rtcUserMemoryWrite(0, &reset_counter, sizeof(reset_counter));
      Serial.println(String(reset_counter));
    }
    else
    {
      Serial.println("Zeroing reset counter.");
      ESP.rtcUserMemoryWrite(0, &reset_counter, sizeof(reset_counter));
      return;
    }
  }

  // With larger values, deep-sleep is unrealiable: it might never wake up and consume a lot of power.
  // Therefore sleep one hour at a time.
  // In reality, the ESP sleeps a bit less than the 60 minutes it is told to.
  if (reset_counter < hours)
  {
    // If this is the first time going to sleep, do the radio calibration once.
    // Otherwise, disable radio (WiFi).
    RFMode wake_mode = waking_from_sleep ? WAKE_RF_DISABLED : WAKE_RFCAL;
    if (reset_counter + 1 == hours)
    {
      // Wake up with radio on if the next power cycle finishes sleeping.
      wake_mode = WAKE_NO_RFCAL;
    }
    //Serial.println("Going to deep-sleep for 1 hour.");
    // 1: WAKE_RFCAL
    // 2: WAKE_NO_RFCAL
    // 4: WAKE_RF_DISABLED
    //Serial.println("Radio mode will be: " + String(wake_mode));
    //ESP.deepSleep(3600*1e6, wake_mode);
    ESP.deepSleep(3600 * 1e6);
  }
  reset_counter = 0;
  ESP.rtcUserMemoryWrite(0, &reset_counter, sizeof(reset_counter));
}

//callback notifying us of the need to save config
void saveConfigCallback()
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

int readbatterylevel()
{
  unsigned int adc = analogRead(A0);
  int batpercent = map(adc, 560, 820, 0, 100);
  return batpercent;
}

void setup()
{

  //Setup and read inputs
  pinMode(IOPIN, INPUT);
  pinMode(TESTPIN, INPUT);
  pinMode(RESETPIN, INPUT_PULLUP);
  testalarm = digitalRead(TESTPIN);
  smokealarm = digitalRead(IOPIN);
  //reset = digitalRead(RESETPIN);
  if (testalarm && !digitalRead(RESETPIN))
    reset = true;

  deepSleepCycle(24);

  Serial.begin(115200);
  Serial.println(testalarm);
  Serial.println(smokealarm);
  Serial.println(reset);

  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin())
  {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json"))
    {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile)
      {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject &json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success())
        {
          Serial.println("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(pub_topic, json["pub_topic"]);
        }
        else
        {
          Serial.println("failed to load json config");
        }
        configFile.close();
      }
    }
  }
  else
  {
    Serial.println("failed to mount FS");
  }
  //end read

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_pub_topic("topic", "publish topic", pub_topic, 32);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
  //wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_pub_topic);

  //reset settings - for testing
  //wifiManager.resetSettings();

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);

  if (reset)
  {
    Serial.println("Resetting WiFi settings.");
    WiFi.disconnect();
    delay(1000);
  }

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("SmartSmokeAP", "password"))
  {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...:)");

  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(pub_topic, custom_pub_topic.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig)
  {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject &json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["pub_topic"] = pub_topic;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile)
    {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, atoi(mqtt_port));

  if (!client.connected())
  {
    reconnect();
  }

  if (!reset)
  {
    if (testalarm)
    {
      snprintf(msg, 50, "TestAlarm");
      Serial.print("Publish message: ");
      Serial.println(msg);
      client.publish(pub_topic, msg);
      delay(1000);
    }
    else if (smokealarm)
    {
        snprintf(msg, 50, "SmokeAlarm");
        Serial.print("Publish message: ");
        Serial.println(msg);
        client.publish(pub_topic, msg);
        delay(1000);
      while (digitalRead(IOPIN))
      {
        if (!client.connected())
        {
          reconnect();
        }
        snprintf(msg, 50, "SmokeAlarm");
        Serial.print("Publish message: ");
        Serial.println(msg);
        client.publish(pub_topic, msg);
        delay(1000);
      }
    }
  }

  snprintf(msg, 50, "Battery:%ld", readbatterylevel());
  Serial.print("Publish message: ");
  Serial.println(msg);
  client.publish(pub_topic, msg);
  delay(1000);

  deepSleepCycle(24, true);

  /*  
  Serial.println("\nGoing to sleep");
  ESP.deepSleep(60e6);
  delay(1000);
  Serial.println("\nSleep unsuccessful");
*/
}

void loop()
{
  // put your main code here, to run repeatedly:
  Serial.println(readbatterylevel());
  delay(1000);
}
