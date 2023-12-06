
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>


const char* ssid     = "David"; // Change this to your WiFi SSID
const char* password = "pepecine"; // Change this to your WiFi password
const int httpPort = 5000;
const String IP = "192.168.32.78";
const String serverName ="http://"+IP+":"+httpPort+"/start";


int POST_Data(String data_to_send);
int GET_Data(String data_to_send);
int connect_to_wifi(String ssid,String password);



int POST_Data(String data_to_send){
  if(WiFi.status()== WL_CONNECTED){
    HTTPClient http;
    http.begin(serverName);

    // If you need Node-RED/server authentication, insert user and password below
    //http.setAuthorization("REPLACE_WITH_SERVER_USERNAME", "REPLACE_WITH_SERVER_PASSWORD");

    // Specify content-type header
    http.addHeader("Content-Type", "application/json");

    // Data to send with HTTP POST

    // Send HTTP POST request
    int httpResponseCode = http.POST(data_to_send);

    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
          
    // Free resources
    http.end();
  }  
  return 0;
}
int GET_Data(String data_to_send){
  // Send HTTP GET request
    //Send an HTTP POST request every 10 minutes
    //Check WiFi connection status
  if(WiFi.status()== WL_CONNECTED){
    HTTPClient http;

    String serverPath = serverName + "?"+data_to_send;
    
    // Your Domain name with URL path or IP address with path
    http.begin(serverPath.c_str());
    
    
    // Send HTTP GET request
    int httpResponseCode = http.GET();
    
    if (httpResponseCode>0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      String payload = http.getString();
      Serial.println(payload);
    }
    else {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
    }
    // Free resources
    http.end();
  }
  else {
    Serial.println("WiFi Disconnected");
    return 1;
  }
  return 0;
}


int connect_to_wifi(String ssid,String password){
  Serial.println();
  Serial.println("******************************************************");
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  int i = 0;
  while (WiFi.status() != WL_CONNECTED) {
    i++;
    delay(500);
    Serial.print(".");
    if(i%5==1){ // Every 5 seconds passed without connection
      switch (WiFi.status()){
      case WL_NO_SHIELD:
        Serial.println("No shield");
        break;
      case WL_IDLE_STATUS:
        Serial.println("Idle");
        break;
      case WL_CONNECT_FAILED:
        Serial.println("Connection failed");
        return 1;
        break;
      case WL_NO_SSID_AVAIL:
        Serial.println("WiFi not found");
        break;
      case WL_SCAN_COMPLETED:
        Serial.println("Scan completed");
      break;
      }
    }
    
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  return 0;
}
