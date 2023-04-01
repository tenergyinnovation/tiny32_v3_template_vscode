#include <WiFi.h>
#include <WiFiClient.h>
#include <Webserver.h>
#include <ESPmDNS.h>
#include <Update.h>

#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* host = "tenergy-iot"; //<-- url for update ota firmware
WebServer serverOTA(8888);

/*
 * Login page
 */

const char* loginIndex = 
 "<form name='loginForm'>"
    "<table width='20%' bgcolor='FFCC00' align='center'>"
        "<tr>"
            "<td colspan=2>"
                "<center><font size=4><b>Tenergy-IoT Login Page</b></font></center>"
                "<br>"
            "</td>"
            "<br>"
            "<br>"
        "</tr>"
        "<td>Username:</td>"
        "<td><input type='text' size=25 name='userid'><br></td>"
        "</tr>"
        "<br>"
        "<br>"
        "<tr>"
            "<td>Password:</td>"
            "<td><input type='Password' size=25 name='pwd'><br></td>"   
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
            "<td><input type='submit' onclick='check(this.form)' value='Login'></td>"
        "</tr>"
    "</table>"
"</form>"
"<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=='user' && form.pwd.value=='password')"  //change user and password for login here
    "{"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Error Password or Username')/*displays error message*/"
    "}"
    "}"
"</script>";
 
/*
 * Server Index Page
 */
 
const char* serverIndex = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Update'>"
    "</form>"
 "<div id='prg'>progress: 0%</div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')" 
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";

 /***********************************************************************
 * FUNCTION:    otaUpdateFunction
 * DESCRIPTION: update OTA via webbrowser with file.bin
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
 void otaUpdateFunction(void)
 {
       /*use mdns for host name resolution*/
  if (!MDNS.begin(host)) { //http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  /*return index page which is stored in serverIndex */
  serverOTA.on("/", HTTP_GET, []() {
    serverOTA.sendHeader("Connection", "close");
    serverOTA.send(200, "text/html", loginIndex);
  });
  serverOTA.on("/serverIndex", HTTP_GET, []() {
    serverOTA.sendHeader("Connection", "close");
    serverOTA.send(200, "text/html", serverIndex);
  });
  /*handling uploading firmware file */
  serverOTA.on("/update", HTTP_POST, []() {
    serverOTA.sendHeader("Connection", "close");
    serverOTA.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = serverOTA.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  serverOTA.begin();
//   serverOTA.handleClient(); //run this command in loop
 }


 /***********************************************************************
 * FUNCTION:    otaUpdateCode
 * DESCRIPTION: update OTA via platform io or Arduino IDE
 * PARAMETERS:  nothing
 * RETURNED:    nothing
 ***********************************************************************/
 void otaUpdateCode(void)
 {
     // หากต้องการเพิ่มเติมคำสั่งอื่นๆ สามารถลบ Comment ออกได้
  // ตั้งค่าพอร์ตเป็นพอร์ต 3232
  // ArduinoOTA.setPort(3232);

  // ตั้งค่า Hostname เป็น esp3232-[MAC]
   ArduinoOTA.setHostname("user");

  // ไม่มีการตรวจสอบความปลอดภัย
   ArduinoOTA.setPassword("password");

  // สามารถตั้งค่า Password ได้ (ใช้การเข้ารหัส MD5 ดีที่สุด)
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

// ส่วนของ OTA
  ArduinoOTA
    .onStart([]() {
      String type;          // ประเภทของ OTA ที่เข้ามา
      if (ArduinoOTA.getCommand() == U_FLASH)         // แบบ U_FLASH
        type = "sketch";
      else          // แบบ U_SPIFFS
        type = "filesystem";

      // NOTE: ถ้าใช้เป็นแบบ SPIFFS อาจใช้คำสั่ง SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })

    // เริ่มทำงาน (รับข้อมูลโปรแกรม) พร้อมแสดงความคืบหน้าทาง Serial Monitor
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })

    // แสดงข้อความต่างๆหากเกิด Error ขึ้น
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

//   ArduinoOTA.handle(); //run this command in loop
 }