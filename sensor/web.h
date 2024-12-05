#ifndef WEB_H
#define WEB_H

#include <WebServer.h>
#include <WebSocketsServer.h>

extern WebServer server;
extern WebSocketsServer webSocket;

void handleRoot() {
    server.send(200, "text/html", WEBPAGE);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED:
            Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num);
            break;
        case WStype_TEXT:
            Serial.printf("[%u] Received text: %s\n", num, payload);
            break;
    }
}

void broadcastSensorData();

#endif

const char WEBPAGE[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <title>Sensor Data Display</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      background-color: #222;
      color: #fff;
      text-align: center;
    }
    .dashboard {
      display: flex;
      justify-content: center;
      margin-top: 50px;
    }
    .stats-panel {
      display: flex;
      flex-direction: column;
    }
    .stat-box {
      padding: 20px;
      margin: 10px;
      background-color: #333;
      border-radius: 10px;
    }
    .stat-label {
      font-size: 1.2em;
    }
    .stat-value {
      font-size: 2em;
      font-weight: bold;
      margin-top: 10px;
    }
  </style>
</head>
<body>
  <h1>Sensor Data Dashboard</h1>
  <div class="dashboard">
    <div class="stats-panel">
      <div class="stat-box">
        <div class="stat-label">Front Distance</div>
        <div id="frontDistance" class="stat-value">0 mm</div>
      </div>
      <div class="stat-box">
        <div class="stat-label">Left Distance</div>
        <div id="leftDistance" class="stat-value">0 mm</div>
      </div>
      <div class="stat-box">
        <div class="stat-label">Right Distance</div>
        <div id="rightDistance" class="stat-value">0 mm</div>
      </div>
    </div>
  </div>

  <script>
    const ws = new WebSocket('ws://' + window.location.hostname + ':81/');
    ws.onmessage = function(event) {
      const data = JSON.parse(event.data);
      document.getElementById('frontDistance').textContent = data.front + ' mm';
      document.getElementById('leftDistance').textContent = data.left + ' mm';
      document.getElementById('rightDistance').textContent = data.right + ' mm';
    };
  </script>
</body>
</html>
)=====";