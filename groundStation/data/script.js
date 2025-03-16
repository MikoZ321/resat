var ws = new WebSocket("ws://" + window.location.host + "/ws");

ws.onmessage = function(event) {
    const data = JSON.parse(event.data);
    document.getElementById("temperature").innerText = data.temperature;
    document.getElementById("pressure").innerText = data.pressure;
    document.getElementById("humidity").innerText = data.humidity;
    document.getElementById("altitude").innerText = data.altitudePressure;
    document.getElementById("latitude").innerText = data.latitude;
    document.getElementById("longitude").innerText = data.longitude;
    document.getElementById("lightLevel").innerText = data.lightLevel;
    document.getElementById("batteryVoltage").innerText = data.batteryVoltage;
    document.getElementById("motorOutputVoltage").innerText = data.motorOutputVoltage;
    document.getElementById("gyroX").innerText = data.gyroX;
    document.getElementById("gyroY").innerText = data.gyroY;
    document.getElementById("gyroZ").innerText = data.gyroZ;
    document.getElementById("accelX").innerText = data.accelX;
    document.getElementById("accelY").innerText = data.accelY;
    document.getElementById("accelZ").innerText = data.accelZ;
    document.getElementById("angularSpeed").innerText = data.angularSpeed;
};