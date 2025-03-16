var ws = new WebSocket("ws://" + window.location.host + "/ws");

ws.onmessage = function(event) {
    const data = JSON.parse(event.data);
    document.getElementById("temperature").innerText = data.temperature;
    document.getElementById("pressure").innerText = data.pressure;
    document.getElementById("humidity").innerText = data.humidity;
    document.getElementById("altitude").innerText = data.altitudeGPS;
    document.getElementById("latitude").innerText = data.latitude;
    document.getElementById("longitude").innerText = data.longitude;
    document.getElementById("lightLevel").innerText = data.lightLevel;
    document.getElementById("batteryVoltage").innerText = data.batteryVoltage;
    document.getElementById("motorOutputVoltage").innerText = data.motorOutputVoltage;
    document.getElementById("gyroX").innerText = data.gyro.x;
    document.getElementById("gyroY").innerText = data.gyro.y;
    document.getElementById("gyroZ").innerText = data.gyro.z;
    document.getElementById("accelX").innerText = data.acceleration.x;
    document.getElementById("accelY").innerText = data.acceleration.y;
    document.getElementById("accelZ").innerText = data.acceleration.z;
    document.getElementById("angularSpeed").innerText = data.angularSpeed;
};