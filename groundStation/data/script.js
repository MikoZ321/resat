var ws = new WebSocket("ws://" + window.location.host + "/ws");

ws.onmessage = function(event) {
    const data = JSON.parse(event.data);
    document.getElementById("temperature").innerText = data.temperature;
    let seconds = int(data.time) / 1000.0;

    if (seconds < 60) {
        document.getElementById("runtime").innerText = `${seconds} s`;
    }
    else {
        let minutes = (seconds - (seconds % 60)) / 60;
        seconds = seconds % 60;
        document.getElementById("runtime").innerText = `${minutes} min ${seconds} s`;

    }
    document.getElementById("pressure").innerText = data.pressure;
    document.getElementById("humidity").innerText = data.humidity;
    document.getElementById("altitude").innerText = data.altitudeGPS;
    document.getElementById("height").innerText = data.height;
    document.getElementById("latitude").innerText = data.latitude;
    document.getElementById("longitude").innerText = data.longitude;
    document.getElementById("batteryVoltage").innerText = data.batteryVoltage;
    document.getElementById("motorOutputVoltage").innerText = data.motorOutputVoltage;
    document.getElementById("gyroX").innerText = data.gyroX;
    document.getElementById("gyroY").innerText = data.gyroY;
    document.getElementById("gyroZ").innerText = data.gyroZ;
    document.getElementById("accelX").innerText = data.accelX;
    document.getElementById("accelY").innerText = data.accelY;
    document.getElementById("accelZ").innerText = data.accelZ;
    document.getElementById("angularSpeed").innerText = data.angularSpeed;
    document.getElementById("rssi").innerText = data.rssi;
};