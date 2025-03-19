const map = L.map('map').setView([37.7749, -122.4194], 13);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);
const marker = L.marker([37.7749, -122.4194]).addTo(map);

var ws = new WebSocket("ws://" + window.location.host + "/ws");

ws.onmessage = function(event) {
    const data = JSON.parse(event.data);
    document.getElementById("temperature").innerText = data.temperature;
    let seconds = Number(data.time) / 1000.0;

    if (seconds < 60) {
        document.getElementById("runtime").innerText = `${seconds} s`;
    }
    else {
        let minutes = (seconds - (seconds % 60)) / 60;
        seconds = seconds % 60;
        document.getElementById("runtime").innerText = `${minutes} min ${seconds} s`;
    }
    document.getElementById("pressure").innerText = data.pressure;
    document.getElementById("altitude").innerText = data.altitudeGPS;
    document.getElementById("height").innerText = data.height;
    document.getElementById("latitude").innerText = data.latitude;
    document.getElementById("longitude").innerText = data.longitude;

    let lat = parseFloat(data.latitude);
    let lon = parseFloat(data.longitude);
    map.setView([lat, lon], 13);
    marker.setLatLng([lat, lon]);

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
    document.getElementById("descentRate").innerText = data.descentRate;
};