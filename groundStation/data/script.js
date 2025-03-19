const BASE_LATITUDE = 37.7749;
const BASE_LONGITUDE = -122.4194;
const ZOOM_LEVEL = 16;

const map = L.map('map').setView([BASE_LATITUDE, BASE_LONGITUDE], ZOOM_LEVEL);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);
const marker = L.marker([BASE_LATITUDE, BASE_LONGITUDE]).addTo(map);

var ws = new WebSocket("ws://" + window.location.host + "/ws");

ws.onmessage = function(event) {
    const data = JSON.parse(event.data);
    document.getElementById("temperature").innerText = `${data.temperature}°C`;
    let seconds = Number(data.time) / 1000.0;

    if (seconds < 60) {
        document.getElementById("runtime").innerText = `${seconds}s`;
    }
    else {
        let minutes = (seconds - (seconds % 60)) / 60;
        seconds = seconds % 60;
        document.getElementById("runtime").innerText = `${minutes}min ${seconds}s`;
    }
    document.getElementById("pressure").innerText = `${data.pressure}hPa`;
    document.getElementById("altitude").innerText = `${data.altitudeGPS}m`;
    document.getElementById("height").innerText = `${data.height}m`;
    document.getElementById("latitude").innerText = data.latitude;
    document.getElementById("longitude").innerText = data.longitude;

    let lat = parseFloat(data.latitude);
    let lon = parseFloat(data.longitude);
    marker.setLatLng([lat, lon]);

    document.getElementById("batteryVoltage").innerText = `${data.batteryVoltage}V`;
    document.getElementById("motorOutputVoltage").innerText = `${data.motorOutputVoltage}V`;
    document.getElementById("gyroX").innerText = data.gyroX;
    document.getElementById("gyroY").innerText = data.gyroY;
    document.getElementById("gyroZ").innerText = data.gyroZ;
    document.getElementById("accelX").innerText = data.accelX;
    document.getElementById("accelY").innerText = data.accelY;
    document.getElementById("accelZ").innerText = data.accelZ;
    document.getElementById("angularSpeed").innerText = `${data.angularSpeed}RPM`;
    document.getElementById("rssi").innerText = data.rssi;
    document.getElementById("descentRate").innerText = `${data.descentRate}m/s`;
    document.getElementById("distance").innerText = `${data.distance}m`;
};