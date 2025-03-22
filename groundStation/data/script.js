const BASE_LATITUDE = 37.7749;
const BASE_LONGITUDE = -122.4194;
const ZOOM_LEVEL = 16;
const CHART_ITEM_MAX = 20;
const MAX_RANGE = 2833;

const map = L.map('map').setView([BASE_LATITUDE, BASE_LONGITUDE], ZOOM_LEVEL);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);
const marker = L.marker([BASE_LATITUDE, BASE_LONGITUDE]).addTo(map);

// Get the canvas contexts
let heightCtx = document.getElementById('heightChart').getContext('2d');
let voltageCtx = document.getElementById('voltageChart').getContext('2d');

// Create the Height vs. Time Chart
let heightChart = new Chart(heightCtx, {
    type: 'line',
    data: {
        labels: [],  // Time labels (X-axis)
        datasets: [{
            label: 'Height (m)',
            data: [],  // AltitudePressure (Y-axis)
            borderColor: 'rgb(75, 192, 192)',
            borderWidth: 2,
            fill: false
        }]
    },
    options: {
        responsive: true,
        scales: {
            x: { title: { display: true, text: 'Time (s)' } },
            y: { title: { display: true, text: 'Height (m)' } }
        }
    }
});

// Create the Angular Speed vs. Voltage Chart
let voltageChart = new Chart(voltageCtx, {
    type: 'scatter',
    data: {
        datasets: [{
            label: 'Voltage vs Angular Speed',
            data: [],  // Points will be { x: angularSpeed, y: voltage }
            borderColor: 'rgb(255, 99, 132)',
            backgroundColor: 'rgba(255, 99, 132, 0.2)',
            borderWidth: 2,
            showLine: true
        }]
    },
    options: {
        responsive: true,
        scales: {
            x: { title: { display: true, text: 'Angular Speed (RPM)' } },
            y: { title: { display: true, text: 'Voltage (V)' } }
        }
    }
});

// Function to update Height vs. Time Chart
function updateHeightChart(time, height) {
    if (heightChart.data.labels.length > CHART_ITEM_MAX) {
        heightChart.data.labels.shift();
        heightChart.data.datasets[0].data.shift();
    }
    
    heightChart.data.labels.push(time);
    heightChart.data.datasets[0].data.push(height);
    heightChart.update();
}

// Function to update Angular Speed vs. Voltage Chart
function updateVoltageChart(angularSpeed, voltage) {
    if (voltageChart.data.datasets[0].data.length > CHART_ITEM_MAX) {
        voltageChart.data.datasets[0].data.shift();
    }

    voltageChart.data.datasets[0].data.push({ x: angularSpeed, y: voltage });
    voltageChart.update();
}

var ws = new WebSocket("ws://" + window.location.host + "/ws");

ws.onmessage = function(event) {
    const data = JSON.parse(event.data);
    document.getElementById("temperature").innerText = `${data.temperature}*C`;
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

    if (data.height) {
        updateHeightChart(parseFloat(seconds), parseFloat(data.height));
    }

    document.getElementById("latitude").innerText = data.latitude;
    document.getElementById("longitude").innerText = data.longitude;

    let lat = parseFloat(data.latitude);
    let lon = parseFloat(data.longitude);
    marker.setLatLng([lat, lon]);

    document.getElementById("batteryVoltage").innerText = `${data.batteryVoltage}V`;
    document.getElementById("motorOutputVoltage").innerText = `${data.motorOutputVoltage}V`;
    document.getElementById("accelX").innerText = data.accelX;
    document.getElementById("accelY").innerText = data.accelY;
    document.getElementById("accelZ").innerText = data.accelZ;
    document.getElementById("angularSpeed").innerText = `${data.angularSpeed}RPM`;

    if (data.angularSpeed && data.motorOutputVoltage) {
        updateVoltageChart(parseFloat(data.angularSpeed), parseFloat(data.motorOutputVoltage));
    }

    document.getElementById("rssi").innerText = data.rssi;
    document.getElementById("descentRate").innerText = `${data.descentRate}m/s`;
    document.getElementById("distance").innerText = `${data.distance}m`;
    document.getElementById("range").innerText = `${MAX_RANGE - Number(data.distance)}m`
};