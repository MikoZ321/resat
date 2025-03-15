var ws = new WebSocket("ws://" + window.location.host + "/ws");

ws.onmessage = function(event) {
    document.getElementById("loradata").innerText = event.data;
};