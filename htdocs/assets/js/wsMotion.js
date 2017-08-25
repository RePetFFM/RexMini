var ws;

var rexminiData;

function onLoad() {
		ws = new WebSocket("ws://rexmini:9090/websocket");

		ws.onmessage = function(e) {
			
			tmpStr = e.data.replace(/\n/g, "<br />");
			if(tmpStr!='') {
				rexminiData = tmpStr.split(" ");	
			}
			
		};

}


function setpoint() {
	tmp_setpoint = document.getElementById('setpoint').value;
	if(document.getElementById('pidSelection').value==0) {
		ws.send('00sg'+tmp_setpoint);	
	}

	if(document.getElementById('pidSelection').value==1) {
		ws.send('01sg'+tmp_setpoint);	
	}
}


function setpointRight(value) {
	ws.send('00sg'+value);
}

function setpointLeft(value) {
	ws.send('01sg'+value);
}
