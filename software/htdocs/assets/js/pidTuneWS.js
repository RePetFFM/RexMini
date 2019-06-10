var ws;

var pidDebugData = [0,0,0,0,0,0];

var rowCounter = 0;

var doReadout = false;

function onLoad() {
		ws = new WebSocket("ws://rexmini:9090/websocket");

		ws.onmessage = function(e) {
			if(rowCounter>20) {
				rowCounter = 0;
				clearDebugOuput();
			}
			document.getElementById('out').innerHTML += e.data.replace(/\n/g, "<br />");
			rowCounter++;

			tmpStr = e.data.replace(/\n/g, "<br />");
			if(tmpStr!='') {
				pidDebugData = tmpStr.split(" ");	
			}
			
		};

		// requestState();
		// setInterval(requestState, 2000);
}

function requestState() {
		ws.send('ff?');
}


function pidReadoutChanged() {
	if(doReadout = document.getElementById('pidreadout').checked) {
		ws.send('ffed0');
		selectedPID = document.getElementById('pidSelection').value;
		selectedPID++;
		ws.send('ffed'+selectedPID);
	} else {
		ws.send('ffed0');
	}
}

function clearDebugOuput() {
	document.getElementById('out').innerHTML = '';
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


function pid_p() {
	tmp = document.getElementById('pid_p').value;
	tmp = Math.round(tmp*10);
	if(document.getElementById('pidSelection').value==0) {
		ws.send('00sp'+tmp);	
	}

	if(document.getElementById('pidSelection').value==1) {
		ws.send('01sp'+tmp);	
	}
}

function pid_i() {
	tmp = document.getElementById('pid_i').value;
	tmp = Math.round(tmp*10);
	if(document.getElementById('pidSelection').value==0) {
		ws.send('00si'+tmp);	
	}

	if(document.getElementById('pidSelection').value==1) {
		ws.send('01si'+tmp);	
	}
}

function pid_d() {
	tmp = document.getElementById('pid_d').value;
	tmp = Math.round(tmp*10);
	if(document.getElementById('pidSelection').value==0) {
		ws.send('00sd'+tmp);	
	}

	if(document.getElementById('pidSelection').value==1) {
		ws.send('01sd'+tmp);	
	}
}

function pid_esum() {
	tmp = document.getElementById('pid_esum').value;
	if(document.getElementById('pidSelection').value==0) {
		ws.send('00se'+tmp);	
	}

	if(document.getElementById('pidSelection').value==1) {
		ws.send('01se'+tmp);	
	}
}