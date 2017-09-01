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


function setCurveFactor() {
	if(document.getElementById('setcurve').value>5) {
		ws.send('00sc'+document.getElementById('setcurve').value);	
	}
	
}

function setForwardSpeed() {
	if(document.getElementById('setforward').value>=0 && document.getElementById('setforward').value!='') {
		ws.send('00sf'+document.getElementById('setforward').value);	
	}
	
}

