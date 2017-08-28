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


function setpointRight(value) {
	ws.send('00sg'+value);
}

function setpointLeft(value) {
	ws.send('01sg'+value);
}
