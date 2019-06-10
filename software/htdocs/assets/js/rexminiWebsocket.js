var ws;
var rexminiData;
var c;
var ctx;
var canvasBuffer;
var ctxBuffer;

function onLoad() {
	ws = new WebSocket("ws://rexmini:9090/websocket");

	ws.onmessage = function(e) {
		if(e.data!='') {
			rexminiTelemetrieParser(e.data);	
		}
	};
	c=document.getElementById("ccdoutput");
	c.width = 130;
	c.height = 100;
	ctx=c.getContext("2d");
	

	canvasBuffer = document.createElement("canvas");
	canvasBuffer.width = c.width;
	canvasBuffer.height = c.height;
	 
	ctxBuffer = canvasBuffer.getContext("2d");

}


function rexminiTelemetrieParser(data) {
	type = data.substring(0, 2);
	data = data.substring(2);
	switch (type) {
		case 'cl':
			scanData = data.split(" ");
			// console.log(scanData[3]);
			// console.log(scanData[0].length);
			if(scanData[0].length==260) {
				
				ctx.beginPath();
				ctx.strokeStyle="#FF0000";
				ctx.moveTo(64,0);
				ctx.lineTo(64,2);
				ctx.stroke();

				scanWidth = parseInt(scanData[1]);

				ctx.beginPath();
				ctx.strokeStyle="#00FF00";
				ctx.moveTo(64-scanWidth,0);
				ctx.lineTo(64-scanWidth,2);
				ctx.stroke();

				ctx.beginPath();
				ctx.strokeStyle="#00FF00";
				ctx.moveTo(64+scanWidth,0);
				ctx.lineTo(64+scanWidth,2);
				ctx.stroke();

				lineCenter = parseInt(scanData[2]);
				lineWidth = parseInt(scanData[3]);
				lineDetected = parseInt(scanData[4]);
				lineKnownWidth = parseInt(scanData[5]);

				$('#linedata').html("lineDetected: "+lineDetected+" linecenter: "+lineCenter+" lineWidth: "+lineWidth+" lineKnownWidth: "+lineKnownWidth);

				ctx.beginPath();
				ctx.strokeStyle="#0000FF";
				ctx.moveTo(lineCenter,0);
				ctx.lineTo(lineCenter,2);
				ctx.stroke();


				console.log(data);	
				for(i=0; i<130; i++) {
					color = scanData[0].substring(i*2,(i*2)+1);
					ctx.beginPath();
					ctx.strokeStyle="#"+color+color+color;
					ctx.moveTo(i,0);
					ctx.lineTo(i,1);
					ctx.stroke();
				}
			}
			translate(0, 2);

			

			// console.log(scanData[1]);
		break; 

		case '0s':
			robotstatus = data.split(" ");
			// console.log(robotstatus);
			$('#rexmode').html("mode: "+robotstatus[0]);

			battery_voltage = (robotstatus[1]/5000);
			
			batprogvalue = ((battery_voltage-6)/2)*100;
			batprogvalue>100 ? batprogvalue = 100 : false;
			$('.progress-bar').css('width', batprogvalue+'%').attr('aria-valuenow', batprogvalue);
			$('.progress-bar').html(Math.round(batprogvalue)+'%');
		break;
	}
}


function translate(x, y) {
    ctxBuffer.clearRect(0,0,canvasBuffer.width,canvasBuffer.height); //clear buffer
    ctxBuffer.drawImage(c,0,0); //store display data in buffer
    ctx.clearRect(0,0,c.width,c.height); //clear display
    ctx.drawImage(canvasBuffer,x,y); //copy buffer to display
}

function sendSerialCommand(cmd) {
	ws.send(cmd);
}
