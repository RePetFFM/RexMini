var canvas;
var ctx;
var w = 0;
var h = 0;

var timer;
var updateStarted = false;
var touches = [];

var controlBox_x = 0;
var controlBox_y = 0;
var controlBox_phi = 0;

var gyro = '';

var serverFeedback = '';

// ajax variables
var moveUpdate = false;
var moveSent = false;

var moveLastchange_x = 0;
var moveLastchange_y = 0;
var moveLastchange_r = 0;

var counterSent = 0;
var counterSuccess = 0;

var mouseControlOn = false;

var mouse_x = 0;
var mouse_y = 0;


function update() {
	if (updateStarted) return;
	updateStarted = true;

	var nw = window.innerWidth;
	var nh = window.innerHeight;

	if ((w != nw) || (h != nh)) {
		w = nw;
		h = nh;
		canvas.style.width = w+'px';
		canvas.style.height = h+'px';
		canvas.width = w;
		canvas.height = h;
	}

	ctx.clearRect(0, 0, w, h);

	// redrawUI();

	var i, len = touches.length;

	ctx.font = "20px Arial";
	ctx.fillText("Touch len: "+len,30,30);

	xpos = controlBox_x-w/2;
	ypos = controlBox_y-h/2;

	controlBox_phi_txt = Math.round(controlBox_phi*1000)/1000;

	// 

	/*
	var alpha = DeviceRotationRate.alpha;
	var beta = DeviceRotationRate.beta;
	var gamma = DeviceRotationRate.gamma;
	gyro = alpha+' - '+beta+' - '+gammaa;
	*/



	serverSentStatus = 'sent: '+counterSent+' success: '+counterSuccess; 

	ctx.fillText("server sent status: "+serverSentStatus,30,150);

	ctx.fillText("server feedback: "+serverFeedback,30,180);

	noMoveUpdate = true;

	if(mouseControlOn) {
		var px = mouse_x;
		var py = mouse_y;

		ctx.beginPath();
		ctx.arc(px, py, 70, 0, 2*Math.PI, true);

		ctx.fillStyle = "rgba(0, 0, 200, 0.2)";
		ctx.fill();

		ctx.lineWidth = 2.0;
		ctx.strokeStyle = "rgba(0, 0, 200, 0.8)";
		ctx.stroke();

		// if only one touch point
		x1 = px;
		y1 = py;	

		controlBox_x = x1;
		controlBox_y = y1;
		controlBox_phi = 0;

		// draw line from screen center to control pivot
		ctx.strokeStyle = "rgba(0, 200, 0, 1)";
		ctx.beginPath();
		ctx.moveTo(w/2,h/2);
		ctx.lineTo(x1,y1);
		ctx.lineWidth = 2.0;
		ctx.setLineDash([5, 5]);
		ctx.stroke();
		ctx.setLineDash([]);

		xpos = controlBox_x-w/2;
		ypos = controlBox_y-h/2;
		
		ypos = -ypos;
		
		xpos = Math.round(300*(xpos/(w/2)));
		ypos = Math.round(300*(ypos/(h/2)));

		deltaX = moveLastchange_x - xpos;
		deltaY = moveLastchange_y - ypos;
		deltaR = moveLastchange_r - Math.round(controlBox_phi);


		ctx.fillText("x: "+xpos,30,60);
		ctx.fillText("y: "+ypos,30,90);
		ctx.fillText("phi: "+controlBox_phi_txt,30,120);


		// moveChanged(xpos,ypos,controlBox_phi);

		if(Math.abs(deltaX+deltaY+deltaR)>0) {
			moveChanged(xpos,ypos,controlBox_phi);
		};

		noMoveUpdate = false;
	}


	if(len>0) {
		for (i=0; i<len; i++) {
			if(i>1) break; // exit if too many fingers
			var touch = touches[i];
			var px = touch.pageX;
			var py = touch.pageY;

			ctx.beginPath();
			ctx.arc(px, py, 70, 0, 2*Math.PI, true);

			ctx.fillStyle = "rgba(0, 0, 200, 0.2)";
			ctx.fill();

			ctx.lineWidth = 2.0;
			ctx.strokeStyle = "rgba(0, 0, 200, 0.8)";
			ctx.stroke();

			if(len==1) {
				// if only one touch point
				x1 = touches[0].pageX;
				y1 = touches[0].pageY;	
				
				controlBox_x = x1;
				controlBox_y = y1;
				controlBox_phi = 0;

				// draw line from screen center to control pivot
				ctx.strokeStyle = "rgba(0, 200, 0, 1)";
				ctx.beginPath();
				ctx.moveTo(w/2,h/2);
				ctx.lineTo(x1,y1);
				ctx.lineWidth = 2.0;
				ctx.setLineDash([5, 5]);
				ctx.stroke();
				ctx.setLineDash([]);
			}

			if(i==1) {
				// if two touch point
				ctx.beginPath();
				ctx.moveTo(touches[0].pageX,touches[0].pageY);
				ctx.lineTo(touches[1].pageX,touches[1].pageY);
				ctx.lineWidth = 3.0;
				ctx.strokeStyle = "rgba(0, 50, 100, 0.3)";
				ctx.stroke();

				x1 = touches[0].pageX;
				y1 = touches[0].pageY;

				x2 = touches[1].pageX;
				y2 = touches[1].pageY;

				if(x1>x2) {
					tx = x1;
					ty = y1;

					x1 = x2;
					y1 = y2;

					x2 = tx;
					y2 = ty;
				}

				deltaX = x2-x1;
				deltaY = y2-y1;

				res = Math.atan(deltaY/deltaX)*180/Math.PI; 
				
				controlBox_x = (x1+x2)/2;
				controlBox_y = (y1+y2)/2;
				controlBox_phi = res;

				// draw line from screen center to control pivot
				ctx.strokeStyle = "rgba(0, 200, 0, 1)";
				ctx.beginPath();
				ctx.moveTo(w/2,h/2);
				ctx.lineTo((x1+x2)/2,(y1+y2)/2);
				ctx.lineWidth = 2.0;
				ctx.setLineDash([5, 5]);
				ctx.stroke();
				ctx.setLineDash([])
			}


			xpos = controlBox_x-w/2;
			ypos = controlBox_y-h/2;
			
			ypos = -ypos;
			
			xpos = Math.round(300*(xpos/(w/2)));
			ypos = Math.round(300*(ypos/(h/2)));

			deltaX = moveLastchange_x - xpos;
			deltaY = moveLastchange_y - ypos;
			deltaR = moveLastchange_r - Math.round(controlBox_phi);


			ctx.fillText("x: "+xpos,30,60);
			ctx.fillText("y: "+ypos,30,90);
			ctx.fillText("phi: "+controlBox_phi_txt,30,120);


			// moveChanged(xpos,ypos,controlBox_phi);

			if(Math.abs(deltaX+deltaY+deltaR)>0) {
				moveChanged(xpos,ypos,controlBox_phi);
			};
		}
		noMoveUpdate = false;
	}

	if(noMoveUpdate) {
		controlBox_x = w/2;
		controlBox_y = h/2;
		controlBox_phi = 0;

		xpos = controlBox_x-w/2;
		ypos = controlBox_y-h/2;

		ypos = -ypos;
		
		xpos = Math.round(300*(xpos/(w/2)));
		ypos = Math.round(300*(ypos/(h/2)));

		deltaX = moveLastchange_x - xpos;
		deltaY = moveLastchange_y - ypos;
		deltaR = moveLastchange_r - Math.round(controlBox_phi);


		ctx.fillText("x: "+xpos,30,60);
		ctx.fillText("y: "+ypos,30,90);
		ctx.fillText("phi: "+controlBox_phi_txt,30,120);


		// moveChanged(xpos,ypos,controlBox_phi);

		if(Math.abs(deltaX+deltaY+deltaR)>0) {
			moveChanged(xpos,ypos,controlBox_phi);
		};
	}
	

	controlBox(controlBox_x,controlBox_y,controlBox_phi,50,100);

	drawUIAxisCenterMarker(w/2,h/2);

	ctx.beginPath();
	ctx.arc(w/2, h/2, 7, 0, 2*Math.PI, true);
	ctx.fillStyle = "rgba(0, 0, 200, 1)";
	ctx.fill();

	updateStarted = false;
}



function moveChanged(x,y,r) {
	moveUpdate = true;
	moveLastchange_x = x;
	moveLastchange_y = y;
	moveLastchange_r = r;
	sendMoveChange();
}

function sendMoveChange() {
	counterSent++;

	if(!moveSent) {
		moveSent = true;
		moveUpdate = false;
		$.ajax({
		// cmd = move
		// variables: x,y,r	
			type: "POST",
			url: "cgi",
			data: {cmd: 'move', x: moveLastchange_x, y: moveLastchange_y, r: moveLastchange_r},
			success: function( data ) {
				counterSuccess++;
				serverFeedback = data;
				moveSent = false;
				if(moveUpdate) sendMoveChange();
		  	}
		});
	}
}


function controlBox(x, y, phi, width,height) {

	phi = Math.PI * (phi/180);

	w2 = width / 2;
	h2 = height / 2;

	xp1 = - w2;
	yp1 = - h2;

	xp2 = + w2;
	yp2 = - h2;

	xp3 = + w2;
	yp3 = + h2;

	xp4 = - w2;
	yp4 = + h2;

	xxp1 = xp1*Math.cos(phi)-yp1*Math.sin(phi);
	yyp1 = xp1*Math.sin(phi)+yp1*Math.cos(phi);

	xxp2 = xp2*Math.cos(phi)-yp2*Math.sin(phi);
	yyp2 = xp2*Math.sin(phi)+yp2*Math.cos(phi);

	xxp3 = xp3*Math.cos(phi)-yp3*Math.sin(phi);
	yyp3 = xp3*Math.sin(phi)+yp3*Math.cos(phi);

	xxp4 = xp4*Math.cos(phi)-yp4*Math.sin(phi);
	yyp4 = xp4*Math.sin(phi)+yp4*Math.cos(phi);

	ctx.fillStyle = "rgba(200, 0, 0, 0.5)";
	ctx.beginPath();
	ctx.moveTo(xxp1+x, yyp1+y);
	ctx.lineTo(xxp2+x, yyp2+y);
	ctx.lineTo(xxp3+x, yyp3+y);
	ctx.lineTo(xxp4+x, yyp4+y);
	ctx.closePath();
	ctx.fill();		
}

function drawUIAxisCenterMarker(x, y) {
	ctx.beginPath();
	ctx.strokeStyle = "rgba(0, 0, 200, 1)";
	ctx.lineWidth = 1.0;
	ctx.moveTo(x-10,y);
	ctx.lineTo(x+10,y);
	ctx.stroke();
	ctx.moveTo(x,y-10);
	ctx.lineTo(x,y+10);
	ctx.stroke();
}



/* Copyright (C) 2007 Richard Atterer, richardÂ©atterer.net
   This program is free software; you can redistribute it and/or modify it
   under the terms of the GNU General Public License, version 2. See the file
   COPYING for details. */

var imageNr = 0; // Serial number of current image
var finished = new Array(); // References to img objects which have finished downloading
var paused = false;

function createImageLayer() {
  var img = new Image();
  img.style.position = "absolute";
  img.style.zIndex = 10;
  img.style.width = 320;
  img.style.height = 280;
  img.onload = imageOnload;
  img.onclick = imageOnclick;
  img.src = "http://littletau:8080/?action=snapshot&n=" + (++imageNr);
  var webcam = document.getElementById("webcam");
  webcam.insertBefore(img, webcam.firstChild);
}

// Two layers are always present (except at the very beginning), to avoid flicker
function imageOnload() {
  this.style.zIndex = 0; // Image finished, bring to front!
  while (1 < finished.length) {
	var del = finished.shift(); // Delete old image(s) from document
	del.parentNode.removeChild(del);
  }
  finished.push(this);
  if (!paused) createImageLayer();
}

function imageOnclick() { // Clicking on the image will pause the stream
  paused = !paused;
  if (!paused) createImageLayer();
}



$(document).ready(function() {
	// createImageLayer();
	canvas = document.getElementById('canvas');
	ctx = canvas.getContext('2d');
	timer = setInterval(update, 100);

	document.getElementById('tiltControl').addEventListener('mousedown',function(event) {
		gyro = 'on';
	});
	document.getElementById('tiltControl').addEventListener('mouseup',function(event) {
		gyro = 'off';
	});

	canvas.addEventListener('touchend', function(event) {
		event.preventDefault();
		touches = [];
		
		controlBox_x = window.innerWidth/2;
		controlBox_y = window.innerHeight/2;
		controlBox_phi = 0;
	});

	canvas.addEventListener('touchmove', function(event) {
		event.preventDefault();
		touches = event.touches;
		mouseControlOn = false;
	});

	canvas.addEventListener('touchstart', function(event) {
		event.preventDefault();
		mouseControlOn = false;
	});

	canvas.addEventListener('touchcancel', function(event) {
		controlBox_x = window.innerWidth/2;
		controlBox_y = window.innerHeight/2;
		controlBox_phi = 0;
	});

	canvas.addEventListener('mouseup',function(event){
		mouseControlOn = false;
	});

	canvas.addEventListener('mousemove',function(event){
		mouse_x = event.pageX;
		mouse_y = event.pageY;
	});

	canvas.addEventListener('mousedown',function(event){
		mouseControlOn = true;
	});
		// event.preventDefault();
		
// console.log('start');

});
