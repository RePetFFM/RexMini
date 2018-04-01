var time = new Date();

var data = [{
	name: 'mesured',
	x: [time], 
	y: [0],
	mode: 'lines',
	line: {color: '#ff0000'}
},{
	name: 'setpoint',
	x: [time], 
	y: [0],
	mode: 'lines',
	line: {color: '#00ff00'}
}]


var layout = {
	autosize: true,
	margin: {
		l: 30,
		r: 30,
		b: 30,
		t: 30,
		pad: 4
	}
};

Plotly.plot('graph', data,layout);  

var cnt = 0;

var interval = setInterval(function() {
	
	var time = new Date();

	d1 = pidDebugData[0];
	d2 = pidDebugData[1];

	if(doReadout) {

		var update = {
		x:  [[time],[time]],
		y: [[d1],[d2]]
		}
		
		var olderTime = time.setSeconds(time.getSeconds() - 10);
		var futureTime = time.setSeconds(time.getSeconds() + 10);
		
		var minuteView = {
					xaxis: {
						type: 'date',
						range: [olderTime,futureTime]
					}
				};
		
		Plotly.relayout('graph', minuteView);
		Plotly.extendTraces('graph', update, [0,1]);

	}
	
	if(cnt === 100) clearInterval(interval);
}, 10);