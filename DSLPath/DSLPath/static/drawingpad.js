if (this.location.pathname == '/') {

	init_drawing_pad();
}

window = this;
function init_drawing_pad() {
	var paint = false;
	var firstStroke = true;

	var colorPurple = "#cb3594";
	var colorGreen = "#659b41";
	var colorYellow = "#ffcf33";
	var colorBrown = "#986928";
	var colorBlack = "#000000";

	var curColor = colorPurple;
	var clickColor = new Array();


	var canvasWidth = window.innerWidth-20;
	var canvasHeight = 0.6 * canvasWidth;


	var clickX = [];
	var clickY = [];
	var clickDrag = [];
	var clickColor = [];
	var clickTime = [];

	var TRAJ_TAU = 50;
	var numResDNN;
	var numResNoDNN;

	// Init the canvas (functionality)
	var canvasDiv = document.getElementById('canvasDiv');
	canvas = document.createElement('canvas');
	canvas.setAttribute('width', canvasWidth);
	canvas.setAttribute('height', canvasHeight);
	canvas.setAttribute('id', 'canvas');
	canvas.style.backgroundColor = 'rgba(255, 255, 255)';


	function onMouseDown(e) {

		var mouseX;
		var mouseY;
		if ((e.pageX && e.pageY)) {
			mouseX = e.pageX - this.offsetLeft;
			mouseY = e.pageY - this.offsetTop;
		} else if (e.targetTouches) {
			mouseX = e.targetTouches[0].cliencX - this.offsetLeft;
			mouseY = e.targetTouches[0].clientY - this.offsetTop;
		}
		paint = true;
		if (firstStroke) {
			var d = new Date();
			startTime = d.getTime();
			addClick(mouseX, mouseY);
			redraw();
		}
	}
	canvas.addEventListener("touchstart", function(e) {
		e.preventDefault();
		onMouseDown(e);
		}, false);
	canvas.addEventListener("mousedown", onMouseDown, false);



	function positionHandler(e) {

		if ((e.pageX && e.pageY)) {
			if (firstStroke && paint) {
				addClick(e.pageX - this.offsetLeft, e.pageY - this.offsetTop, true);
				redraw();
			}
		} else if (e.changedTouches) {
			var posX = e.changedTouches[0].pageX - this.offsetLeft;
			var posY = e.changedTouches[0].pageY - this.offsetTop;
			if (posX < 0 || posX > canvasWidth || posY < 0 || posY > canvasHeight) {
				firstStroke=false;
			}
			if (firstStroke && paint) {
				addClick(posX, posY, true);
				redraw();
			}
		}
	}
	canvas.addEventListener('mousemove', positionHandler, false);
	canvas.addEventListener('touchstart', positionHandler, false);
	canvas.addEventListener('touchmove', positionHandler, false);

	function onStrokeEnd(e){
		paint = false;
		firstStroke = false;
	};
	canvas.addEventListener('touchend', onStrokeEnd, false);
	canvas.onmouseup = onStrokeEnd

	canvas.onmouseleave = onStrokeEnd;
	canvas.ontouchcancel = onStrokeEnd;

	canvasDiv.appendChild(canvas);

	if(typeof G_vmlCanvasManager != 'undefined') {
		canvas = G_vmlCanvasManager.initElement(canvas);
	}
	context = canvas.getContext("2d");
	context.strokeRect(0, 0, canvasWidth, canvasHeight);

	// Text Field
	var textfield = document.getElementById('trajName');
	var traj_name = textfield.value;
	textfield.oninput = function(e) {
		traj_name = textfield.value;
	}


	// Init buttons
	clearButton = document.getElementById('clearButton');
	clearButton.onclick = function(e) {
		clickX = [];
		clickY = [];
		clickDrag = [];
		clickColor = [];
		clickTime = [];
		firstStroke = true;
		traj_name = "";
		textfield.value = "";
		redraw();
	}
	submitButton = document.getElementById('submitButton');
	submitButton.onclick = function(e) {
		// Trajectory time
		var d = new Date();
		var currTimeString = d.getTime().toString();
		var currTimeUTCString = d.toUTCString();

		// des_traj_raw is an array of objects
		var des_traj_raw = [];

		var prev = 0;
		for (var i = 0; i < clickX.length; i++) {
			var vx = 0;
			var vy = 0;
			if (i != 0) {
				vx = (clickX[i] - clickX[i-1]) / (clickTime[i] - clickTime[i-1]);
				vy = (clickY[i] - clickY[i-1]) / (clickTime[i] - clickTime[i-1]);
			}
			var obj = {"time":clickTime[i], "x":clickX[i], "y":clickY[i], "vx":vx, "vy":vy};

			des_traj_raw.push(obj);
		}

		// Now extract des_traj from des_traj_raw
		var des_traj = [];
		time_th = TRAJ_TAU;
		var start = 0;
		for (var i = 0; i < des_traj_raw.length; i++) {
			if (des_traj_raw[i].time > time_th) {
				time_th += TRAJ_TAU;
				des_traj.push(average(des_traj_raw, start, i, time_th));
				start = i;
			}
		}

		// Give des_traj to the server, which handles everything else.
		var xhttp = new XMLHttpRequest();
		if (window.XMLHttpRequest) {
			xhttp = new XMLHttpRequest();
		} else {
			alert("Your browser doesn't support XMLHttpRequest!");
		}
		xhttp.onreadystatechange = function() {
			if (xhttp.readyState == 4 && xhttp.status == 200) {
				var rt = xhttp.responseText;
				document.getElementById("txtHint").innerHTML = rt;

			}
		}
		orig_range = [[0, canvasWidth], [canvasHeight, 0]];
		xhttp.open("POST",
			"des_path_submission?traj_name=" + traj_name +
			"&des_traj=" + JSON.stringify(des_traj) +
			"&curr_time=" + currTimeUTCString +
			"&orig_range=" + JSON.stringify(orig_range),
			true);
		xhttp.send();

		// Also starts a timer to periodically send AJAX for result
		query_res_num();
		setInterval(query_res_num_and_update, 10000);

	}

	// Init time
	var d = new Date();
	var startTime = d.getTime();

	function average(des_traj_raw, start, end, time) {
		var avg_x = 0;
		var avg_y = 0;
		var avg_vx = 0;
		var avg_vy = 0;
		for (var i = start; i < end; i++) {
			avg_x += des_traj_raw[i].x;

			avg_y += des_traj_raw[i].y;
			avg_vx += des_traj_raw[i].vx;
			avg_vy += des_traj_raw[i].vy;
		}
		var n = end - start;

		avg_x /= n;
		avg_y /= n;
		avg_vx /= n;
		avg_vy /= n;

		var obj = {'time':time, 'x':avg_x, 'y':avg_y, 'vx':avg_vx, 'vy':avg_vy};
		return obj;
	}

	function addClick(x, y, dragging) {
		clickX.push(x);
		clickY.push(y);
		clickDrag.push(dragging);
		clickColor.push(curColor);
		var d = new Date();
		clickTime.push(d.getTime() - startTime);
	}

	function redraw(){
		context.clearRect(0, 0, context.canvas.width, context.canvas.height); // Clears the canvas

		context.strokeStyle = "#df4b26";
		context.lineJoin = "round";
		context.lineWidth = 5;
		context.strokeRect(0, 0, canvasWidth, canvasHeight);

		for(var i=0; i < clickX.length; i++) {
			context.beginPath();
			if(clickDrag[i] && i){
				context.moveTo(clickX[i-1], clickY[i-1]);
			}else{
				context.moveTo(clickX[i]-1, clickY[i]);
			}
			context.lineTo(clickX[i], clickY[i]);
			context.closePath();
			context.stroke();
		}

	}

}

function query_res_num() {
	// Called to initialize numResDNN and numResNoDNN
	var xhttp = new XMLHttpRequest();
	if (window.XMLHttpRequest) {
		xhttp = new XMLHttpRequest();
	} else {
		alert("Your browser doesn't support XMLHttpRequest!");
	}
	xhttp.onreadystatechange = function() {
		if (xhttp.readyState == 4 && xhttp.status == 200) {
			var rt = xhttp.responseText;

			var r = JSON.parse(rt)
			numResDNN = parseInt(r['numResDNN'])
			numResNoDNN = parseInt(r['numResNoDNN'])
			console.log("Initial status: numResDNN=" + String(numResDNN) + ", numResNoDNN=" + String(numResNoDNN));
		}
	}
	xhttp.open("GET", "query_for_result", true);
	xhttp.send();

}

function query_res_num_and_update() {
	var num_d;
	var num_nd;
	var xhttp = new XMLHttpRequest();
	if (window.XMLHttpRequest) {
		xhttp = new XMLHttpRequest();
	} else {
		alert("Your browser doesn't support XMLHttpRequest!");
	}
	xhttp.onreadystatechange = function() {
		if (xhttp.readyState == 4 && xhttp.status == 200) {
			var rt = xhttp.responseText;

			var r = JSON.parse(rt)
			num_d = parseInt(r['numResDNN'])
			num_nd = parseInt(r['numResNoDNN'])
			console.log("query_res_num_and_update(). d=" + String(num_d) + ", nd=" + String(num_nd));
			if(num_d > numResDNN) {
				console.log('hasDNN new data found!');
				var hasdnn = true;
				update_result_display(hasdnn);
			}
			if (num_nd > numResNoDNN) {
				console.log('noDNN new data found!');
				var hasdnn = false;
				update_result_display(hasdnn);
			}
		}
	}
	xhttp.open("GET", "query_for_result", true);
	xhttp.send();

	return [num_nd, num_d];
}
