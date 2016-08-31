var dpBtn = document.getElementById('nav_drawingpad');
var glBtn = document.getElementById('nav_gallery');

console.log('document.innerHTML=' + document.innerHTML);
dpBtn.onclick = function(e) {
	dpBtn.className = dpBtn.className.replace( /(?:^|\s)navbtn_unselect(?!\S)/g, 'navbtn_selected' );
	glBtn.className = glBtn.className.replace( /(?:^|\s)navbtn_selected(?!\S)/g, 'navbtn_unselect' );

	var xhttp = new XMLHttpRequest();
	if (window.XMLHttpRequest) {
		xhttp = new XMLHttpRequest();
	} else {
		alert("Your browser doesn't support XMLHttpRequest!");
	}
	xhttp.onreadystatechange = function() {
		if (xhttp.readyState == 4 && xhttp.status == 200) {
			var rt = xhttp.responseText;
			console.log('action.js render_drawingpad: ' + rt);
			document.write(rt);
			// FIXME - the new contents got attached to below...
		}
	}
	xhttp.open("GET", "/", true);
	xhttp.send();
}

glBtn.onclick = function(e) {
	glBtn.className = glBtn.className.replace( /(?:^|\s)navbtn_unselect(?!\S)/g, 'navbtn_selected' );
	dpBtn.className = dpBtn.className.replace( /(?:^|\s)navbtn_selected(?!\S)/g, 'navbtn_unselect' );

	var xhttp = new XMLHttpRequest();
	if (window.XMLHttpRequest) {
		xhttp = new XMLHttpRequest();
	} else {
		alert("Your browser doesn't support XMLHttpRequest!");
	}
	xhttp.onreadystatechange = function() {
		if (xhttp.readyState == 4 && xhttp.status == 200) {
			var rt = xhttp.responseText;
			console.log('action.js render_gallery: ' + rt);
			document.write(rt);
		}
	}
	xhttp.open("GET", "gallery", true);
	xhttp.send();
}
