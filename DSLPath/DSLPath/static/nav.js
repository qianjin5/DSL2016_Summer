
loc = this.location;

var nav_drawingPad = document.getElementById('nav_drawingPad');
var nav_gallery = document.getElementById('nav_gallery');

nav_drawingPad.onclick = goToDrawingPad;
function goToDrawingPad(e) {
	loc.assign('http://' + loc.hostname + '/');
}

nav_gallery.onclick = goToGallery;
function goToGallery(e) {
	
	loc.assign('http://' + loc.hostname + '/gallery');
}
