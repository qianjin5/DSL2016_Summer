function update_result_display(hasdnn) {
	/*Request for the newest result*/
	var xhttp;
	if (window.XMLHttpRequest) {
		xhttp = new XMLHttpRequest();
	} else {
		xhttp = new ActiveXObject("Microsoft.XMLHTTP");
	}
	xhttp.onreadystatechange = function() {
		if (xhttp.readyState == 4 && xhttp.status == 200) {
			var rt = xhttp.responseText;
			data = JSON.parse(rt);

			display_data(data, hasdnn);
		}
	}
	xhttp.open("GET", "get_trial_result?hasdnn=" + String(hasdnn), true);
	xhttp.send();
}

function display_data(data, hasdnn) {

	if (hasdnn) {
		str = "<p><h5>DNN utilized: </h5></p>" +
		"<p><h6>traj_id=" + data['traj_id'] + "; " +
		"name=" + data['trajname'] + "; " +
		"average error=" + data['avgerr'] + "</h6></p>";
		var container_id_str = 'resData_DNN_display';
		document.getElementById(container_id_str).innerHTML = str;
		plot_graph(container_id_str, data, hasdnn);
	} else {
		str = "<p><h5>DNN not utilized: </h5></p>" +
		"<p><h6>traj_id=" + data['traj_id'] + "; " +
		"name=" + data['trajname'] + "; " +
		"average error=" + data['avgerr'] + "</h6></p>";
		var container_id_str = 'resData_NoDNN_display';
		document.getElementById(container_id_str).innerHTML = str;
		plot_graph(container_id_str, data, hasdnn);
	}

}
