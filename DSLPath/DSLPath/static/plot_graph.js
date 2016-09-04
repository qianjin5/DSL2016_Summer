
function plot_graph(container_id_str, data, hasdnn) {

	var margin = {left: 30, right:30, top:50, bottom:30};
	var svg_x_range = window.innerWidth - 20;
	var svg_y_range = svg_x_range * 0.60;
	var width = svg_x_range - margin.left - margin.right;
	var height = svg_y_range - margin.top - margin.bottom;
	var svg = d3.select('#' + container_id_str).append("svg")
            .attr("width", width + margin.left + margin.right)
            .attr("height", height + margin.top + margin.bottom)
            .attr("border", "black")
            .append("g")
            .attr("transform", "translate(" + margin.left + "," + margin.top + ")");
	//console.log('plot_graph(). data:' + JSON.stringify(data));


	var phys_range = JSON.parse(data['phys_range'])

	//console.log('data phys_range:' + phys_range);
	var xdomain = phys_range[0];
	//console.log('xdomain: ' + xdomain);
	var ydomain = phys_range[1];


	//Axes
	var xScale = d3.scale.linear()
		.range([0, width])
		.domain(xdomain);

	var yScale = d3.scale.linear()
		.range([height, 0])
		.domain(ydomain);

	var xAxis = d3.svg.axis()
		.scale(xScale)
		.orient("bottom")
		.ticks(10);

    var yAxis = d3.svg.axis()
		.scale(yScale)
		.orient("left")
		.ticks(8);

    svg.append("g")
		.attr("class", "x axis")
		.attr("transform", "translate(0," + height + ")")
		.call(xAxis);

	svg.append("g")
		.attr("class", "y axis")
		.call(yAxis);

	// Lines
	var lineFunc = d3.svg.line()
		.x(function(d, i) {
			return xScale(d.x);
		})
		.y(function(d, i) {
			return yScale(d.y);
		})
		.interpolate('linear');

	var estTrajData = data4plt(data['est_traj']);
	var refTrajData = data4plt(data['ref_traj']);
	var desTrajData = data4plt(data['des_traj']);

	svg.append("path")
		.attr('class', 'est_traj')
		.attr('d', lineFunc(estTrajData));

	svg.append("path")
		.attr('class', 'ref_traj')
		.attr('d', lineFunc(refTrajData))
		.style('stroke-dasharray', ("3, 3"));

	svg.append("path")
		.attr('class', 'des_traj')
		.attr('d', lineFunc(desTrajData))
		.style('stroke-dasharray', ("3, 3"));

	//legend
	svg.append("g")
		.attr("id", "legend")
		.attr("transform", "translate(0, -40)");

	var legend = svg.select("#legend");
	legend.append("rect")
		.attr("x", 0)
		.attr("y", 0)
		.attr("width", width)
		.attr("height", margin.top - 10)
		.attr("class", "legendbox");

	if (hasdnn) {
		var w = width;
		var h = margin.top - 10;
		var wu = w / 3;
		var tw = 0.3;

		legend.append("line")
			.attr("x1", 0)
			.attr("y1", h / 2)
			.attr("x2", 0 + tw * wu)
			.attr("y2", h / 2)
			.attr("class", "des_traj");

		legend.append("text")
			.attr("x", 0 + tw * wu)
			.attr("y", 0.8*h)
			.text("desired trajectory");

		legend.append("line")
			.attr("x1", wu)
			.attr("y1", h / 2)
			.attr("x2", wu + tw * wu)
			.attr("y2", h / 2)
			.attr("class", "ref_traj");

		legend.append("text")
			.attr("x", wu + tw * wu)
			.attr("y", 0.8*h)
			.text("reference trajectory");

		legend.append("line")
			.attr("x1", wu*2)
			.attr("y1", h / 2)
			.attr("x2", wu*2 + tw * wu)
			.attr("y2", h / 2)
			.attr("class", "est_traj");

		legend.append("text")
			.attr("x", wu*2 + tw * wu)
			.attr("y", 0.8*h)
			.text("actual trajectory");
	} else {
		var w = width;
		var h = margin.top - 10;
		var wu = w / 2;
		var tw = 0.3;

		legend.append("line")
			.attr("x1", 0)
			.attr("y1", h / 2)
			.attr("x2", 0 + tw * wu)
			.attr("y2", h / 2)
			.attr("class", "des_traj");

		legend.append("text")
			.attr("x", 0 + tw * wu)
			.attr("y", 0.8*h)
			.text("desired trajectory");

		legend.append("line")
			.attr("x1", wu)
			.attr("y1", h / 2)
			.attr("x2", wu + tw * wu)
			.attr("y2", h / 2)
			.attr("class", "est_traj");

		legend.append("text")
			.attr("x", wu + tw * wu)
			.attr("y", 0.8*h)
			.text("actual trajectory");
	}
}

function data4plt(sLOL) {
	lineData = [];
	LOL = JSON.parse(sLOL);

	for (var i = 0; i < LOL[0].length; i++) {
		pt = {'x': LOL[0][i], 'y': LOL[2][i]};
		lineData.push(pt);
		/*if (i == 10) {
			console.log('lineData:' + String(lineData));
		}*/
	}
	return lineData;
}
