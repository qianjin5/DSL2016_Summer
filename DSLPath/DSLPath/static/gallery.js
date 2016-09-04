var window = this;

if (window.location.pathname == '/gallery') {

    get_trajs_and_plot();
}

function get_trajs_and_plot() {
    var promise = new Promise(function(resolve, reject) {
        var xhttp;
        if (window.XMLHttpRequest) {
            xhttp = new XMLHttpRequest();
        } else {
            xhttp = new ActiveXObject("Microsoft.XMLHTTP");
        }
        xhttp.onreadystatechange = function() {
            if (xhttp.readyState == 4 && xhttp.status == 200) {
                var rt = xhttp.responseText;
                resolve(rt);
                //console.log('resolved!' + rt);
            }
        };
        xhttp.open("GET", "get_all_trajectories", true);
        xhttp.send();
    });
    promise.then(plot_data,
        function(error) {
            console.log(error);
        });
}

function plot_data(data) {

    data_arr = JSON.parse(data);
    N = data_arr.length;

    /*media query*/
    var container = document.getElementById('gallery-container');
    var container_width = parseInt(window.getComputedStyle(container, null).getPropertyValue("width"));
    console.log('container_width: ' + container_width);
    var w = 0;
    var h = 0;
    if (container_width > 768) {
        w = container_width / 3;
        h = w;
    } else if (container_width > 300) {
        w = container_width / 2;
        h = w;
    } else {
        w = container_width;
        h = w;
    }
    // Create the svg blocks with calculated width and height
    var i = 0;


    for (var i = 0; i < data_arr.length; i++) {
        var data_obj = data_arr[i];
        var tid = data_obj.traj_id;
        var divname = "traj" + tid;
        var box = document.createElement('span');
        box.setAttribute("id", divname);
        box.setAttribute("style", "{display: inline-block; width:" + w + "; height:" + h + ";} ");
        container.appendChild(box);
        var svg = append_svgfig(w, h, data_obj, divname);


        box.onclick = function(x) {
            return function() {
                console.log('svg #' + x + ' clicked!');
                window.location.assign('http://' + window.location.hostname + '/viewDetail?traj_id=' + x);
            };
        }(tid);
    }
}


function viewTrajDetails(id) {
    console.log('Traj #' + id + " selected!");
    loc = window.location;
    loc.assign('http://' + loc.hostname + '/viewDetail' + '?id=' + id);
}

function append_svgfig(w, h, data_obj, divname) {

    margin = {'left': 10, 'right': 10, 'top': 10, 'bottom': 10};
    margin_style_txt = margin.top + "px " + margin.right + "px " + margin.bottom + "px " + margin.left + "px";

    offsetSize = {'width': w - margin.left - margin.right,
        'height': h - margin.top - margin.bottom};

    padding = {'left': 100, 'right': 20, 'top': 60, 'bottom': 50};

    clientSize = {'width': offsetSize.width - padding.left - padding.right,
        'height': offsetSize.height - padding.top - padding.bottom};


    svg = d3.select('#' + divname).append("svg")
        .attr("width", offsetSize.width)
        .attr("height", offsetSize.height)
        .attr("border", "black")
        .attr("class", "svg_selectable")
        .attr("style", "margin: " + margin_style_txt);

    // Title
    svg.append("text")
        .text("#" + data_obj.traj_id + ": " + data_obj.trajname)
        .attr("x", function(d) {

            return w / 2 - this.getBBox().width / 2;
        })
        .attr("y", 0.5 * padding.top)
        .classed("figtitle", true);

    // Figure
    fig = svg.append("g")
        .attr("transform", "translate(" + padding.left + "," + padding.top + ")");
    // Note that these trajectories are fetched directly from drawing pad table (DSLPath_tbl), so they don't have phys_range yet; only orig_range. However, we are anyways mapping it to the target range.
    var orig_range = JSON.parse(data_obj['orig_range']);
    var xdomain = orig_range[0];
    var ydomain = orig_range[1];


    //Axes
    var xScale = d3.scale.linear()
        .range([0, clientSize.width])
        .domain(xdomain);

    var yScale = d3.scale.linear()
        .range([clientSize.height, 0])
        .domain(ydomain);

    var xAxis = d3.svg.axis()
        .scale(xScale)
        .orient("bottom")
        .ticks(4);

    var yAxis = d3.svg.axis()
        .scale(yScale)
        .orient("left")
        .ticks(8);

    var tmph = offsetSize.height - padding.bottom;
    svg.append("g")
        .attr("class", "x axis")
        .attr("transform", "translate(" + 0.5 * padding.left + ", " + tmph + ")")
        .call(xAxis);

    svg.append("g")
        .attr("class", "y axis")
        .attr("transform", "translate(" + 0.5 * padding.left + ", " + padding.top + ")")
        .call(yAxis);

    // Path plot
    var lineFunc = d3.svg.line()
        .x(function(d, i) {
            return xScale(parseInt(d.x));
        })
        .y(function(d, i) {
            return yScale(parseInt(d.y));
        })
        .interpolate('linear');

    var trajData = JSON.parse(data_obj['destraj']);

    fig.append("path")
        .attr('class', 'des_traj')
        .attr('d', lineFunc(trajData))
        .style('stroke-dasharray', ("3, 3"));

    return svg;
}
