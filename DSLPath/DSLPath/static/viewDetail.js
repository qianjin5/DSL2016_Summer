window = this;

if (window.location.pathname == '/viewDetail') {
    // Assume only one parameter exist
    var prm = window.location.search.split('=');
    var traj_id = parseInt(prm[1]);
    console.log('viewDetail: traj_id=' + traj_id);
    render_detail(traj_id);
}

function render_detail(traj_id) {
    var promise = new Promise(function(resolve, reject) {
        var xhttp;
        if (window.XMLHttpRequest) {
            xhttp = new XMLHttpRequest();
        } else {
            xhttp = new ActiveXObject("Microsoft.XMLHTTP");
        }
        xhttp.onreadystatechange = function() {
            if (xhttp.readyState == 4 && xhttp.status == 200) {
                var data = xhttp.responseText;
                resolve(JSON.parse(data));
                //console.log('resolved! data=' + data);
            }
        };
        xhttp.open("GET", "getTrajDetailById?traj_id=" + traj_id, true);
        xhttp.send();
    });
    promise.then(function(data) {
        var basicInfoArea = document.getElementById('basic-info');
        var infonode = document.createElement('div');
        infonode.setAttribute('id', 'infonode');
        var paragraph = document.createElement('h2');
        if (data['nodnn'] != undefined) {
            d = data['nodnn'];
            var txt;

            txt = 'Traj #' + d.traj_id + ": " + d.trajname;
            paragraph.innerHTML = txt;
            infonode.appendChild(paragraph);

        } else if (data['hasdnn'] != undefined) {
            d = data['hasdnn'];
            var txt;
            txt = 'Traj #' + d.traj_id + ": " + d.trajname;
            paragraph.innerHTML = txt;
            infonode.appendChild(paragraph);
        } else {
            txt = document.createTextNode('There is no running result for this trajectory yet!');
            paragraph.innerHTML = txt;
            infonode.appendChild(paragraph);
        }

        basicInfoArea.appendChild(infonode);
        return data;
    });

    promise.then(function(data) {
        if (data['nodnn'] == undefined) {
            return data;
        }
        var infoarea = document.getElementById('nodnn-info');
        var paragraph = document.createElement('p');
        paragraph.innerHTML = "Average Error without DNN: " + data['nodnn'].avgerr;
        infoarea.appendChild(paragraph);
        plot_graph("fig-nodnn", data['nodnn'], false);

        return data;
        },
        function (error) {
            console.log(error);
        }
    );

    promise.then(function(data) {
        if (data['hasdnn'] == undefined) {
            return data;
        }
        var infoarea = document.getElementById('hasdnn-info');
        var paragraph = document.createElement('p');
        paragraph.innerHTML = "Average Error with DNN: " + data['hasdnn'].avgerr;
        infoarea.appendChild(paragraph);
        plot_graph("fig-hasdnn", data['hasdnn'], true);
        return data;
        },
        function(error) {
            console.log(error);
        }
    );

}
