//////////////////////////////////////////////////////////////////////

var chart;
var host = "vibue.com";
//var host = "localhost";

//////////////////////////////////////////////////////////////////////

function date_format(x) {

    return new Date(x).toLocaleString('en-US', {

        day: 'numeric',
        month: 'short',
        hour: 'numeric',
        minute: 'numeric'
    });
}

//////////////////////////////////////////////////////////////////////

function set_cookie(name, value, days) {

    days = days || 400;
    var expires = "";
    var date = new Date();
    date.setTime(date.getTime() + (days * 24 * 60 * 60 * 1000));
    expires = "; expires=" + date.toUTCString();
    document.cookie = name + "=" + (value || "") + expires + "; path=/";
}

//////////////////////////////////////////////////////////////////////

function get_cookie(name) {

    var f = name + "=";
    for (var c of document.cookie.split(';')) {
        var t = c.trimStart();
        if (t.indexOf(f) == 0) {
            return t.substring(f.length);
        }
    }
    return null;
}

//////////////////////////////////////////////////////////////////////

function erase_cookie(name) {

    document.cookie = name + '=; Path=/; Expires=Thu, 01 Jan 1970 00:00:01 GMT;';
}

//////////////////////////////////////////////////////////////////////

function set_device(ev, name, address) {
    ev.preventDefault()
    set_cookie("device_name", name);
    set_cookie("device_address", address);
    show_graph();
}

//////////////////////////////////////////////////////////////////////

function refresh_devices() {

    var xhr = new XMLHttpRequest();
    xhr.responseType = 'json';

    var url = new URL(`https://${host}/devices`);
    url.searchParams.set('nonce', new Date().getMilliseconds());

    xhr.open('GET', url);

    xhr.onload = function () {

        if (xhr.status != 200) {
            alert(xhr.response.info.join('\n'));

        } else {

            var items = ""

            var rows = xhr.response["rows"];
            var devices = xhr.response["device"];
            for (var i = 0; i < rows; ++i) {
                var device = devices[i];
                var name = device['name']
                var addr = device['address']
                items += `<a onClick='set_device(event, "${name}", "${addr}")' class="dropdown-item" href="${name}"><span class='text-info'>${device['address']}</span><span>&nbsp;${device['name']}</span></a>`
            }
            var dropdown = document.getElementById('dropdown_device_items');
            dropdown.innerHTML = items;
        }
    }
    xhr.send();
}

//////////////////////////////////////////////////////////////////////

function show_graph(idx) {

    var now = new Date(new Date().setDate(new Date().getDate() + 1));
    var then = new Date(new Date().setDate(now.getDate() - 121));

    var graph_num_str = idx || get_cookie("graph_num") || "0";
    var device_name = get_cookie("device_name") || "?";
    var device_address = get_cookie("device_address") || "84f3eb536123";

    set_cookie("graph_num", graph_num_str);
    set_cookie("device_name", device_name);
    set_cookie("device_address", device_address);

    var graph_num = parseInt(graph_num_str);

    for (var i = 0; i < 3; ++i) {
        var btn = document.getElementById(`button${i}`);
        if (i == graph_num) {
            btn.classList.add('highlighted');
            btn.classList.remove('inactive');
        } else {
            btn.classList.remove('highlighted');
            btn.classList.add('inactive');
        }
    }

    var device_name_input_field = document.getElementById('device_address');
    device_name_input_field.value = `${device_name} (${device_address})`;

    var url = new URL(`https://${host}/readings`);
    url.searchParams.set('from', then.toISOString());
    url.searchParams.set('to', now.toISOString());
    url.searchParams.set('device', device_address);
    url.searchParams.set('nonce', new Date().getMilliseconds());

    var xhr = new XMLHttpRequest();
    xhr.responseType = 'json';

    xhr.open('GET', url);

    xhr.onload = function () {

        if (xhr.status != 200) {
            alert(xhr.response.info.join('\n'));

        } else {

            var settings = [
                {
                    field: 'distance',
                    color: '#32CD32',
                    lineColor: '#167016',
                    reverse: true,
                    format: function (x) {
                        return `${+parseFloat(x / 10).toFixed(1)}cm`
                    }
                },
                {
                    field: 'vbat',
                    color: '#FF8C00',
                    lineColor: '#804000',
                    reverse: false,
                    format: function (x) {
                        return `${+parseFloat(x / 100).toFixed(2)}v`
                    }
                },
                {
                    field: 'rssi',
                    color: '#4080FF',
                    lineColor: '#203890',
                    reverse: false,
                    format: function (x) {
                        return `${x}dBm`
                    }
                },
            ];

            var config = settings[graph_num]

            var datasets = []
            var data = [];

            var rsp = xhr.response;

            for (var i = 0; i < rsp.rows; ++i) {
                data.push({
                    x: new Date(rsp.time[i] * 1000),
                    y: rsp[config.field][i]
                });
            }
            datasets.push({
                label: config.field,
                labelPosition: "none",
                data: data,
                showLine: true,
                borderColor: config.lineColor,
                backgroundColor: config.color,
                pointBorderColor: config.color,
                lineTension: 0.01,
                fill: false,
            });

            if (chart !== undefined) {
                chart.destroy();
            }

            chart = new Chart(document.getElementById('graph'), {
                type: 'scatter',
                data: {
                    datasets: datasets,
                    labels: data
                },
                options: {
                    animation: {
                        duration: 100
                    },
                    plugins: {
                        tooltip: {
                            callbacks: {
                                title: function (item, data) {
                                    return config.field;
                                },
                                label: function (item, data) {
                                    return `${config.format(item.parsed.y)} at ${date_format(item.parsed.x)}`
                                }
                            }
                        },
                        legend: {
                            display: false,
                        },
                    },
                    responsive: true,
                    scales: {
                        x: {
                            ticks: {
                                color: '#999',
                                callback: function (val) {
                                    return date_format(val);
                                }
                            },
                            grid: {
                                color: '#333'
                            }
                        },
                        y: {
                            position: 'left',
                            reverse: config.reverse,
                            ticks: {
                                color: '#999',
                                callback: function (val) {
                                    return config.format(val);
                                }
                            },
                            grid: {
                                drawOnChartArea: true,
                                color: '#777'
                            }
                        }
                    }
                },
            }
            );
        }
    };

    xhr.send();
}

(function () {
    refresh_devices();
    show_graph();
})();
