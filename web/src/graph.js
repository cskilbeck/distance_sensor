//////////////////////////////////////////////////////////////////////

var chart;
var host = "vibue.com";
//var host = "localhost";

//////////////////////////////////////////////////////////////////////

function date_format(x) {

    return new Date(x).toLocaleString('en-GB', {

        day: 'numeric',
        month: 'short',
        hour: 'numeric',
        minute: 'numeric',
        timezone: 'UTC'
    });
}

//////////////////////////////////////////////////////////////////////

function setCookie(name, value, days) {

    var expires = "";
    if (days) {
        var date = new Date();
        date.setTime(date.getTime() + (days * 24 * 60 * 60 * 1000));
        expires = "; expires=" + date.toUTCString();
    }
    document.cookie = name + "=" + (value || "") + expires + "; path=/";
}

//////////////////////////////////////////////////////////////////////

function getCookie(name) {

    var nameEQ = name + "=";
    var ca = document.cookie.split(';');
    for (var i = 0; i < ca.length; i++) {
        var c = ca[i];
        while (c.charAt(0) == ' ') {
            c = c.substring(1, c.length);
        }
        if (c.indexOf(nameEQ) == 0) {
            return c.substring(nameEQ.length, c.length);
        }
    }
    return null;
}

//////////////////////////////////////////////////////////////////////

function eraseCookie(name) {

    document.cookie = name + '=; Path=/; Expires=Thu, 01 Jan 1970 00:00:01 GMT;';
}

//////////////////////////////////////////////////////////////////////

function set_device(name, address) {
    setCookie("device_name", name, 9999);
    setCookie("device_address", address, 9999);
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
                items += `<a onClick='set_device("${name}", "${addr}")' class="dropdown-item" href="#"><span class='text-info'>${device['address']}</span><span>&nbsp;${device['name']}</span></a>`
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

    var graph_num_str = idx || getCookie("graph_num") || "0";
    var device_name = getCookie("device_name") || "?";
    var device_address = getCookie("device_address") || "84f3eb536123";

    setCookie("graph_num", graph_num_str, 9999);
    setCookie("device_name", device_name, 9999);
    setCookie("device_address", device_address, 9999);

    var graph_num = parseInt(graph_num_str);

    for (var i = 0; i < 3; ++i) {
        var btn = document.getElementById(`button${i}`);
        if (i == graph_num) {
            btn.classList.add('highlighted');
        } else {
            btn.classList.remove('highlighted');
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
                            display: true,
                            position: 'bottom',
                            labels: {
                                color: config.color
                            }
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
