//////////////////////////////////////////////////////////////////////

var chart;

//////////////////////////////////////////////////////////////////////

function show_graph(idx) {

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

    var now = new Date(new Date().setDate(new Date().getDate() + 1));
    var then = new Date(new Date().setDate(now.getDate() - 121));

    var host = "vibue.com";
    //var host = "localhost";

    var device_name_input_field = document.getElementById('device_address');
    var device_name = device_name_input_field.value;
    console.log(`From text box: [${device_name}]`);

    if (device_name == null || device_name == "") {
        device_name = getCookie("device_name");
        console.log(`From cookie: [${device_name}]`);
        if (device_name == null || device_name == "") {
            console.log("Default device_name");
            device_name = '84f3eb536123';
        }
    }

    setCookie("device_name", device_name, 9999);

    device_name_input_field.value = device_name;

    var url = new URL(`https://${host}/readings`);
    url.searchParams.set('from', then.toISOString());
    url.searchParams.set('to', now.toISOString());
    url.searchParams.set('device', device_name);
    console.log(device_name);
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
                    reverse: true,
                    format: function (x) {
                        return `${x / 10}cm`
                    }
                },
                {
                    field: 'vbat',
                    color: '#FF8C00',
                    reverse: false,
                    format: function (x) {
                        return `${x / 100}v`
                    }
                },
                {
                    field: 'rssi',
                    color: '#4169E1',
                    reverse: false,
                    format: function (x) {
                        return `${x}dBm`
                    }
                },
            ];

            var config = settings[idx]

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
                borderColor: config.color,
                backgroundColor: config.color,
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

(function () { show_graph(0); })();
