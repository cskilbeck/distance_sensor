var chart;

function show_graph(idx) {

    var date_format = function (x) {

        return new Date(x).toLocaleString('en-GB', {

            day: 'numeric',
            month: 'short',
            hour: 'numeric',
            minute: 'numeric',
            timezone: 'UTC'
        });
    };

    var now = new Date(new Date().setDate(new Date().getDate() + 1));
    var then = new Date(new Date().setDate(now.getDate() - 121));

    var host = "vibue.com";
    //var host = "localhost";

    var url = new URL(`https://${host}/readings`);
    url.searchParams.set('from', then.toISOString());
    url.searchParams.set('to', now.toISOString());
    url.searchParams.set('device', '84f3eb536123');
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
                lineTension: 0,
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
