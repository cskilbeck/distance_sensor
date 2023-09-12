//////////////////////////////////////////////////////////////////////

// the chart object which is replaced each time a new graph is shown

let chart;

// where to get the data from

let host = "vibue.com";
//let host = "localhost";

// graph settings (formats, colors etc) - see init_settings()

let settings;

//////////////////////////////////////////////////////////////////////
// format a date for the chart axes and popups

function date_format(x) {

    return new Date(x).toLocaleString('en-US', {

        day: 'numeric',
        month: 'short',
        hour: 'numeric',
        minute: 'numeric'
    });
}

//////////////////////////////////////////////////////////////////////
// convert a Number to a 32 bit hex string

function hex32(number) {

    if (number < 0) {
        number = 0xFFFFFFFF + number + 1;
    }
    let n = number.toString(16).toUpperCase();
    while (n.length < 8) {
        n = '0' + n;
    }
    return n;
}

//////////////////////////////////////////////////////////////////////
// get background color of an element as a 32 bit Number (with alpha = ff)

function get_button_color(id) {

    let button = document.getElementById(id);
    let style = getComputedStyle(button);
    let color = style.getPropertyValue('background-color');

    // @DODGY this relies on the color being serialized as rgb(r,g,b) or rgba(r,g,b,a)
    // which is not guaranteed by the spec although in practice it's fine for now
    let [r, g, b] = color.match(/\d+/g).map(Number);

    return (r << 24) | (g << 16) | (b << 8) | 0xff;
}

//////////////////////////////////////////////////////////////////////
// set a cookie

function set_cookie(name, value, days) {

    days = days || 400;
    let expires = "";
    let date = new Date();
    date.setTime(date.getTime() + (days * 24 * 60 * 60 * 1000));
    expires = "; expires=" + date.toUTCString();
    document.cookie = name + "=" + (value || "") + expires + "; path=/";
}

//////////////////////////////////////////////////////////////////////
// get value of a cookie (or null)

function get_cookie(name) {

    let f = name + "=";
    for (let c of document.cookie.split(';')) {
        let t = c.trimStart();
        if (t.indexOf(f) == 0) {
            return t.substring(f.length);
        }
    }
    return null;
}

//////////////////////////////////////////////////////////////////////
// erase a cookie

function erase_cookie(name) {

    document.cookie = name + '=; Path=/; Expires=Thu, 01 Jan 1970 00:00:01 GMT;';
}

//////////////////////////////////////////////////////////////////////
// choose a device (called by the device dropdown)

function set_device(ev, name, address) {

    ev.preventDefault()
    set_cookie("device_name", name);
    set_cookie("device_address", address);
    show_graph();
}

//////////////////////////////////////////////////////////////////////
// init the settings for each type of graph

function init_settings() {

    // graph colors track the button colors

    let color0 = get_button_color('button0');
    let color1 = get_button_color('button1');
    let color2 = get_button_color('button2');

    let alpha = 0xa0;

    let alpha0 = (color0 & 0xffffff00) | alpha;
    let alpha1 = (color1 & 0xffffff00) | alpha;
    let alpha2 = (color2 & 0xffffff00) | alpha;

    settings = [
        {
            field: 'distance',
            color: `#${hex32(color0)}`,
            lineColor: `#${hex32(alpha0)}`,
            reverse: true,
            format: function (x) {
                return `${+parseFloat(x / 10).toFixed(1)}cm`
            }
        },
        {
            field: 'vbat',
            color: `#${hex32(color1)}`,
            lineColor: `#${hex32(alpha1)}`,
            reverse: false,
            format: function (x) {
                return `${+parseFloat(x / 100).toFixed(2)}v`
            }
        },
        {
            field: 'rssi',
            color: `#${hex32(color2)}`,
            lineColor: `#${hex32(alpha2)}`,
            reverse: false,
            format: function (x) {
                return `${x}dBm`
            }
        },
    ];
}

//////////////////////////////////////////////////////////////////////
// get all the devices, populate the dropdown and show the graph

function refresh_devices() {

    let xhr = new XMLHttpRequest();
    xhr.responseType = 'json';

    let url = new URL(`https://${host}/devices`);
    url.searchParams.set('nonce', Math.random());

    xhr.open('GET', url);

    xhr.onload = function () {

        if (xhr.status != 200) {
            alert(xhr.response.info.join('\n'));

        } else {

            let items = ""

            let rows = xhr.response["rows"];
            let devices = xhr.response["device"];
            for (let i = 0; i < rows; ++i) {
                let device = devices[i];
                let name = device['name']
                let addr = device['address']
                items += `<a onClick='set_device(event, "${name}", "${addr}")' class="dropdown-item" href="${name}"><span class='text-info'>${device['address']}</span><span>&nbsp;${device['name']}</span></a>`
            }
            let dropdown = document.getElementById('dropdown_device_items');
            dropdown.innerHTML = items;
            show_graph();
        }
    }
    xhr.send();
}

//////////////////////////////////////////////////////////////////////
// show one of the graphs

function show_graph(idx) {

    let now = new Date(new Date().setDate(new Date().getDate() + 1));
    let then = new Date(new Date().setDate(now.getDate() - 120));

    let graph_num_str = idx || get_cookie("graph_num") || "0";
    let device_name = get_cookie("device_name") || "?";
    let device_address = get_cookie("device_address") || "84f3eb536123";

    set_cookie("graph_num", graph_num_str);
    set_cookie("device_name", device_name);
    set_cookie("device_address", device_address);

    let graph_num = parseInt(graph_num_str);

    for (let i = 0; i < 3; ++i) {
        let btn = document.getElementById(`button${i}`);
        if (i == graph_num) {
            btn.classList.add('highlighted');
            btn.classList.remove('inactive');
        } else {
            btn.classList.remove('highlighted');
            btn.classList.add('inactive');
        }
    }

    let device_name_input_field = document.getElementById('device_address');
    device_name_input_field.value = `${device_name} (${device_address})`;

    let url = new URL(`https://${host}/readings`);
    url.searchParams.set('from', then.toISOString());
    url.searchParams.set('to', now.toISOString());
    url.searchParams.set('device', device_address);
    url.searchParams.set('nonce', Math.random());

    let xhr = new XMLHttpRequest();
    xhr.responseType = 'json';

    xhr.open('GET', url);

    xhr.onload = function () {

        if (xhr.status != 200) {
            alert(xhr.response.info.join('\n'));

        } else {

            let config = settings[graph_num]

            let datasets = []
            let data = [];

            let rsp = xhr.response;

            for (let i = 0; i < rsp.rows; ++i) {
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

//////////////////////////////////////////////////////////////////////
// main

(function () {
    init_settings();
    refresh_devices();
})();
