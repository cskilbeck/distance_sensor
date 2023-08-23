//////////////////////////////////////////////////////////////////////

#pragma once

//////////////////////////////////////////////////////////////////////

typedef void (*wifi_callback)();

extern wifi_callback on_wifi_connected;
extern wifi_callback on_wifi_disconnected;

//////////////////////////////////////////////////////////////////////

void wifi_init();
void wifi_deinit();

int8 wifi_get_rssi();