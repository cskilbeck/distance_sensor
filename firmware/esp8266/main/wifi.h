#pragma once

void init_wifi();
void deinit_wifi();

typedef void (*wifi_callback)();

extern wifi_callback on_wifi_connected;
extern wifi_callback on_wifi_disconnected;
