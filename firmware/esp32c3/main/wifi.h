//////////////////////////////////////////////////////////////////////

#pragma once

//////////////////////////////////////////////////////////////////////
// events returned from wifi_wait_for_event()

uint32 constexpr wifi_event_connected = BIT0;
uint32 constexpr wifi_event_disconnected = BIT1;

//////////////////////////////////////////////////////////////////////

esp_err_t wifi_init();
esp_err_t wifi_factory_reset();
uint32 wifi_wait_for_event(TickType_t timeout);
int8 wifi_get_rssi();