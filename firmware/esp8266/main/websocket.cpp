// //////////////////////////////////////////////////////////////////////

// #if defined(_WIN32)

// #define WIN32_LEAN_AND_MEAN

// #include <windows.h>
// #include <winsock2.h>
// #include <ws2tcpip.h>
// #include <cstdint>
// #include <stdlib.h>
// #include <stdio.h>
// #include <errno.h>

// #pragma comment(lib, "Ws2_32.lib")
// #pragma comment(lib, "Mswsock.lib")
// #pragma comment(lib, "AdvApi32.lib")

#define ESP_LOG_NOP(x, y, ...) \
    do {                       \
    } while(0)

#define ESP_LOG(x, y, ...)      \
    do {                        \
        printf(y, __VA_ARGS__); \
        printf("\n");           \
    } while(0)

// #define ESP_LOGE(x, y, ...) ESP_LOG(x, y, __VA_ARGS__)
// #define ESP_LOGW(x, y, ...) ESP_LOG(x, y, __VA_ARGS__)
// #define ESP_LOGI(x, y, ...) ESP_LOG(x, y, __VA_ARGS__)
// #define ESP_LOGV(x, y, ...) ESP_LOG(x, y, __VA_ARGS__)
// #define ESP_LOGD(x, y, ...) ESP_LOG_NOP(x, y, __VA_ARGS__)

// #define countof _countof

// #define ERR_WOULD_BLOCK WSAEWOULDBLOCK

// //////////////////////////////////////////////////////////////////////

// #else

// #include <string.h>
// #include <sys/param.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/event_groups.h"
// #include "esp_system.h"
// #include "esp_wifi.h"
// #include "esp_event_loop.h"
// #include "esp_log.h"
// #include "nvs_flash.h"

// #include "lwip/err.h"
// #include "lwip/sockets.h"
// #include "lwip/sys.h"
// #include <lwip/netdb.h>

// #include <errno.h>

// #define ERR_WOULD_BLOCK EWOULDBLOCK


// #endif

// //////////////////////////////////////////////////////////////////////

// #include "util.h"
// #include "websocket.h"

// //////////////////////////////////////////////////////////////////////

// namespace
// {
//     using websocket::byte;
//     using websocket::client;
//     using websocket::ring_buffer;
//     using opcode = client::opcode;

//     char const *TAG = "websocket";

//     byte socket_buffer[512];

//     byte const mask_bytes[4] = { 0x12, 0x34, 0x56, 0x78 };

//     char const http_header_id[] = "HTTP/1.1 101";

//     char const *opcode_names[static_cast<int>(client::opcode::opcode_count)] = {
//         "continuation",    // 0
//         "text",            // 1
//         "binary",          // 2
//         "?3",              // 3
//         "?4",              // 4
//         "?5",              // 5
//         "?6",              // 6
//         "?7",              // 7
//         "close",           // 8
//         "ping",            // 9
//         "pong",            // 10
//     };

//     //////////////////////////////////////////////////////////////////////
//     // write_line like printf but to a socket via a buffer

//     int write_line(int sock, char *buffer, size_t buffer_len, char const *fmt, ...)
//     {
//         va_list v;
//         va_start(v, fmt);
//         int len = vsnprintf(buffer, buffer_len, fmt, v);
//         return send(sock, buffer, len, 0);
//     }

//     //////////////////////////////////////////////////////////////////////
//     // read_line
//     // read a line from a socket
//     // returns length of line not including null terminator
//     // or -1 if there was a recv() error
//     // or -2 if line overflowed

//     int read_line(int sock, char *buffer, size_t buffer_len)
//     {
//         char prev_chr = 0;
//         char *p = buffer;
//         for(size_t i = 0; i < buffer_len - 1; ++i) {
//             char c;
//             if(recv(sock, &c, 1, 0) <= 0) {
//                 return -1;
//             }
//             *p++ = c;
//             if(c == '\n' && prev_chr == '\r') {
//                 size_t len = p - buffer;
//                 *p++ = 0;
//                 ESP_LOGV(TAG, "RESPONSE: %.*s", len - 2, buffer);
//                 return static_cast<int>(len);
//             }
//             prev_chr = c;
//         }
//         return -2;
//     }

//     //////////////////////////////////////////////////////////////////////

//     char const *opcode_name(opcode code)
//     {
//         int x = static_cast<uint8_t>(code);
//         if(x < countof(opcode_names)) {
//             return opcode_names[x];
//         }
//         return "??";
//     }

//     //////////////////////////////////////////////////////////////////////
//     // get header size for a given payload size

//     int get_tx_header_size(int size)
//     {
//         if(size < 126) {
//             return 6;
//         }
//         if(size < 65536) {
//             return 8;
//         }
//         return 14;
//     }

//     //////////////////////////////////////////////////////////////////////
//     // workarounds for win32 vs others

//     int socket_error()
//     {
// #if defined(_WIN32)
//         return WSAGetLastError();
// #else
//         return errno;
// #endif
//     }

//     int set_non_blocking(int socket)
//     {
// #if defined(_WIN32)
//         u_long mode = 1;
//         return ioctlsocket(socket, FIONBIO, &mode);
// #else
//         return fcntl(socket, F_SETFL, O_NONBLOCK);
// #endif
//     }

// }    // namespace

// //////////////////////////////////////////////////////////////////////

// namespace websocket
// {
//     byte const client::mask_bytes[4] = { 0x12, 0x34, 0x56, 0x78 };

//     int client::open(char const *address, char const *port, char const *path)
//     {
//         struct addrinfo hints;
//         struct addrinfo *result = null;

//         memset(&hints, 0, sizeof(hints));
//         hints.ai_family = AF_UNSPEC;
//         hints.ai_socktype = SOCK_STREAM;
//         hints.ai_protocol = IPPROTO_TCP;

//         auto err = [&](int n, char const *reason = null) {
//             if(n < 0) {
//                 int err = socket_error();
//                 if(sock > 0) {
//                     closesocket(sock);
//                     sock = -1;
//                 }
//                 if(reason != null) {
//                     ESP_LOGE(TAG, "%s: %d", reason, err);
//                 }
//             }
//             freeaddrinfo(result);
//             return n;
//         };

//         current_state = state::closed;
//         sock = -1;

//         if(getaddrinfo(address, port, &hints, &result) != 0) {
//             return err(-1, "getaddrinfo() failed");
//         }

//         sock = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
//         if(sock < 0) {
//             return err(-1, "socket() failed");
//         }

//         for(auto ad = result; ad != null; ad = ad->ai_next) {
//             ESP_LOGI(TAG, "LEN: %d", ad->ai_addrlen);
//             struct sockaddr_in *a = (struct sockaddr_in *)(ad->ai_addr);
//             uint32_t d = a->sin_addr.s_addr;
//             ESP_LOGI(TAG, "ADDR: %08x (%d.%d.%d.%d)", d, d & 0xff, (d >> 8) & 0xff, (d >> 16) & 0xff, (d >> 24) & 0xff);
//         }

//         if(connect(sock, result->ai_addr, result->ai_addrlen) != 0) {
//             return err(-1, "connect() failed");
//         }

//         current_state = state::connecting;

//         ESP_LOGI(TAG, "socket connected to %s:%s", address, port);

//         // hijack tx buffer for headers
//         char *buf = reinterpret_cast<char *>(txbuf.data);
//         size_t len = countof(txbuf.data);

//         // clang-format off
//         if(write_line(sock, buf, len, "GET /%s HTTP/1.1\r\n", path) < 0 ||
//            write_line(sock, buf, len, "Host: %s:%s\r\n", address, port) < 0 ||
//            write_line(sock, buf, len, "Upgrade: websocket\r\n") < 0 ||
//            write_line(sock, buf, len, "Connection: Upgrade\r\n") < 0 ||
//            write_line(sock, buf, len, "X-DeviceID: 54321\r\n") < 0 ||
//            write_line(sock, buf, len, "Sec-WebSocket-Key: x3JJHMbDL1EzLkh9GBhXDw==\r\n") < 0 ||
//            write_line(sock, buf, len, "Sec-WebSocket-Version: 13\r\n") < 0 ||
//            write_line(sock, buf, len, "\r\n") < 0) {

//             return err(-1, "write_line failed");
//         }
//         // clang-format on

//         if(read_line(sock, buf, len) < 0) {
//             return err(-1, "error reading status line");
//         }

//         if(memcmp(buf, http_header_id, countof(http_header_id) - 1) != 0) {
//             ESP_LOGE(TAG, "bad status: %s", buf);
//             return err(-1);
//         }

//         // flush response headers for now
//         while(true) {
//             if(read_line(sock, buf, len) < 0) {
//                 return err(-1, "error reading response headers");
//             }
//             if(buf[0] == '\r' && buf[1] == '\n') {
//                 break;
//             }
//         }

//         // configure socket
//         int flag = 1;
//         if(setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(flag)) < 0) {
//             return err(-1, "setsockopt failed");
//         }

//         if(set_non_blocking(sock) < 0) {
//             return err(-1, "set_non_blocking failed");
//         }

//         ESP_LOGI(TAG, "websocket connected to %s:%s", address, port);
//         current_state = state::open;
//         txbuf.reset();
//         rxbuf.reset();

//         return err(0);
//     }

//     //////////////////////////////////////////////////////////////////////
//     // send close msg to server

//     void client::_close(char const *reason)
//     {
//         if(current_state == state::closing || current_state == state::closed) {
//             return;
//         }
//         send_data(opcode::close, reinterpret_cast<byte const *>(reason), strlen(reason));
//         current_state = state::closing;
//     }

//     //////////////////////////////////////////////////////////////////////

//     bool client::connected() const
//     {
//         return current_state != state::closed;
//     }

//     //////////////////////////////////////////////////////////////////////

//     int client::send_ping()
//     {
//         return send_data(opcode::ping, null, 0);
//     }

//     //////////////////////////////////////////////////////////////////////

//     int client::send_string(char const *message, size_t len)
//     {
//         return send_data(opcode::text, reinterpret_cast<byte const *>(message), len);
//     }

//     //////////////////////////////////////////////////////////////////////

//     int client::send_binary(byte const *message, size_t len)
//     {
//         return send_data(opcode::binary, message, len);
//     }

//     //////////////////////////////////////////////////////////////////////
//     // send and recv outstanding data

//     void client::update()
//     {
//         if(current_state == state::closed) {
//             return;
//         }

//         // RECV into rxbuf until WOULDBLOCK or rxbuf is full
//         while(rxbuf.free() != 0) {
//             int ret = recv(sock, (char *)socket_buffer, countof(socket_buffer), 0);
//             if(ret == 0) {
//                 break;
//             }
//             if(ret < 0) {
//                 int err = socket_error();
//                 if(err != ERR_WOULD_BLOCK) {
//                     closesocket(sock);
//                     current_state = state::closed;
//                     ESP_LOGE(TAG, "Error during recv %d", err);
//                 }
//                 break;
//             }
//             ESP_LOGD(TAG, "recv(sock, buf, %d, 0 returned %d", rxbuf.free(), ret);
//             rxbuf.push(socket_buffer, ret);
//         }

//         // SEND from txbuf until WOULDBLOCK or none left to send
//         while(txbuf.size != 0) {
//             int s = txbuf.pop(socket_buffer, countof(socket_buffer));
//             int ret = ::send(sock, reinterpret_cast<char const *>(socket_buffer), s, 0);
//             if(ret < 0) {
//                 int err = socket_error();
//                 if(err != ERR_WOULD_BLOCK) {
//                     closesocket(sock);
//                     current_state = state::closed;
//                     ESP_LOGE(TAG, "Error during send %d", err);
//                 }
//                 break;
//             }
//         }

//         // if closing and all tx sent, then close
//         if(current_state == state::closing) {
//             ESP_LOGI(TAG, "Finished sending close msg");
//             closesocket(sock);
//             current_state = state::closed;
//             return;
//         }

//         if(rxbuf.size < 2) {
//             return;
//         }

//         size_t header_size = 2;

//         byte d0 = rxbuf.at(0);
//         // int final_bit = (d0 & 0x80) != 0;
//         opcode code = static_cast<opcode>(d0 & 0x0f);

//         byte d1 = rxbuf.at(1);
//         int mask_bit = (d1 & 0x80) != 0;
//         int payload_size = d1 & 0x7f;

//         // long payloads not supported
//         if(payload_size == 127) {
//             return;
//         }

//         if(payload_size == 126) {
//             header_size += 2;
//             payload_size = (static_cast<uint16_t>(rxbuf.at(2) << 8)) | rxbuf.at(3);
//         }

//         if(mask_bit) {
//             header_size += 4;
//         }

//         if(rxbuf.size < header_size) {
//             return;
//         }

//         byte mask[4];

//         if(mask_bit) {
//             for(int i = 0; i < 4; ++i) {
//                 mask[i] = rxbuf.at(header_size - 4 + i);
//             }
//         } else {
//             memset(mask, 0, 4);
//         }

//         if(rxbuf.size < (header_size + payload_size)) {
//             return;
//         }

//         rxbuf.flush(header_size);

//         if(mask_bit) {
//             for(size_t i = 0; i != payload_size; ++i) {
//                 rxbuf.at(i) ^= mask[i & 0x3];
//             }
//         }

//         ESP_LOGI(TAG, "RECV %-10s (%2d) mask %d, size %-5d", opcode_name(code), static_cast<int>(code), mask_bit != 0, payload_size);

//         switch(code) {
//         case opcode::continuation:
//             break;
//         case opcode::text:
//             rxbuf.pop(socket_buffer, payload_size);
//             on_message(socket_buffer, payload_size);
//             break;
//         case opcode::binary:
//             rxbuf.pop(socket_buffer, payload_size);
//             on_message(socket_buffer, payload_size);
//             break;
//         case opcode::ping:
//             send_data(opcode::pong, rxbuf, payload_size);
//             break;
//         case opcode::pong:
//             break;
//         case opcode::close:
//             closesocket(sock);
//             current_state = state::closed;
//             break;
//         default:
//             ESP_LOGE(TAG, "Unknown opcode %d (size %d)", static_cast<int>(code), payload_size);
//             break;
//         }
//     }

//     //////////////////////////////////////////////////////////////////////

//     int client::setup_header(opcode code, size_t size)
//     {
//         size_t header_size = get_tx_header_size(size);

//         if(txbuf.free() < (size + header_size)) {
//             return -1;
//         }

//         if(current_state == state::closing || current_state == state::closed) {
//             ESP_LOGE(TAG, "Can't send %d, state = %d", static_cast<int>(code), static_cast<int>(current_state));
//             return -2;
//         }

//         ESP_LOGI(TAG, "SEND %-10s (%2d) mask %d, size %-5d (header is %d bytes)", opcode_name(code), static_cast<int>(code), 1, size, header_size);

//         // always fin - spanned payloads not supported
//         txbuf.push(0x80 | static_cast<byte>(code));

//         byte mask_bit = 0x80;

//         if(size < 126) {
//             txbuf.push(static_cast<byte>(size) | mask_bit);

//         } else if(size < 65536) {
//             txbuf.push(126 | mask_bit);
//             txbuf.push((size >> 8) & 0xff);
//             txbuf.push(size & 0xff);
//         }

//         txbuf.push(mask_bytes, 4);

//         return 0;
//     }

//     //////////////////////////////////////////////////////////////////////

//     int client::send_data(opcode code, byte const *msg, size_t size)
//     {
//         if(setup_header(code, size) < 0) {
//             ESP_LOGE(TAG, "No room to send %d bytes for opcode %s", size, opcode_name(code));
//             return -1;
//         }

//         for(int i = 0; i != size; ++i) {
//             txbuf.push(msg[i] ^ mask_bytes[i & 3]);
//         }
//         return size;
//     }

//     //////////////////////////////////////////////////////////////////////

// }    // namespace websocket
