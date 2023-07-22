//////////////////////////////////////////////////////////////////////
// Very very basic WebSocket client
//
// 1. call connect()
// 2. call update() every 10..100 ms
// 3. call send(...) to add things to the send buffer
//      the send buffer will be flushed inside update()
//      messages received inside update() will trigger on_message callback
//
// synchronous, polling architecture
// no memory allocations
// memory usage around 3 * buffer_size
//
// payload size limited by buffer size (eg 512 - 8 = 504 bytes)
// only tested against Gorilla/go websocket server
// it's not thread safe (really not)
// continuations not supported
// tested on Win32 and ESP8266
// no UTF8 testing on text payloads (why the hell did they put that in the spec?)
//
//////////////////////////////////////////////////////////////////////

#pragma once

//////////////////////////////////////////////////////////////////////

namespace websocket
{
    //////////////////////////////////////////////////////////////////////

    static nullptr_t constexpr null = nullptr;
    using byte = uint8_t;

    //////////////////////////////////////////////////////////////////////
    // circular buffer for sending and receiving

    template <int N> struct ring_buffer
    {
        static int constexpr capacity = N;

        size_t front = 0;
        size_t size = 0;

        byte data[capacity];

        size_t free() const
        {
            return capacity - size;
        }

        size_t back() const
        {
            return (front + size) % capacity;
        }

        bool empty() const
        {
            return size == 0;
        }

        void reset()
        {
            front = 0;
            size = 0;
        }

        void push(byte c)
        {
            if(free() > 0) {
                data[back()] = c;
                size += 1;
            }
        }

        int inc(int n)
        {
            return (front + n) % capacity;
        }

        int _min(int a, int b)
        {
            return (a < b) ? a : b;
        }

        void push(byte const *p, size_t n)
        {
            n = _min(free(), n);
            size_t f = back();
            size += n;
            size_t r = _min(capacity - f, n);
            if(r != 0) {
                memcpy(data + f, p, r);
            }
            if(n > r) {
                memcpy(data, p + r, n - r);
            }
        }

        int pop()
        {
            if(empty()) {
                return -1;
            }
            byte c = data[front];
            front = inc(1);
            size -= 1;
            return c;
        }

        int pop(byte *dst, size_t n)
        {
            n = _min(size, n);
            for(size_t i = 0; i < n; ++i) {
                *dst++ = data[front];
                front = inc(1);
            }
            size -= n;
            return n;
        }

        byte &at(size_t n)
        {
            return data[inc(n)];
        }

        void flush(size_t n)
        {
            if(n > size) {
                n = size;
            }
            size -= n;
            front = inc(n);
        }
    };

    //////////////////////////////////////////////////////////////////////

    struct client
    {
        typedef void (*callback)(byte const *, int);

        callback on_message;

        int open(char const *address, char const *port, char const *path);

        void _close(char const *reason);
        bool connected() const;
        void update();

        int send_ping();
        int send_string(char const *message, size_t len);
        int send_binary(byte const *message, size_t len);

        //////////////////////////////////////////////////////////////////////

        enum class state
        {
            connecting,
            open,
            closing,
            closed,
        };

        enum class opcode : byte
        {
            continuation = 0,
            text = 1,
            binary = 2,
            close = 8,
            ping = 9,
            pong = 10,
            opcode_count = 11
        };

        int send_data(opcode type, byte const *msg, size_t message_size);
        int setup_header(opcode type, size_t message_size);

        template <int N> int send_data(client::opcode opcode, ring_buffer<N> buffer, size_t size)
        {
            if(setup_header(opcode, size) < 0) {
                return -1;
            }
            for(int i = 0; i != size; ++i) {
                txbuf.push(buffer.pop() ^ mask_bytes[i & 3]);
            }
            return size;
        }

        state current_state = state::closed;
        int sock = -1;

        static byte const mask_bytes[4];

        ring_buffer<128> rxbuf;
        ring_buffer<128> txbuf;
    };
}    // namespace websocket
