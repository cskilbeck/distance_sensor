#pragma once

#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

//////////////////////////////////////////////////////////////////////

#define SUPPRESS_DEPRECATED _Pragma("GCC diagnostic push") _Pragma("GCC diagnostic ignored \"-Wdeprecated-declarations\"")
#define SUPPRESS_POP _Pragma("GCC diagnostic pop")

//////////////////////////////////////////////////////////////////////

#define ASSERT(x)                                                                                                            \
    do {                                                                                                                     \
        if(!(x)) {                                                                                                           \
            printf("\033[0;31m\n\n=====\n\nASSERT '%s' failed\nLINE %d\nFILE %s\n\n=====\n\033[0m", #x, __LINE__, __FILE__); \
            fflush(stdout);                                                                                                  \
            fsync(fileno(stdout));                                                                                           \
            abort();                                                                                                         \
        }                                                                                                                    \
    } while(false)

//////////////////////////////////////////////////////////////////////

void log_buffer(char const *tag, void const *buffer, uint16_t buff_len, esp_log_level_t log_level);
esp_err_t i2c_master_write_write_device(i2c_port_t i2c_num, uint8_t dev_addr, uint8_t const *b1, size_t s1, uint8_t const *b2, size_t s2, TickType_t timeout);

//////////////////////////////////////////////////////////////////////

#define MAYBE_UNUSED __attribute__((unused))

//#define NO_LOGGING

#if defined(NO_LOGGING)

#define LOG_NOP \
    do {        \
    } while(0)

#define LOG_CONTEXT(x)

#define LOG_ERROR(...) LOG_NOP
#define LOG_WARN(...) LOG_NOP
#define LOG_INFO(...) LOG_NOP
#define LOG_VERBOSE(...) LOG_NOP
#define LOG_DEBUG(...) LOG_NOP

#define LOG_SET_LEVEL(...) LOG_NOP

#define LOG_BUFFER(...) LOG_NOP

#define LOG_FLUSH(...) LOG_NOP

#else

#define LOG_CONTEXT(x) static char const *__log_tag MAYBE_UNUSED = x

#define LOG_ERROR(...) ESP_LOGE(__log_tag, __VA_ARGS__)
#define LOG_WARN(...) ESP_LOGW(__log_tag, __VA_ARGS__)
#define LOG_INFO(...) ESP_LOGI(__log_tag, __VA_ARGS__)
#define LOG_VERBOSE(...) ESP_LOGV(__log_tag, __VA_ARGS__)
#define LOG_DEBUG(...) ESP_LOGD(__log_tag, __VA_ARGS__)

#define LOG_SET_LEVEL(channel, level) esp_log_level_set(channel, level)

#define LOG_BUFFER(level, data, len)                 \
    do {                                             \
        if(LOG_LOCAL_LEVEL >= level)                 \
            log_buffer(__log_tag, data, len, level); \
    } while(0)

#define LOG_FLUSH(...)         \
    do {                       \
        fflush(stdout);        \
        fsync(fileno(stdout)); \
    } while(0)

#endif

//////////////////////////////////////////////////////////////////////

#define ESP_RET(x)                                                                        \
    do {                                                                                  \
        esp_err_t __err = (x);                                                            \
        if(__err != ESP_OK) {                                                             \
            LOG_ERROR("%s failed: 0x%08x (%s)", #x, (uint)__err, esp_err_to_name(__err)); \
            return __err;                                                                 \
        }                                                                                 \
    } while(0)

//////////////////////////////////////////////////////////////////////

#define ESP_LOG(x)                                                                        \
    do {                                                                                  \
        esp_err_t __err = (x);                                                            \
        if(__err != ESP_OK) {                                                             \
            LOG_ERROR("%s failed: 0x%08x (%s)", #x, (uint)__err, esp_err_to_name(__err)); \
        }                                                                                 \
    } while(0)

//////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
}
#endif

//////////////////////////////////////////////////////////////////////
// templates not allowed in extern "C" section

#ifdef __cplusplus

#include <utility>

namespace
{
    //////////////////////////////////////////////////////////////////////

    inline uint8_t int_to_bcd(int n)
    {
        return (n % 10) | ((n / 10) << 4);
    }

    //////////////////////////////////////////////////////////////////////

    inline int bcd_to_int(uint8_t n)
    {
        return (n & 0xf) + ((n >> 4) * 10);
    }

    //////////////////////////////////////////////////////////////////////

    template <typename T, size_t N> constexpr size_t countof(T const (&)[N]) noexcept
    {
        return N;
    }

    //////////////////////////////////////////////////////////////////////

    template <typename T> T min(T x, T y)
    {
        return (x <= y) ? x : y;
    }

    //////////////////////////////////////////////////////////////////////

    template <typename T> T max(T x, T y)
    {
        return (x >= y) ? x : y;
    }

    //////////////////////////////////////////////////////////////////////

    template <typename F> class defer_finalizer
    {
        F f;
        bool moved;

    public:
        template <typename T> defer_finalizer(T &&f_) : f(std::forward<T>(f_)), moved(false)
        {
        }

        defer_finalizer(const defer_finalizer &) = delete;

        defer_finalizer(defer_finalizer &&other) : f(std::move(other.f)), moved(other.moved)
        {
            other.moved = true;
        }

        ~defer_finalizer()
        {
            if(!moved) {
                f();
            }
        }
    };

    //////////////////////////////////////////////////////////////////////

    static struct
    {
        template <typename F> defer_finalizer<F> operator<<(F &&f)
        {
            return defer_finalizer<F>(std::forward<F>(f));
        }
    } deferrer __attribute__((__used__));

}    // namespace

//////////////////////////////////////////////////////////////////////

#define DEFER_TOKENPASTE(x, y) x##y
#define DEFER_TOKENPASTE2(x, y) DEFER_TOKENPASTE(x, y)
#define SCOPED auto DEFER_TOKENPASTE2(__deferred_lambda_call, __COUNTER__) = deferrer <<
#define DEFER(X) \
    SCOPED[=]    \
    {            \
        X;       \
    };

#endif
