#pragma once

#include <utility>

namespace
{
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

    static struct
    {
        template <typename F> defer_finalizer<F> operator<<(F &&f)
        {
            return defer_finalizer<F>(std::forward<F>(f));
        }
    } deferrer __attribute__((__used__));

}    // namespace

#define DEFER_TOKENPASTE(x, y) x##y
#define DEFER_TOKENPASTE2(x, y) DEFER_TOKENPASTE(x, y)
#define SCOPED auto DEFER_TOKENPASTE2(__deferred_lambda_call, __COUNTER__) = deferrer <<
#define DEFER(X) \
    SCOPED[=]    \
    {            \
        X;       \
    };
