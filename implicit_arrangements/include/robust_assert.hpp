#pragma once

#include <sstream>

template <typename T>
void throw_LA_ASSERT(const T msg, const char* file, int line)
{
    std::ostringstream oss;
    oss << "Robustness error: " << file << " line: " << line << " " << msg;
    throw std::runtime_error(oss.str());
}

#define LA_GET_3RD_ARG_HELPER(arg1, arg2, arg3, ...) arg3

#define LA_Assert_1(cond) \
    if (!(cond)) { throw_LA_ASSERT("", __FILE__, __LINE__); }

#define LA_Assert_2(cond, msg) \
    if (!(cond)) { throw_LA_ASSERT(msg, __FILE__, __LINE__); }

#if (_MSC_VER && !(__clang__ || __GNUC__)) // special VS variadics

#define LA_MSVS_EXPAND(x) x

#define LA_ASSERT(...)                                                           \
    LA_MSVS_EXPAND(LA_GET_3RD_ARG_HELPER(__VA_ARGS__, LA_Assert_2, LA_Assert_1)) \
    LA_MSVS_EXPAND((__VA_ARGS__))

#else
#define LA_ASSERT(...) LA_GET_3RD_ARG_HELPER(__VA_ARGS__, LA_Assert_2, LA_Assert_1)(__VA_ARGS__)
#endif

#ifndef RELEASE_BRANCH
#define ROBUST_ASSERT(x) \
    do {                 \
        LA_ASSERT(x);    \
    } while (0)
#else
#define ROBUST_ASSERT(x) ((void)0)
#endif