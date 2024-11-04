#pragma once

#include <stack>
#include <unordered_map>
#include <iostream>

#include <tbb/tick_count.h>

class ScopedTimer
{
public:
    ScopedTimer(const char* label) : m_label(label) { m_tic = tbb::tick_count::now(); }

    auto toc() const
    {
        auto                          t = tbb::tick_count::now();
        std::chrono::duration<double> d = t - m_tic;
        return d.count();
    }

    friend class labelled_timers_manager;

private:
    const char*     m_label{};
    tbb::tick_count m_tic{};
};

struct timer_statistics {
    double elapsed_time{};
    size_t count{};
};

class labelled_timers_manager
{
public:
    labelled_timers_manager() = default;

    void clear() { m_timer_statistics.clear(); }

    void print() const{
        for (const auto& [label, stats] : m_timer_statistics) {
            std::cout << label << ": " << stats.elapsed_time / stats.count << "s" << std::endl;
        }
    }

    void push_timer(const char* label) { m_label_stack.emplace(label); }

    void pop_timer(const char* label)
    {
        const auto& timer = m_label_stack.top();
        if (strcmp(timer.m_label, label) != 0) {
            std::cerr << "Error: popped timer label does not match the top of the stack" << std::endl;
        }
        m_timer_statistics[timer.m_label].elapsed_time += timer.toc();
        m_timer_statistics[timer.m_label].count++;
        m_label_stack.pop();
    }

private:
    std::stack<ScopedTimer>                                m_label_stack{};
    std::unordered_map<const char*, timer_statistics> m_timer_statistics{};
};