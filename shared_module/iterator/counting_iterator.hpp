#pragma once

#include <iterator>

template <typename T>
class counting_iterator
{
public:
    using iterator_category = std::random_access_iterator_tag;
    using value_type        = T;
    using difference_type   = std::ptrdiff_t;
    using pointer           = T*;
    using reference         = T&;

    constexpr counting_iterator() noexcept : current_(0) {}

    constexpr counting_iterator(T value) noexcept : current_(value) {}

    constexpr counting_iterator& operator++() noexcept
    {
        ++current_;
        return *this;
    }

    constexpr counting_iterator operator++(int) noexcept
    {
        auto tmp = *this;
        ++current_;
        return tmp;
    }

    constexpr counting_iterator& operator--() noexcept
    {
        --current_;
        return *this;
    }

    constexpr counting_iterator operator--(int) noexcept
    {
        auto tmp = *this;
        --current_;
        return tmp;
    }

    constexpr counting_iterator& operator+=(difference_type n) noexcept
    {
        current_ += n;
        return *this;
    }

    constexpr counting_iterator& operator-=(difference_type n) noexcept
    {
        current_ -= n;
        return *this;
    }

    constexpr counting_iterator operator+(difference_type n) const noexcept
    {
        auto tmp    = *this;
        return tmp += n;
    }

    constexpr counting_iterator operator-(difference_type n) const noexcept
    {
        auto tmp    = *this;
        return tmp -= n;
    }

    constexpr difference_type operator-(const counting_iterator& other) const noexcept { return current_ - other.current_; }

    constexpr reference operator*() const noexcept { return current_; }

    constexpr pointer operator->() const noexcept { return &current_; }

    constexpr reference operator[](difference_type n) const noexcept { return current_ + n; }

    constexpr bool operator==(const counting_iterator& other) const noexcept { return current_ == other.current_; }

    constexpr bool operator!=(const counting_iterator& other) const noexcept { return current_ != other.current_; }

    constexpr bool operator<(const counting_iterator& other) const noexcept { return current_ < other.current_; }

    constexpr bool operator<=(const counting_iterator& other) const noexcept { return current_ <= other.current_; }

    constexpr bool operator>(const counting_iterator& other) const noexcept { return current_ > other.current_; }

    constexpr bool operator>=(const counting_iterator& other) const noexcept { return current_ >= other.current_; }

    constexpr counting_iterator& operator=(const counting_iterator& other) noexcept
    {
        current_ = other.current_;
        return *this;
    }

private:
    T current_;
};