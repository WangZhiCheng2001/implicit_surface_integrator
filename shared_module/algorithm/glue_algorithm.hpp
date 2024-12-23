#pragma once

#include <iterator>
#include <type_traits>
#include <variant>
#include <algorithm>
#include <execution>

#include <iterator/counting_iterator.hpp>

// HINT: c++17 does not support std::is_constant_evaluated() yet, so we use __builtin_is_constant_evaluated() __implemented by
// each compiler instead. and to be said, __builtin_is_constant_evaluated() is tested to be valid under c++17 with gcc, clang
// and msvc.
// HINT: for algorithm listed below, we use oneDPL library to accelerate it when it is executed under runtime.
// Othereise, we will use s__imple loop and local variables to __implement it, since these features are guaranteed to be valid
// since c++14, but some algorithms may not be to evaluated as constant expressions under c++17.

// MODIFY: we do not support  version function for now, since it is useless in our project

namespace detail
{
using execution_policy_variant_t = std::
    variant<std::execution::sequenced_policy, std::execution::parallel_policy, std::execution::parallel_unsequenced_policy>;

template <typename _Tp>
auto execution_policy_selector(_Tp N) -> execution_policy_variant_t
{
    static_assert(!std::is_floating_point_v<_Tp>);
    if (N <= 16)
        return std::execution::seq;
    else if (N % 4 == 0)
        return std::execution::par_unseq;
    else
        return std::execution::par;
}

template <typename _Tp>
auto execution_policy_selector_simd_only(_Tp N) -> execution_policy_variant_t
{
    static_assert(!std::is_floating_point_v<_Tp>);
    if (N <= 16)
        return std::execution::seq;
    else
        return std::execution::par_unseq;
}

static double thread_overload_factor = 1.5;

template <typename _Tp>
static inline auto decide_available_threads(_Tp max_works_expected)
{
    static_assert(!std::is_floating_point_v<_Tp>);
    return std::min(static_cast<size_t>(max_works_expected),
                    static_cast<size_t>(thread_overload_factor * std::thread::hardware_concurrency()));
}
} // namespace detail

namespace algorithm
{
enum class ExecutionPolicySelector : bool { all, simd_only };

template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all, typename _Tp, typename UnaryFunc>
inline void for_loop(_Tp first, _Tp last, UnaryFunc f)
{
    static_assert(std::is_integral_v<_Tp>);
    detail::execution_policy_variant_t policy;
    if constexpr (Policy == ExecutionPolicySelector::all)
        policy = detail::execution_policy_selector(last - first);
    else
        policy = detail::execution_policy_selector_simd_only(last - first);
    std::visit([&](auto&& exec) { std::for_each(exec, counting_iterator<_Tp>(first), counting_iterator<_Tp>(last), f); },
               policy);
}

template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all, typename InputIt, typename UnaryFunc>
inline void for_each(InputIt first, InputIt last, UnaryFunc f)
{
    detail::execution_policy_variant_t policy;
    if constexpr (Policy == ExecutionPolicySelector::all)
        policy = detail::execution_policy_selector(std::distance(first, last));
    else
        policy = detail::execution_policy_selector_simd_only(std::distance(first, last));
    std::visit([&](auto&& exec) { std::for_each(exec, first, last, f); }, policy);
}

template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all, typename ForwardIt>
inline ForwardIt min_element(ForwardIt first, ForwardIt last)
{
    detail::execution_policy_variant_t policy;
    if constexpr (Policy == ExecutionPolicySelector::all)
        policy = detail::execution_policy_selector(std::distance(first, last));
    else
        policy = detail::execution_policy_selector_simd_only(std::distance(first, last));
    return std::visit([&](auto&& exec) { return std::min_element(exec, first, last); }, policy);
}

template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all, typename ForwardIt>
inline ForwardIt max_element(ForwardIt first, ForwardIt last)
{
    detail::execution_policy_variant_t policy;
    if constexpr (Policy == ExecutionPolicySelector::all)
        policy = detail::execution_policy_selector(std::distance(first, last));
    else
        policy = detail::execution_policy_selector_simd_only(std::distance(first, last));
    return std::visit([&](auto&& exec) { return std::max_element(exec, first, last); }, policy);
}

template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all, typename ForwardIt>
inline void fill(ForwardIt first, ForwardIt last, const typename std::iterator_traits<ForwardIt>::value_type& value)
{
    detail::execution_policy_variant_t policy;
    if constexpr (Policy == ExecutionPolicySelector::all)
        policy = detail::execution_policy_selector(std::distance(first, last));
    else
        policy = detail::execution_policy_selector_simd_only(std::distance(first, last));
    std::visit([&](auto&& exec) { std::fill(exec, first, last, value); }, policy);
}

template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all, typename OutputIt>
inline OutputIt fill_n(OutputIt first, std::size_t count, const typename std::iterator_traits<OutputIt>::value_type& value)
{
    detail::execution_policy_variant_t policy;
    if constexpr (Policy == ExecutionPolicySelector::all)
        policy = detail::execution_policy_selector(count);
    else
        policy = detail::execution_policy_selector_simd_only(count);
    return std::visit([&](auto&& exec) { return std::fill_n(exec, first, count, value); }, policy);
}

template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all, typename InputIt, typename OutputIt>
inline OutputIt copy(InputIt first, InputIt last, OutputIt d_first)
{
    detail::execution_policy_variant_t policy;
    if constexpr (Policy == ExecutionPolicySelector::all)
        policy = detail::execution_policy_selector(std::distance(first, last));
    else
        policy = detail::execution_policy_selector_simd_only(std::distance(first, last));
    return std::visit([&](auto&& exec) { return std::copy(exec, first, last, d_first); }, policy);
}

template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all, typename ForwardIt1, typename ForwardIt2>
inline ForwardIt2 swap_ranges(ForwardIt1 first1, ForwardIt1 last1, ForwardIt2 first2)
{
    detail::execution_policy_variant_t policy;
    if constexpr (Policy == ExecutionPolicySelector::all)
        policy = detail::execution_policy_selector(std::distance(first1, last1));
    else
        policy = detail::execution_policy_selector_simd_only(std::distance(first1, last1));
    return std::visit([&](auto&& exec) { return std::swap_ranges(exec, first1, last1, first2); }, policy);
}

template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all, typename InputIt>
inline typename std::iterator_traits<InputIt>::difference_type
    count(InputIt first, InputIt last, const typename std::iterator_traits<InputIt>::value_type& value)
{
    detail::execution_policy_variant_t policy;
    if constexpr (Policy == ExecutionPolicySelector::all)
        policy = detail::execution_policy_selector(std::distance(first, last));
    else
        policy = detail::execution_policy_selector_simd_only(std::distance(first, last));
    return std::visit([&](auto&& exec) { return std::count(exec, first, last, value); }, policy);
}

template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all, typename InputIt>
inline void replace(InputIt                                                   first,
                    InputIt                                                   last,
                    const typename std::iterator_traits<InputIt>::value_type& old_value,
                    const typename std::iterator_traits<InputIt>::value_type& new_value)
{
    detail::execution_policy_variant_t policy;
    if constexpr (Policy == ExecutionPolicySelector::all)
        policy = detail::execution_policy_selector(std::distance(first, last));
    else
        policy = detail::execution_policy_selector_simd_only(std::distance(first, last));
    std::visit([&](auto&& exec) { std::replace(exec, first, last, old_value, new_value); }, policy);
}

template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all, typename InputIt>
inline InputIt find(InputIt first, InputIt last, const typename std::iterator_traits<InputIt>::value_type& value)
{
    detail::execution_policy_variant_t policy;
    if constexpr (Policy == ExecutionPolicySelector::all)
        policy = detail::execution_policy_selector(std::distance(first, last));
    else
        policy = detail::execution_policy_selector_simd_only(std::distance(first, last));
    return std::visit([&](auto&& exec) { return std::find(exec, first, last, value); }, policy);
}

template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all, typename InputIt, typename UnaryPred>
inline InputIt find_if(InputIt first, InputIt last, UnaryPred p)
{
    detail::execution_policy_variant_t policy;
    if constexpr (Policy == ExecutionPolicySelector::all)
        policy = detail::execution_policy_selector(std::distance(first, last));
    else
        policy = detail::execution_policy_selector_simd_only(std::distance(first, last));
    return std::visit([&](auto&& exec) { return std::find_if(exec, first, last, p); }, policy);
}

template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all, typename InputIt, typename UnaryPred>
inline bool all_of(InputIt first, InputIt last, UnaryPred pred)
{
    detail::execution_policy_variant_t policy;
    if constexpr (Policy == ExecutionPolicySelector::all)
        policy = detail::execution_policy_selector(std::distance(first, last));
    else
        policy = detail::execution_policy_selector_simd_only(std::distance(first, last));
    return std::visit([&](auto&& exec) { return std::all_of(exec, first, last, pred); }, policy);
}

template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all, typename InputIt, typename UnaryPred>
inline bool any_of(InputIt first, InputIt last, UnaryPred pred)
{
    detail::execution_policy_variant_t policy;
    if constexpr (Policy == ExecutionPolicySelector::all)
        policy = detail::execution_policy_selector(std::distance(first, last));
    else
        policy = detail::execution_policy_selector_simd_only(std::distance(first, last));
    return std::visit([&](auto&& exec) { return std::any_of(exec, first, last, pred); }, policy);
}

template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all, typename InputIt, typename UnaryPred>
inline bool none_of(InputIt first, InputIt last, UnaryPred pred)
{
    detail::execution_policy_variant_t policy;
    if constexpr (Policy == ExecutionPolicySelector::all)
        policy = detail::execution_policy_selector(std::distance(first, last));
    else
        policy = detail::execution_policy_selector_simd_only(std::distance(first, last));
    return std::visit([&](auto&& exec) { return std::none_of(exec, first, last, pred); }, policy);
}

template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all, typename InputIt, typename T, typename BinaryOp>
inline T reduce(InputIt first, InputIt last, T init, BinaryOp op)
{
    detail::execution_policy_variant_t policy;
    if constexpr (Policy == ExecutionPolicySelector::all)
        policy = detail::execution_policy_selector(std::distance(first, last));
    else
        policy = detail::execution_policy_selector_simd_only(std::distance(first, last));
    return std::visit([&](auto&& exec) { return std::reduce(exec, first, last, init, op); }, policy);
}

template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all, typename InputIt, typename OutputIt, typename UnaryOp>
inline OutputIt transform(InputIt first, InputIt last, OutputIt d_first, UnaryOp unary_op)
{
    detail::execution_policy_variant_t policy;
    if constexpr (Policy == ExecutionPolicySelector::all)
        policy = detail::execution_policy_selector(std::distance(first, last));
    else
        policy = detail::execution_policy_selector_simd_only(std::distance(first, last));
    return std::visit([&](auto&& exec) { return std::transform(exec, first, last, d_first, unary_op); }, policy);
}

template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all,
          typename InputIt1,
          typename InputIt2,
          typename OutputIt,
          typename BinaryOp>
inline OutputIt transform(InputIt1 first1, InputIt1 last1, InputIt2 first2, OutputIt d_first, BinaryOp binary_op)
{
    detail::execution_policy_variant_t policy;
    if constexpr (Policy == ExecutionPolicySelector::all)
        policy = detail::execution_policy_selector(std::distance(first1, last1));
    else
        policy = detail::execution_policy_selector_simd_only(std::distance(first1, last1));
    return std::visit([&](auto&& exec) { return std::transform(exec, first1, last1, first2, d_first, binary_op); }, policy);
}

template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all,
          typename InputIt1,
          typename InputIt2,
          typename T,
          typename BinaryOp1,
          typename BinaryOp2>
inline T transform_reduce(InputIt1 first1, InputIt1 last1, InputIt2 first2, T init, BinaryOp1 reduce, BinaryOp2 transform)
{
    detail::execution_policy_variant_t policy;
    if constexpr (Policy == ExecutionPolicySelector::all)
        policy = detail::execution_policy_selector(std::distance(first1, last1));
    else
        policy = detail::execution_policy_selector_simd_only(std::distance(first1, last1));
    return std::visit([&](auto&& exec) { return std::transform_reduce(exec, first1, last1, first2, init, reduce, transform); },
                      policy);
}

template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all,
          typename InputIt1,
          typename T,
          typename BinaryOp,
          typename UnaryOp>
inline T transform_reduce(InputIt1 first, InputIt1 last, T init, BinaryOp reduce, UnaryOp transform)
{
    detail::execution_policy_variant_t policy;
    if constexpr (Policy == ExecutionPolicySelector::all)
        policy = detail::execution_policy_selector(std::distance(first, last));
    else
        policy = detail::execution_policy_selector_simd_only(std::distance(first, last));
    return std::visit([&](auto&& exec) { return std::transform_reduce(exec, first, last, init, reduce, transform); }, policy);
}

template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all,
          typename InputIt,
          typename OutputIt,
          typename T,
          typename BinaryOp>
inline OutputIt inclusive_scan(InputIt first, InputIt last, OutputIt d_first, T init, BinaryOp binary_op)
{
    detail::execution_policy_variant_t policy;
    if constexpr (Policy == ExecutionPolicySelector::all)
        policy = detail::execution_policy_selector(std::distance(first, last));
    else
        policy = detail::execution_policy_selector_simd_only(std::distance(first, last));
    return std::visit([&](auto&& exec) { return std::inclusive_scan(exec, first, last, d_first, binary_op, init); }, policy);
}

template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all,
          typename InputIt,
          typename OutputIt,
          typename T,
          typename BinaryOp>
inline OutputIt exclusive_scan(InputIt first, InputIt last, OutputIt d_first, T init, BinaryOp binary_op)
{
    detail::execution_policy_variant_t policy;
    if constexpr (Policy == ExecutionPolicySelector::all)
        policy = detail::execution_policy_selector(std::distance(first, last));
    else
        policy = detail::execution_policy_selector_simd_only(std::distance(first, last));
    return std::visit([&](auto&& exec) { return std::exclusive_scan(exec, first, last, d_first, init, binary_op); }, policy);
}

// template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all, typename ForwardIt>
// inline ForwardIt shift_left(ForwardIt first, ForwardIt last, typename std::iterator_traits<ForwardIt>::difference_type n)
// {
//     detail::execution_policy_variant_t policy;
//     if constexpr (Policy == ExecutionPolicySelector::all)
//         policy = detail::execution_policy_selector(std::distance(first, last));
//     else
//         policy = detail::execution_policy_selector_simd_only(std::distance(first, last));
//     return std::visit([&](auto&& exec) { return std::shift_left(exec, first, last, n); }, policy);
// }

// template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all, typename ForwardIt>
// inline ForwardIt shift_right(ForwardIt first, ForwardIt last, typename std::iterator_traits<ForwardIt>::difference_type n)
// {
//     detail::execution_policy_variant_t policy;
//     if constexpr (Policy == ExecutionPolicySelector::all)
//         policy = detail::execution_policy_selector(std::distance(first, last));
//     else
//         policy = detail::execution_policy_selector_simd_only(std::distance(first, last));
//     return std::visit([&](auto&& exec) { return std::shift_right(exec, first, last, n); }, policy);
// }

template <ExecutionPolicySelector Policy = ExecutionPolicySelector::all, typename ForwardIt>
ForwardIt rotate(ForwardIt first, ForwardIt middle, ForwardIt last)
{
    detail::execution_policy_variant_t policy;
    if constexpr (Policy == ExecutionPolicySelector::all)
        policy = detail::execution_policy_selector(std::distance(first, last));
    else
        policy = detail::execution_policy_selector_simd_only(std::distance(first, last));
    return std::visit([&](auto&& exec) { return std::rotate(exec, first, middle, last); }, policy);
}
} // namespace algorithm