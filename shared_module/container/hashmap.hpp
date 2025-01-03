#pragma once

#include <parallel_hashmap/phmap.h>
#include <memory/memory_pool.hpp>

// TODO: replace std::shared_mutex with std::shared_mutex
// which needs to change the internal usage of mutexes in phmap

template <class T,
          class Hash  = phmap::priv::hash_default_hash<T>,
          class Eq    = phmap::priv::hash_default_eq<T>,
          class Alloc = std::allocator<T>>
using flat_hash_set = phmap::flat_hash_set<T, Hash, Eq, Alloc>;

template <class K,
          class V,
          class Hash  = phmap::priv::hash_default_hash<K>,
          class Eq    = phmap::priv::hash_default_eq<K>,
          class Alloc = std::allocator<std::pair<const K, V>>>
using flat_hash_map = phmap::flat_hash_map<K, V, Hash, Eq, Alloc>;

template <class T,
          class Hash  = phmap::priv::hash_default_hash<T>,
          class Eq    = phmap::priv::hash_default_eq<T>,
          class Alloc = std::allocator<T>,
          size_t N    = 4, // 2**N submaps
          class Mutex = std::shared_mutex>
using parallel_flat_hash_set = phmap::parallel_flat_hash_set<T, Hash, Eq, Alloc, N, Mutex>;

template <class K,
          class V,
          class Hash  = phmap::priv::hash_default_hash<K>,
          class Eq    = phmap::priv::hash_default_eq<K>,
          class Alloc = std::allocator<std::pair<const K, V>>,
          size_t N    = 4, // 2**N submaps
          class Mutex = std::shared_mutex>
using parallel_flat_hash_map = phmap::parallel_flat_hash_map<K, V, Hash, Eq, Alloc, N, Mutex>;

template <class T,
          class Hash  = phmap::priv::hash_default_hash<T>,
          class Eq    = phmap::priv::hash_default_eq<T>,
          class Alloc = tbb::tbb_allocator<T>>
using flat_hash_set_mp = phmap::flat_hash_set<T, Hash, Eq, Alloc>;

template <class K,
          class V,
          class Hash  = phmap::priv::hash_default_hash<K>,
          class Eq    = phmap::priv::hash_default_eq<K>,
          class Alloc = tbb::tbb_allocator<std::pair<const K, V>>>
using flat_hash_map_mp = phmap::flat_hash_map<K, V, Hash, Eq, Alloc>;

template <class T,
          class Hash  = phmap::priv::hash_default_hash<T>,
          class Eq    = phmap::priv::hash_default_eq<T>,
          class Alloc = tbb::tbb_allocator<T>,
          size_t N    = 4, // 2**N submaps
          class Mutex = std::shared_mutex>
using parallel_flat_hash_set_mp = phmap::parallel_flat_hash_set<T, Hash, Eq, Alloc, N, Mutex>;

template <class K,
          class V,
          class Hash  = phmap::priv::hash_default_hash<K>,
          class Eq    = phmap::priv::hash_default_eq<K>,
          class Alloc = tbb::tbb_allocator<std::pair<const K, V>>,
          size_t N    = 4, // 2**N submaps
          class Mutex = std::shared_mutex>
using parallel_flat_hash_map_mp = phmap::parallel_flat_hash_map<K, V, Hash, Eq, Alloc, N, Mutex>;