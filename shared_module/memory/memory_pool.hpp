#pragma once

#define TBB_PREVIEW_MEMORY_POOL 1
#include <tbb/tbb_allocator.h>
#include <tbb/memory_pool.h>

class ScalableMemoryPoolSingleton
{
public:
    static auto &instance()
    {
        static thread_local tbb::memory_pool<tbb::tbb_allocator<char>> _instance;
        return _instance;
    }
};

#if _MSC_VER && !defined(__INTEL_COMPILER)
// Workaround for erroneous "unreferenced parameter" warning in method destroy.
#pragma warning(push)
#pragma warning(disable : 4100)
#endif

template <typename T>
class ScalableMemoryPoolAllocator
{
protected:
    template <typename U>
    friend class ScalableMemoryPoolAllocator;
    template <typename V, typename U>
    friend bool operator==(const ScalableMemoryPoolAllocator<V> &a, const ScalableMemoryPoolAllocator<U> &b);
    template <typename V, typename U>
    friend bool operator!=(const ScalableMemoryPoolAllocator<V> &a, const ScalableMemoryPoolAllocator<U> &b);

public:
    typedef T                 value_type;
    typedef value_type       *pointer;
    typedef const value_type *const_pointer;
    typedef value_type       &reference;
    typedef const value_type &const_reference;
    typedef size_t            size_type;
    typedef ptrdiff_t         difference_type;

    template <typename U>
    struct rebind {
        typedef ScalableMemoryPoolAllocator<U> other;
    };

    ScalableMemoryPoolAllocator() : m_allocator(ScalableMemoryPoolSingleton::instance()) {}

    ScalableMemoryPoolAllocator(const ScalableMemoryPoolAllocator &src) throw() : m_allocator(src.m_allocator) {}

    template <typename U>
    ScalableMemoryPoolAllocator(const ScalableMemoryPoolAllocator<U> &src) throw() : m_allocator(src.m_allocator)
    {
    }

    inline pointer address(reference x) const { return &x; }

    inline const_pointer address(const_reference x) const { return &x; }

    //! Allocate space for n objects.
    inline pointer allocate(size_type n, const void * /*hint*/ = nullptr) { return m_allocator.allocate(n); }

    //! Free previously allocated block of memory.
    inline void deallocate(pointer p, size_type s) { m_allocator.deallocate(p, s); }

    //! Largest value for which method allocate might succeed.
    inline size_type max_size() const throw()
    {
        size_type max = static_cast<size_type>(-1) / sizeof(value_type);
        return (max > 0 ? max : 1);
    }

    //! Copy-construct value at location pointed to by p.

    template <typename U, typename... Args>
    inline void construct(U *p, Args &&...args)
    {
        ::new ((void *)p) U(std::forward<Args>(args)...);
    }

    //! Destroy value at location pointed to by p.
    inline void destroy(pointer p) { p->~value_type(); }

private:
    tbb::memory_pool_allocator<T> m_allocator{};
};

#if _MSC_VER && !defined(__INTEL_COMPILER)
#pragma warning(pop)
#endif // warning 4100 is back

template <typename V, typename U>
inline bool operator==(const ScalableMemoryPoolAllocator<V> &a, const ScalableMemoryPoolAllocator<U> &b)
{
    return a.m_allocator == b.m_allocator;
}

template <typename V, typename U>
inline bool operator!=(const ScalableMemoryPoolAllocator<V> &a, const ScalableMemoryPoolAllocator<U> &b)
{
    return a.m_allocator != b.m_allocator;
}