#pragma once

#include <tbb/tbbmalloc_proxy.h>

#define EIGEN_NO_CUDA                     1
#define EIGEN_USE_MKL                     1
#define EIGEN_USE_MKL_ALL                 1
#define EIGEN_USE_THREADS                 1
// just an experiment, since it seems OKay to mix oneTBB and OpenMP, see
// https://oneapi-src.github.io/oneTBB/main/tbb_userguide/appendix_B.html
// #define EIGEN_DONT_PARALLELIZE \
//     1 // use this macro to disable multi-threading in Eigen, since Eigen's multi-threading is
//       // restricted to use OpenMP
#define EIGEN_INITIALIZE_MATRICES_BY_ZERO 1
#define EIGEN_NO_AUTOMATIC_RESIZING       1
// #define EIGEN_MAX_CPP_VER                 17
#include <Eigen/Dense>

// namespace BlobtreeIntegral::IMath
// {
// using namespace Eigen;

// static aligned_allocator<double> g_aligned_allocator{};

// static ThreadPool       g_thread_pool(nbThreads());
// static ThreadPoolDevice g_tensor_thread_pool_handle(&g_thread_pool, nbThreads());
// } // namespace BlobtreeIntegral::IMath