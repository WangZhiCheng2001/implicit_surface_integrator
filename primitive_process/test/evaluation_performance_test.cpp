#include <timer/scoped_timer.hpp>

#include <internal_process_api.hpp>

int main()
{
    labelled_timers_manager timer{};

    box_descriptor_t box{
        {0., 0., 0.},
        {1., 1., 1.}
    };

    timer.push_timer("run-time under 10^6 loops (old)");

    for (auto i = 0; i < 1'000'000; ++i) { auto result = evaluate(box, Eigen::Vector3d::Ones() * 2); }

    timer.pop_timer("run-time under 10^6 loops (old)");

    timer.print();

    return 0;
}