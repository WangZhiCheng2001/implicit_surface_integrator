#include <iostream>

#include <environment.h>
#include <execution.h>

int main()
{
    std::cout << "Setting environments..." << std::endl;
    setting_descriptor setting_desc{21};
    update_setting(setting_desc);
    update_environment();

    std::cout << "Setting scene..." << std::endl;
    update_scene();

    std::cout << "Executing solver..." << std::endl;
    execute_solver();

    std::cout << "Time statistics: " << std::endl;
    print_statistics();

    return 0;
}