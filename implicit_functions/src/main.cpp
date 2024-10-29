#include <iostream>
#include <fstream>

#include <nlohmann/json.hpp>

#include <load_functions.hpp>

load_functions_result_t load_functions(const std::string& filename)
{
    using json = nlohmann::json;

    std::ifstream fin(filename.c_str());
    if (!fin) {
        std::cout << "Failed to open function file: " << filename << std::endl;
        return {};
    }

    json data;
    fin >> data;
    fin.close();

    load_functions_result_t result{};
    const auto              num_functions = data.size();
    result.functions.reserve(num_functions);
    for (size_t j = 0; j < num_functions; ++j) {
        const auto type = data[j]["type"].get<std::string>();
        if (type == "plane") {
            Eigen::Vector3d point{}, normal{};
            for (auto i = 0; i < 3; ++i) {
                point[i]  = data[j]["point"][i].get<double>();
                normal[i] = data[j]["normal"][i].get<double>();
            }
            result.functions.emplace_back(PlaneDistanceFunction<double, 3>{point, normal});
        } else if (type == "line") {
            Eigen::Vector3d point{}, direction{};
            for (auto i = 0; i < 3; ++i) {
                point[i]     = data[j]["point"][i].get<double>();
                direction[i] = data[j]["direction"][i].get<double>();
            }
            result.functions.emplace_back(CylinderDistanceFunction<double, 3>{point, direction, 0});
        } else if (type == "cylinder") {
            Eigen::Vector3d axis_point{}, axis_direction{};
            if (data[j].contains("axis_point1") && data[j].contains("axis_point2")) {
                Eigen::Vector3d axis_point_{};
                for (auto i = 0; i < 3; ++i) {
                    axis_point[i]  = data[j]["axis_point1"][i].get<double>();
                    axis_point_[i] = data[j]["axis_point2"][i].get<double>();
                }
                axis_direction = axis_point_ - axis_point;
            } else {
                for (auto i = 0; i < 3; ++i) {
                    axis_point[i]     = data[j]["axis_point"][i].get<double>();
                    axis_direction[i] = data[j]["axis_direction"][i].get<double>();
                }
            }
            double radius = data[j]["radius"].get<double>();
            result.functions.emplace_back(CylinderDistanceFunction<double, 3>{axis_point, axis_direction, radius});
        } else if (type == "sphere") {
            Eigen::Vector3d center{};
            for (auto i = 0; i < 3; ++i) { center[i] = data[j]["center"][i].get<double>(); }
            double radius = data[j]["radius"].get<double>();
            result.functions.emplace_back(SphereDistanceFunction<double, 3>{center, radius});
        } else if (type == "cone") {
            Eigen::Vector3d apex_point{}, axis_direction{};
            for (auto i = 0; i < 3; ++i) {
                apex_point[i]     = data[j]["apex_point"][i].get<double>();
                axis_direction[i] = data[j]["axis_direction"][i].get<double>();
            }
            double apex_angle = data[j]["apex_angle"].get<double>();
            result.functions.emplace_back(ConeDistanceFunction<double, 3>{apex_point, axis_direction, apex_angle});
        } else if (type == "constant") {
            double value = data[j]["value"].get<double>();
            result.functions.emplace_back(ConstantFunction<double, 3>{value});
        } else {
            std::cout << "Unsupported function type: " << type << std::endl;
            return {};
        }
    }

    result.success = true;
    return result;
}