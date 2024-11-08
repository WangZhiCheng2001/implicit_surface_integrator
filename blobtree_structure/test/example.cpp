#pragma once
#include "blobtree.h"
#include "internal_api.hpp"
#include <iostream>
#include <vector>

typedef virtual_node_t BodyTag;
typedef raw_vector3d_t Vector3D;

class Interface
{
public:
    Interface() = default;

    BodyTag createEmpty()
    {
        constant_descriptor_t descriptor;
        descriptor.value = 0;
        return blobtree_new_virtual_node(descriptor);
    }

    BodyTag createPlane(const Vector3D& point, const Vector3D& normal)
    {
        plane_descriptor_t descriptor;
        descriptor.normal = normal;
        descriptor.point  = point;
        return blobtree_new_virtual_node(descriptor);
    }

    BodyTag createSphere(const Vector3D& center, const double radius)
    {
        sphere_descriptor_t descriptor;
        descriptor.center = center;
        descriptor.radius = radius;
        return blobtree_new_virtual_node(descriptor);
    }

    BodyTag createCylinder(const Vector3D& bottomPoint, const double radius, const Vector3D& offset)
    {
        cylinder_descriptor_t descriptor;
        descriptor.bottom_origion = bottomPoint;
        descriptor.radius         = radius;
        descriptor.offset         = offset;
        return blobtree_new_virtual_node(descriptor);
    }

    BodyTag createCone(const Vector3D& topPoint, const Vector3D& bottomPoint, const double radius1, const double radius2)
    {
        cone_descriptor_t descriptor;
        descriptor.top_point    = topPoint;
        descriptor.bottom_point = bottomPoint;
        descriptor.radius1      = radius1;
        descriptor.radius2      = radius2;
        return blobtree_new_virtual_node(descriptor);
    }

    BodyTag createBox(const Vector3D& leftBottomPoint, const double length, const double width, const double height)
    {
        box_descriptor_t descriptor;
        auto             half_size = Vector3D{length / 2.0, width / 2.0, height / 2.0};
        descriptor.half_size       = half_size;
        descriptor.center = {leftBottomPoint.x + half_size.x, leftBottomPoint.y + half_size.y, leftBottomPoint.z + half_size.z};

        return blobtree_new_virtual_node(descriptor);
    }

    BodyTag createMesh(const std::vector<Vector3D>            points,
                       const std::vector<int>                 indexs,
                       const std::vector<std::pair<int, int>> faces)
    {
        mesh_descriptor_t descriptor;

        descriptor.points = new Vector3D[points.size()];
        memcpy(descriptor.points, points.data(), points.size() * sizeof(Vector3D));

        descriptor.indexs = new int[indexs.size()];
        memcpy(descriptor.indexs, indexs.data(), indexs.size() * sizeof(double));

        descriptor.faces = new int*[faces.size()];
        for (int i = 0; i < faces.size(); ++i) {
            descriptor.faces[i]    = new int[2];
            descriptor.faces[i][0] = faces[i].first;
            descriptor.faces[i][1] = faces[i].second;
        }

        descriptor.point_number = points.size();
        descriptor.face_number  = faces.size();

        return blobtree_new_virtual_node(descriptor);
    }

    BodyTag createExtrude(const std::vector<Vector3D>& points, const std::vector<double>& bulges, const Vector3D& extusion)
    {
        extrude_descriptor_t descriptor;

        descriptor.points = new Vector3D[points.size()];
        memcpy(descriptor.points, points.data(), points.size() * sizeof(Vector3D));

        descriptor.bulges = new double[bulges.size()];
        memcpy(descriptor.bulges, bulges.data(), bulges.size() * sizeof(double));

        descriptor.extusion     = extusion;
        descriptor.edges_number = bulges.size();

        return blobtree_new_virtual_node(descriptor);
    }

    void booleanUnion(BodyTag& body1, BodyTag& body2) { virtual_node_boolean_union(&body1, &body2); }

    void booleanIntersect(BodyTag& body1, BodyTag& body2) { virtual_node_boolean_intersect(&body1, &body2); }

    void booleanDifference(BodyTag& body1, BodyTag& body2) { virtual_node_boolean_difference(&body1, &body2); }

    void split(BodyTag& body, const Vector3D& basePoint, const Vector3D& normal)
    {
        virtual_node_split(&body, basePoint, normal);
    }

    void offset(BodyTag& body, const Vector3D& direction, const double length)
    {
        virtual_node_offset(&body, direction, length);
    }

    void offset(BodyTag& body, const Vector3D& offset) { virtual_node_offset(&body, offset); }

    double getArea(BodyTag& body) { return 0.0; }

    double getVolume(BodyTag& body) { return 0.0; }

    std::pair<double, double> getAreaAndVolume(BodyTag& body) { return std::make_pair(0.0, 0.0); }
};

void test()
{
    Interface scene = Interface{};

    std::vector<Vector3D> points;
    std::vector<double>   bulges;
    Vector3D              extusion;
    Vector3D              topPoint, bottomPoint, basePoint, leftBottomPoint;
    Vector3D              direction;

    double                    offset, radius, radius1, radius2, length, width, height, areaDifference, volumeDifference;
    std::pair<double, double> before, after;

    /* 体1 */
    points.clear();
    bulges.clear();
    points.push_back(Vector3D{-32450.000001122418, -3.8391231093737305e-07, 0.0000000000000000});
    points.push_back(Vector3D{32449.999998877582, -3.8392495305561452e-07, 0.0000000000000000});
    bulges.push_back(0.99999999999999989);
    bulges.push_back(0.99999999999999989);
    extusion  = Vector3D{0.0000000000000000, 0.0000000000000000, 3300.0000000000000};
    auto tag1 = scene.createExtrude(points, bulges, extusion);

    /* 体2 */
    points.clear();
    bulges.clear();
    points.push_back(Vector3D{-32450.000001122418, -3.8391231093737305e-07, 0.0000000000000000});
    points.push_back(Vector3D{32449.999998877582, -3.8392495305561452e-07, 0.0000000000000000});
    points.push_back(Vector3D{33538.999998877582, -3.8392534654100421e-07, 0.0000000000000000});
    points.push_back(Vector3D{-33539.000001122418, -3.8391228016184551e-07, 0.0000000000000000});
    bulges.push_back(0.99999999999999989);
    bulges.push_back(0.0000000000000000);
    bulges.push_back(-0.99999999999999989);
    bulges.push_back(0.0000000000000000);
    extusion  = Vector3D{0.0000000000000000, 0.0000000000000000, 3300.0000000000000};
    auto tag2 = scene.createExtrude(points, bulges, extusion);

    /* 体3 */
    points.clear();
    bulges.clear();
    points.push_back(Vector3D{-32450.000001122418, -3.8391231093737305e-07, 0.0000000000000000});
    points.push_back(Vector3D{32449.999998877582, -3.8392495305561452e-07, 0.0000000000000000});
    points.push_back(Vector3D{33538.999998877582, -3.8392534654100421e-07, 0.0000000000000000});
    points.push_back(Vector3D{-33539.000001122418, -3.8391228016184551e-07, 0.0000000000000000});
    bulges.push_back(0.99999999999999989);
    bulges.push_back(0.0000000000000000);
    bulges.push_back(-0.99999999999999989);
    bulges.push_back(0.0000000000000000);
    extusion  = Vector3D{0.0000000000000000, 0.0000000000000000, 3300.0000000000000};
    auto tag3 = scene.createExtrude(points, bulges, extusion);

    /* 体4 */
    topPoint    = Vector3D{-1.1224183253943920e-06, -3.8391322798592142e-07, 0.0000000000000000};
    bottomPoint = Vector3D{-1.1224183253943920e-06, -3.8391322798592142e-07, 3300.0000000000000000};
    radius1     = 32450.000000000000;
    radius2     = 33539.000000000000;
    auto tag4   = scene.createCone(topPoint, bottomPoint, radius1, radius2);

    /* 体3被切割 */
    direction = Vector3D{0, 0, 1};
    basePoint = Vector3D{-1.1224183253943920e-06, -3.8391322798592142e-07, 0.0000000000000000};
    scene.split(tag3, basePoint, direction);

    /* 体3和体4布尔差 */
    scene.booleanDifference(tag3, tag4);

    /* 体5 */
    points.clear();
    bulges.clear();
    points.push_back(Vector3D{-32450.000001122418, -3.8391231093737305e-07, 0.0000000000000000});
    points.push_back(Vector3D{32449.999998877582, -3.8392495305561452e-07, 0.0000000000000000});
    points.push_back(Vector3D{33538.999998877582, -3.8392534654100421e-07, 0.0000000000000000});
    points.push_back(Vector3D{-33539.000001122418, -3.8391228016184551e-07, 0.0000000000000000});
    bulges.push_back(0.99999999999999989);
    bulges.push_back(0.0000000000000000);
    bulges.push_back(-0.99999999999999989);
    bulges.push_back(0.0000000000000000);
    extusion  = Vector3D{0.0000000000000000, 0.0000000000000000, 3300.0000000000000};
    auto tag5 = scene.createExtrude(points, bulges, extusion);

    /* 体5被切割 */
    direction = Vector3D{0, 0, 1};
    basePoint = Vector3D{-1.1224183253943920e-06, -3.8391322798592142e-07, 3300.0000000000000000};
    scene.split(tag5, basePoint, direction);

    /* 体3和体5布尔并 */
    scene.booleanUnion(tag3, tag5);

    /* 体6 */
    points.clear();
    bulges.clear();
    points.push_back(Vector3D{-32450.000001122418, -3.8391231093737305e-07, 0.0000000000000000});
    points.push_back(Vector3D{32449.999998877582, -3.8392495305561452e-07, 0.0000000000000000});
    points.push_back(Vector3D{33538.999998877582, -3.8392534654100421e-07, 0.0000000000000000});
    points.push_back(Vector3D{-33539.000001122418, -3.8391228016184551e-07, 0.0000000000000000});
    bulges.push_back(0.99999999999999989);
    bulges.push_back(0.0000000000000000);
    bulges.push_back(-0.99999999999999989);
    bulges.push_back(0.0000000000000000);
    extusion  = Vector3D{0.0000000000000000, 0.0000000000000000, 3300.0000000000000};
    auto tag6 = scene.createExtrude(points, bulges, extusion);

    /* 体6体被切割 */
    direction = Vector3D{0, 0, 1};
    basePoint = Vector3D{-1.1224183253943920e-06, -3.8391322798592142e-07, 3300.0000000000000000};
    scene.split(tag6, basePoint, direction);

    /* 体3和体6布尔并 */
    scene.booleanUnion(tag3, tag6);

    /* 体2和体3布尔差 */
    scene.booleanDifference(tag2, tag3);

    /* 体7 */
    points.clear();
    bulges.clear();
    points.push_back(Vector3D{32449.999998877582, -3.8392495305561452e-07, 0.0000000000000000});
    points.push_back(Vector3D{-32450.000001122418, -3.8391231093737305e-07, 0.0000000000000000});
    points.push_back(Vector3D{-33539.000001122418, -3.8391191745198337e-07, 0.0000000000000000});
    points.push_back(Vector3D{33538.999998877582, -3.8392498383114206e-07, 0.0000000000000000});
    bulges.push_back(0.99999999999999989);
    bulges.push_back(0.0000000000000000);
    bulges.push_back(-0.99999999999999989);
    bulges.push_back(0.0000000000000000);
    extusion  = Vector3D{0.0000000000000000, 0.0000000000000000, 3300.0000000000000};
    auto tag7 = scene.createExtrude(points, bulges, extusion);

    /* 体8 */
    points.clear();
    bulges.clear();
    points.push_back(Vector3D{32449.999998877582, -3.8392495305561452e-07, 0.0000000000000000});
    points.push_back(Vector3D{-32450.000001122418, -3.8391231093737305e-07, 0.0000000000000000});
    points.push_back(Vector3D{-33539.000001122418, -3.8391191745198337e-07, 0.0000000000000000});
    points.push_back(Vector3D{33538.999998877582, -3.8392498383114206e-07, 0.0000000000000000});
    bulges.push_back(0.99999999999999989);
    bulges.push_back(0.0000000000000000);
    bulges.push_back(-0.99999999999999989);
    bulges.push_back(0.0000000000000000);
    extusion  = Vector3D{0.0000000000000000, 0.0000000000000000, 3300.0000000000000};
    auto tag8 = scene.createExtrude(points, bulges, extusion);

    /* 体9 */
    topPoint    = Vector3D{-1.1224183253943920e-06, -3.8392403600706615e-07, 0.0000000000000000};
    bottomPoint = Vector3D{-1.1224183253943920e-06, -3.8392403600706615e-07, 3300.0000000000000000};
    radius1     = 32450.000000000000;
    radius2     = 33539.000000000000;
    auto tag9   = scene.createCone(topPoint, bottomPoint, radius1, radius2);

    /* 体8被切割 */
    direction = Vector3D{0, 0, 1};
    basePoint = Vector3D{-1.1224183253943920e-06, -3.8392403600706615e-07, 3300.0000000000000000};
    scene.split(tag8, basePoint, direction);

    /* 体8和体9布尔差 */
    scene.booleanDifference(tag8, tag9);

    /* 体10 */
    points.clear();
    bulges.clear();
    points.push_back(Vector3D{32449.999998877582, -3.8392495305561452e-07, 0.0000000000000000});
    points.push_back(Vector3D{-32450.000001122418, -3.8391231093737305e-07, 0.0000000000000000});
    points.push_back(Vector3D{-33539.000001122418, -3.8391191745198337e-07, 0.0000000000000000});
    points.push_back(Vector3D{33538.999998877582, -3.8392498383114206e-07, 0.0000000000000000});
    bulges.push_back(0.99999999999999989);
    bulges.push_back(0.0000000000000000);
    bulges.push_back(-0.99999999999999989);
    bulges.push_back(0.0000000000000000);
    extusion   = Vector3D{0.0000000000000000, 0.0000000000000000, 3300.0000000000000};
    auto tag10 = scene.createExtrude(points, bulges, extusion);

    /* 体10被切割 */
    direction = Vector3D{0, 0, 1};
    basePoint = Vector3D{-1.1224183253943920e-06, -3.8392403600706615e-07, 3300.0000000000000000};
    scene.split(tag10, basePoint, direction);

    /* 体8和体10布尔并 */
    scene.booleanUnion(tag8, tag10);

    /* 体11 */
    points.clear();
    bulges.clear();
    points.push_back(Vector3D{32449.999998877582, -3.8392495305561452e-07, 0.0000000000000000});
    points.push_back(Vector3D{-32450.000001122418, -3.8391231093737305e-07, 0.0000000000000000});
    points.push_back(Vector3D{-33539.000001122418, -3.8391191745198337e-07, 0.0000000000000000});
    points.push_back(Vector3D{33538.999998877582, -3.8392498383114206e-07, 0.0000000000000000});
    bulges.push_back(0.99999999999999989);
    bulges.push_back(0.0000000000000000);
    bulges.push_back(-0.99999999999999989);
    bulges.push_back(0.0000000000000000);
    extusion   = Vector3D{0.0000000000000000, 0.0000000000000000, 3300.0000000000000};
    auto tag11 = scene.createExtrude(points, bulges, extusion);

    /* 体11被切割 */
    direction = Vector3D{0, 0, 1};
    basePoint = Vector3D{-1.1224183253943920e-06, -3.8392403600706615e-07, 3300.0000000000000000};
    scene.split(tag11, basePoint, direction);

    /* 体8和体11布尔并 */
    scene.booleanUnion(tag8, tag11);

    /* 体7和体8布尔差 */
    scene.booleanDifference(tag7, tag8);

    /* 体2和体7布尔并 */
    scene.booleanUnion(tag2, tag7);

    /* 体1和体2布尔并 */
    scene.booleanUnion(tag1, tag2);

    /* 体1 Z轴偏移 -3600 */
    direction = Vector3D{0, 0, 1};
    offset    = -3600;
    scene.offset(tag1, direction, offset);

#ifdef _DEBUG
    output_blobtree(tag1);
#endif // _DEBUG

    /* 体111 */
    points.clear();
    bulges.clear();
    points.push_back(Vector3D{-32050.000001122418, -3.8396319723688066e-07, 0.0000000000000000});
    points.push_back(Vector3D{32049.999998877582, -3.8387224776670337e-07, 0.0000000000000000});
    bulges.push_back(0.99999999999999989);
    bulges.push_back(0.99999999999999989);
    extusion    = Vector3D{0.0000000000000000, 0.0000000000000000, 600.0000000000000};
    auto tag111 = scene.createExtrude(points, bulges, extusion);

    /* 体0 */
    points.clear();
    bulges.clear();
    points.push_back(Vector3D{-11798.670446418590, 5999.2409883221799, 0.0000000000000000});
    points.push_back(Vector3D{11801.337366082498, 5999.2409883221590, 0.0000000000000000});
    bulges.push_back(0.99999999999999989);
    bulges.push_back(0.99999999999999989);
    extusion  = Vector3D{0.0000000000000000, 0.0000000000000000, 600.0000000000000};
    auto tag0 = scene.createExtrude(points, bulges, extusion);

    /* 体111和体0布尔差 */
    scene.booleanDifference(tag111, tag0);

    /* 体111 Z轴偏移 -3500 */
    direction = Vector3D{0, 0, 1};
    offset    = -3500;
    scene.offset(tag111, direction, offset);

    /* 体12 */
    points.clear();
    bulges.clear();
    points.push_back(Vector3D{676.33403607269543, 5999.2409883221790, 0.0000000000000000});
    points.push_back(Vector3D{-673.66711640879771, 5999.2409883221790, 0.0000000000000000});
    bulges.push_back(0.99999999999999989);
    bulges.push_back(0.99999999999999989);
    extusion   = Vector3D{0.0000000000000000, 0.0000000000000000, 600.0000000000000};
    auto tag12 = scene.createExtrude(points, bulges, extusion);

    /* 体12 Z轴偏移 -3500 */
    direction = Vector3D{0, 0, 1};
    offset    = -3500;
    scene.offset(tag12, direction, offset);

    /* 体111和体12布尔并，保留体12 */
    scene.booleanUnion(tag111, tag12);

    /* 体13 */
    points.clear();
    bulges.clear();
    points.push_back(Vector3D{-2398.6665401680511, 5999.2409883221771, 0.0000000000000000});
    points.push_back(Vector3D{2401.3334598319489, 5999.2409883221790, 0.0000000000000000});
    bulges.push_back(0.99999999999999989);
    bulges.push_back(0.99999999999999989);
    extusion   = Vector3D{0.0000000000000000, 0.0000000000000000, 600.0000000000000};
    auto tag13 = scene.createExtrude(points, bulges, extusion);

    /* 体14 */
    points.clear();
    bulges.clear();
    points.push_back(Vector3D{-673.66711640879771, 5999.2409883221790, 0.0000000000000000});
    points.push_back(Vector3D{676.33403607269543, 5999.2409883221790, 0.0000000000000000});
    bulges.push_back(-0.99999999999999989);
    bulges.push_back(-0.99999999999999989);
    extusion   = Vector3D{0.0000000000000000, 0.0000000000000000, 600.0000000000000};
    auto tag14 = scene.createExtrude(points, bulges, extusion);

    /* 体13和体14布尔差	 */
    scene.booleanDifference(tag13, tag14);

    /* 体13 Z轴偏移 -3500 */
    direction = Vector3D{0, 0, 1};
    offset    = -3500;
    scene.offset(tag13, direction, offset);

    /* 体11和体13布尔并，保留体13 */
    scene.booleanUnion(tag111, tag13);

    before = scene.getAreaAndVolume(tag1);
    /* 体1和体111布尔差 */
    scene.booleanDifference(tag1, tag111);
    after = scene.getAreaAndVolume(tag1);

    /* 获取体1 布尔前后的体积差和面积差 */
    areaDifference   = after.first - before.first;
    volumeDifference = after.second - before.second;
    std::cout << areaDifference << ", " << volumeDifference << std::endl;

    /* 定义空体 扣减体1 */
    auto subTag1 = scene.createEmpty();
    auto cycle   = [&scene, &subTag1](const Vector3D& point) {
        std::vector<Vector3D> points;
        std::vector<double>   bulges;
        Vector3D              extusion;
        Vector3D              direction;
        double                offset;

        /* 循环体1 */
        points.clear();
        bulges.clear();
        points.push_back(Vector3D{-1000.0000000000000, 1000.0000000000001, 0.0000000000000000});
        points.push_back(Vector3D{-1000.0000000000000, -1000.0000000000000, 0.0000000000000000});
        points.push_back(Vector3D{1000.0000000000000, -1000.0000000000000, 0.0000000000000000});
        points.push_back(Vector3D{1000.0000000000000, 1000.0000000000000, 0.0000000000000000});
        bulges.push_back(0.0000000000000000);
        bulges.push_back(0.0000000000000000);
        bulges.push_back(0.0000000000000000);
        bulges.push_back(0.0000000000000000);
        extusion       = Vector3D{0.0000000000000000, 0.0000000000000000, 600.0000000000000};
        auto cycleTag1 = scene.createExtrude(points, bulges, extusion);

        /* 循环体1 偏移+ 传入的X,Y,Z坐标值  */
        scene.offset(cycleTag1, point);

        /* 循环体2 */
        auto   leftBottomPoint = Vector3D{point.x - 1000, point.y - 1000, point.z + 0.0000000000005};
        double length          = 2000.0000000000036;
        double width           = 2000.0000000000036;
        double height          = 600;
        auto   cycleTag2       = scene.createBox(leftBottomPoint, length, width, height);

        /* 循环体 */
        points.clear();
        bulges.clear();
        points.push_back(Vector3D{-32050.000001122418, -3.8396319723688066e-07, 0.0000000000000000});
        points.push_back(Vector3D{32049.999998877582, -3.8387224776670337e-07, 0.0000000000000000});
        bulges.push_back(0.99999999999999989);
        bulges.push_back(0.99999999999999989);
        extusion       = Vector3D{0.0000000000000000, 0.0000000000000000, 600.0000000000000};
        auto cycleTag3 = scene.createExtrude(points, bulges, extusion);

        /* 循环体3 Z轴偏移 -2900 */
        direction = Vector3D{0, 0, 1};
        offset    = -2900;
        scene.offset(cycleTag3, direction, offset);

        /* 循环体3和体2布尔交 */
        scene.booleanIntersect(cycleTag3, cycleTag2);

        /* 循环体1和体3布尔交 */
        scene.booleanIntersect(cycleTag1, cycleTag3);

        /* 扣减体1和循环体1布尔并 */
        scene.booleanUnion(subTag1, cycleTag1);
    };

    points.clear();
    points.push_back(Vector3D{-21595.036456438422, -21609.173181957169, -2900.0000000000000});
    points.push_back(Vector3D{-13505.711180076411, -13505.717935507526, -2900.0000000000000});
    points.push_back(Vector3D{-2987.8729842756293, -18864.829751913778, -2900.0000000000000});
    points.push_back(Vector3D{2987.9196792985895, -18864.830728476278, -2900.0000000000000});
    points.push_back(Vector3D{13505.770265236089, -13505.722818320026, -2900.0000000000000});
    points.push_back(Vector3D{17018.254640236089, -8671.2047519137759, -2900.0000000000000});
    points.push_back(Vector3D{27220.281983986089, -13869.397623007526, -2900.0000000000000});
    points.push_back(Vector3D{21602.139405861089, -21602.103677695028, -2900.0000000000000});
    points.push_back(Vector3D{30173.926515236089, -4778.9669589450259, -2900.0000000000000});
    points.push_back(Vector3D{18864.887452736089, -2987.8275546481514, -2900.0000000000000});
    points.push_back(Vector3D{-18864.815672263911, -2987.8811435153389, -2900.0000000000000});
    points.push_back(Vector3D{-30173.856687888911, -4779.0570468356509, -2900.0000000000000});
    points.push_back(Vector3D{-13869.422117576411, -27220.218912070028, -2900.0000000000000});
    points.push_back(Vector3D{-11781.052000388911, -23121.591958945028, -2900.0000000000000});
    points.push_back(Vector3D{-8671.2131332014105, -17018.193521445028, -2900.0000000000000});
    points.push_back(Vector3D{-27220.229734763911, -13869.391763632526, -2900.0000000000000});
    points.push_back(Vector3D{-17018.194578513911, -8671.1974276950259, -2900.0000000000000});
    points.push_back(Vector3D{-30173.856687888911, 4779.0965176174741, -2900.0000000000000});
    points.push_back(Vector3D{-25630.485594138911, 4059.4959316799741, -2900.0000000000000});
    points.push_back(Vector3D{-18864.815672263911, 2987.9134121487236, -2900.0000000000000});
    points.push_back(Vector3D{-15863.798094138911, 8083.0535488674741, -2900.0000000000000});
    points.push_back(Vector3D{-27220.216062888911, 13869.449056679974, -2900.0000000000000});
    points.push_back(Vector3D{-21602.091062888911, 21602.130697304972, -2900.0000000000000});
    points.push_back(Vector3D{-18349.393797263911, 18349.437337929972, -2900.0000000000000});
    points.push_back(Vector3D{-13908.684812888911, 13908.728353554974, -2900.0000000000000});
    points.push_back(Vector3D{-9584.8552230451605, 18811.339681679972, -2900.0000000000000});
    points.push_back(Vector3D{-13869.425047263911, 27220.249837929972, -2900.0000000000000});
    points.push_back(Vector3D{-4779.0493148420355, 30173.900228554972, -2900.0000000000000});
    points.push_back(Vector3D{4779.0900894548395, 30173.904134804972, -2900.0000000000000});
    points.push_back(Vector3D{13869.430421486089, 27220.275228554972, -2900.0000000000000});
    points.push_back(Vector3D{9584.8806167985895, 18811.363119179972, -2900.0000000000000});
    points.push_back(Vector3D{13908.739015236089, 13908.728353554974, -2900.0000000000000});
    points.push_back(Vector3D{18349.453858986089, 18349.433431679972, -2900.0000000000000});
    points.push_back(Vector3D{21602.147218361089, 21602.130697304972, -2900.0000000000000});
    points.push_back(Vector3D{27220.268312111089, 13869.454916054974, -2900.0000000000000});
    points.push_back(Vector3D{15863.852296486089, 8083.0574551174741, -2900.0000000000000});
    points.push_back(Vector3D{18864.879640236089, 2987.9153652737236, -2900.0000000000000});
    points.push_back(Vector3D{25630.543702736089, 4059.4900723049741, -2900.0000000000000});
    points.push_back(Vector3D{30173.912843361089, 4779.0896816799741, -2900.0000000000000});
    points.push_back(Vector3D{13869.432374611089, -27220.242349570028, -2900.0000000000000});
    points.push_back(Vector3D{-4776.1108367703273, -30098.869302695028, -2900.0000000000000});
    points.push_back(Vector3D{4779.0915542985895, -30098.869302695028, -2900.0000000000000});
    points.push_back(Vector3D{-23121.594969138911, -11781.033365195026, -2900.0000000000000});
    points.push_back(Vector3D{23121.637452736089, 11781.092611367474, -2900.0000000000000});
    points.push_back(Vector3D{11781.053468361089, -23121.619302695028, -2900.0000000000000});
    points.push_back(Vector3D{-23121.583250388911, 11781.089681679974, -2900.0000000000000});
    for (auto& point : points) { cycle(point); }

    before = scene.getAreaAndVolume(tag1);
    /* 体1和扣减体1布尔差 */
    scene.booleanDifference(tag1, subTag1);
    after = scene.getAreaAndVolume(tag1);

    /* 获取体1 布尔前后的体积差和面积差 */
    areaDifference   = after.first - before.first;
    volumeDifference = after.second - before.second;
    std::cout << areaDifference << ", " << volumeDifference << std::endl;

    /* 体15 Ent1.bool */
    bottomPoint  = Vector3D{0.0000, 0.0000, -3600.0000};
    radius       = 32150.0;
    direction    = Vector3D{0, 0, 100};
    auto tag15   = scene.createCylinder(bottomPoint, radius, direction);
    bottomPoint  = Vector3D{1.3330, 5999.2412, -3600.0000};
    radius       = 11700.43125;
    direction    = Vector3D{0, 0, 100};
    auto tag15_1 = scene.createCylinder(bottomPoint, radius, direction);
    scene.booleanDifference(tag15, tag15_1);

    /* 体1和体15布尔差 */
    scene.booleanDifference(tag1, tag15);

    /* 体16 */
    points.clear();
    bulges.clear();
    points.push_back(Vector3D{-2503.6291053659488, 5999.2409883221790, -3500.0000000000000});
    points.push_back(Vector3D{2501.2960250298465, 5999.2409883221790, -3500.0000000000000});
    bulges.push_back(0.99999999999999989);
    bulges.push_back(0.99999999999999989);
    extusion   = Vector3D{-0.0000000000000000, -0.0000000000000000, -100.00000000000000};
    auto tag16 = scene.createExtrude(points, bulges, extusion);

    before = scene.getAreaAndVolume(tag1);
    /* 体1和体16布尔差 */
    scene.booleanDifference(tag1, tag16);
    after = scene.getAreaAndVolume(tag1);

    /* 获取体1 布尔前后的体积差和面积差 */
    areaDifference   = after.first - before.first;
    volumeDifference = after.second - before.second;
    std::cout << areaDifference << ", " << volumeDifference << std::endl;

    /* 体17 */
    points.clear();
    bulges.clear();
    points.push_back(Vector3D{2079.8334638622341, 5999.2409883221844, 0.0000000000000000});
    points.push_back(Vector3D{-2070.1665441983364, 5999.2409883221844, 0.0000000000000000});
    points.push_back(Vector3D{-1320.1665441983364, 5999.2409883221844, 0.0000000000000000});
    points.push_back(Vector3D{1329.8334638622341, 5999.2409883221844, 0.0000000000000000});
    bulges.push_back(0.99999999999999989);
    bulges.push_back(0.0000000000000000);
    bulges.push_back(-0.99999999999999989);
    bulges.push_back(0.0000000000000000);
    extusion   = Vector3D{0.0000000000000000, 0.0000000000000000, 2750.0000000000000};
    auto tag17 = scene.createExtrude(points, bulges, extusion);

    /* 体17 Z轴偏移 -2900 */
    direction = Vector3D{0, 0, 1};
    offset    = -2900;
    scene.offset(tag17, direction, offset);

    /* 体1和体17布尔差 */
    scene.booleanDifference(tag1, tag17);

    /* 体18 */
    points.clear();
    bulges.clear();
    points.push_back(Vector3D{-2070.1665441983364, 5999.2409883221844, 0.0000000000000000});
    points.push_back(Vector3D{2079.8334638622341, 5999.2409883221844, 0.0000000000000000});
    points.push_back(Vector3D{1329.8334638622341, 5999.2409883221844, 0.0000000000000000});
    points.push_back(Vector3D{-1320.1665441983364, 5999.2409883221844, 0.0000000000000000});
    bulges.push_back(0.99999999999999989);
    bulges.push_back(0.0000000000000000);
    bulges.push_back(-0.99999999999999989);
    bulges.push_back(0.0000000000000000);
    extusion   = Vector3D{0.0000000000000000, 0.0000000000000000, 2750.0000000000000};
    auto tag18 = scene.createExtrude(points, bulges, extusion);

    /* 体18 Z轴偏移 -2900 */
    direction = Vector3D{0, 0, 1};
    offset    = -2900;
    scene.offset(tag18, direction, offset);

    /* 体1和体18布尔差 */
    scene.booleanDifference(tag1, tag18);

    /* 体19 */
    points.clear();
    bulges.clear();
    points.push_back(Vector3D{8760.4644464528919, -16861.795338417785, 0.0000000000000000});
    points.push_back(Vector3D{5893.9646280986062, -18064.309214597473, 0.0000000000000000});
    points.push_back(Vector3D{6248.1524494726591, -18208.769763083357, 0.0000000000000000});
    points.push_back(Vector3D{8651.1159137951836, -17197.859075721935, 0.0000000000000000});
    bulges.push_back(-0.039803147455118697);
    bulges.push_back(0.0000000000000000);
    bulges.push_back(0.032943018176267741);
    bulges.push_back(0.0000000000000000);
    extusion   = Vector3D{0.0000000000000000, 0.0000000000000000, 2850.0000000000000};
    auto tag19 = scene.createExtrude(points, bulges, extusion);

    /* 体19 Z轴偏移 -2900 */
    direction = Vector3D{0, 0, 1};
    offset    = -2900;
    scene.offset(tag19, direction, offset);

    /* 体1和体19布尔差 */
    scene.booleanDifference(tag1, tag19);

    /* 体20 */
    points.clear();
    bulges.clear();
    points.push_back(Vector3D{8472.1973651856952, -23124.378331468590, 0.0000000000000000});
    points.push_back(Vector3D{11234.316117012168, -21717.002618593218, 0.0000000000000000});
    points.push_back(Vector3D{10898.067054009927, -21607.748991111552, 0.0000000000000000});
    points.push_back(Vector3D{8581.4512221945606, -22788.128674499021, 0.0000000000000000});
    bulges.push_back(0.0000000000000000);
    bulges.push_back(0.0000000000000000);
    bulges.push_back(0.0000000000000000);
    bulges.push_back(0.0000000000000000);
    extusion   = Vector3D{0.0000000000000000, 0.0000000000000000, 2850.0000000000000};
    auto tag20 = scene.createExtrude(points, bulges, extusion);

    /* 体20 Z轴偏移 -2900 */
    direction = Vector3D{0, 0, 1};
    offset    = -2900;
    scene.offset(tag20, direction, offset);

    /* 体1和体20布尔差 */
    scene.booleanDifference(tag1, tag20);

    /* 体21 */
    points.clear();
    bulges.clear();
    points.push_back(Vector3D{11234.316117012169, -21717.002618593222, 0.0000000000000000});
    points.push_back(Vector3D{8760.4644464528919, -16861.795338417785, 0.0000000000000000});
    points.push_back(Vector3D{8651.1159137951836, -17197.859075721935, 0.0000000000000000});
    points.push_back(Vector3D{10898.067054009929, -21607.748991111555, 0.0000000000000000});
    bulges.push_back(0.0000000000000000);
    bulges.push_back(0.0000000000000000);
    bulges.push_back(0.0000000000000000);
    bulges.push_back(0.0000000000000000);
    extusion   = Vector3D{0.0000000000000000, 0.0000000000000000, 2850.0000000000000};
    auto tag21 = scene.createExtrude(points, bulges, extusion);

    /* 体21 Z轴偏移 -2900 */
    direction = Vector3D{0, 0, 1};
    offset    = -2900;
    scene.offset(tag21, direction, offset);

    /* 体1和体21布尔差 */
    scene.booleanDifference(tag1, tag21);

    /* 体22 */
    points.clear();
    bulges.clear();
    points.push_back(Vector3D{5893.9646280986062, -18064.309214597473, 0.0000000000000000});
    points.push_back(Vector3D{8472.1973651856970, -23124.378331468593, 0.0000000000000000});
    points.push_back(Vector3D{8581.4512221945624, -22788.128674499025, 0.0000000000000000});
    points.push_back(Vector3D{6248.1524494726591, -18208.769763083357, 0.0000000000000000});
    bulges.push_back(0.0000000000000000);
    bulges.push_back(0.0000000000000000);
    bulges.push_back(0.0000000000000000);
    bulges.push_back(0.0000000000000000);
    extusion   = Vector3D{0.0000000000000000, 0.0000000000000000, 2850.0000000000000};
    auto tag22 = scene.createExtrude(points, bulges, extusion);

    /* 体22 Z轴偏移 -2900 */
    direction = Vector3D{0, 0, 1};
    offset    = -2900;
    scene.offset(tag22, direction, offset);

    before = scene.getAreaAndVolume(tag1);
    /* 体1和体22布尔差 */
    scene.booleanDifference(tag1, tag22);
    after = scene.getAreaAndVolume(tag1);

    /* 获取体1 布尔前后的体积差和面积差 */
    areaDifference   = after.first - before.first;
    volumeDifference = after.second - before.second;
    std::cout << areaDifference << ", " << volumeDifference << std::endl;

    /* 体23 */
    points.clear();
    bulges.clear();
    points.push_back(Vector3D{7380.7150904719992, -20706.098050612181, 0.0000000000000000});
    points.push_back(Vector3D{9920.1451018985645, -19412.300707257506, 0.0000000000000000});
    points.push_back(Vector3D{9829.3529561354007, -19234.096368736471, 0.0000000000000000});
    points.push_back(Vector3D{7289.9229447088355, -20527.893712091147, 0.0000000000000000});
    bulges.push_back(0.0000000000000000);
    bulges.push_back(0.0000000000000000);
    bulges.push_back(0.0000000000000000);
    bulges.push_back(0.0000000000000000);
    extusion   = Vector3D{0.0000000000000000, 0.0000000000000000, 2850.0000000000000};
    auto tag23 = scene.createExtrude(points, bulges, extusion);

    /* 体23 Z轴偏移 -2900 */
    direction = Vector3D{0, 0, 1};
    offset    = -2900;
    scene.offset(tag23, direction, offset);

    before = scene.getAreaAndVolume(tag1);
    /* 体1和体23布尔差 */
    scene.booleanDifference(tag1, tag23);
    after = scene.getAreaAndVolume(tag1);

    /* 获取体1 布尔前后的体积差和面积差 */
    areaDifference   = after.first - before.first;
    volumeDifference = after.second - before.second;
    std::cout << areaDifference << ", " << volumeDifference << std::endl;

    /* 以下为一个循环，凸度组固定，但点组不一样，其他操作一致 */
    auto cycle1 = [&scene, &tag1](const Vector3D& point1, const Vector3D point2) {
        /* 循环体 */
        const std::vector<Vector3D> points   = {point1, point2};
        std::vector<double>         bulges   = {0.99999999999999989, 0.99999999999999989};
        Vector3D                    extusion = Vector3D{0.0000000000000000, 0.0000000000000000, 2850.0000000000000};
        auto                        cycleTag = scene.createExtrude(points, bulges, extusion);

        /* 循环体 Z轴偏移 -2900 */
        Vector3D direction = Vector3D{0, 0, 1};
        double   offset    = -2900;
        scene.offset(cycleTag, direction, offset);

        /* 体1和循环体布尔差 */
        scene.booleanDifference(tag1, cycleTag);
    };

    points.clear();
    bulges.clear();
    points.push_back(Vector3D{25330.544191030145, 4059.4890957491152, 0.0000000000000000});
    points.push_back(Vector3D{25930.544191030145, 4059.4890957491152, 0.0000000000000000});
    points.push_back(Vector3D{2637.9248093705974, -18864.831701145784, 0.0000000000000000});
    points.push_back(Vector3D{3337.9248093705974, -18864.831701145784, 0.0000000000000000});
    points.push_back(Vector3D{-19214.815183982137, -2987.8808536118449, 0.0000000000000000});
    points.push_back(Vector3D{-18514.815183982137, -2987.8808536118449, 0.0000000000000000});
    points.push_back(Vector3D{13155.774659765651, -13505.730142548899, 0.0000000000000000});
    points.push_back(Vector3D{13855.774659765651, -13505.730142548899, 0.0000000000000000});
    points.push_back(Vector3D{-23421.584711222677, 11781.088222738428, 0.0000000000000000});
    points.push_back(Vector3D{-22821.584711222677, 11781.088222738428, 0.0000000000000000});
    points.push_back(Vector3D{-21952.083741631068, -21602.101728577210, 0.0000000000000000});
    points.push_back(Vector3D{-21252.083741631068, -21602.101728577210, 0.0000000000000000});
    points.push_back(Vector3D{-3337.8756971632247, -18864.831700995899, 0.0000000000000000});
    points.push_back(Vector3D{-2637.8756971632247, -18864.831700995899, 0.0000000000000000});
    points.push_back(Vector3D{22821.635010756669, 11781.088216097109, 0.0000000000000000});
    points.push_back(Vector3D{23421.635010756669, 11781.088216097109, 0.0000000000000000});
    points.push_back(Vector3D{8321.2448748154566, -17018.220376670986, 0.0000000000000000});
    points.push_back(Vector3D{9021.2448748154566, -17018.220376670986, 0.0000000000000000});
    points.push_back(Vector3D{-1871.5656374104437, -9922.6107521542435, 0.0000000000000000});
    points.push_back(Vector3D{-1271.5656374104440, -9922.6107521542435, 0.0000000000000000});
    points.push_back(Vector3D{26920.265871888550, 13869.448081589901, 0.0000000000000000});
    points.push_back(Vector3D{27520.265871888550, 13869.448081589901, 0.0000000000000000});
    points.push_back(Vector3D{29873.915284768678, 4779.0894375231728, 0.0000000000000000});
    points.push_back(Vector3D{30473.915284768678, 4779.0894375231728, 0.0000000000000000});
    points.push_back(Vector3D{1271.6142550302902, -9922.6210172974243, 0.0000000000000000});
    points.push_back(Vector3D{1871.6142550302900, -9922.6210172974243, 0.0000000000000000});
    points.push_back(Vector3D{7608.7448703002883, -7908.7010952570563, 0.0000000000000000});
    points.push_back(Vector3D{8208.7448703002883, -7908.7010952570563, 0.0000000000000000});
    points.push_back(Vector3D{11431.053468361089, -23121.619302695020, 0.0000000000000000});
    points.push_back(Vector3D{12131.053468361089, -23121.619302695020, 0.0000000000000000});
    points.push_back(Vector3D{21252.143796979217, -21602.113447324591, 0.0000000000000000});
    points.push_back(Vector3D{21952.143796979217, -21602.113447324591, 0.0000000000000000});
    points.push_back(Vector3D{26870.284425387275, -13869.400552711551, 0.0000000000000000});
    points.push_back(Vector3D{27570.284425387275, -13869.400552711551, 0.0000000000000000});
    points.push_back(Vector3D{16668.254151753965, -8671.2113439714158, 0.0000000000000000});
    points.push_back(Vector3D{17368.254151753965, -8671.2113439714158, 0.0000000000000000});
    points.push_back(Vector3D{21302.142824993702, 21602.129722254482, 0.0000000000000000});
    points.push_back(Vector3D{21902.142824993702, 21602.129722254482, 0.0000000000000000});
    points.push_back(Vector3D{18514.882078273164, -2987.8308623732737, 0.0000000000000000});
    points.push_back(Vector3D{19214.882078273164, -2987.8308623732737, 0.0000000000000000});
    points.push_back(Vector3D{-27570.225340223697, -13869.390298805301, 0.0000000000000000});
    points.push_back(Vector3D{-26870.225340223697, -13869.390298805301, 0.0000000000000000});
    points.push_back(Vector3D{15563.854737884307, 8083.0486660407769, 0.0000000000000000});
    points.push_back(Vector3D{16163.854737884307, 8083.0486660407769, 0.0000000000000000});
    points.push_back(Vector3D{29823.926027013978, -4778.9706820347201, 0.0000000000000000});
    points.push_back(Vector3D{30523.926027013978, -4778.9706820347201, 0.0000000000000000});
    points.push_back(Vector3D{-25930.485105855274, 4059.4890957491152, 0.0000000000000000});
    points.push_back(Vector3D{-25330.485105855274, 4059.4890957491152, 0.0000000000000000});
    points.push_back(Vector3D{-15906.105228972388, 2471.7791264631815, 0.0000000000000000});
    points.push_back(Vector3D{-15306.105228972388, 2471.7791264631815, 0.0000000000000000});
    points.push_back(Vector3D{-14208.685301157355, 13908.728353561615, 0.0000000000000000});
    points.push_back(Vector3D{-13608.685301157355, 13908.728353561615, 0.0000000000000000});
    points.push_back(Vector3D{15306.173097269726, 2471.7791347907769, 0.0000000000000000});
    points.push_back(Vector3D{15906.173097269726, 2471.7791347907769, 0.0000000000000000});
    points.push_back(Vector3D{-30523.856199605914, -4779.0607699771499, 0.0000000000000000});
    points.push_back(Vector3D{-29823.856199605914, -4779.0607699771499, 0.0000000000000000});
    points.push_back(Vector3D{13608.744379809476, 13908.728344973482, 0.0000000000000000});
    points.push_back(Vector3D{14208.744379809476, 13908.728344973482, 0.0000000000000000});
    points.push_back(Vector3D{-19164.815183980274, 2987.9090176032769, 0.0000000000000000});
    points.push_back(Vector3D{-18564.815183980274, 2987.9090176032769, 0.0000000000000000});
    points.push_back(Vector3D{18049.454334036331, 18349.429508093293, 0.0000000000000000});
    points.push_back(Vector3D{18649.454334036331, 18349.429508093293, 0.0000000000000000});
    points.push_back(Vector3D{18564.882081644726, 2987.9090176032769, 0.0000000000000000});
    points.push_back(Vector3D{19164.882081644726, 2987.9090176032769, 0.0000000000000000});
    points.push_back(Vector3D{-18649.394773816515, 18349.429525433647, 0.0000000000000000});
    points.push_back(Vector3D{-18049.394773816515, 18349.429525433647, 0.0000000000000000});
    points.push_back(Vector3D{-16163.795652730274, 8083.0486660407769, 0.0000000000000000});
    points.push_back(Vector3D{-15563.795652730274, 8083.0486660407769, 0.0000000000000000});
    points.push_back(Vector3D{-13089.545652741857, 6516.6287441791064, 0.0000000000000000});
    points.push_back(Vector3D{-12489.545652741857, 6516.6287441791064, 0.0000000000000000});
    points.push_back(Vector3D{-12432.865969395847, 1921.6695098721102, 0.0000000000000000});
    points.push_back(Vector3D{-11832.865969395847, 1921.6695098721102, 0.0000000000000000});
    points.push_back(Vector3D{11832.924069322587, 1921.6787872227869, 0.0000000000000000});
    points.push_back(Vector3D{12432.924069322587, 1921.6787872227869, 0.0000000000000000});
    points.push_back(Vector3D{-14219.424558988423, -27220.219888640844, 0.0000000000000000});
    points.push_back(Vector3D{-13519.424558988423, -27220.219888640844, 0.0000000000000000});
    points.push_back(Vector3D{-8208.6860335274250, -7908.7010897282598, 0.0000000000000000});
    points.push_back(Vector3D{-7608.6860335274250, -7908.7010897282598, 0.0000000000000000});
    points.push_back(Vector3D{11531.074457970564, 23121.638501036170, 0.0000000000000000});
    points.push_back(Vector3D{12031.074457970564, 23121.638501036170, 0.0000000000000000});
    points.push_back(Vector3D{-21902.083733627456, 21602.129728298649, 0.0000000000000000});
    points.push_back(Vector3D{-21302.083733627456, 21602.129728298649, 0.0000000000000000});
    points.push_back(Vector3D{-5079.0456527201459, 30173.900228539613, 0.0000000000000000});
    points.push_back(Vector3D{-4479.0456527201459, 30173.900228539613, 0.0000000000000000});
    points.push_back(Vector3D{-3223.9956954814261, 18461.557455070157, 0.0000000000000000});
    points.push_back(Vector3D{-2623.9956954814261, 18461.557455070157, 0.0000000000000000});
    points.push_back(Vector3D{2624.0444348900346, 18461.557454720794, 0.0000000000000000});
    points.push_back(Vector3D{3224.0444348900346, 18461.557454720794, 0.0000000000000000});
    points.push_back(Vector3D{-8405.0161077585071, 15906.991054460479, 0.0000000000000000});
    points.push_back(Vector3D{-7805.0161077585071, 15906.991054460479, 0.0000000000000000});
    points.push_back(Vector3D{4479.0942398422048, 30173.900228537284, 0.0000000000000000});
    points.push_back(Vector3D{5079.0942398422048, 30173.900228537284, 0.0000000000000000});
    points.push_back(Vector3D{-30473.855223292368, 4779.0894371719478, 0.0000000000000000});
    points.push_back(Vector3D{-29873.855223292368, 4779.0894371719478, 0.0000000000000000});
    points.push_back(Vector3D{-9834.8557113136048, 18811.339681665777, 0.0000000000000000});
    points.push_back(Vector3D{-9334.8557113136048, 18811.339681665777, 0.0000000000000000});
    points.push_back(Vector3D{13569.434806959180, 27220.270333465378, 0.0000000000000000});
    points.push_back(Vector3D{14169.434806959180, 27220.270333465378, 0.0000000000000000});
    points.push_back(Vector3D{13452.094479468651, -2178.0609130331286, 0.0000000000000000});
    points.push_back(Vector3D{14052.094479468651, -2178.0609130331286, 0.0000000000000000});
    points.push_back(Vector3D{7635.4943863257067, -4043.3108920587547, 0.0000000000000000});
    points.push_back(Vector3D{8235.4943863257067, -4043.3108920587547, 0.0000000000000000});
    points.push_back(Vector3D{-5030.7456039006938, -9284.6107577729126, 0.0000000000000000});
    points.push_back(Vector3D{-4430.7456039006938, -9284.6107577729126, 0.0000000000000000});
    points.push_back(Vector3D{9334.8840347801452, 18811.357259790777, 0.0000000000000000});
    points.push_back(Vector3D{9834.8840347801452, 18811.357259790777, 0.0000000000000000});
    points.push_back(Vector3D{-3675.6656478448422, 21628.976400436615, 0.0000000000000000});
    points.push_back(Vector3D{-3175.6656478448422, 21628.976400436615, 0.0000000000000000});
    points.push_back(Vector3D{3809.4943863322260, 25630.529134790777, 0.0000000000000000});
    points.push_back(Vector3D{4309.4943863322260, 25630.529134790777, 0.0000000000000000});
    points.push_back(Vector3D{4430.7846695982153, -9284.6210116034345, 0.0000000000000000});
    points.push_back(Vector3D{5030.7846695982153, -9284.6210116034345, 0.0000000000000000});
    points.push_back(Vector3D{-13855.715086132754, -13505.720376673606, 0.0000000000000000});
    points.push_back(Vector3D{-13155.715086132754, -13505.720376673606, 0.0000000000000000});
    points.push_back(Vector3D{-12131.054926622368, -23121.600743537347, 0.0000000000000000});
    points.push_back(Vector3D{-11431.054926622368, -23121.600743537347, 0.0000000000000000});
    points.push_back(Vector3D{-10578.765375099087, -1627.9909095301846, 0.0000000000000000});
    points.push_back(Vector3D{-9978.7653750990867, -1627.9909095301846, 0.0000000000000000});
    points.push_back(Vector3D{3175.7141128947260, 21628.976400436615, 0.0000000000000000});
    points.push_back(Vector3D{3675.7141128947260, 21628.976400436615, 0.0000000000000000});
    points.push_back(Vector3D{11239.354737904854, 11539.338705116606, 0.0000000000000000});
    points.push_back(Vector3D{11839.354737904854, 11539.338705116606, 0.0000000000000000});
    points.push_back(Vector3D{-17318.196043352364, -8671.2008456719195, 0.0000000000000000});
    points.push_back(Vector3D{-16718.196043352364, -8671.2008456719195, 0.0000000000000000});
    points.push_back(Vector3D{-14169.424558975501, 27220.248861367945, 0.0000000000000000});
    points.push_back(Vector3D{-13569.424558975501, 27220.248861367945, 0.0000000000000000});
    points.push_back(Vector3D{9978.8439957064693, -1627.9708957259863, 0.0000000000000000});
    points.push_back(Vector3D{10578.843995706469, -1627.9708957259863, 0.0000000000000000});
    points.push_back(Vector3D{13519.435304297367, -27220.250162088065, 0.0000000000000000});
    points.push_back(Vector3D{14219.435304297367, -27220.250162088065, 0.0000000000000000});
    points.push_back(Vector3D{-3550.8955367645249, -6380.2710360850760, 0.0000000000000000});
    points.push_back(Vector3D{-2950.8955367645249, -6380.2710360850760, 0.0000000000000000});
    points.push_back(Vector3D{10709.754156451614, -5609.7207374573773, 0.0000000000000000});
    points.push_back(Vector3D{11309.754156451614, -5609.7207374573773, 0.0000000000000000});
    points.push_back(Vector3D{12489.604245378461, 6516.6389926101110, 0.0000000000000000});
    points.push_back(Vector3D{13089.604245378461, 6516.6389926101110, 0.0000000000000000});
    points.push_back(Vector3D{-8235.4355452836026, -4043.3108920448431, 0.0000000000000000});
    points.push_back(Vector3D{-7635.4355452836026, -4043.3108920448431, 0.0000000000000000});
    points.push_back(Vector3D{2950.9448257543263, -6380.2808017734642, 0.0000000000000000});
    points.push_back(Vector3D{3550.9448257543263, -6380.2808017734642, 0.0000000000000000});
    points.push_back(Vector3D{-11309.696043347765, -5609.7207431293646, 0.0000000000000000});
    points.push_back(Vector3D{-10709.696043347765, -5609.7207431293646, 0.0000000000000000});
    points.push_back(Vector3D{-11839.285398820706, 11539.328939490792, 0.0000000000000000});
    points.push_back(Vector3D{-11239.285398820706, 11539.328939490792, 0.0000000000000000});
    points.push_back(Vector3D{-14052.005613667367, -2178.0908755634300, 0.0000000000000000});
    points.push_back(Vector3D{-13452.005613667367, -2178.0908755634300, 0.0000000000000000});
    points.push_back(Vector3D{-9021.2153273623553, -17018.201329904943, 0.0000000000000000});
    points.push_back(Vector3D{-8321.2153273623553, -17018.201329904943, 0.0000000000000000});
    points.push_back(Vector3D{-4309.4457381869433, 25630.529134784956, 0.0000000000000000});
    points.push_back(Vector3D{-3809.4457381869433, 25630.529134784956, 0.0000000000000000});
    points.push_back(Vector3D{-27520.215574607428, 13869.448080100890, 0.0000000000000000});
    points.push_back(Vector3D{-26920.215574607428, 13869.448080100890, 0.0000000000000000});
    points.push_back(Vector3D{-12031.065183990693, 23121.618978540777, 0.0000000000000000});
    points.push_back(Vector3D{-11531.065183990693, 23121.618978540777, 0.0000000000000000});
    points.push_back(Vector3D{7805.0446787483525, 15907.018391851525, 0.0000000000000000});
    points.push_back(Vector3D{8405.0446787483525, 15907.018391851525, 0.0000000000000000});
    points.push_back(Vector3D{4354.0942397164181, -30098.870279427545, 0.0000000000000000});
    points.push_back(Vector3D{5204.0942397164044, -30098.870279427545, 0.0000000000000000});
    points.push_back(Vector3D{-5204.0456527279457, -30098.870279264738, 0.0000000000000000});
    points.push_back(Vector3D{-4354.0456527279594, -30098.870279264738, 0.0000000000000000});
    points.push_back(Vector3D{-23421.612136159150, -11781.067532595984, 0.0000000000000000});
    points.push_back(Vector3D{-22821.612136159150, -11781.067532595984, 0.0000000000000000});
    for (size_t i = 0; i < points.size(); i += 2) { cycle1(points[i], points[i + 1]); }

    /* 以下为一个循环，凸度组固定，但点组不一样，其他操作一致 */
    auto cycle2 = [&scene, &tag1](const Vector3D& point1, const Vector3D point2, const Vector3D point3, const Vector3D point4) {
        /* 循环体 */
        const std::vector<Vector3D> points   = {point1, point2, point3, point4};
        std::vector<double>         bulges   = {0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 0.0000000000000000};
        Vector3D                    extusion = Vector3D{0.0000000000000000, 0.0000000000000000, 2850.0000000000000};
        auto                        cycleTag = scene.createExtrude(points, bulges, extusion);

        /* 循环体 Z轴偏移 -2900 */
        Vector3D direction = Vector3D{0, 0, 1};
        double   offset    = -2900;
        scene.offset(cycleTag, direction, offset);

        /* 体1和循环体布尔差 */
        scene.booleanDifference(tag1, cycleTag);
    };

    points.clear();
    bulges.clear();
    points.push_back(Vector3D{-24963.009860716236, 7446.7650615308812, 0.0000000000000000});
    points.push_back(Vector3D{-24459.052558350675, 7366.9792723925602, 0.0000000000000000});
    points.push_back(Vector3D{-24388.685618648018, 7811.4435496937649, 0.0000000000000000});
    points.push_back(Vector3D{-24833.149895949224, 7881.8104893964210, 0.0000000000000000});
    points.push_back(Vector3D{24963.106150213611, 7446.7650615305229, 0.0000000000000000});
    points.push_back(Vector3D{24459.148847848050, 7366.9792723922019, 0.0000000000000000});
    points.push_back(Vector3D{24388.781908145393, 7811.4435496934066, 0.0000000000000000});
    points.push_back(Vector3D{24833.246185446598, 7881.8104893960626, 0.0000000000000000});
    points.push_back(Vector3D{6085.2898662776533, -18440.931060827090, 0.0000000000000000});
    points.push_back(Vector3D{6530.7930936759531, -18213.935742862825, 0.0000000000000000});
    points.push_back(Vector3D{6326.4973075081125, -17812.982838204356, 0.0000000000000000});
    points.push_back(Vector3D{5880.9940801098128, -18039.978156168621, 0.0000000000000000});
    points.push_back(Vector3D{9198.0021988374647, -24549.990396445304, 0.0000000000000000});
    points.push_back(Vector3D{9643.5067429911614, -24322.997662776565, 0.0000000000000000});
    points.push_back(Vector3D{9439.2132826892957, -23922.043573038238, 0.0000000000000000});
    points.push_back(Vector3D{8993.7087385355990, -24149.036306706977, 0.0000000000000000});

    before = scene.getAreaAndVolume(tag1);
    for (size_t i = 0; i < points.size(); i += 4) { cycle2(points[i], points[i + 1], points[i + 2], points[i + 3]); }
    after = scene.getAreaAndVolume(tag1);

#ifdef _DEBUG
    // output_blobtree(tag1);
#endif // _DEBUG

    /* 获取体1 布尔前后的体积差和面积差 */
    areaDifference   = after.first - before.first;
    volumeDifference = after.second - before.second;
    std::cout << areaDifference << ", " << volumeDifference << std::endl;
}

int main()
{
    test();
    return 0;
}
