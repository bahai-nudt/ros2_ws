#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include "ikd_Tree.h"
#include "lidar_types.h"

namespace {

using msf::lidar::PointType;
using msf::lidar::PointVector;

PointType MakePoint(float x, float y, float z) {
    PointType p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.intensity = 0.0f;
    return p;
}

bool NearPoint(const PointType& p, float x, float y, float z, float tol = 1.0e-4f) {
    return std::fabs(p.x - x) <= tol &&
           std::fabs(p.y - y) <= tol &&
           std::fabs(p.z - z) <= tol;
}

// 建树 + 最近邻：验证 Build / Nearest_Search / validnum
bool TestBuildAndNearestSearch() {
    PointVector points;
    points.push_back(MakePoint(0.0f, 0.0f, 0.0f));
    points.push_back(MakePoint(1.0f, 0.0f, 0.0f));
    points.push_back(MakePoint(0.0f, 2.0f, 0.0f));
    points.push_back(MakePoint(0.0f, 0.0f, 3.0f));
    points.push_back(MakePoint(2.0f, 2.0f, 2.0f));

    auto tree = std::make_unique<KD_TREE<PointType>>(0.5f, 0.6f, 0.2f);

    tree->Build(points);

    if (tree->validnum() != 5) {
        return false;
    }

    PointVector nearest;
    std::vector<float> distances;
    tree->Nearest_Search(MakePoint(0.1f, 0.1f, 0.1f), 1, nearest, distances);
    if (nearest.size() != 1 || !NearPoint(nearest[0], 0.0f, 0.0f, 0.0f)) {
        return false;
    }

    nearest.clear();
    distances.clear();
    tree->Nearest_Search(MakePoint(1.1f, 0.0f, 0.0f), 1, nearest, distances);
    if (nearest.size() != 1 || !NearPoint(nearest[0], 1.0f, 0.0f, 0.0f)) {
        return false;
    }

    return true;
}

// 盒式搜索：验证 Box_Search 返回包围盒内的点
bool TestBoxSearch() {
    PointVector points;
    points.push_back(MakePoint(0.0f, 0.0f, 0.0f));
    points.push_back(MakePoint(1.0f, 0.0f, 0.0f));
    points.push_back(MakePoint(0.0f, 2.0f, 0.0f));

    auto tree = std::make_unique<KD_TREE<PointType>>(0.5f, 0.6f, 0.2f);
    tree->Build(points);

    BoxPointType box;
    box.vertex_min[0] = 0.5f;
    box.vertex_min[1] = -0.5f;
    box.vertex_min[2] = -0.5f;
    box.vertex_max[0] = 1.5f;
    box.vertex_max[1] = 0.5f;
    box.vertex_max[2] = 0.5f;

    PointVector found;
    tree->Box_Search(box, found);
    if (found.size() != 1 || !NearPoint(found[0], 1.0f, 0.0f, 0.0f)) {
        return false;
    }
    return true;
}

// 删除 + 再插入：验证 Delete_Points / Add_Points 后有效点数与最近邻恢复
bool TestDeleteAndAdd() {
    PointVector points;
    points.push_back(MakePoint(0.0f, 0.0f, 0.0f));
    points.push_back(MakePoint(1.0f, 0.0f, 0.0f));
    points.push_back(MakePoint(0.0f, 2.0f, 0.0f));

    auto tree = std::make_unique<KD_TREE<PointType>>(0.5f, 0.6f, 0.2f);
    tree->Build(points);

    PointVector to_delete;
    to_delete.push_back(MakePoint(1.0f, 0.0f, 0.0f));
    tree->Delete_Points(to_delete);
    if (tree->validnum() != 2) {
        return false;
    }

    PointVector nearest;
    std::vector<float> distances;
    tree->Nearest_Search(MakePoint(1.1f, 0.0f, 0.0f), 1, nearest, distances);
    if (nearest.size() != 1 || !NearPoint(nearest[0], 0.0f, 0.0f, 0.0f)) {
        return false;
    }

    PointVector to_add;
    to_add.push_back(MakePoint(1.0f, 0.0f, 0.0f));
    tree->Add_Points(to_add, false);
    if (tree->validnum() != 3) {
        return false;
    }

    nearest.clear();
    distances.clear();
    tree->Nearest_Search(MakePoint(1.1f, 0.0f, 0.0f), 1, nearest, distances);
    if (nearest.size() != 1 || !NearPoint(nearest[0], 1.0f, 0.0f, 0.0f)) {
        return false;
    }
    return true;
}

}  // namespace

int main() {
    struct TestCase {
        const char* name;
        bool (*fn)();
    };

    const TestCase tests[] = {
        {"build_and_nearest_search", TestBuildAndNearestSearch},
        {"box_search", TestBoxSearch},
        {"delete_and_add", TestDeleteAndAdd},
    };

    int failed = 0;
    for (const auto& test : tests) {
        if (!test.fn()) {
            std::cerr << "[FAIL] " << test.name << std::endl;
            ++failed;
        } else {
            std::cout << "[PASS] " << test.name << std::endl;
        }
    }
    return failed == 0 ? 0 : 1;
}
