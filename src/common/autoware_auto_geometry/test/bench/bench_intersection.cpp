#include <iostream>
#include <boost/geometry.hpp>
#include <boost/geometry/io/io.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <geometry/common_2d.hpp>
#include <list>
#include <geometry/intersection.hpp>
#include <benchmark/benchmark.h>


namespace bg = boost::geometry;
typedef bg::model::point<autoware::common::types::float32_t, 2, bg::cs::cartesian> point_type;
typedef bg::model::ring<point_type> polygon_type;

using Point = geometry_msgs::msg::Point32;

struct TestPoint
{
    autoware::common::types::float32_t x;
    autoware::common::types::float32_t y;

};

static void bench_convex_polygon_intersection_2d(benchmark::State& state) {

    std::list<TestPoint> polygon1{{6, 1},{ 6, 8},{ 2, 8},{-2, 4},{ 1, 1},{ 6, 1}};
    std::list<TestPoint> polygon2{{1, 3},{2, 6},{4, 2},{1, 3}};

    for (auto _ : state)
        autoware::common::geometry::convex_polygon_intersection2d(polygon1, polygon2);
}

// Register the function as a benchmark

BENCHMARK(bench_convex_polygon_intersection_2d);















