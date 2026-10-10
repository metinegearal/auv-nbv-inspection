#include <cmath>
#include <cstdlib>
#include "active_inspection_planner/use_cases/mapping.hpp"

void require(bool condition) {if(!condition) {std::abort();}}
struct Map final : inspection::MapReader, inspection::MapWriter
{
  inspection::Cloud cloud; inspection::Colors colors;
  Eigen::Vector3d origin; double range = 0.;
  std::vector<inspection::MapLeaf> leaves() const override {return {};}
  std::size_t count() const override {return cloud.size();}
  void insert(
    const inspection::Cloud & points, const inspection::Colors & rgb,
    const Eigen::Vector3d & location, double max_range) override
  {
    cloud = points; colors = rgb; origin = location; range = max_range;
  }
};
int main()
{
  Map map;
  const inspection::MapperConfig config;
  inspection::MapUpdate update(config, map, map);
  update.process_frame(Eigen::Vector3d(10., 20., -30.), 0., 0., 0.,
    inspection::ColorImage{2, 2, {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {10, 11, 12}}},
    inspection::DepthImage{2, 2, {.5f, 4.f, 0.f, 2.f}});
  require(map.cloud.size() == 1 && map.colors[0] == std::array<unsigned char, 3>({12, 11, 10}));
  require((map.origin - Eigen::Vector3d(10.35, 20.3875, -30.5)).norm() < 1e-12);
  require((map.cloud[0].cast<double>() - Eigen::Vector3d(10.35 + std::sqrt(2.), 20.3875,
    -30.5 - std::sqrt(2.))).norm() < 3e-6);
  require(map.range == 4.);
  const auto path = SearchPolicy::astar_3d(Eigen::Vector3d(.05, .05, .05),
      Eigen::Vector3d(.25, .05, .05), {}, .1);
  require(path.size() == 3);
  require((path.front() - Eigen::Vector3d(.05, .05, .05)).norm() < 1e-12);
  require((path.back() - Eigen::Vector3d(.25, .05, .05)).norm() < 1e-12);
  require(SearchPolicy::check_line_of_sight(path.front(), path.back(), {}, .1));
  std::string error;
  require(SearchPolicy::astar_3d(path.front(), path.back(), {}, .1, 0,
    [&error](const std::string & value){error = value;}).empty());
  require(error == "A* failed to find a path.\n");
}
