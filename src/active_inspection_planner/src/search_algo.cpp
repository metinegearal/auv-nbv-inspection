#include "search_algo.hpp"
#include <iostream>

// --- Helper structures for A* ---
struct AStarNode
{
    VoxelIdx idx;
    double f_score;
    bool operator>(const AStarNode &o) const { return f_score > o.f_score; }
};

inline double heuristic(const VoxelIdx &a, const VoxelIdx &b)
{
    // Manhattan distance is optimal for 6-way voxel movement
    return std::abs(a.x - b.x) + std::abs(a.y - b.y) + std::abs(a.z - b.z);
}

bool find_nearest_free_voxel(
    const VoxelIdx &origin, const VoxelSet &occ_set, int max_radius, VoxelIdx &result)
{
    if (!occ_set.count(origin))
    {
        result = origin;
        return true;
    }

    std::queue<std::pair<VoxelIdx, int>> pending;
    VoxelSet visited;
    pending.push({origin, 0});
    visited.insert(origin);

    const std::vector<VoxelIdx> directions = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

    while (!pending.empty())
    {
        const auto [current, distance] = pending.front();
        pending.pop();

        if (distance >= max_radius)
            continue;

        for (const auto &direction : directions)
        {
            VoxelIdx neighbor = {
                current.x + direction.x,
                current.y + direction.y,
                current.z + direction.z};

            if (!visited.insert(neighbor).second)
                continue;

            if (!occ_set.count(neighbor))
            {
                result = neighbor;
                return true;
            }

            pending.push({neighbor, distance + 1});
        }
    }

    return false;
}

std::vector<Eigen::Vector3d> SearchAlgo::astar_3d(
    const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_pos,
    const VoxelSet &occ_set, double res, int max_steps)
{
    VoxelIdx start_idx = {
        static_cast<int>(std::floor(start_pos.x() / res)),
        static_cast<int>(std::floor(start_pos.y() / res)),
        static_cast<int>(std::floor(start_pos.z() / res))};
    VoxelIdx goal_idx = {
        static_cast<int>(std::floor(goal_pos.x() / res)),
        static_cast<int>(std::floor(goal_pos.y() / res)),
        static_cast<int>(std::floor(goal_pos.z() / res))};

    constexpr int endpoint_search_radius = 5;
    VoxelIdx free_start_idx;
    VoxelIdx free_goal_idx;
    const bool start_was_occupied = occ_set.count(start_idx) != 0;
    const bool goal_was_occupied = occ_set.count(goal_idx) != 0;

    if (!find_nearest_free_voxel(start_idx, occ_set, endpoint_search_radius, free_start_idx) ||
        !find_nearest_free_voxel(goal_idx, occ_set, endpoint_search_radius, free_goal_idx))
    {
        // std::cerr << "A* Error: Start or Goal is inside an obstacle!\n";
        return {};
    }

    // if (start_was_occupied)
    // {
    //     std::cerr << "A* Warning: shifted occupied start voxel to a nearby free voxel.\n";
    // }
    // if (goal_was_occupied)
    // {
    //     std::cerr << "A* Warning: shifted occupied goal voxel to a nearby free voxel.\n";
    // }

    start_idx = free_start_idx;
    goal_idx = free_goal_idx;

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
    open_set.push({start_idx, 0.0});

    std::unordered_map<VoxelIdx, VoxelIdx, VoxelHash> came_from;
    std::unordered_map<VoxelIdx, double, VoxelHash> g_score;
    g_score[start_idx] = 0.0;

    std::vector<VoxelIdx> directions = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

    int steps = 0;
    while (!open_set.empty() && steps < max_steps)
    {
        steps++;
        VoxelIdx current = open_set.top().idx;
        open_set.pop();

        if (current == goal_idx)
        {
            std::vector<Eigen::Vector3d> path;
            VoxelIdx curr = current;
            while (came_from.find(curr) != came_from.end())
            {
                path.push_back({(curr.x + 0.5) * res, (curr.y + 0.5) * res, (curr.z + 0.5) * res});
                curr = came_from[curr];
            }
            path.push_back({(start_idx.x + 0.5) * res, (start_idx.y + 0.5) * res, (start_idx.z + 0.5) * res});
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const auto &d : directions)
        {
            VoxelIdx neighbor = {current.x + d.x, current.y + d.y, current.z + d.z};

            if (occ_set.count(neighbor))
                continue;

            double tentative_g = g_score[current] + 1.0;

            if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor])
            {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g;
                double f = tentative_g + heuristic(neighbor, goal_idx);
                open_set.push({neighbor, f});
            }
        }
    }
    std::cerr << "A* failed to find a path.\n";
    return {};
}

bool SearchAlgo::check_line_of_sight(
    const Eigen::Vector3d &p_start, const Eigen::Vector3d &p_end,
    const VoxelSet &occ_set, double res, int gap_multiplier)
{
    Eigen::Vector3d ray_vec = p_end - p_start;
    double dist = ray_vec.norm();
    int steps = static_cast<int>(dist / res);
    int radius = gap_multiplier / 2;

    if (steps <= radius + 1)
        return true;

    // Precompute spherical offsets
    std::vector<VoxelIdx> offsets;
    for (int dx = -radius; dx <= radius; ++dx)
    {
        for (int dy = -radius; dy <= radius; ++dy)
        {
            for (int dz = -radius; dz <= radius; ++dz)
            {
                if (dx * dx + dy * dy + dz * dz <= radius * radius)
                {
                    offsets.push_back({dx, dy, dz});
                }
            }
        }
    }

    // Walk along the line
    for (int i = 1; i < steps - radius; ++i)
    {
        Eigen::Vector3d p = p_start + (ray_vec * (static_cast<double>(i) / steps));
        VoxelIdx base_idx = {
            static_cast<int>(std::floor(p.x() / res)),
            static_cast<int>(std::floor(p.y() / res)),
            static_cast<int>(std::floor(p.z() / res))};

        for (const auto &offset : offsets)
        {
            VoxelIdx check_idx = {base_idx.x + offset.x, base_idx.y + offset.y, base_idx.z + offset.z};
            if (occ_set.count(check_idx))
            {
                return false; // Collision in the gap
            }
        }
    }
    return true;
}