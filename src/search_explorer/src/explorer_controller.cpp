#include "explorer_controller.h"

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <visualization_msgs/Marker.h>

namespace {

struct CoveragePlannerCell {
    int x;
    int y;
};

struct CoveragePlannerGridView {
    int width = 0;
    int height = 0;
    std::vector<int8_t> occupancy;

    bool isInside(int x, int y) const {
        return x >= 0 && y >= 0 && x < width && y < height;
    }

    bool isFree(int x, int y) const {
        return isInside(x, y) && occupancy[static_cast<size_t>(y) * width + x] == 0;
    }
};

struct CoveragePlannerConfig {
    int lane_spacing_cells = 1;
    int sample_spacing_cells = 1;
    int min_lane_span_cells = 1;
};

struct CoveragePlannerComponent {
    std::vector<uint8_t> mask;
    int free_count = 0;
    int min_x = 0;
    int max_x = -1;
    int min_y = 0;
    int max_y = -1;

    bool empty() const {
        return free_count <= 0;
    }
};

double clampValue(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
}

double pointDistance(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

double pointDistance(double ax, double ay, double bx, double by) {
    return std::hypot(ax - bx, ay - by);
}

int coverageFlatIndex(int x, int y, int width) {
    return y * width + x;
}

struct EdgeRouteCell {
    int x;
    int y;

    bool operator==(const EdgeRouteCell& other) const {
        return x == other.x && y == other.y;
    }
};

struct EdgeRouteGrid {
    int width = 0;
    int height = 0;
    double resolution = 0.10;
    std::vector<int8_t> occupancy;

    bool isInside(int x, int y) const {
        return x >= 0 && y >= 0 && x < width && y < height;
    }
};

struct EdgeRouteConfig {
    double desired_center_offset_m = 0.72;
    double offset_tolerance_m = 0.10;
    double waypoint_spacing_m = 0.60;
    double object_standoff_m = 0.50;
    int min_route_points = 8;
};

inline int edgeFlatIndex(int x, int y, int width) {
    return y * width + x;
}

inline bool isFreeRouteCell(const EdgeRouteGrid& grid,
                            const std::vector<uint8_t>& free_mask,
                            int x,
                            int y) {
    if (!grid.isInside(x, y)) {
        return false;
    }
    const size_t index = static_cast<size_t>(edgeFlatIndex(x, y, grid.width));
    return index < free_mask.size() &&
           index < grid.occupancy.size() &&
           free_mask[index] != 0 &&
           grid.occupancy[index] == 0;
}

inline std::vector<int> computeEdgeDistanceCells(const EdgeRouteGrid& grid,
                                                 const std::vector<uint8_t>& free_mask) {
    const size_t total = static_cast<size_t>(grid.width) * static_cast<size_t>(grid.height);
    std::vector<int> distance(total, -1);
    std::queue<EdgeRouteCell> pending;

    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            const bool is_boundary = x == 0 || y == 0 ||
                                     x == grid.width - 1 || y == grid.height - 1;
            if (isFreeRouteCell(grid, free_mask, x, y) && !is_boundary) {
                continue;
            }
            const size_t index = static_cast<size_t>(edgeFlatIndex(x, y, grid.width));
            distance[index] = 0;
            pending.push(EdgeRouteCell{x, y});
        }
    }

    const int offsets[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };
    while (!pending.empty()) {
        const EdgeRouteCell current = pending.front();
        pending.pop();
        const int current_distance =
            distance[static_cast<size_t>(edgeFlatIndex(current.x, current.y, grid.width))];
        for (const auto& offset : offsets) {
            const int nx = current.x + offset[0];
            const int ny = current.y + offset[1];
            if (!grid.isInside(nx, ny)) {
                continue;
            }
            const size_t next_index = static_cast<size_t>(edgeFlatIndex(nx, ny, grid.width));
            if (distance[next_index] >= 0) {
                continue;
            }
            distance[next_index] = current_distance + 1;
            pending.push(EdgeRouteCell{nx, ny});
        }
    }

    return distance;
}

inline std::vector<uint8_t> extractDominantSafetyComponent(
    const EdgeRouteGrid& grid,
    const std::vector<uint8_t>& safe_mask,
    const EdgeRouteCell& preferred_seed,
    int min_preferred_cells,
    int& selected_count,
    int& largest_count) {
    selected_count = 0;
    largest_count = 0;
    const size_t total = static_cast<size_t>(grid.width) * static_cast<size_t>(grid.height);
    std::vector<uint8_t> empty(total, 0);
    if (grid.width <= 0 || grid.height <= 0 || safe_mask.size() != total) {
        return empty;
    }

    auto collect = [&](const EdgeRouteCell& seed, std::vector<uint8_t>& component) -> int {
        component.assign(total, 0);
        if (!grid.isInside(seed.x, seed.y)) {
            return 0;
        }
        const size_t seed_index = static_cast<size_t>(edgeFlatIndex(seed.x, seed.y, grid.width));
        if (safe_mask[seed_index] == 0) {
            return 0;
        }

        std::queue<EdgeRouteCell> pending;
        pending.push(seed);
        component[seed_index] = 1;
        int count = 0;
        const int offsets[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        while (!pending.empty()) {
            const EdgeRouteCell current = pending.front();
            pending.pop();
            ++count;
            for (const auto& offset : offsets) {
                const int nx = current.x + offset[0];
                const int ny = current.y + offset[1];
                if (!grid.isInside(nx, ny)) {
                    continue;
                }
                const size_t next = static_cast<size_t>(edgeFlatIndex(nx, ny, grid.width));
                if (safe_mask[next] == 0 || component[next] != 0) {
                    continue;
                }
                component[next] = 1;
                pending.push(EdgeRouteCell{nx, ny});
            }
        }
        return count;
    };

    std::vector<uint8_t> preferred;
    int preferred_count = collect(preferred_seed, preferred);

    std::vector<uint8_t> largest(total, 0);
    std::vector<uint8_t> seen(total, 0);
    std::queue<EdgeRouteCell> pending;
    const int offsets[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            const size_t start = static_cast<size_t>(edgeFlatIndex(x, y, grid.width));
            if (safe_mask[start] == 0 || seen[start] != 0) {
                continue;
            }
            std::vector<uint8_t> component(total, 0);
            pending.push(EdgeRouteCell{x, y});
            seen[start] = 1;
            component[start] = 1;
            int count = 0;
            while (!pending.empty()) {
                const EdgeRouteCell current = pending.front();
                pending.pop();
                ++count;
                for (const auto& offset : offsets) {
                    const int nx = current.x + offset[0];
                    const int ny = current.y + offset[1];
                    if (!grid.isInside(nx, ny)) {
                        continue;
                    }
                    const size_t next = static_cast<size_t>(edgeFlatIndex(nx, ny, grid.width));
                    if (safe_mask[next] == 0 || seen[next] != 0) {
                        continue;
                    }
                    seen[next] = 1;
                    component[next] = 1;
                    pending.push(EdgeRouteCell{nx, ny});
                }
            }
            if (count > largest_count) {
                largest_count = count;
                largest.swap(component);
            }
        }
    }

    if (preferred_count >= std::max(1, min_preferred_cells) &&
        preferred_count >= largest_count / 3) {
        selected_count = preferred_count;
        return preferred;
    }

    selected_count = largest_count;
    return largest;
}

inline std::vector<uint8_t> expandSafeMaskToRawFreeComponent(
    const EdgeRouteGrid& grid,
    const std::vector<uint8_t>& safe_mask) {
    const size_t total = static_cast<size_t>(grid.width) * static_cast<size_t>(grid.height);
    std::vector<uint8_t> raw_component(total, 0);
    if (grid.width <= 0 || grid.height <= 0 || safe_mask.size() != total) {
        return raw_component;
    }

    std::queue<EdgeRouteCell> pending;
    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            const size_t index = static_cast<size_t>(edgeFlatIndex(x, y, grid.width));
            if (safe_mask[index] == 0 || grid.occupancy[index] != 0 ||
                raw_component[index] != 0) {
                continue;
            }
            raw_component[index] = 1;
            pending.push(EdgeRouteCell{x, y});
        }
    }

    const int offsets[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    while (!pending.empty()) {
        const EdgeRouteCell current = pending.front();
        pending.pop();
        for (const auto& offset : offsets) {
            const int nx = current.x + offset[0];
            const int ny = current.y + offset[1];
            if (!grid.isInside(nx, ny)) {
                continue;
            }
            const size_t next = static_cast<size_t>(edgeFlatIndex(nx, ny, grid.width));
            if (raw_component[next] != 0 || grid.occupancy[next] != 0) {
                continue;
            }
            raw_component[next] = 1;
            pending.push(EdgeRouteCell{nx, ny});
        }
    }

    return raw_component;
}

inline bool findFreeBounds(const EdgeRouteGrid& grid,
                           const std::vector<uint8_t>& free_mask,
                           int& min_x,
                           int& min_y,
                           int& max_x,
                           int& max_y) {
    min_x = grid.width;
    min_y = grid.height;
    max_x = -1;
    max_y = -1;
    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            if (!isFreeRouteCell(grid, free_mask, x, y)) {
                continue;
            }
            min_x = std::min(min_x, x);
            min_y = std::min(min_y, y);
            max_x = std::max(max_x, x);
            max_y = std::max(max_y, y);
        }
    }
    return max_x >= min_x && max_y >= min_y;
}

inline double routeCellDistance(const EdgeRouteCell& a, const EdgeRouteCell& b) {
    return std::hypot(static_cast<double>(a.x - b.x), static_cast<double>(a.y - b.y));
}

inline bool nearestBandCell(const EdgeRouteGrid& grid,
                            const std::vector<uint8_t>& free_mask,
                            const std::vector<int>& distance,
                            const EdgeRouteCell& desired,
                            int target_distance,
                            int tolerance,
                            int search_radius,
                            EdgeRouteCell& output) {
    bool found = false;
    double best_score = std::numeric_limits<double>::infinity();
    EdgeRouteCell best{0, 0};
    for (int dy = -search_radius; dy <= search_radius; ++dy) {
        for (int dx = -search_radius; dx <= search_radius; ++dx) {
            const int nx = desired.x + dx;
            const int ny = desired.y + dy;
            if (!isFreeRouteCell(grid, free_mask, nx, ny)) {
                continue;
            }
            const size_t index = static_cast<size_t>(edgeFlatIndex(nx, ny, grid.width));
            if (index >= distance.size() || distance[index] < 0) {
                continue;
            }
            const int distance_error = std::abs(distance[index] - target_distance);
            if (distance_error > tolerance) {
                continue;
            }
            const double desired_error = std::hypot(static_cast<double>(dx),
                                                    static_cast<double>(dy));
            const double score = static_cast<double>(distance_error) * 10.0 + desired_error;
            if (score < best_score) {
                best_score = score;
                best = EdgeRouteCell{nx, ny};
                found = true;
            }
        }
    }

    if (!found) {
        return false;
    }
    output = best;
    return true;
}

inline void appendRouteCell(std::vector<EdgeRouteCell>& route,
                            const EdgeRouteCell& cell,
                            int min_spacing_cells) {
    if (!route.empty() && route.back() == cell) {
        return;
    }
    if (!route.empty() &&
        routeCellDistance(route.back(), cell) <
            static_cast<double>(std::max(1, min_spacing_cells)) * 0.45) {
        return;
    }
    route.push_back(cell);
}

inline long long crossProduct(const EdgeRouteCell& origin,
                              const EdgeRouteCell& a,
                              const EdgeRouteCell& b) {
    return static_cast<long long>(a.x - origin.x) * static_cast<long long>(b.y - origin.y) -
           static_cast<long long>(a.y - origin.y) * static_cast<long long>(b.x - origin.x);
}

std::vector<EdgeRouteCell> computeConvexHullCells(const EdgeRouteGrid& grid,
                                                  const std::vector<uint8_t>& free_mask) {
    std::vector<EdgeRouteCell> cells;
    cells.reserve(free_mask.size() / 4);
    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            if (isFreeRouteCell(grid, free_mask, x, y)) {
                cells.push_back(EdgeRouteCell{x, y});
            }
        }
    }

    if (cells.size() < 3) {
        return cells;
    }

    std::sort(cells.begin(), cells.end(),
              [](const EdgeRouteCell& a, const EdgeRouteCell& b) {
                  if (a.x == b.x) {
                      return a.y < b.y;
                  }
                  return a.x < b.x;
              });
    cells.erase(std::unique(cells.begin(), cells.end(),
                            [](const EdgeRouteCell& a, const EdgeRouteCell& b) {
                                return a.x == b.x && a.y == b.y;
                            }),
                cells.end());

    std::vector<EdgeRouteCell> hull;
    hull.reserve(cells.size() * 2);
    for (const EdgeRouteCell& cell : cells) {
        while (hull.size() >= 2 &&
               crossProduct(hull[hull.size() - 2], hull.back(), cell) <= 0) {
            hull.pop_back();
        }
        hull.push_back(cell);
    }

    const size_t lower_size = hull.size();
    for (auto it = cells.rbegin(); it != cells.rend(); ++it) {
        while (hull.size() > lower_size &&
               crossProduct(hull[hull.size() - 2], hull.back(), *it) <= 0) {
            hull.pop_back();
        }
        hull.push_back(*it);
    }

    if (!hull.empty()) {
        hull.pop_back();
    }

    if (hull.size() < 3) {
        return hull;
    }

    long long twice_area = 0;
    for (size_t i = 0; i < hull.size(); ++i) {
        const EdgeRouteCell& a = hull[i];
        const EdgeRouteCell& b = hull[(i + 1) % hull.size()];
        twice_area += static_cast<long long>(a.x) * static_cast<long long>(b.y) -
                      static_cast<long long>(b.x) * static_cast<long long>(a.y);
    }
    if (twice_area < 0) {
        std::reverse(hull.begin(), hull.end());
    }

    return hull;
}

struct EdgePathNode {
    EdgeRouteCell cell;
    double g_cost;
    double f_cost;

    bool operator>(const EdgePathNode& other) const {
        return f_cost > other.f_cost;
    }
};

inline double edgeRoutePenalty(int distance_cell, int target_distance) {
    const int error = std::abs(distance_cell - target_distance);
    const double penalty = static_cast<double>(error * error) * 0.12;
    if (distance_cell < std::max(1, target_distance - 3)) {
        return penalty + 5.0;
    }
    if (distance_cell > target_distance + 3) {
        return penalty + 1.5;
    }
    return penalty;
}

bool edgeBiasedSearch(const EdgeRouteGrid& grid,
                      const std::vector<uint8_t>& free_mask,
                      const std::vector<int>& distance,
                      const EdgeRouteCell& start,
                      const EdgeRouteCell& goal,
                      int target_distance,
                      std::vector<EdgeRouteCell>& output) {
    if (!isFreeRouteCell(grid, free_mask, start.x, start.y) ||
        !isFreeRouteCell(grid, free_mask, goal.x, goal.y)) {
        return false;
    }

    const size_t total = static_cast<size_t>(grid.width) * static_cast<size_t>(grid.height);
    const auto flatIndex = [width = grid.width](int x, int y) -> size_t {
        return static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);
    };

    const size_t start_index = flatIndex(start.x, start.y);
    const size_t goal_index = flatIndex(goal.x, goal.y);
    std::vector<double> g_cost(total, std::numeric_limits<double>::infinity());
    std::vector<EdgeRouteCell> parent(total, EdgeRouteCell{-1, -1});
    std::vector<uint8_t> closed(total, 0);
    std::priority_queue<EdgePathNode, std::vector<EdgePathNode>, std::greater<EdgePathNode>> open;

    g_cost[start_index] = 0.0;
    open.push(EdgePathNode{start, 0.0, routeCellDistance(start, goal)});

    const int offsets[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };

    int iteration = 0;
    const int max_iterations = grid.width * grid.height * 4;
    while (!open.empty()) {
        if (++iteration > max_iterations) {
            return false;
        }

        const EdgePathNode current = open.top();
        open.pop();

        const size_t current_index = flatIndex(current.cell.x, current.cell.y);
        if (closed[current_index] != 0) {
            continue;
        }
        closed[current_index] = 1;

        if (current_index == goal_index) {
            output.clear();
            EdgeRouteCell trace = goal;
            while (trace.x >= 0 && trace.y >= 0) {
                output.push_back(trace);
                if (trace == start) {
                    break;
                }
                const size_t trace_index = flatIndex(trace.x, trace.y);
                trace = parent[trace_index];
            }
            if (output.empty() || !(output.back() == start)) {
                return false;
            }
            std::reverse(output.begin(), output.end());
            return true;
        }

        for (const auto& offset : offsets) {
            const int nx = current.cell.x + offset[0];
            const int ny = current.cell.y + offset[1];
            if (!isFreeRouteCell(grid, free_mask, nx, ny)) {
                continue;
            }
            const size_t next_index = flatIndex(nx, ny);
            if (closed[next_index] != 0) {
                continue;
            }

            const bool diagonal = offset[0] != 0 && offset[1] != 0;
            const double step_cost = diagonal ? 1.41421356237 : 1.0;
            const int distance_cell = (next_index < distance.size() && distance[next_index] >= 0)
                                          ? distance[next_index]
                                          : target_distance;
            const double tentative = current.g_cost + step_cost + edgeRoutePenalty(distance_cell, target_distance);
            if (tentative < g_cost[next_index]) {
                g_cost[next_index] = tentative;
                parent[next_index] = current.cell;
                open.push(EdgePathNode{
                    EdgeRouteCell{nx, ny},
                    tentative,
                    tentative + routeCellDistance(EdgeRouteCell{nx, ny}, goal)
                });
            }
        }
    }

    return false;
}

void appendStitchedRouteCell(std::vector<EdgeRouteCell>& route,
                             const EdgeRouteCell& cell,
                             int min_spacing_cells) {
    if (route.empty()) {
        route.push_back(cell);
        return;
    }
    if (route.back() == cell) {
        return;
    }
    if (routeCellDistance(route.back(), cell) <
        static_cast<double>(std::max(1, min_spacing_cells)) * 0.85) {
        return;
    }
    route.push_back(cell);
}

std::vector<EdgeRouteCell> stitchRouteAnchors(
    const EdgeRouteGrid& grid,
    const std::vector<uint8_t>& free_mask,
    const std::vector<int>& distance,
    const std::vector<EdgeRouteCell>& anchors,
    int target_distance,
    int min_spacing_cells) {
    std::vector<EdgeRouteCell> route;
    if (anchors.empty()) {
        return route;
    }
    if (anchors.size() == 1U) {
        route.push_back(anchors.front());
        return route;
    }

    std::vector<EdgeRouteCell> path;
    route.reserve(anchors.size() * std::max(1, min_spacing_cells));
    for (size_t i = 0; i < anchors.size(); ++i) {
        const EdgeRouteCell& start = anchors[i];
        const EdgeRouteCell& goal = anchors[(i + 1) % anchors.size()];
        path.clear();
        if (!edgeBiasedSearch(grid, free_mask, distance, start, goal, target_distance, path)) {
            appendStitchedRouteCell(route, start, min_spacing_cells);
            appendStitchedRouteCell(route, goal, min_spacing_cells);
            continue;
        }
        for (const EdgeRouteCell& cell : path) {
            appendStitchedRouteCell(route, cell, min_spacing_cells);
        }
    }

    if (route.size() >= 2 &&
        route.front() == route.back()) {
        route.pop_back();
    }
    return route;
}

std::vector<EdgeRouteCell> buildConvexEdgeAnchors(
    const EdgeRouteGrid& grid,
    const std::vector<uint8_t>& route_mask,
    const std::vector<int>& distance,
    const std::vector<uint8_t>& hull_mask,
    const EdgeRouteConfig& config) {
    std::vector<EdgeRouteCell> hull = computeConvexHullCells(grid, hull_mask);
    if (hull.size() < 3) {
        return std::vector<EdgeRouteCell>();
    }

    const int target_distance = std::max(
        1, static_cast<int>(std::round(config.desired_center_offset_m / grid.resolution)));
    const int tolerance = std::max(
        1, static_cast<int>(std::ceil(config.offset_tolerance_m / grid.resolution)));
    const int sample_step = std::max(
        1, static_cast<int>(std::round(config.waypoint_spacing_m / grid.resolution)));
    const int search_radius = std::max(target_distance + tolerance + 4, sample_step);

    std::vector<EdgeRouteCell> anchors;
    anchors.reserve(hull.size() * 4);

    for (size_t i = 0; i < hull.size(); ++i) {
        const EdgeRouteCell& a = hull[i];
        const EdgeRouteCell& b = hull[(i + 1) % hull.size()];
        const double edge_x = static_cast<double>(b.x - a.x);
        const double edge_y = static_cast<double>(b.y - a.y);
        const double length = std::hypot(edge_x, edge_y);
        if (length < 1e-6) {
            continue;
        }

        const double inward_x = -edge_y / length;
        const double inward_y = edge_x / length;
        const int sample_count = std::max(
            1, static_cast<int>(std::ceil(length / static_cast<double>(sample_step))));

        for (int sample = 0; sample <= sample_count; ++sample) {
            const double t = static_cast<double>(sample) / static_cast<double>(sample_count);
            const double desired_x = static_cast<double>(a.x) + edge_x * t +
                                     inward_x * static_cast<double>(target_distance);
            const double desired_y = static_cast<double>(a.y) + edge_y * t +
                                     inward_y * static_cast<double>(target_distance);

            EdgeRouteCell snapped{0, 0};
            if (nearestBandCell(grid, route_mask, distance,
                                EdgeRouteCell{static_cast<int>(std::round(desired_x)),
                                              static_cast<int>(std::round(desired_y))},
                                target_distance, tolerance, search_radius, snapped)) {
                appendRouteCell(anchors, snapped, std::max(1, sample_step / 2));
            }
        }
    }

    if (anchors.size() >= 2 &&
        routeCellDistance(anchors.front(), anchors.back()) < static_cast<double>(std::max(1, sample_step)) * 0.45) {
        anchors.pop_back();
    }

    return anchors;
}

inline std::vector<EdgeRouteCell> angleSortedFallbackRoute(
    const EdgeRouteGrid& grid,
    const std::vector<uint8_t>& free_mask,
    const std::vector<int>& distance,
    const EdgeRouteConfig& config,
    int target_distance,
    int tolerance,
    int min_spacing_cells) {
    std::vector<EdgeRouteCell> candidates;
    double cx = 0.0;
    double cy = 0.0;
    int count = 0;
    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            if (!isFreeRouteCell(grid, free_mask, x, y)) {
                continue;
            }
            const size_t index = static_cast<size_t>(edgeFlatIndex(x, y, grid.width));
            if (index >= distance.size() ||
                std::abs(distance[index] - target_distance) > tolerance) {
                continue;
            }
            candidates.push_back(EdgeRouteCell{x, y});
            cx += static_cast<double>(x);
            cy += static_cast<double>(y);
            ++count;
        }
    }

    if (candidates.empty() || count == 0) {
        return std::vector<EdgeRouteCell>();
    }
    cx /= static_cast<double>(count);
    cy /= static_cast<double>(count);

    std::sort(candidates.begin(), candidates.end(),
              [cx, cy](const EdgeRouteCell& a, const EdgeRouteCell& b) {
                  const double aa = std::atan2(static_cast<double>(a.y) - cy,
                                               static_cast<double>(a.x) - cx);
                  const double ab = std::atan2(static_cast<double>(b.y) - cy,
                                               static_cast<double>(b.x) - cx);
                  if (aa == ab) {
                      return routeCellDistance(a, EdgeRouteCell{
                          static_cast<int>(std::round(cx)),
                          static_cast<int>(std::round(cy))}) <
                             routeCellDistance(b, EdgeRouteCell{
                          static_cast<int>(std::round(cx)),
                          static_cast<int>(std::round(cy))});
                  }
                  return aa < ab;
              });

    std::vector<EdgeRouteCell> route;
    for (const EdgeRouteCell& candidate : candidates) {
        appendRouteCell(route, candidate, min_spacing_cells);
    }
    (void)config;
    return route;
}

inline std::vector<EdgeRouteCell> buildEdgeFollowingRoute(
    const EdgeRouteGrid& grid,
    const std::vector<uint8_t>& free_mask,
    const EdgeRouteConfig& config) {
    if (grid.width <= 0 || grid.height <= 0 || grid.resolution <= 0.0 ||
        free_mask.size() != static_cast<size_t>(grid.width * grid.height)) {
        return std::vector<EdgeRouteCell>();
    }

    std::vector<uint8_t> raw_mask = expandSafeMaskToRawFreeComponent(grid, free_mask);

    int min_x = 0;
    int min_y = 0;
    int max_x = 0;
    int max_y = 0;
    if (!findFreeBounds(grid, raw_mask, min_x, min_y, max_x, max_y)) {
        return std::vector<EdgeRouteCell>();
    }

    const std::vector<int> distance = computeEdgeDistanceCells(grid, raw_mask);
    const int target_distance = std::max(
        1, static_cast<int>(std::round(config.desired_center_offset_m / grid.resolution)));
    const int tolerance = std::max(
        1, static_cast<int>(std::ceil(config.offset_tolerance_m / grid.resolution)));
    const int sample_step = std::max(
        1, static_cast<int>(std::round(config.waypoint_spacing_m / grid.resolution)));
    std::vector<EdgeRouteCell> route = buildConvexEdgeAnchors(grid, free_mask, distance, raw_mask, config);
    if (static_cast<int>(route.size()) < config.min_route_points) {
        const int left = min_x;
        const int right = max_x;
        const int bottom = min_y;
        const int top = max_y;

        std::vector<EdgeRouteCell> desired;
        for (int x = left; x <= right; x += sample_step) {
            desired.push_back(EdgeRouteCell{x, bottom});
        }
        for (int y = bottom + sample_step; y <= top; y += sample_step) {
            desired.push_back(EdgeRouteCell{right, y});
        }
        for (int x = right - sample_step; x >= left; x -= sample_step) {
            desired.push_back(EdgeRouteCell{x, top});
        }
        for (int y = top - sample_step; y > bottom; y -= sample_step) {
            desired.push_back(EdgeRouteCell{left, y});
        }

        route.clear();
        const int search_radius = std::max(target_distance + tolerance + 2, sample_step);
        route.reserve(desired.size());
        for (const EdgeRouteCell& target : desired) {
            EdgeRouteCell candidate{0, 0};
            if (nearestBandCell(grid, free_mask, distance, target,
                                target_distance, tolerance, search_radius, candidate)) {
                appendRouteCell(route, candidate, std::max(1, sample_step / 2));
            }
        }

        if (static_cast<int>(route.size()) < config.min_route_points) {
            route = angleSortedFallbackRoute(grid, free_mask, distance, config,
                                             target_distance, tolerance, sample_step);
        }
    }

    if (route.size() >= 2) {
        const std::vector<EdgeRouteCell> stitched =
            stitchRouteAnchors(grid, free_mask, distance, route,
                               target_distance, std::max(1, sample_step));
        if (static_cast<int>(stitched.size()) >= config.min_route_points) {
            return stitched;
        }
    }

    return route;
}

inline bool chooseObjectApproachCell(const EdgeRouteGrid& grid,
                                     const std::vector<uint8_t>& free_mask,
                                     const EdgeRouteCell& start,
                                     const EdgeRouteCell& object,
                                     const EdgeRouteConfig& config,
                                     EdgeRouteCell& approach) {
    if (grid.width <= 0 || grid.height <= 0 || grid.resolution <= 0.0 ||
        free_mask.size() != static_cast<size_t>(grid.width * grid.height)) {
        return false;
    }

    double dx = static_cast<double>(start.x - object.x);
    double dy = static_cast<double>(start.y - object.y);
    const double norm = std::hypot(dx, dy);
    if (norm < 1e-6) {
        dx = -1.0;
        dy = 0.0;
    } else {
        dx /= norm;
        dy /= norm;
    }

    const double standoff_cells =
        std::max(1.0, config.object_standoff_m / grid.resolution);
    const EdgeRouteCell desired{
        static_cast<int>(std::round(static_cast<double>(object.x) + dx * standoff_cells)),
        static_cast<int>(std::round(static_cast<double>(object.y) + dy * standoff_cells))
    };

    bool found = false;
    double best_score = std::numeric_limits<double>::infinity();
    EdgeRouteCell best{0, 0};
    const int search_radius = std::max(3, static_cast<int>(std::ceil(standoff_cells)) + 3);
    for (int radius = 0; radius <= search_radius; ++radius) {
        for (int y = desired.y - radius; y <= desired.y + radius; ++y) {
            for (int x = desired.x - radius; x <= desired.x + radius; ++x) {
                if (std::max(std::abs(x - desired.x), std::abs(y - desired.y)) != radius) {
                    continue;
                }
                if (!isFreeRouteCell(grid, free_mask, x, y)) {
                    continue;
                }
                const double desired_error =
                    std::hypot(static_cast<double>(x - desired.x),
                               static_cast<double>(y - desired.y));
                const double start_error =
                    std::hypot(static_cast<double>(x - start.x),
                               static_cast<double>(y - start.y));
                const double object_distance =
                    std::hypot(static_cast<double>(x - object.x),
                               static_cast<double>(y - object.y));
                const double standoff_error = std::fabs(object_distance - standoff_cells);
                const double score = desired_error + standoff_error * 2.0 + start_error * 0.02;
                if (score < best_score) {
                    best_score = score;
                    best = EdgeRouteCell{x, y};
                    found = true;
                }
            }
        }
        if (found) {
            approach = best;
            return true;
        }
    }

    return false;
}

CoveragePlannerCell nearestFreeCell(const CoveragePlannerGridView& grid,
                                    const CoveragePlannerCell& start) {
    if (grid.isFree(start.x, start.y)) {
        return start;
    }

    std::vector<uint8_t> visited(static_cast<size_t>(grid.width * grid.height), 0);
    std::queue<CoveragePlannerCell> queue;
    if (grid.isInside(start.x, start.y)) {
        queue.push(start);
        visited[static_cast<size_t>(coverageFlatIndex(start.x, start.y, grid.width))] = 1;
    }

    const int offsets[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    while (!queue.empty()) {
        const CoveragePlannerCell current = queue.front();
        queue.pop();
        for (const auto& offset : offsets) {
            const CoveragePlannerCell next{current.x + offset[0], current.y + offset[1]};
            if (!grid.isInside(next.x, next.y)) {
                continue;
            }
            const int index = coverageFlatIndex(next.x, next.y, grid.width);
            if (visited[static_cast<size_t>(index)]) {
                continue;
            }
            visited[static_cast<size_t>(index)] = 1;
            if (grid.isFree(next.x, next.y)) {
                return next;
            }
            queue.push(next);
        }
    }

    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            if (grid.isFree(x, y)) {
                return CoveragePlannerCell{x, y};
            }
        }
    }
    return start;
}

std::vector<uint8_t> collectComponentMask(const CoveragePlannerGridView& grid,
                                          const CoveragePlannerCell& seed) {
    std::vector<uint8_t> component(static_cast<size_t>(grid.width * grid.height), 0);
    if (!grid.isFree(seed.x, seed.y)) {
        return component;
    }

    std::queue<CoveragePlannerCell> queue;
    queue.push(seed);
    component[static_cast<size_t>(coverageFlatIndex(seed.x, seed.y, grid.width))] = 1;

    const int offsets[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    while (!queue.empty()) {
        const CoveragePlannerCell current = queue.front();
        queue.pop();
        for (const auto& offset : offsets) {
            const CoveragePlannerCell next{current.x + offset[0], current.y + offset[1]};
            if (!grid.isFree(next.x, next.y)) {
                continue;
            }
            const int index = coverageFlatIndex(next.x, next.y, grid.width);
            if (component[static_cast<size_t>(index)]) {
                continue;
            }
            component[static_cast<size_t>(index)] = 1;
            queue.push(next);
        }
    }

    return component;
}

CoveragePlannerComponent buildComponentFromMask(const CoveragePlannerGridView& grid,
                                                std::vector<uint8_t> mask) {
    CoveragePlannerComponent component;
    component.mask = std::move(mask);
    component.min_x = grid.width;
    component.min_y = grid.height;

    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            if (component.mask[static_cast<size_t>(coverageFlatIndex(x, y, grid.width))] == 0) {
                continue;
            }
            ++component.free_count;
            component.min_x = std::min(component.min_x, x);
            component.max_x = std::max(component.max_x, x);
            component.min_y = std::min(component.min_y, y);
            component.max_y = std::max(component.max_y, y);
        }
    }

    if (component.empty()) {
        component.min_x = 0;
        component.max_x = -1;
        component.min_y = 0;
        component.max_y = -1;
    }

    return component;
}

CoveragePlannerComponent collectLargestFreeComponent(const CoveragePlannerGridView& grid) {
    CoveragePlannerComponent best;
    std::vector<uint8_t> visited(static_cast<size_t>(grid.width * grid.height), 0);

    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            if (!grid.isFree(x, y)) {
                continue;
            }

            const int seed_index = coverageFlatIndex(x, y, grid.width);
            if (visited[static_cast<size_t>(seed_index)] != 0) {
                continue;
            }

            std::vector<uint8_t> mask(static_cast<size_t>(grid.width * grid.height), 0);
            std::queue<CoveragePlannerCell> queue;
            queue.push(CoveragePlannerCell{x, y});
            visited[static_cast<size_t>(seed_index)] = 1;
            mask[static_cast<size_t>(seed_index)] = 1;

            const int offsets[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
            while (!queue.empty()) {
                const CoveragePlannerCell current = queue.front();
                queue.pop();
                for (const auto& offset : offsets) {
                    const CoveragePlannerCell next{current.x + offset[0], current.y + offset[1]};
                    if (!grid.isFree(next.x, next.y)) {
                        continue;
                    }
                    const int next_index = coverageFlatIndex(next.x, next.y, grid.width);
                    if (visited[static_cast<size_t>(next_index)] != 0) {
                        continue;
                    }
                    visited[static_cast<size_t>(next_index)] = 1;
                    mask[static_cast<size_t>(next_index)] = 1;
                    queue.push(next);
                }
            }

            CoveragePlannerComponent candidate = buildComponentFromMask(grid, std::move(mask));
            if (candidate.free_count > best.free_count) {
                best = std::move(candidate);
            }
        }
    }

    return best;
}

std::vector<CoveragePlannerCell> dedupeRoute(const std::vector<CoveragePlannerCell>& route) {
    std::vector<CoveragePlannerCell> result;
    result.reserve(route.size());
    for (const CoveragePlannerCell& cell : route) {
        if (result.empty() || result.back().x != cell.x || result.back().y != cell.y) {
            result.push_back(cell);
        }
    }
    return result;
}

std::vector<CoveragePlannerCell> buildSweepRoute(const CoveragePlannerGridView& grid,
                                                 const std::vector<uint8_t>& component,
                                                 const CoveragePlannerConfig& config,
                                                 bool horizontal) {
    std::vector<CoveragePlannerCell> route;
    if (grid.width <= 0 || grid.height <= 0) {
        return route;
    }

    const int lane_step = std::max(1, config.lane_spacing_cells);
    const int sample_step = std::max(1, config.sample_spacing_cells);
    const int min_lane_span = std::max(1, config.min_lane_span_cells);
    bool reverse = false;

    auto componentAt = [&](int x, int y) -> bool {
        return grid.isInside(x, y) &&
               component[static_cast<size_t>(coverageFlatIndex(x, y, grid.width))] != 0;
    };

    auto longestContiguousLane = [&](int lane_index) {
        std::vector<CoveragePlannerCell> best_lane;
        std::vector<CoveragePlannerCell> current_lane;

        const int primary_limit = horizontal ? grid.width : grid.height;
        for (int primary = 0; primary < primary_limit; ++primary) {
            const int x = horizontal ? primary : lane_index;
            const int y = horizontal ? lane_index : primary;

            if (componentAt(x, y)) {
                current_lane.push_back(CoveragePlannerCell{x, y});
                continue;
            }

            if (current_lane.size() > best_lane.size()) {
                best_lane = current_lane;
            }
            current_lane.clear();
        }

        if (current_lane.size() > best_lane.size()) {
            best_lane = current_lane;
        }
        return best_lane;
    };

    if (horizontal) {
        for (int y = 0; y < grid.height; y += lane_step) {
            std::vector<CoveragePlannerCell> lane = longestContiguousLane(y);
            if (static_cast<int>(lane.size()) < min_lane_span) {
                continue;
            }
            if (reverse) {
                std::reverse(lane.begin(), lane.end());
            }
            for (size_t i = 0; i < lane.size(); i += static_cast<size_t>(sample_step)) {
                route.push_back(lane[i]);
            }
            if (route.empty() || route.back().x != lane.back().x || route.back().y != lane.back().y) {
                route.push_back(lane.back());
            }
            reverse = !reverse;
        }
    } else {
        for (int x = 0; x < grid.width; x += lane_step) {
            std::vector<CoveragePlannerCell> lane = longestContiguousLane(x);
            if (static_cast<int>(lane.size()) < min_lane_span) {
                continue;
            }
            if (reverse) {
                std::reverse(lane.begin(), lane.end());
            }
            for (size_t i = 0; i < lane.size(); i += static_cast<size_t>(sample_step)) {
                route.push_back(lane[i]);
            }
            if (route.empty() || route.back().x != lane.back().x || route.back().y != lane.back().y) {
                route.push_back(lane.back());
            }
            reverse = !reverse;
        }
    }

    return dedupeRoute(route);
}

}  // namespace

constexpr int8_t ExplorerController::FREE_CELL;
constexpr int8_t ExplorerController::UNKNOWN_CELL;
constexpr int8_t ExplorerController::OCCUPIED_CELL;

ExplorerController::ExplorerController()
    : pnh_("~")
    , tf_listener_(tf_buffer_)
    , route_index_(0)
    , route_computed_(false)
    , total_circuits_(0)
    , coverage_cells_total_(0)
    , map_received_(false)
    , local_map_received_(false)
    , pose_received_(false)
    , robot_yaw_(0.0)
    , state_(RobotState::INIT)
    , has_active_goal_(false)
    , object_goal_active_(false)
    , navigating_to_object_(false)
    , last_object_goal_time_(0)
    , last_object_goal_pos_{}
    , last_selected_wp_(-1)
    , dynamic_obstacle_active_(false)
    , scan_msg_ct_(0)
    , min_front_scan_range_(std::numeric_limits<double>::infinity())
    , stuck_ctr_(0)
    , last_sx_(0.0)
    , last_sy_(0.0)
    , last_stuck_check_time_(0)
    , pose_msg_ct_(0)
    , path_ok_(0)
    , path_fail_(0)
    , replan_ct_(0)
    , wp_skipped_(0)
    , wp_visited_(0)
    , total_dist_(0.0)
    , current_goal_start_distance_(0.0)
    , latest_obstacle_warning_(false)
    , soft_avoid_active_(false)
    , last_soft_avoid_time_(0) {
    pnh_.param<double>("max_linear_speed", max_lin_, 0.13);
    pnh_.param<double>("max_angular_speed", max_ang_, 0.50);
    pnh_.param<double>("goal_tolerance", goal_tol_, 0.30);
    pnh_.param<double>("lookahead_distance", lookahead_, 0.50);
    pnh_.param<double>("path_replan_threshold", path_replan_thresh_, 1.00);
    pnh_.param<double>("dynamic_clear_time", dynamic_clear_time_, 3.0);
    pnh_.param<double>("inflation_radius", inflation_radius_, 0.30);
    pnh_.param<double>("route_safety_radius", route_safety_radius_, 0.50);
    pnh_.param<double>("coverage_range", cov_range_, COVERAGE_RANGE);
    pnh_.param<double>("coverage_half_angle_deg", cov_half_angle_, COVERAGE_HALF_ANGLE_DEG);
    pnh_.param<double>("align_angular_speed", align_ang_, 0.40);
    pnh_.param<double>("route_waypoint_spacing", wp_spacing_, 0.60);
    pnh_.param<double>("waypoint_coverage_radius", wp_cov_radius_, 2.0);
    pnh_.param<double>("scan_coverage_target", scan_cov_target_, 0.80);
    pnh_.param<double>("health_report_interval", health_interval_, 2.0);
    pnh_.param<bool>("phase1_static_global_only", phase1_static_global_only_, true);
    pnh_.param<bool>("enable_object_goal", enable_object_goal_, false);
    pnh_.param<bool>("use_edge_route", use_edge_route_, true);
    pnh_.param<double>("edge_side_clearance", edge_side_clearance_, 0.22);
    pnh_.param<double>("edge_preferred_center_distance", edge_preferred_center_distance_, 0.70);
    pnh_.param<double>("edge_min_center_distance", edge_min_center_distance_, 0.60);
    pnh_.param<double>("edge_max_center_distance", edge_max_center_distance_, 0.85);
    pnh_.param<double>("edge_offset_tolerance", edge_offset_tolerance_, 0.12);
    pnh_.param<int>("min_static_obstacle_cluster_cells", min_static_obstacle_cluster_cells_, 4);
    pnh_.param<int>("edge_reachable_search_window", edge_reachable_search_window_, 10);
    pnh_.param<double>("object_standoff_distance", object_standoff_, 0.50);
    pnh_.param<bool>("require_local_map_before_navigation",
                     require_local_map_before_navigation_, true);
    pnh_.param<bool>("startup_gate_enabled", startup_gate_enabled_, true);
    pnh_.param<double>("startup_stable_duration", startup_stable_duration_, 3.0);
    pnh_.param<double>("startup_pose_max_age", startup_pose_max_age_, 1.5);
    pnh_.param<double>("startup_local_map_max_age", startup_local_map_max_age_, 1.5);
    pnh_.param<double>("startup_scan_max_age", startup_scan_max_age_, 1.0);
    pnh_.param<double>("laser_front_angle_deg", laser_front_angle_, 90.0);
    pnh_.param<double>("front_block_replan_delay", front_block_replan_delay_, 1.8);
    pnh_.param<int>("front_block_skip_count", front_block_skip_count_, 3);
    pnh_.param<int>("startup_min_pose_messages", startup_min_pose_messages_, 5);
    pnh_.param<bool>("startup_require_clear_warning", startup_require_clear_warning_, true);

    if (phase1_static_global_only_) {
        require_local_map_before_navigation_ = false;
        startup_require_clear_warning_ = false;
    }

    max_lin_ = clampValue(max_lin_, 0.05, 0.13);
    inflation_radius_ = clampValue(inflation_radius_, 0.55, 0.80);
    route_safety_radius_ = clampValue(route_safety_radius_, ROBOT_ROTATION_RADIUS, 0.70);
    wp_spacing_ = clampValue(wp_spacing_, 0.40, 0.80);
    edge_side_clearance_ = clampValue(edge_side_clearance_, 0.15, 0.30);
    edge_preferred_center_distance_ =
        clampValue(edge_preferred_center_distance_, 0.60, 0.85);
    edge_min_center_distance_ =
        clampValue(edge_min_center_distance_, ROBOT_ROTATION_RADIUS, edge_preferred_center_distance_);
    edge_max_center_distance_ =
        clampValue(edge_max_center_distance_, edge_preferred_center_distance_, 1.10);
    edge_offset_tolerance_ =
        std::max(edge_offset_tolerance_, edge_preferred_center_distance_ - edge_min_center_distance_);
    edge_offset_tolerance_ =
        std::max(edge_offset_tolerance_, edge_max_center_distance_ - edge_preferred_center_distance_);
    edge_center_offset_ = edge_preferred_center_distance_;
    edge_side_clearance_ = std::max(edge_side_clearance_, 0.20);
    edge_offset_tolerance_ = std::max(edge_offset_tolerance_, 0.08);
    min_static_obstacle_cluster_cells_ = std::max(1, min_static_obstacle_cluster_cells_);
    edge_reachable_search_window_ = std::max(3, std::min(edge_reachable_search_window_, 20));
    object_standoff_ = clampValue(object_standoff_, ROBOT_ROTATION_RADIUS, 1.20);
    cov_range_ = clampValue(cov_range_, 0.50, 2.50);
    cov_half_angle_ = clampValue(cov_half_angle_, 5.0, 90.0) * M_PI / 180.0;
    startup_stable_duration_ = clampValue(startup_stable_duration_, 0.5, 20.0);
    startup_pose_max_age_ = clampValue(startup_pose_max_age_, 0.3, POSE_STALE_TIMEOUT);
    startup_local_map_max_age_ = clampValue(startup_local_map_max_age_, 0.3, LOCAL_MAP_STALE_TIMEOUT);
    startup_scan_max_age_ = clampValue(startup_scan_max_age_, 0.2, 5.0);
    laser_front_angle_ = angles::normalize_angle(laser_front_angle_ * M_PI / 180.0);
    front_block_replan_delay_ = clampValue(front_block_replan_delay_, 0.5, 6.0);
    front_block_skip_count_ = std::max(1, front_block_skip_count_);
    startup_min_pose_messages_ = std::max(1, startup_min_pose_messages_);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    coverage_goal_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/coverage_goal", 1);
    global_path_pub_ = nh_.advertise<nav_msgs::Path>("/global_path", 1);
    coverage_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/coverage_markers", 1);

    map_sub_ = nh_.subscribe("/map", 1, &ExplorerController::mapCallback, this);
    pose_sub_ = nh_.subscribe("/amcl_pose", 5, &ExplorerController::poseCallback, this);
    scan_sub_ = nh_.subscribe("/scan", 1, &ExplorerController::scanCallback, this);
    if (!phase1_static_global_only_) {
        warning_sub_ = nh_.subscribe("/obstacle_warning", 5, &ExplorerController::warningCallback, this);
        dynamic_points_sub_ = nh_.subscribe("/dynamic_obstacles", 5, &ExplorerController::dynamicPointsCallback, this);
        local_map_sub_ = nh_.subscribe("/local_dynamic_map", 5, &ExplorerController::localMapCallback, this);
    }
    if (!phase1_static_global_only_ || enable_object_goal_) {
        object_goal_sub_ = nh_.subscribe("/object_goal", 5, &ExplorerController::objectGoalCallback, this);
    }

    control_timer_ = nh_.createTimer(ros::Duration(0.1), &ExplorerController::controlLoop, this);
    exploration_timer_ = nh_.createTimer(ros::Duration(0.5), &ExplorerController::explorationLoop, this);
    dynamic_decay_timer_ = nh_.createTimer(ros::Duration(1.0), &ExplorerController::dynamicDecayLoop, this);
    coverage_update_timer_ = nh_.createTimer(ros::Duration(0.2), &ExplorerController::coverageUpdateLoop, this);
    health_report_timer_ = nh_.createTimer(ros::Duration(health_interval_), &ExplorerController::healthReportLoop, this);

    state_enter_time_ = ros::Time::now();
    controller_start_time_ = state_enter_time_;
    startup_gate_ready_since_ = ros::Time(0);
    startup_gate_released_ = !startup_gate_enabled_;
    last_report_ = state_enter_time_;
    last_pose_msg_time_ = ros::Time(0);
    last_map_msg_time_ = ros::Time(0);
    last_lmap_msg_time_ = ros::Time(0);
    last_scan_msg_time_ = ros::Time(0);
    last_scan_stamp_ = ros::Time(0);
    last_goal_progress_time_ = ros::Time(0);
    front_block_since_ = ros::Time(0);

    ROS_INFO("============================================================");
    ROS_INFO("[EXPLORER] Scout Mini coverage explorer started");
    ROS_INFO("[EXPLORER] Phase1 static global only=%s",
             phase1_static_global_only_ ? "ON" : "OFF");
    ROS_INFO("[EXPLORER] Coverage sector: range=%.2fm half_angle=%.1fdeg",
             cov_range_, cov_half_angle_ * 180.0 / M_PI);
    ROS_INFO("[EXPLORER] Route spacing=%.2fm planner=%s center_clearance=%.2fm safe_range=%.2f-%.2fm",
             wp_spacing_, use_edge_route_ ? "edge-follow-first" : "sweep",
             edge_center_offset_, edge_min_center_distance_, edge_max_center_distance_);
    ROS_INFO("[EXPLORER] Inflation radius=%.2fm route_safety=%.2fm obstacle_cluster_filter=%d cells route_window=%d",
             inflation_radius_, route_safety_radius_, min_static_obstacle_cluster_cells_,
             edge_reachable_search_window_);
    ROS_INFO("[EXPLORER] Input topics: %s",
             phase1_static_global_only_
                 ? "/map /amcl_pose /scan"
                 : "/map /local_dynamic_map /amcl_pose /obstacle_warning /dynamic_obstacles /object_goal");
    ROS_INFO("[EXPLORER] Goal tolerance=%.2fm lookahead=%.2fm inflation=%.2fm",
             goal_tol_, lookahead_, inflation_radius_);
    ROS_INFO("[EXPLORER] Startup gate=%s stable=%.1fs pose_source=map->base_link TF scan_age<=%.1fs local_age<=%.1fs min_amcl_msgs=%d diagnostic_only clear_warning=%s",
             startup_gate_enabled_ ? "ON" : "OFF",
             startup_stable_duration_,
             startup_scan_max_age_,
             startup_local_map_max_age_,
             startup_min_pose_messages_,
             startup_require_clear_warning_ ? "Y" : "N");
    ROS_INFO("[EXPLORER] LiDAR front angle=%.1fdeg in laser frame",
             laser_front_angle_ * 180.0 / M_PI);
    ROS_INFO("============================================================");
}

void ExplorerController::spin() {
    ros::spin();
}

std::string ExplorerController::stateName(RobotState state) const {
    switch (state) {
        case RobotState::INIT: return "INIT";
        case RobotState::ALIGN: return "ALIGN";
        case RobotState::EXPLORE: return "EXPLORE";
        case RobotState::NAVIGATE: return "NAVIGATE";
        case RobotState::SCAN: return "SCAN";
        case RobotState::AVOID: return "AVOID";
        case RobotState::STUCK: return "STUCK";
        case RobotState::DONE: return "DONE";
        default: return "UNKNOWN";
    }
}

std::string ExplorerController::formatPose() const {
    std::ostringstream oss;
    oss << "(" << std::fixed << std::setprecision(2)
        << robot_pose_.position.x << ", "
        << robot_pose_.position.y << ", yaw="
        << robot_yaw_ * 180.0 / M_PI << "deg)";
    return oss.str();
}

void ExplorerController::setState(RobotState new_state, const std::string& reason) {
    if (state_ == new_state) {
        return;
    }

    ROS_INFO("[STATE] %s -> %s | %s",
             stateName(state_).c_str(),
             stateName(new_state).c_str(),
             reason.c_str());
    state_ = new_state;
    state_enter_time_ = ros::Time::now();
    front_block_since_ = ros::Time(0);
}

void ExplorerController::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    static_map_ = *msg;
    map_received_ = true;
    last_map_msg_time_ = ros::Time::now();

    const int total = msg->info.width * msg->info.height;
    int free_count = 0;
    int occupied_count = 0;
    int unknown_count = 0;
    for (int8_t cell : msg->data) {
        if (cell == FREE_CELL) {
            ++free_count;
        } else if (cell >= 50) {
            ++occupied_count;
        } else {
            ++unknown_count;
        }
    }

    coverage_grid_.assign(total, 0);
    dynamic_layer_.assign(total, FREE_CELL);
    dynamic_timestamps_.assign(total, ros::Time(0));
    coverage_cells_total_ = free_count;

    route_computed_ = false;
    main_route_.clear();
    waypoints_.clear();
    route_index_ = 0;
    total_circuits_ = 0;
    last_selected_wp_ = -1;
    has_active_goal_ = false;
    global_path_.clear();

    ROS_INFO("[DATA] /map received: %dx%d res=%.3f free=%d occupied=%d unknown=%d",
             msg->info.width, msg->info.height, msg->info.resolution,
             free_count, occupied_count, unknown_count);
}

void ExplorerController::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    const ros::Time now = ros::Time::now();
    const double new_yaw = tf2::getYaw(msg->pose.pose.orientation);

    if (pose_received_) {
        const double jump = pointDistance(robot_pose_.position.x, robot_pose_.position.y,
                                          msg->pose.pose.position.x, msg->pose.pose.position.y);
        total_dist_ += jump;
        if (jump > 0.60) {
            ROS_WARN("[POSE] Large AMCL jump detected: %.2fm -> (%.2f, %.2f, yaw=%.2fdeg)",
                     jump,
                     msg->pose.pose.position.x,
                     msg->pose.pose.position.y,
                     new_yaw * 180.0 / M_PI);
        }
    } else {
        ROS_INFO("[POSE] First AMCL pose: (%.2f, %.2f, %.1fdeg)",
                 msg->pose.pose.position.x,
                 msg->pose.pose.position.y,
                 new_yaw * 180.0 / M_PI);
    }

    robot_pose_ = msg->pose.pose;
    robot_yaw_ = new_yaw;
    pose_received_ = true;
    last_pose_msg_time_ = now;
    ++pose_msg_ct_;

    static ros::Time last_record_time(0);
    if ((now - last_record_time).toSec() >= 0.5) {
        recordPos(robot_pose_.position.x, robot_pose_.position.y);
        last_record_time = now;
    }

    ROS_INFO_THROTTLE(2.0, "[POSE] pose=%s msgs=%d",
                      formatPose().c_str(), pose_msg_ct_);
}

bool ExplorerController::refreshPoseFromTf(const std::string& reason) {
    try {
        const geometry_msgs::TransformStamped transform =
            tf_buffer_.lookupTransform("map", "base_link",
                                       ros::Time(0), ros::Duration(0.05));
        geometry_msgs::Pose pose;
        pose.position.x = transform.transform.translation.x;
        pose.position.y = transform.transform.translation.y;
        pose.position.z = transform.transform.translation.z;
        pose.orientation = transform.transform.rotation;
        const double new_yaw = tf2::getYaw(pose.orientation);

        const ros::Time now = ros::Time::now();
        if (!pose_received_) {
            ROS_INFO("[POSE] First TF pose: (%.2f, %.2f, %.1fdeg)",
                     pose.position.x, pose.position.y, new_yaw * 180.0 / M_PI);
        } else {
            const double jump = pointDistance(robot_pose_.position.x, robot_pose_.position.y,
                                              pose.position.x, pose.position.y);
            if (jump > 0.60) {
                ROS_WARN("[POSE] Large TF pose jump detected: %.2fm -> (%.2f, %.2f, yaw=%.2fdeg)",
                         jump, pose.position.x, pose.position.y, new_yaw * 180.0 / M_PI);
            }
        }

        robot_pose_ = pose;
        robot_yaw_ = new_yaw;
        pose_received_ = true;
        last_pose_msg_time_ = now;

        static ros::Time last_record_time(0);
        if ((now - last_record_time).toSec() >= 0.5) {
            recordPos(robot_pose_.position.x, robot_pose_.position.y);
            last_record_time = now;
        }

        ROS_INFO_THROTTLE(2.0, "[POSE-TF] refreshed from map->base_link for %s: %s",
                          reason.c_str(), formatPose().c_str());
        return true;
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(2.0, "[POSE-TF] map->base_link unavailable for %s: %s",
                          reason.c_str(), ex.what());
        return false;
    }
}

bool ExplorerController::hasFreshLocalMap() const {
    return local_map_received_ &&
           last_lmap_msg_time_.toSec() > 0.0 &&
           (ros::Time::now() - last_lmap_msg_time_).toSec() <= LOCAL_MAP_STALE_TIMEOUT;
}

bool ExplorerController::hasFreshScan() const {
    if (scan_msg_ct_ <= 0 ||
        last_scan_msg_time_.toSec() <= 0.0 ||
        last_scan_stamp_.toSec() <= 0.0) {
        return false;
    }

    const ros::Time now = ros::Time::now();
    const double rx_age = (now - last_scan_msg_time_).toSec();
    const double stamp_age = (now - last_scan_stamp_).toSec();
    return rx_age <= startup_scan_max_age_ &&
           stamp_age <= startup_scan_max_age_ &&
           stamp_age >= -0.20;
}

bool ExplorerController::updateStartupGate() {
    if (!startup_gate_enabled_ || startup_gate_released_) {
        return true;
    }

    const ros::Time now = ros::Time::now();
    std::vector<std::string> missing;
    bool pose_tf_ok = false;

    if (!map_received_) {
        missing.push_back("map");
    } else {
        pose_tf_ok = refreshPoseFromTf("startup gate");
        if (!pose_tf_ok) {
            missing.push_back("map->base_link TF pose");
        } else if (pose_msg_ct_ < startup_min_pose_messages_) {
            ROS_INFO_THROTTLE(5.0,
                              "[STARTUP-GATE] AMCL messages=%d/%d, but current TF pose is valid; startup is not blocked",
                              pose_msg_ct_, startup_min_pose_messages_);
        }
    }

    if (!pose_received_) {
        missing.push_back("pose");
    }

    if (!hasFreshScan()) {
        std::ostringstream reason;
        reason << "scan ";
        if (scan_msg_ct_ <= 0 || last_scan_msg_time_.toSec() <= 0.0) {
            reason << "none";
        } else {
            const double rx_age = (now - last_scan_msg_time_).toSec();
            const double stamp_age = last_scan_stamp_.toSec() > 0.0
                                         ? (now - last_scan_stamp_).toSec()
                                         : -1.0;
            reason << "rx_age=" << std::fixed << std::setprecision(1) << rx_age << "s";
            if (stamp_age < 0.0) {
                reason << " stamp_age=none";
            } else {
                reason << " stamp_age=" << std::fixed << std::setprecision(1) << stamp_age << "s";
            }
        }
        missing.push_back(reason.str());
    }

    if (!phase1_static_global_only_ && !hasFreshLocalMap()) {
        const double local_age = last_lmap_msg_time_.toSec() > 0.0
                                     ? (now - last_lmap_msg_time_).toSec()
                                     : -1.0;
        std::ostringstream reason;
        reason << "local_map_age ";
        if (local_age < 0.0) {
            reason << "none";
        } else {
            reason << std::fixed << std::setprecision(1) << local_age << "s";
        }
        missing.push_back(reason.str());
    } else if (!phase1_static_global_only_) {
        const double local_age = (now - last_lmap_msg_time_).toSec();
        if (local_age > startup_local_map_max_age_) {
            std::ostringstream reason;
            reason << "local_map_age " << std::fixed << std::setprecision(1) << local_age << "s";
            missing.push_back(reason.str());
        }
    }

    if (startup_require_clear_warning_ && latest_obstacle_warning_) {
        missing.push_back("obstacle_warning");
    }

    if (!missing.empty() || !pose_tf_ok) {
        startup_gate_ready_since_ = ros::Time(0);
        std::ostringstream joined;
        for (size_t i = 0; i < missing.size(); ++i) {
            if (i > 0) {
                joined << ", ";
            }
            joined << missing[i];
        }
        const double waited = (now - controller_start_time_).toSec();
        ROS_WARN_THROTTLE(1.0,
                          "[STARTUP-GATE] Holding robot still for %.1fs, waiting: %s",
                          waited,
                          joined.str().c_str());
        if (waited > 8.0 && !hasFreshScan()) {
            if (phase1_static_global_only_) {
                ROS_WARN_THROTTLE(3.0,
                                  "[STARTUP-GATE] /scan is not fresh. Phase1 still needs fresh lidar for emergency braking; check start_lidar.sh, /velodyne_points, cloud_to_scan, and /use_sim_time.");
            } else {
                ROS_WARN_THROTTLE(3.0,
                                  "[STARTUP-GATE] /scan is not fresh, so /local_dynamic_map cannot become reliable. Check start_lidar.sh, /velodyne_points, cloud_to_scan, and /use_sim_time.");
            }
        } else if (!phase1_static_global_only_ &&
                   waited > 8.0 && hasFreshScan() && !hasFreshLocalMap()) {
            ROS_WARN_THROTTLE(3.0,
                              "[STARTUP-GATE] /scan is fresh but /local_dynamic_map is missing; check obstacle_detector TF/localization logs.");
        }
        eStop();
        return false;
    }

    if (startup_gate_ready_since_.toSec() == 0.0) {
        startup_gate_ready_since_ = now;
        ROS_INFO("[STARTUP-GATE] Inputs are ready, checking continuous stability for %.1fs",
                 startup_stable_duration_);
        eStop();
        return false;
    }

    const double stable_time = (now - startup_gate_ready_since_).toSec();
    if (stable_time < startup_stable_duration_) {
        ROS_INFO_THROTTLE(1.0,
                          "[STARTUP-GATE] Stable for %.1f/%.1fs, robot remains stopped",
                          stable_time,
                          startup_stable_duration_);
        eStop();
        return false;
    }

    startup_gate_released_ = true;
    ROS_INFO("[STARTUP-GATE] Released after %.1fs from controller start; navigation may begin",
             (now - controller_start_time_).toSec());
    return true;
}

void ExplorerController::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ++scan_msg_ct_;
    last_scan_msg_time_ = ros::Time::now();
    last_scan_stamp_ = msg->header.stamp;

    const double stamp_age = msg->header.stamp.toSec() > 0.0
                                 ? (last_scan_msg_time_ - msg->header.stamp).toSec()
                                 : -1.0;
    if (stamp_age > startup_scan_max_age_ || stamp_age < -0.20) {
        ROS_WARN_THROTTLE(1.0,
                          "[SCAN-IN] /scan received but timestamp is not fresh: stamp_age=%.2fs msgs=%d",
                          stamp_age, scan_msg_ct_);
    } else {
        ROS_INFO_THROTTLE(5.0,
                          "[SCAN-IN] /scan fresh: stamp_age=%.2fs msgs=%d ranges=%zu",
                          stamp_age, scan_msg_ct_, msg->ranges.size());
    }

    // Extract minimum range in the robot-front arc directly from LiDAR for emergency braking.
    // On this Scout Mini setup the LiDAR 0deg points to the robot's right side, so the
    // robot-front direction is configurable and defaults to +90deg in the laser frame.
    double min_range = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
        const double front_error = std::fabs(angles::normalize_angle(angle - laser_front_angle_));
        if (front_error > 2.0 * M_PI / 9.0) { continue; }  // 40 degrees
        const double r = static_cast<double>(msg->ranges[i]);
        if (r >= static_cast<double>(msg->range_min) &&
            r <= static_cast<double>(msg->range_max) &&
            std::isfinite(r)) {
            min_range = std::min(min_range, r);
        }
    }
    min_front_scan_range_ = min_range;
}

void ExplorerController::warningCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (phase1_static_global_only_) {
        ROS_INFO_THROTTLE(5.0, "[PHASE1] Ignoring /obstacle_warning in static global mode");
        return;
    }
    latest_obstacle_warning_ = msg->data;
    const bool previous = dynamic_obstacle_active_;
    dynamic_obstacle_active_ = msg->data;
    if (!previous && dynamic_obstacle_active_) {
        last_dynamic_alert_time_ = ros::Time::now();
        ROS_WARN("[SAFETY] /obstacle_warning raised");
    } else if (previous && !dynamic_obstacle_active_) {
        soft_avoid_active_ = false;
        ROS_INFO("[SAFETY] /obstacle_warning cleared");
    }
}

void ExplorerController::dynamicPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (phase1_static_global_only_) {
        ROS_INFO_THROTTLE(5.0, "[PHASE1] Ignoring /dynamic_obstacles in static global mode");
        return;
    }
    if (!map_received_) {
        return;
    }

    sensor_msgs::PointCloud2 map_cloud;
    if (!xformCloud(*msg, map_cloud)) {
        ROS_WARN_THROTTLE(2.0, "[DYN] Failed to transform dynamic obstacle cloud to map frame");
        return;
    }
    updateDynamicLayer(map_cloud);
}

void ExplorerController::localMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    if (phase1_static_global_only_) {
        ROS_INFO_THROTTLE(5.0, "[PHASE1] Ignoring /local_dynamic_map in static global mode");
        return;
    }
    local_dynamic_map_ = *msg;
    local_map_received_ = true;
    last_lmap_msg_time_ = ros::Time::now();

    if (local_dynamic_map_.header.frame_id != "map") {
        ROS_WARN_THROTTLE(3.0, "[LOCAL MAP] Expected map frame but got '%s'",
                          local_dynamic_map_.header.frame_id.c_str());
    } else {
        ROS_INFO_THROTTLE(5.0, "[LOCAL MAP] frame=map size=%dx%d res=%.3f",
                          msg->info.width, msg->info.height, msg->info.resolution);
    }
}

void ExplorerController::objectGoalCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    if (!enable_object_goal_) {
        ROS_INFO_THROTTLE(5.0, "[OBJECT] /object_goal disabled; set enable_object_goal:=true to activate");
        return;
    }
    // Do not interrupt escape or terminal states
    if (state_ == RobotState::STUCK || state_ == RobotState::DONE) {
        ROS_INFO_THROTTLE(2.0, "[OBJECT] Ignored /object_goal in state %s", stateName(state_).c_str());
        return;
    }
    // Debounce: ignore if same area received within 3s
    const ros::Time now = ros::Time::now();
    const double dist_from_last = pointDistance(msg->point.x, msg->point.y,
                                                last_object_goal_pos_.x, last_object_goal_pos_.y);
    if (last_object_goal_time_.toSec() > 0.0 &&
        (now - last_object_goal_time_).toSec() < 3.0 &&
        dist_from_last < 0.30) {
        ROS_INFO_THROTTLE(2.0, "[OBJECT] Debounced /object_goal (%.2fm, %.1fs ago)",
                          dist_from_last, (now - last_object_goal_time_).toSec());
        return;
    }
    last_object_goal_time_ = now;
    last_object_goal_pos_ = msg->point;

    if (!map_received_ || !pose_received_) {
        if (!map_received_ || !refreshPoseFromTf("object goal")) {
            ROS_WARN_THROTTLE(2.0, "[OBJECT] Ignored object goal because map/pose not ready");
            return;
        }
    }

    object_goal_ = *msg;
    object_goal_active_ = true;

    int object_x = 0;
    int object_y = 0;
    if (!worldToMap(msg->point.x, msg->point.y, object_x, object_y)) {
        ROS_WARN("[OBJECT] Goal outside static map: (%.2f, %.2f)",
                 msg->point.x, msg->point.y);
        object_goal_active_ = false;
        return;
    }

    std::vector<geometry_msgs::Point> path;
    GridCell approach_cell{0, 0};
    if (!planToObjectGoal(msg->point.x, msg->point.y, approach_cell, path)) {
        ++path_fail_;
        ROS_WARN("[OBJECT] Failed to plan to object approach near map=(%d,%d)",
                 object_x, object_y);
        object_goal_active_ = false;
        return;
    }

    ++path_ok_;
    global_path_ = path;
    current_goal_cell_ = approach_cell;
    has_active_goal_ = true;
    navigating_to_object_ = true;
    last_goal_progress_time_ = ros::Time::now();
    current_goal_start_distance_ = pointDistance(robot_pose_.position.x, robot_pose_.position.y,
                                                 path.back().x, path.back().y);
    visPath(path);
    visGoal(current_goal_cell_);
    setState(RobotState::NAVIGATE, "object goal received");

    ROS_INFO("[OBJECT] Accepted object map=(%d,%d) world=(%.2f,%.2f), approach=(%d,%d)",
             object_x, object_y, msg->point.x, msg->point.y,
             approach_cell.x, approach_cell.y);
}

void ExplorerController::controlLoop(const ros::TimerEvent&) {
    if (map_received_ && !pose_received_) {
        refreshPoseFromTf("startup");
    }

    if (!map_received_ || !pose_received_) {
        ROS_INFO_THROTTLE(3.0, "[STATE] Waiting for /map and map->base_link pose");
        eStop();
        return;
    }

    if (!updateStartupGate()) {
        return;
    }

    if ((ros::Time::now() - last_pose_msg_time_).toSec() > POSE_STALE_TIMEOUT) {
        const double stale_age = (ros::Time::now() - last_pose_msg_time_).toSec();
        if (!refreshPoseFromTf("stale AMCL pose")) {
            ROS_WARN_THROTTLE(2.0, "[POSE] AMCL pose is stale for %.1fs and TF fallback failed",
                              stale_age);
            eStop();
            return;
        }
    }

    const bool driving_state = state_ == RobotState::ALIGN ||
                               state_ == RobotState::EXPLORE ||
                               state_ == RobotState::NAVIGATE;
    if (driving_state && !hasFreshScan()) {
        ROS_WARN_THROTTLE(1.0,
                          "[SAFETY] /scan is stale during driving; stopping until lidar data is fresh");
        eStop();
        return;
    }
    if (require_local_map_before_navigation_ &&
        driving_state &&
        !navigating_to_object_ &&
        !hasFreshLocalMap()) {
        ROS_WARN_THROTTLE(2.0,
                          "[LOCAL MAP] /local_dynamic_map is not fresh; continuing slowly on static map until it arrives");
    }

    static int visual_counter = 0;
    if (++visual_counter >= 20) {
        visual_counter = 0;
        pubCovMarkers();
        pubRouteMarkers();
    }

    switch (state_) {
        case RobotState::INIT:
            if (route_computed_ && !main_route_.empty()) {
                setState(RobotState::ALIGN, "main route ready");
            }
            return;
        case RobotState::ALIGN:
            handleAlign();
            return;
        case RobotState::SCAN:
            handleScan();
            return;
        case RobotState::AVOID:
            handleAvoid();
            return;
        case RobotState::STUCK:
            handleStuck();
            return;
        case RobotState::DONE:
            eStop();
            return;
        case RobotState::EXPLORE:
        case RobotState::NAVIGATE:
            break;
    }

    const ros::Time now = ros::Time::now();
    if (!phase1_static_global_only_ &&
        latest_obstacle_warning_ && state_ != RobotState::AVOID) {
        if (state_ == RobotState::NAVIGATE && has_active_goal_ && !global_path_.empty()) {
            if (!soft_avoid_active_) {
                soft_avoid_active_ = true;
                last_soft_avoid_time_ = now;
                ROS_WARN("[SAFETY] Detector warning during navigation, slowing and replanning before hard avoid");
            }
        } else {
            setState(RobotState::AVOID, "detector warning active without navigable path");
            eStop();
            return;
        }
    } else if (!latest_obstacle_warning_) {
        soft_avoid_active_ = false;
    }

    if (isStuck()) {
        setState(RobotState::STUCK, "robot motion stalled");
        return;
    }

    if (state_ == RobotState::NAVIGATE) {
        double goal_world_x = 0.0;
        double goal_world_y = 0.0;
        mapToWorld(current_goal_cell_.x, current_goal_cell_.y, goal_world_x, goal_world_y);
        const double remaining = pointDistance(robot_pose_.position.x, robot_pose_.position.y,
                                               goal_world_x, goal_world_y);
        if (remaining < goal_tol_) {
            if (navigating_to_object_) {
                const double target_yaw = std::atan2(object_goal_.point.y - robot_pose_.position.y,
                                                     object_goal_.point.x - robot_pose_.position.x);
                const double yaw_error = angles::normalize_angle(target_yaw - robot_yaw_);
                if (std::fabs(yaw_error) > ALIGN_TOLERANCE_DEG * M_PI / 180.0) {
                    publishCmdVel(0.0, clampValue(2.0 * yaw_error, -max_ang_, max_ang_));
                    ROS_INFO_THROTTLE(1.0,
                                      "[OBJECT] Holding approach point, aligning head to object err=%.1fdeg",
                                      yaw_error * 180.0 / M_PI);
                    return;
                }

                eStop();
                has_active_goal_ = false;
                global_path_.clear();
                object_goal_active_ = false;
                navigating_to_object_ = false;
                setState(RobotState::DONE, "object reached and robot head aligned");
                return;
            }

            if (route_index_ >= 0 && route_index_ < static_cast<int>(waypoints_.size())) {
                waypoints_[route_index_].visited = true;
                waypoints_[route_index_].times_reached++;
                waypoints_[route_index_].last_visit = ros::Time::now();
                ++wp_visited_;
            }
            has_active_goal_ = false;
            global_path_.clear();
            navigating_to_object_ = false;
            setState(RobotState::SCAN, "reached active route waypoint");
            return;
        }

        if (remaining + 0.10 < current_goal_start_distance_) {
            current_goal_start_distance_ = remaining;
            last_goal_progress_time_ = ros::Time::now();
        } else if (last_goal_progress_time_.toSec() > 0.0 &&
                   (now - last_goal_progress_time_).toSec() > GOAL_PROGRESS_TIMEOUT) {
            ++replan_ct_;
            replanToCurrentGoal("progress timeout");
        }

        if (!phase1_static_global_only_ && isPathBlocked()) {
            ++replan_ct_;
            replanToCurrentGoal("dynamic obstacle on path");
        }
    }

    if (global_path_.empty()) {
        eStop();
        return;
    }

    geometry_msgs::Point look_ahead;
    if (!calcLookAhead(global_path_, look_ahead)) {
        ++replan_ct_;
        replanToCurrentGoal("lookahead unavailable");
        return;
    }

    const double dx = look_ahead.x - robot_pose_.position.x;
    const double dy = look_ahead.y - robot_pose_.position.y;
    const double target_yaw = std::atan2(dy, dx);
    const double yaw_error = angles::normalize_angle(target_yaw - robot_yaw_);

    double angular = clampValue(2.5 * yaw_error, -max_ang_, max_ang_);
    double linear = max_lin_ * std::max(0.0, 1.0 - std::fabs(yaw_error) / (M_PI / 3.0));
    if (std::fabs(yaw_error) > M_PI_2) {
        linear = 0.0;
    } else if (!navigating_to_object_) {
        linear = std::max(linear, max_lin_ * 0.18);
    }

    if (!phase1_static_global_only_ &&
        local_map_received_ &&
        (now - last_lmap_msg_time_).toSec() <= LOCAL_MAP_STALE_TIMEOUT) {
        const double front_probe = std::max(0.55, ROBOT_LENGTH * 0.70);
        const double side_probe = std::max(0.22, ROBOT_WIDTH * 0.50);  // 覆盖机器人实际半宽(0.285m)，防止侧面剐蹭
        bool hard_block = false;
        bool soft_block = false;
        for (double lateral = -side_probe; lateral <= side_probe + 1e-6; lateral += side_probe) {
            const double probe_x = robot_pose_.position.x +
                                   front_probe * std::cos(robot_yaw_) -
                                   lateral * std::sin(robot_yaw_);
            const double probe_y = robot_pose_.position.y +
                                   front_probe * std::sin(robot_yaw_) +
                                   lateral * std::cos(robot_yaw_);
            // 静态地图检查：局部动态地图不含静态墙，必须单独检查
            int probe_mx = 0;
            int probe_my = 0;
            if (worldToMap(probe_x, probe_y, probe_mx, probe_my) &&
                isCellBlocked(probe_mx, probe_my)) {
                hard_block = true;
                break;
            }
            if (isLocalMapAreaBlocked(probe_x, probe_y, 0.13, 0.35, 3)) {
                hard_block = true;
                break;
            }
            if (isLocalMapAreaBlocked(probe_x, probe_y, 0.18, 0.18, 2)) {
                soft_block = true;
            }
        }
        if (hard_block) {
            if (!latest_obstacle_warning_ &&
                !navigating_to_object_ &&
                handleFrontStaticBlock(now, look_ahead.x, look_ahead.y)) {
                return;
            }
            linear = 0.0;
            if (latest_obstacle_warning_) {
                angular = clampValue(angular, -max_ang_ * 0.45, max_ang_ * 0.45);
            } else {
                const double turn_sign = yaw_error >= 0.0 ? 1.0 : -1.0;
                angular = clampValue(turn_sign * max_ang_ * 0.75,
                                     -max_ang_ * 0.75,
                                     max_ang_ * 0.75);
            }
            ROS_WARN_THROTTLE(1.0,
                              "[LOCAL MAP] Front safety zone occupied, stopping forward motion%s",
                              latest_obstacle_warning_ ? " because dynamic warning is active" : "; treating as static-route blockage");
        } else if (soft_block) {
            front_block_since_ = ros::Time(0);
            linear = std::min(linear, max_lin_ * 0.35);
            ROS_WARN_THROTTLE(1.5, "[LOCAL MAP] Front area partially occupied, slowing down");
        }
    } else if (require_local_map_before_navigation_ && !navigating_to_object_) {
        front_block_since_ = ros::Time(0);
        linear = std::min(linear, max_lin_ * 0.45);
        angular = clampValue(angular, -max_ang_ * 0.80, max_ang_ * 0.80);
    } else {
        front_block_since_ = ros::Time(0);
    }

    if (soft_avoid_active_) {
        linear = std::min(linear, max_lin_ * 0.22);
        angular = clampValue(angular, -max_ang_ * 0.75, max_ang_ * 0.75);
        if ((now - last_soft_avoid_time_).toSec() >= 2.0) {
            ++replan_ct_;
            replanToCurrentGoal("soft obstacle warning");
            last_soft_avoid_time_ = now;
        }
        ROS_WARN_THROTTLE(1.0, "[SAFETY] Soft avoid active, speed limited while following/replanning path");
    }

    // Direct LiDAR emergency brake — independent of localization.
    // Hard stop distance: robot half-length (0.32m) + minimum clearance (0.15m) = 0.47m.
    // Slow zone: hard_stop + 0.25m = 0.72m.
    if (std::isfinite(min_front_scan_range_)) {
        const double estop_dist = ROBOT_LENGTH * 0.5 + 0.15;   // 0.47m — minimum 15cm clearance
        const double slow_dist  = estop_dist + 0.25;            // 0.72m — begin deceleration
        if (min_front_scan_range_ < estop_dist) {
            linear = 0.0;
            ROS_WARN_THROTTLE(0.5, "[SAFETY] LiDAR emergency stop: front=%.2fm < %.2fm",
                              min_front_scan_range_, estop_dist);
        } else if (min_front_scan_range_ < slow_dist) {
            linear = std::min(linear, max_lin_ * 0.30);
            ROS_WARN_THROTTLE(1.0, "[SAFETY] LiDAR slow: front=%.2fm", min_front_scan_range_);
        }
    }

    publishCmdVel(linear, angular);
}

void ExplorerController::explorationLoop(const ros::TimerEvent&) {
    if (!map_received_ || !pose_received_) {
        return;
    }
    if (startup_gate_enabled_ && !startup_gate_released_) {
        return;
    }

    int robot_x = 0;
    int robot_y = 0;
    if (!worldToMap(robot_pose_.position.x, robot_pose_.position.y, robot_x, robot_y)) {
        ROS_ERROR_THROTTLE(2.0, "[ROUTE] Robot is outside the static map");
        return;
    }

    if (!route_computed_) {
        planMainRoute();
        route_computed_ = true;
        if (main_route_.empty()) {
            setState(RobotState::DONE, "failed to compute main route");
            eStop();
            return;
        }
        route_index_ = findRouteIndexWithHeading(robot_x, robot_y);
        ROS_INFO("[ROUTE] Main route ready: %zu waypoints, start index=%d",
                 main_route_.size(), route_index_);
        return;
    }

    if (state_ == RobotState::INIT) {
        setState(RobotState::ALIGN, "route already computed");
        return;
    }

    if (state_ == RobotState::EXPLORE) {
        handleExplore();
    }
}

void ExplorerController::handleAlign() {
    if (main_route_.size() < 2) {
        setState(RobotState::EXPLORE, "route too short for align");
        return;
    }

    int robot_x = 0;
    int robot_y = 0;
    if (!worldToMap(robot_pose_.position.x, robot_pose_.position.y, robot_x, robot_y)) {
        return;
    }

    const int nearest = findRouteIndexWithHeading(robot_x, robot_y);
    route_index_ = nearest;
    const double target_heading = computeWaypointHeading(nearest);
    const double yaw_error = angles::normalize_angle(target_heading - robot_yaw_);

    ROS_INFO_THROTTLE(1.5, "[ALIGN] nearest=%d target=%.1fdeg robot=%.1fdeg err=%.1fdeg",
                      nearest,
                      target_heading * 180.0 / M_PI,
                      robot_yaw_ * 180.0 / M_PI,
                      yaw_error * 180.0 / M_PI);

    if (std::fabs(yaw_error) <= ALIGN_TOLERANCE_DEG * M_PI / 180.0) {
        eStop();
        setState(RobotState::EXPLORE, "heading aligned with coverage sweep tangent");
        return;
    }

    if (std::fabs(yaw_error) > 45.0 * M_PI / 180.0) {
        ROS_WARN("[ALIGN] Large heading error %.1fdeg; skip pure spin and let path follower turn while moving",
                 yaw_error * 180.0 / M_PI);
        eStop();
        setState(RobotState::EXPLORE, "large heading error, avoid in-place spin");
        return;
    }

    if ((ros::Time::now() - state_enter_time_).toSec() > 25.0) {
        ROS_WARN("[ALIGN] Timeout, entering explore with current heading");
        setState(RobotState::EXPLORE, "align timeout");
        return;
    }

    publishCmdVel(0.0, std::copysign(align_ang_, yaw_error));
}

void ExplorerController::handleExplore() {
    if (require_local_map_before_navigation_ && !hasFreshLocalMap()) {
        ROS_WARN_THROTTLE(2.0,
                          "[LOCAL MAP] Exploration is using static map only until local map is fresh");
    }

    int robot_x = 0;
    int robot_y = 0;
    if (!worldToMap(robot_pose_.position.x, robot_pose_.position.y, robot_x, robot_y)) {
        return;
    }

    if (enable_object_goal_ && object_goal_active_ && !has_active_goal_) {
        std::vector<geometry_msgs::Point> path;
        GridCell approach_cell{0, 0};
        if (planToObjectGoal(object_goal_.point.x, object_goal_.point.y,
                             approach_cell, path)) {
            ++path_ok_;
            global_path_ = path;
            current_goal_cell_ = approach_cell;
            has_active_goal_ = true;
            navigating_to_object_ = true;
            last_goal_progress_time_ = ros::Time::now();
            current_goal_start_distance_ =
                pointDistance(robot_pose_.position.x, robot_pose_.position.y,
                              path.back().x, path.back().y);
            visPath(path);
            visGoal(current_goal_cell_);
            setState(RobotState::NAVIGATE, "resume object goal after interruption");
            return;
        }
        ROS_WARN("[OBJECT] Dropping object goal because replanning failed");
        object_goal_active_ = false;
        navigating_to_object_ = false;
    }

    int next_index = findNextUnvisitedWaypoint(route_index_);
    if (next_index < 0) {
        const double global_cov = getGlobalCov();
        ++total_circuits_;
        ROS_INFO("[ROUTE] Completed one coverage sweep, coverage=%.1f%% circuits=%d",
                 global_cov * 100.0, total_circuits_);
        if (global_cov >= COVERAGE_TARGET || total_circuits_ >= 3) {
            setState(RobotState::DONE, "coverage target reached or max circuits reached");
            eStop();
            return;
        }

        for (WaypointInfo& waypoint : waypoints_) {
            waypoint.times_reached = 0;
            waypoint.visited = false;
        }
        last_selected_wp_ = -1;
        route_index_ = findRouteIndexWithHeading(robot_x, robot_y);
        next_index = findNextUnvisitedWaypoint(route_index_);
        if (next_index < 0) {
            setState(RobotState::DONE, "unable to find next waypoint after reset");
            eStop();
            return;
        }
    }

    int preferred_index = next_index;
    if (preferred_index == last_selected_wp_ && main_route_.size() > 1) {
        preferred_index = (preferred_index + 1) % static_cast<int>(main_route_.size());
        ROS_WARN("[ROUTE] Forced skip to avoid selecting the same waypoint repeatedly");
    }

    std::vector<geometry_msgs::Point> path;
    int planned_index = -1;
    if (!findReachableRouteIndex(robot_x, robot_y, preferred_index, planned_index, path)) {
        ++path_fail_;
        if (preferred_index >= 0 && preferred_index < static_cast<int>(waypoints_.size())) {
            waypoints_[preferred_index].times_reached++;
            if (waypoints_[preferred_index].times_reached >= MAX_WAYPOINT_VISITS) {
                waypoints_[preferred_index].visited = true;
                waypoints_[preferred_index].last_visit = ros::Time::now();
            }
        }
        route_index_ = (preferred_index + 3) % static_cast<int>(main_route_.size());
        last_selected_wp_ = -1;
        stuck_ctr_ = STUCK_THRESHOLD;
        ROS_WARN("[ROUTE] No reachable route waypoint from map=(%d,%d), preferred=%d, advancing to %d",
                 robot_x, robot_y, preferred_index, route_index_);
        return;
    }

    if (planned_index != preferred_index) {
        ROS_WARN("[ROUTE] Preferred waypoint %d unreachable, using nearby reachable waypoint %d",
                 preferred_index, planned_index);
    }

    const double selected_cov = getWaypointCov(main_route_[planned_index]);
    if (selected_cov >= scan_cov_target_) {
        if (planned_index >= 0 && planned_index < static_cast<int>(waypoints_.size())) {
            waypoints_[planned_index].visited = true;
            waypoints_[planned_index].times_reached++;
            waypoints_[planned_index].last_visit = ros::Time::now();
            waypoints_[planned_index].last_coverage = selected_cov;
        }
        ++wp_skipped_;
        route_index_ = (planned_index + 1) % static_cast<int>(main_route_.size());
        last_selected_wp_ = planned_index;
        ROS_INFO("[ROUTE] Skip waypoint %d because local coverage is already %.0f%%",
                 planned_index, selected_cov * 100.0);
        return;
    }

    last_selected_wp_ = planned_index;
    route_index_ = planned_index;

    GridCell target_cell = main_route_[route_index_];

    ++path_ok_;
    global_path_ = path;
    current_goal_cell_ = target_cell;
    has_active_goal_ = true;
    navigating_to_object_ = false;
    object_goal_active_ = false;

    double goal_world_x = 0.0;
    double goal_world_y = 0.0;
    mapToWorld(target_cell.x, target_cell.y, goal_world_x, goal_world_y);
    current_goal_start_distance_ = pointDistance(robot_pose_.position.x, robot_pose_.position.y,
                                                 goal_world_x, goal_world_y);
    last_goal_progress_time_ = ros::Time::now();
    visPath(path);
    visGoal(target_cell);
    setState(RobotState::NAVIGATE, "route waypoint selected");

    ROS_INFO("[PLAN] route_index=%d target=(%d,%d) world=(%.2f,%.2f) coverage=%.0f%%",
             route_index_, target_cell.x, target_cell.y, goal_world_x, goal_world_y,
             waypoints_[route_index_].last_coverage * 100.0);
}

void ExplorerController::handleScan() {
    const double elapsed = (ros::Time::now() - state_enter_time_).toSec();
    const double local_cov = getRobotLocalCov();

    bool finished = false;
    if (elapsed >= MIN_SCAN_TIME && local_cov >= scan_cov_target_) {
        ROS_INFO("[SCAN] Coverage reached %.0f%% in %.1fs", local_cov * 100.0, elapsed);
        finished = true;
    }
    if (elapsed >= MAX_SCAN_TIME) {
        ROS_WARN("[SCAN] Timeout %.1fs, local coverage %.0f%%", elapsed, local_cov * 100.0);
        finished = true;
    }

    if (finished) {
        markCoverageDisk(robot_pose_.position.x, robot_pose_.position.y, cov_range_);
        eStop();
        setState(RobotState::EXPLORE, "scan complete");
        return;
    }

    const double scan_direction = (route_index_ % 2 == 0) ? 1.0 : -1.0;
    publishCmdVel(0.0, scan_direction * std::min(align_ang_, max_ang_ * 0.55));
    ROS_INFO_THROTTLE(2.0, "[SCAN] elapsed=%.1fs local_coverage=%.0f%%",
                      elapsed, local_cov * 100.0);
}

void ExplorerController::handleAvoid() {
    const double elapsed = (ros::Time::now() - state_enter_time_).toSec();
    if (!dynamic_obstacle_active_) {
        has_active_goal_ = false;
        global_path_.clear();
        setState(RobotState::EXPLORE, "dynamic obstacle cleared");
        return;
    }

    if (elapsed < 3.0) {
        eStop();
    } else {
        has_active_goal_ = false;
        global_path_.clear();
        setState(RobotState::EXPLORE, "avoid wait timeout, retry edge route planning");
    }
}

void ExplorerController::handleStuck() {
    const double elapsed = (ros::Time::now() - state_enter_time_).toSec();

    // ── Phase 0 (0 – 1.5s): stop completely ──────────────────────────────────
    // The robot has been physically stopped. Allow sensors to settle and let
    // the scan accumulate a fresh picture of the surroundings.
    if (elapsed < 1.5) {
        eStop();
        ROS_INFO_THROTTLE(0.5, "[ESCAPE] Phase 0: stopped, waiting for sensors (%.1fs/1.5s)", elapsed);
        return;
    }

    // ── Phase 1 (1.5 – 4.0s): rotate to find a clear heading ────────────────
    // We rotate at 80% max angular speed.  If the front scan clears (>0.72m)
    // before the phase timeout we exit early and proceed to phase 2 immediately
    // by pretending we are already in phase 2.
    if (elapsed < 4.0) {
        const double estop_dist = ROBOT_LENGTH * 0.5 + 0.15;   // 0.47m
        const double clear_dist = estop_dist + 0.25;            // 0.72m
        const bool front_clear = !std::isfinite(min_front_scan_range_) ||
                                  min_front_scan_range_ >= clear_dist;
        if (front_clear && elapsed > 2.0) {
            // Front is clear — skip straight to phase 2 by setting state_enter_time_
            // to make elapsed jump beyond 4.0 on the very next call.
            // Achieve this by just driving forward now (treat as phase 2 start).
            publishCmdVel(max_lin_ * 0.45, 0.0);
            ROS_INFO_THROTTLE(0.5,
                              "[ESCAPE] Phase 1→2 early: front=%.2fm, moving forward",
                              min_front_scan_range_);
            return;
        }
        publishCmdVel(0.0, max_ang_ * 0.80);
        ROS_INFO_THROTTLE(0.5, "[ESCAPE] Phase 1: rotating to find clear heading (%.1fs/4.0s)", elapsed);
        return;
    }

    // ── Phase 2 (4.0 – 8.0s): drive forward in the clear direction ──────────
    if (elapsed < 8.0) {
        const double estop_dist = ROBOT_LENGTH * 0.5 + 0.15;   // 0.47m — hard stop
        if (std::isfinite(min_front_scan_range_) && min_front_scan_range_ < estop_dist) {
            // Hit another wall — rotate a bit more, then try forward again.
            publishCmdVel(0.0, max_ang_ * 0.80);
            ROS_WARN_THROTTLE(0.5,
                              "[ESCAPE] Phase 2: front blocked (%.2fm), rotating",
                              min_front_scan_range_);
        } else {
            publishCmdVel(max_lin_ * 0.50, 0.0);
            ROS_INFO_THROTTLE(0.5, "[ESCAPE] Phase 2: driving forward (%.1fs/8.0s)", elapsed);
        }
        return;
    }

    // ── Escape complete ───────────────────────────────────────────────────────
    // Advance route_index by 6 so we target a fresh area of the route,
    // reset stuck counter, and return to EXPLORE.
    stuck_ctr_ = 0;
    has_active_goal_ = false;
    global_path_.clear();
    front_block_since_ = ros::Time(0);
    if (!main_route_.empty()) {
        int best = -1;
        double best_dist = std::numeric_limits<double>::max();
        for (int i = 0; i < static_cast<int>(main_route_.size()); ++i) {
            if (waypoints_[i].visited || waypoints_[i].times_reached >= MAX_WAYPOINT_VISITS) {
                continue;
            }
            double wx = 0.0;
            double wy = 0.0;
            mapToWorld(main_route_[i].x, main_route_[i].y, wx, wy);
            const double d = pointDistance(robot_pose_.position.x, robot_pose_.position.y, wx, wy);
            if (d < best_dist) {
                best_dist = d;
                best = i;
            }
        }
        // If nearest unvisited equals the waypoint that just caused STUCK,
        // it is physically untraversable. Force-skip it and re-select.
        if (best >= 0 && best == route_index_ &&
            best < static_cast<int>(waypoints_.size())) {
            waypoints_[best].times_reached = MAX_WAYPOINT_VISITS;
            waypoints_[best].visited = true;
            ROS_WARN("[ESCAPE] wp=%d reselected after STUCK, force-skipping (physically blocked)", best);
            best = -1;
            double best_dist2 = std::numeric_limits<double>::max();
            for (int i = 0; i < static_cast<int>(main_route_.size()); ++i) {
                if (waypoints_[i].visited || waypoints_[i].times_reached >= MAX_WAYPOINT_VISITS) continue;
                double wx2 = 0.0, wy2 = 0.0;
                mapToWorld(main_route_[i].x, main_route_[i].y, wx2, wy2);
                const double d2 = pointDistance(robot_pose_.position.x, robot_pose_.position.y, wx2, wy2);
                if (d2 < best_dist2) { best_dist2 = d2; best = i; }
            }
        }
        route_index_ = (best >= 0) ? best : (route_index_ + 6) % static_cast<int>(main_route_.size());
        last_selected_wp_ = -1;
    }
    ROS_INFO("[ESCAPE] 3-phase escape complete after %.1fs; nearest unvisited wp=%d, resuming exploration",
             elapsed, route_index_);
    setState(RobotState::EXPLORE, "3-phase escape complete");
}

void ExplorerController::planMainRoute() {
    main_route_.clear();
    dominant_safe_mask_.clear();
    edge_distance_cells_.clear();

    CoveragePlannerConfig config;
    config.lane_spacing_cells = std::max(1, static_cast<int>(std::ceil(
        wp_spacing_ / static_map_.info.resolution)));
    config.sample_spacing_cells = std::max(1, config.lane_spacing_cells / 2);
    config.min_lane_span_cells = std::max(2, config.lane_spacing_cells);

    CoveragePlannerGridView grid;
    grid.width = static_cast<int>(static_map_.info.width);
    grid.height = static_cast<int>(static_map_.info.height);
    grid.occupancy.assign(static_map_.data.begin(), static_map_.data.end());

    const std::vector<uint8_t> safe_mask =
        buildStaticSafetyMask(route_safety_radius_, min_static_obstacle_cluster_cells_);
    EdgeRouteGrid edge_grid;
    edge_grid.width = grid.width;
    edge_grid.height = grid.height;
    edge_grid.resolution = static_map_.info.resolution;
    edge_grid.occupancy = grid.occupancy;

    EdgeRouteCell preferred_seed{0, 0};
    bool have_preferred_seed = false;
    if (pose_received_) {
        int start_x = 0;
        int start_y = 0;
        if (worldToMap(robot_pose_.position.x, robot_pose_.position.y, start_x, start_y)) {
            preferred_seed = EdgeRouteCell{start_x, start_y};
            have_preferred_seed = true;
        }
    }
    int selected_safe_count = 0;
    int largest_safe_count = 0;
    dominant_safe_mask_ = extractDominantSafetyComponent(
        edge_grid,
        safe_mask,
        preferred_seed,
        std::max(200, static_cast<int>(safe_mask.size() / 100)),
        selected_safe_count,
        largest_safe_count);
    if (selected_safe_count > 0) {
        ROS_INFO("[ROUTE] Dominant safe component selected: cells=%d largest=%d seed=%s",
                 selected_safe_count,
                 largest_safe_count,
                 have_preferred_seed ? "robot_pose" : "largest_only");
    } else {
        dominant_safe_mask_ = safe_mask;
    }
    const std::vector<uint8_t>& route_safe_mask =
        selected_safe_count > 0 ? dominant_safe_mask_ : safe_mask;
    const std::vector<uint8_t> raw_route_mask =
        expandSafeMaskToRawFreeComponent(edge_grid, route_safe_mask);
    edge_distance_cells_ = computeEdgeDistanceCells(edge_grid, raw_route_mask);

    CoveragePlannerGridView safe_grid = grid;
    if (route_safe_mask.size() == static_cast<size_t>(grid.width * grid.height)) {
        for (int y = 0; y < grid.height; ++y) {
            for (int x = 0; x < grid.width; ++x) {
                const int index = y * grid.width + x;
                safe_grid.occupancy[index] = route_safe_mask[static_cast<size_t>(index)] != 0
                    ? FREE_CELL
                    : OCCUPIED_CELL;
            }
        }
    }

    CoveragePlannerComponent component;
    if (pose_received_) {
        int start_x = 0;
        int start_y = 0;
        if (worldToMap(robot_pose_.position.x, robot_pose_.position.y, start_x, start_y)) {
            const CoveragePlannerCell seed =
                nearestFreeCell(safe_grid, CoveragePlannerCell{start_x, start_y});
            component = buildComponentFromMask(safe_grid, collectComponentMask(safe_grid, seed));
        }
    }
    if (component.empty()) {
        component = collectLargestFreeComponent(safe_grid);
    }
    if (component.empty()) {
        ROS_WARN("[ROUTE] No traversable free-space component found in map");
        return;
    }

    const CoveragePlannerComponent largest_component = collectLargestFreeComponent(safe_grid);
    if (!largest_component.empty() &&
        component.free_count < std::max(200, largest_component.free_count / 3)) {
        ROS_WARN("[ROUTE] Start safe component is too small (%d cells), using largest safe component (%d cells)",
                 component.free_count, largest_component.free_count);
        component = largest_component;
    }

    if (use_edge_route_) {
        EdgeRouteConfig edge_config;
        edge_config.desired_center_offset_m = edge_center_offset_;
        edge_config.offset_tolerance_m = edge_offset_tolerance_;
        edge_config.waypoint_spacing_m = std::max(0.40, std::min(wp_spacing_, 0.80));
        edge_config.object_standoff_m = object_standoff_;
        edge_config.min_route_points = 8;

        const std::vector<EdgeRouteCell> edge_route =
            buildEdgeFollowingRoute(edge_grid, route_safe_mask, edge_config);
        main_route_.reserve(edge_route.size());
        for (const EdgeRouteCell& cell : edge_route) {
            main_route_.push_back(GridCell{cell.x, cell.y});
        }

        if (!main_route_.empty()) {
            int in_band = 0;
            int too_close = 0;
            int too_far = 0;
            for (const EdgeRouteCell& cell : edge_route) {
                const size_t index = static_cast<size_t>(edgeFlatIndex(cell.x, cell.y, edge_grid.width));
                if (index >= edge_distance_cells_.size() || edge_distance_cells_[index] < 0) {
                    continue;
                }
                const double distance_m =
                    static_cast<double>(edge_distance_cells_[index]) * edge_grid.resolution;
                if (distance_m < edge_min_center_distance_) {
                    ++too_close;
                } else if (distance_m > edge_max_center_distance_) {
                    ++too_far;
                } else {
                    ++in_band;
                }
            }
            ROS_INFO("[ROUTE] Edge-follow route generated: %zu waypoints, center_offset=%.2fm safe_band=%.2f-%.2fm",
                     main_route_.size(), edge_center_offset_,
                     edge_min_center_distance_, edge_max_center_distance_);
            ROS_INFO("[ROUTE] Edge distance band: in=%d too_close=%d too_far=%d dominant_safe=%d largest_safe=%d",
                     in_band, too_close, too_far, selected_safe_count, largest_safe_count);
        }
    }

    if (main_route_.empty()) {
        const bool horizontal =
            (component.max_x - component.min_x) >= (component.max_y - component.min_y);
        const std::vector<CoveragePlannerCell> route =
            buildSweepRoute(safe_grid, component.mask, config, horizontal);
        main_route_.reserve(route.size());
        for (const CoveragePlannerCell& cell : route) {
            main_route_.push_back(GridCell{cell.x, cell.y});
        }
        ROS_WARN("[ROUTE] Falling back to area sweep route with %zu waypoints",
                 main_route_.size());
    }

    if (main_route_.empty()) {
        main_route_.clear();
        ROS_WARN("[ROUTE] Sweep planner did not generate usable waypoints");
        return;
    }

    rebuildWaypointCache();
    ROS_INFO("[ROUTE] Coverage route generated with %zu waypoints from %d-cell safe component",
             main_route_.size(), component.free_count);
}

void ExplorerController::rebuildWaypointCache() {
    waypoints_.clear();
    waypoints_.reserve(main_route_.size());
    for (size_t i = 0; i < main_route_.size(); ++i) {
        WaypointInfo info;
        info.cell = main_route_[i];
        info.index = static_cast<int>(i);
        waypoints_.push_back(info);
    }
}

int ExplorerController::findNearestRouteIndex(int rx, int ry) const {
    if (main_route_.empty()) {
        return -1;
    }

    int best_index = 0;
    double best_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < main_route_.size(); ++i) {
        const double dist = pointDistance(static_cast<double>(main_route_[i].x),
                                          static_cast<double>(main_route_[i].y),
                                          static_cast<double>(rx),
                                          static_cast<double>(ry));
        if (dist < best_dist) {
            best_dist = dist;
            best_index = static_cast<int>(i);
        }
    }
    return best_index;
}

int ExplorerController::findRouteIndexWithHeading(int rx, int ry) const {
    if (main_route_.empty()) {
        return -1;
    }

    const int nearest = findNearestRouteIndex(rx, ry);
    if (nearest < 0 || main_route_.size() < 2) {
        return nearest;
    }

    const double resolution = std::max(0.01f, static_map_.info.resolution);
    const double near_window_m = 1.50;
    const double near_window_cells = near_window_m / resolution;
    const double max_considered_cells = near_window_cells * near_window_cells;

    int best_index = nearest;
    double best_score = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < main_route_.size(); ++i) {
        const GridCell& cell = main_route_[i];
        const double dx = static_cast<double>(cell.x - rx);
        const double dy = static_cast<double>(cell.y - ry);
        const double dist_sq = dx * dx + dy * dy;
        if (dist_sq > max_considered_cells) {
            continue;
        }

        const double heading = computeWaypointHeading(static_cast<int>(i));
        const double heading_error = std::fabs(angles::normalize_angle(heading - robot_yaw_));
        double entry_x = 0.0;
        double entry_y = 0.0;
        mapToWorld(cell.x, cell.y, entry_x, entry_y);
        if (std::hypot(entry_x - robot_pose_.position.x,
                       entry_y - robot_pose_.position.y) < 0.20) {
            const GridCell& next_cell = main_route_[(i + 1) % main_route_.size()];
            mapToWorld(next_cell.x, next_cell.y, entry_x, entry_y);
        }
        const double entry_heading = std::atan2(entry_y - robot_pose_.position.y,
                                                entry_x - robot_pose_.position.x);
        const double entry_error = std::fabs(angles::normalize_angle(entry_heading - robot_yaw_));
        const double score = std::sqrt(dist_sq) * resolution +
                             heading_error * 0.45 +
                             entry_error * 0.70;
        if (score < best_score) {
            best_score = score;
            best_index = static_cast<int>(i);
        }
    }

    if (best_index != nearest) {
        ROS_INFO("[ROUTE] Heading-aware start index %d instead of nearest %d",
                 best_index, nearest);
    }
    return best_index;
}

int ExplorerController::findNextUnvisitedWaypoint(int start) {
    if (main_route_.empty()) {
        return -1;
    }

    const int route_size = static_cast<int>(main_route_.size());
    for (int offset = 0; offset < route_size; ++offset) {
        const int index = (start + offset) % route_size;
        WaypointInfo& info = waypoints_[index];
        info.last_coverage = getWaypointCov(info.cell);

        if (!use_edge_route_ && info.last_coverage >= WAYPOINT_SKIP_RATIO) {
            ++wp_skipped_;
            continue;
        }
        if (info.visited) {
            ++wp_skipped_;
            continue;
        }
        if (info.times_reached >= MAX_WAYPOINT_VISITS) {
            ++wp_skipped_;
            continue;
        }

        return index;
    }

    return -1;
}

void ExplorerController::markCoverageSector(double rx, double ry, double ryaw) {
    if (!map_received_ || coverage_grid_.empty()) {
        return;
    }

    const double resolution = static_map_.info.resolution;
    const int radius_cells = static_cast<int>(std::ceil(cov_range_ / resolution));

    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            const double local_x = static_cast<double>(dx) * resolution;
            const double local_y = static_cast<double>(dy) * resolution;
            const double dist = std::hypot(local_x, local_y);
            if (dist > cov_range_ || dist < resolution * 0.5) {
                continue;
            }

            const double rel_angle = std::atan2(local_y, local_x);
            if (std::fabs(rel_angle) > cov_half_angle_) {
                continue;
            }

            const double world_x = rx + local_x * std::cos(ryaw) - local_y * std::sin(ryaw);
            const double world_y = ry + local_x * std::sin(ryaw) + local_y * std::cos(ryaw);
            int mx = 0;
            int my = 0;
            if (!worldToMap(world_x, world_y, mx, my)) {
                continue;
            }
            const int index = my * static_map_.info.width + mx;
            if (static_map_.data[index] == FREE_CELL) {
                coverage_grid_[index] = 1;
            }
        }
    }
}

void ExplorerController::markCoverageDisk(double wx, double wy, double radius) {
    if (!map_received_ || coverage_grid_.empty()) {
        return;
    }

    int center_x = 0;
    int center_y = 0;
    if (!worldToMap(wx, wy, center_x, center_y)) {
        return;
    }

    const int radius_cells = std::max(1, static_cast<int>(std::ceil(radius / static_map_.info.resolution)));
    const int width = static_map_.info.width;
    const int height = static_map_.info.height;
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            if (dx * dx + dy * dy > radius_cells * radius_cells) {
                continue;
            }
            const int nx = center_x + dx;
            const int ny = center_y + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                continue;
            }
            const int index = ny * width + nx;
            if (static_map_.data[index] == FREE_CELL) {
                coverage_grid_[index] = 1;
            }
        }
    }
}

double ExplorerController::getLocalCov(int mx, int my, double radius) const {
    if (!map_received_ || coverage_grid_.empty()) {
        return 0.0;
    }

    const int radius_cells = std::max(1, static_cast<int>(std::ceil(radius / static_map_.info.resolution)));
    const int width = static_map_.info.width;
    int free_count = 0;
    int covered_count = 0;
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            if (dx * dx + dy * dy > radius_cells * radius_cells) {
                continue;
            }
            const int nx = mx + dx;
            const int ny = my + dy;
            if (!isInMap(nx, ny)) {
                continue;
            }
            const int index = ny * width + nx;
            if (static_map_.data[index] != FREE_CELL) {
                continue;
            }
            ++free_count;
            if (coverage_grid_[index] == 1) {
                ++covered_count;
            }
        }
    }
    if (free_count == 0) {
        return 0.0;
    }
    return static_cast<double>(covered_count) / static_cast<double>(free_count);
}

double ExplorerController::getRobotLocalCov() const {
    int mx = 0;
    int my = 0;
    if (!worldToMap(robot_pose_.position.x, robot_pose_.position.y, mx, my)) {
        return 0.0;
    }

    const double resolution = static_map_.info.resolution;
    const int radius_cells = std::max(1, static_cast<int>(std::ceil(cov_range_ / resolution)));
    const int width = static_map_.info.width;
    int free_count = 0;
    int covered_count = 0;
    const double cos_yaw = std::cos(robot_yaw_);
    const double sin_yaw = std::sin(robot_yaw_);

    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            const int nx = mx + dx;
            const int ny = my + dy;
            if (!isInMap(nx, ny)) {
                continue;
            }
            const double local_x = (static_cast<double>(nx) + 0.5 - (static_cast<double>(mx) + 0.5)) * resolution;
            const double local_y = (static_cast<double>(ny) + 0.5 - (static_cast<double>(my) + 0.5)) * resolution;
            const double dist = std::hypot(local_x, local_y);
            if (dist > cov_range_ || dist < resolution * 0.5) {
                continue;
            }

            const double robot_x = local_x * cos_yaw + local_y * sin_yaw;
            const double robot_y = -local_x * sin_yaw + local_y * cos_yaw;
            const double rel_angle = std::atan2(robot_y, robot_x);
            if (std::fabs(rel_angle) > cov_half_angle_) {
                continue;
            }

            const int index = ny * width + nx;
            if (static_map_.data[index] != FREE_CELL) {
                continue;
            }
            ++free_count;
            if (coverage_grid_[index] == 1) {
                ++covered_count;
            }
        }
    }

    if (free_count == 0) {
        return 0.0;
    }
    return static_cast<double>(covered_count) / static_cast<double>(free_count);
}

double ExplorerController::getWaypointCov(const GridCell& waypoint) const {
    return getLocalCov(waypoint.x, waypoint.y, wp_cov_radius_);
}

double ExplorerController::getGlobalCov() const {
    if (!map_received_ || coverage_cells_total_ <= 0 || coverage_grid_.empty()) {
        return 0.0;
    }

    const int width = static_map_.info.width;
    const int height = static_map_.info.height;
    int covered_count = 0;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const int index = y * width + x;
            if (static_map_.data[index] == FREE_CELL && coverage_grid_[index] == 1) {
                ++covered_count;
            }
        }
    }
    return static_cast<double>(covered_count) / static_cast<double>(coverage_cells_total_);
}

std::vector<uint8_t> ExplorerController::buildPlanningFreeMask(bool use_inflation) const {
    std::vector<uint8_t> mask;
    if (!map_received_) {
        return mask;
    }

    const int width = static_cast<int>(static_map_.info.width);
    const int height = static_cast<int>(static_map_.info.height);
    mask.assign(static_cast<size_t>(width * height), 0);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const int index = y * width + x;
            if (static_map_.data[index] != FREE_CELL) {
                continue;
            }
            if (use_inflation && isCellBlocked(x, y)) {
                continue;
            }
            mask[static_cast<size_t>(index)] = 1;
        }
    }
    return mask;
}

std::vector<uint8_t> ExplorerController::buildStaticSafetyMask(
    double safety_radius,
    int min_obstacle_cluster_cells) const {
    std::vector<uint8_t> mask;
    if (!map_received_) {
        return mask;
    }

    const int width = static_cast<int>(static_map_.info.width);
    const int height = static_cast<int>(static_map_.info.height);
    const size_t total = static_cast<size_t>(width) * static_cast<size_t>(height);
    mask.assign(total, 0);

    std::vector<uint8_t> obstacle(total, 0);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const size_t index = static_cast<size_t>(y * width + x);
            const int8_t cost = static_map_.data[index];
            if (cost >= 50 || cost == UNKNOWN_CELL) {
                obstacle[index] = 1;
            }
        }
    }

    if (min_obstacle_cluster_cells > 1) {
        std::vector<uint8_t> filtered(total, 0);
        std::vector<uint8_t> seen(total, 0);
        std::vector<size_t> component;
        std::queue<size_t> pending;
        const int offsets[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                const size_t start = static_cast<size_t>(y * width + x);
                if (obstacle[start] == 0 || seen[start] != 0) {
                    continue;
                }
                component.clear();
                pending.push(start);
                seen[start] = 1;
                while (!pending.empty()) {
                    const size_t current = pending.front();
                    pending.pop();
                    component.push_back(current);
                    const int cx = static_cast<int>(current % static_cast<size_t>(width));
                    const int cy = static_cast<int>(current / static_cast<size_t>(width));
                    for (const auto& offset : offsets) {
                        const int nx = cx + offset[0];
                        const int ny = cy + offset[1];
                        if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                            continue;
                        }
                        const size_t next = static_cast<size_t>(ny * width + nx);
                        if (obstacle[next] == 0 || seen[next] != 0) {
                            continue;
                        }
                        seen[next] = 1;
                        pending.push(next);
                    }
                }
                if (static_cast<int>(component.size()) >= min_obstacle_cluster_cells) {
                    for (const size_t index : component) {
                        filtered[index] = 1;
                    }
                }
            }
        }
        obstacle.swap(filtered);
    }

    const int safety_cells = std::max(1, static_cast<int>(std::ceil(
        safety_radius / std::max(0.01, static_cast<double>(static_map_.info.resolution)))));
    std::vector<uint8_t> unsafe(total, 0);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const size_t obs_index = static_cast<size_t>(y * width + x);
            if (obstacle[obs_index] == 0) {
                continue;
            }
            for (int dy = -safety_cells; dy <= safety_cells; ++dy) {
                for (int dx = -safety_cells; dx <= safety_cells; ++dx) {
                    if (dx * dx + dy * dy > safety_cells * safety_cells) {
                        continue;
                    }
                    const int nx = x + dx;
                    const int ny = y + dy;
                    if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                        continue;
                    }
                    unsafe[static_cast<size_t>(ny * width + nx)] = 1;
                }
            }
        }
    }

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const size_t index = static_cast<size_t>(y * width + x);
            if (static_map_.data[index] == FREE_CELL && unsafe[index] == 0) {
                mask[index] = 1;
            }
        }
    }
    return mask;
}

void ExplorerController::recordPos(double x, double y) {
    pos_history_.push_back(PosRecord{x, y, ros::Time::now()});
    while (static_cast<int>(pos_history_.size()) > POSITION_HISTORY_SIZE) {
        pos_history_.pop_front();
    }
}

double ExplorerController::lastVisitAge(double x, double y, double radius) const {
    const ros::Time now = ros::Time::now();
    for (auto it = pos_history_.rbegin(); it != pos_history_.rend(); ++it) {
        if (pointDistance(it->x, it->y, x, y) <= radius) {
            return (now - it->stamp).toSec();
        }
    }
    return -1.0;
}

double ExplorerController::computeWaypointHeading(int route_index) const {
    if (main_route_.size() < 2) {
        return robot_yaw_;
    }

    const int last_index = static_cast<int>(main_route_.size()) - 1;
    const int current = std::max(0, std::min(route_index, last_index));
    const int prev_index = std::max(0, current - 1);
    const int next_index = std::min(last_index, current + 1);

    GridCell from = main_route_[current];
    GridCell to = main_route_[next_index];
    if (current == last_index) {
        from = main_route_[prev_index];
        to = main_route_[current];
    } else if (current > 0) {
        from = main_route_[prev_index];
        to = main_route_[next_index];
    }

    double from_x = 0.0;
    double from_y = 0.0;
    double to_x = 0.0;
    double to_y = 0.0;
    mapToWorld(from.x, from.y, from_x, from_y);
    mapToWorld(to.x, to.y, to_x, to_y);
    return std::atan2(to_y - from_y, to_x - from_x);
}

bool ExplorerController::worldToMap(double wx, double wy, int& mx, int& my) const {
    if (!map_received_) {
        return false;
    }

    const double origin_x = static_map_.info.origin.position.x;
    const double origin_y = static_map_.info.origin.position.y;
    const double resolution = static_map_.info.resolution;

    mx = static_cast<int>(std::floor((wx - origin_x) / resolution));
    my = static_cast<int>(std::floor((wy - origin_y) / resolution));
    return isInMap(mx, my);
}

void ExplorerController::mapToWorld(int mx, int my, double& wx, double& wy) const {
    const double origin_x = static_map_.info.origin.position.x;
    const double origin_y = static_map_.info.origin.position.y;
    const double resolution = static_map_.info.resolution;

    wx = origin_x + (static_cast<double>(mx) + 0.5) * resolution;
    wy = origin_y + (static_cast<double>(my) + 0.5) * resolution;
}

bool ExplorerController::isInMap(int mx, int my) const {
    return map_received_ &&
           mx >= 0 &&
           my >= 0 &&
           mx < static_map_.info.width &&
           my < static_map_.info.height;
}

int8_t ExplorerController::getStaticCost(int mx, int my) const {
    if (!isInMap(mx, my)) {
        return OCCUPIED_CELL;
    }
    return static_map_.data[my * static_map_.info.width + mx];
}

int8_t ExplorerController::getDynamicCost(int mx, int my) const {
    if (phase1_static_global_only_) {
        return FREE_CELL;
    }
    if (!isInMap(mx, my) || dynamic_layer_.empty()) {
        return FREE_CELL;
    }
    return dynamic_layer_[my * static_map_.info.width + mx];
}

int8_t ExplorerController::getTotalCost(int mx, int my) const {
    const int8_t static_cost = getStaticCost(mx, my);
    if (static_cost >= 50 || static_cost == UNKNOWN_CELL) {
        return OCCUPIED_CELL;
    }
    if (getDynamicCost(mx, my) >= 50) {
        return OCCUPIED_CELL;
    }
    return FREE_CELL;
}

bool ExplorerController::isCellBlocked(int mx, int my) const {
    if (!isInMap(mx, my)) {
        return true;
    }
    const size_t cell_index =
        static_cast<size_t>(my) * static_cast<size_t>(static_map_.info.width) +
        static_cast<size_t>(mx);
    const bool have_filtered_static_mask =
        dominant_safe_mask_.size() == static_map_.data.size();
    if (have_filtered_static_mask) {
        if (cell_index >= dominant_safe_mask_.size() ||
            dominant_safe_mask_[cell_index] == 0 ||
            getDynamicCost(mx, my) >= 50) {
            return true;
        }
        return false;
    }
    if (getStaticCost(mx, my) != FREE_CELL || getDynamicCost(mx, my) >= 50) {
        return true;
    }

    double wx = 0.0;
    double wy = 0.0;
    mapToWorld(mx, my, wx, wy);
    const int inflation_cells = std::max(1, static_cast<int>(std::ceil(
        inflation_radius_ / static_map_.info.resolution)));
    int static_blocked_count = 0;
    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
        for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
            if (dx * dx + dy * dy > inflation_cells * inflation_cells) {
                continue;
            }
            if (!isInMap(mx + dx, my + dy)) {
                ++static_blocked_count;
                continue;
            }
            if (getDynamicCost(mx + dx, my + dy) >= 50) {
                return true;
            }
            const int8_t static_cost = getStaticCost(mx + dx, my + dy);
            if (static_cost >= 50 || static_cost == UNKNOWN_CELL) {
                if (dx * dx + dy * dy <= 1) {
                    return true;
                }
                ++static_blocked_count;
            }
        }
    }

    return static_blocked_count >= 3;
}

bool ExplorerController::isLocalMapBlocked(double wx, double wy) const {
    if (!local_map_received_ || local_dynamic_map_.header.frame_id != "map") {
        return false;
    }

    const double origin_x = local_dynamic_map_.info.origin.position.x;
    const double origin_y = local_dynamic_map_.info.origin.position.y;
    const double resolution = local_dynamic_map_.info.resolution;

    const int mx = static_cast<int>(std::floor((wx - origin_x) / resolution));
    const int my = static_cast<int>(std::floor((wy - origin_y) / resolution));
    if (mx < 0 || my < 0 ||
        mx >= static_cast<int>(local_dynamic_map_.info.width) ||
        my >= static_cast<int>(local_dynamic_map_.info.height)) {
        return false;
    }
    return local_dynamic_map_.data[my * local_dynamic_map_.info.width + mx] >= 50;
}

bool ExplorerController::isLocalMapAreaBlocked(double wx,
                                               double wy,
                                               double radius,
                                               double min_ratio,
                                               int min_hits) const {
    if (!local_map_received_ || local_dynamic_map_.header.frame_id != "map" ||
        local_dynamic_map_.info.resolution <= 0.0) {
        return false;
    }

    const double origin_x = local_dynamic_map_.info.origin.position.x;
    const double origin_y = local_dynamic_map_.info.origin.position.y;
    const double resolution = local_dynamic_map_.info.resolution;
    const int center_x = static_cast<int>(std::floor((wx - origin_x) / resolution));
    const int center_y = static_cast<int>(std::floor((wy - origin_y) / resolution));
    const int radius_cells = std::max(1, static_cast<int>(std::ceil(radius / resolution)));
    int checked = 0;
    int occupied = 0;

    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            if (dx * dx + dy * dy > radius_cells * radius_cells) {
                continue;
            }
            const int mx = center_x + dx;
            const int my = center_y + dy;
            if (mx < 0 || my < 0 ||
                mx >= static_cast<int>(local_dynamic_map_.info.width) ||
                my >= static_cast<int>(local_dynamic_map_.info.height)) {
                continue;
            }
            ++checked;
            if (local_dynamic_map_.data[my * local_dynamic_map_.info.width + mx] >= 50) {
                ++occupied;
            }
        }
    }

    if (checked == 0) {
        return false;
    }
    const double ratio = static_cast<double>(occupied) / static_cast<double>(checked);
    return occupied >= min_hits && ratio >= min_ratio;
}

bool ExplorerController::handleFrontStaticBlock(const ros::Time& now,
                                                double lookahead_x,
                                                double lookahead_y) {
    // Record the time when the block first appeared.
    if (front_block_since_.toSec() == 0.0) {
        front_block_since_ = now;
        ROS_WARN("[ROUTE] Front area blocked by local/static map; trying brief heading change before skipping waypoint");
        return false;
    }

    const double blocked_for = (now - front_block_since_).toSec();

    // Phase 1 (<1.0s): keep turning — the control loop is already rotating,
    // so we just let it run and do nothing here.
    if (blocked_for < front_block_replan_delay_) {
        ROS_WARN_THROTTLE(0.8,
                          "[ROUTE] Front static blockage for %.1fs/%.1fs near lookahead=(%.2f,%.2f)",
                          blocked_for, front_block_replan_delay_, lookahead_x, lookahead_y);
        return false;
    }

    // Phase 2 (1.0-3.0s): try to replan A* around the obstacle without skipping waypoints.
    if (blocked_for < 3.0) {
        ++replan_ct_;
        int robot_x = 0;
        int robot_y = 0;
        if (worldToMap(robot_pose_.position.x, robot_pose_.position.y, robot_x, robot_y) &&
            has_active_goal_) {
            std::vector<geometry_msgs::Point> new_path;
            if (planToGoal(GridCell{robot_x, robot_y}, current_goal_cell_, new_path)) {
                global_path_ = new_path;
                visPath(new_path);
                last_goal_progress_time_ = now;
                front_block_since_ = ros::Time(0);   // blockage resolved by replan
                ROS_WARN("[ROUTE] Replanned around front blockage successfully");
                return true;   // handled — caller should NOT stop linear further
            }
        }
        ROS_WARN_THROTTLE(1.0,
                          "[ROUTE] Replan failed, still blocked for %.1fs",
                          blocked_for);
        return false;
    }

    // Phase 3 (>3.0s): give up and enter the escape (STUCK) state.
    front_block_since_ = ros::Time(0);
    stuck_ctr_ = STUCK_THRESHOLD;   // force isStuck() to return true next cycle
    has_active_goal_ = false;
    global_path_.clear();
    eStop();
    setState(RobotState::STUCK, "front blockage persisted >3s, starting escape manoeuvre");
    return true;
}

double ExplorerController::heuristic(const GridCell& a, const GridCell& b) const {
    const int dx = std::abs(a.x - b.x);
    const int dy = std::abs(a.y - b.y);
    return static_cast<double>(std::max(dx, dy)) +
           0.414 * static_cast<double>(std::min(dx, dy));
}

std::vector<GridCell> ExplorerController::getNeighbors(const GridCell& cell) const {
    std::vector<GridCell> neighbors;
    neighbors.reserve(8);
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) {
                continue;
            }
            const GridCell next{cell.x + dx, cell.y + dy};
            if (!isInMap(next.x, next.y) || isCellBlocked(next.x, next.y)) {
                continue;
            }
            if (dx != 0 && dy != 0) {
                if (isCellBlocked(cell.x + dx, cell.y) ||
                    isCellBlocked(cell.x, cell.y + dy)) {
                    continue;
                }
            }
            neighbors.push_back(next);
        }
    }
    return neighbors;
}

bool ExplorerController::aStarSearch(const GridCell& start,
                                     const GridCell& goal,
                                     std::vector<GridCell>& output) {
    if (!isInMap(start.x, start.y) || !isInMap(goal.x, goal.y) ||
        isCellBlocked(goal.x, goal.y)) {
        return false;
    }
    GridCell search_start = start;
    if (isCellBlocked(search_start.x, search_start.y)) {
        CoveragePlannerGridView planning_grid;
        planning_grid.width = static_cast<int>(static_map_.info.width);
        planning_grid.height = static_cast<int>(static_map_.info.height);
        planning_grid.occupancy.assign(
            static_cast<size_t>(planning_grid.width * planning_grid.height),
            OCCUPIED_CELL);
        const std::vector<uint8_t> planning_mask = buildPlanningFreeMask(true);
        if (planning_mask.size() == planning_grid.occupancy.size()) {
            for (int y = 0; y < planning_grid.height; ++y) {
                for (int x = 0; x < planning_grid.width; ++x) {
                    const int index = y * planning_grid.width + x;
                    if (planning_mask[static_cast<size_t>(index)] != 0) {
                        planning_grid.occupancy[static_cast<size_t>(index)] = FREE_CELL;
                    }
                }
            }
        }
        const CoveragePlannerCell nearest =
            nearestFreeCell(planning_grid, CoveragePlannerCell{start.x, start.y});
        if (!planning_grid.isFree(nearest.x, nearest.y)) {
            return false;
        }
        search_start = GridCell{nearest.x, nearest.y};
        ROS_WARN_THROTTLE(1.0,
                          "[PLAN] Robot start cell is inside safety inflation; planning from nearest safe cell (%d,%d)",
                          search_start.x, search_start.y);
    }

    const int width = static_map_.info.width;
    const int height = static_map_.info.height;
    const size_t total = static_cast<size_t>(width) * static_cast<size_t>(height);
    auto flatIndex = [width](int x, int y) -> size_t {
        return static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);
    };

    std::vector<double> g_cost(total, std::numeric_limits<double>::infinity());
    std::vector<GridCell> parent(total, GridCell{-1, -1});
    std::vector<bool> closed(total, false);
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open;

    const size_t start_index = flatIndex(search_start.x, search_start.y);
    g_cost[start_index] = 0.0;
    open.push(AStarNode{search_start, 0.0, heuristic(search_start, goal), search_start});

    int iteration = 0;
    const int max_iterations = width * height * 4;
    while (!open.empty()) {
        if (++iteration > max_iterations) {
            return false;
        }

        AStarNode current = open.top();
        open.pop();

        const size_t current_index = flatIndex(current.cell.x, current.cell.y);
        if (closed[current_index]) {
            continue;
        }
        closed[current_index] = true;

        if (current.cell == goal) {
            output.clear();
            GridCell trace = goal;
            while (trace != search_start) {
                output.push_back(trace);
                trace = parent[flatIndex(trace.x, trace.y)];
                if (trace.x < 0 || trace.y < 0) {
                    return false;
                }
            }
            output.push_back(search_start);
            std::reverse(output.begin(), output.end());
            return true;
        }

        for (const GridCell& next : getNeighbors(current.cell)) {
            const size_t next_index = flatIndex(next.x, next.y);
            if (closed[next_index]) {
                continue;
            }

            const bool diagonal = next.x != current.cell.x && next.y != current.cell.y;
            const double step = diagonal ? 1.414 : 1.0;
            const double tentative = current.g_cost + step;
            if (tentative < g_cost[next_index]) {
                g_cost[next_index] = tentative;
                parent[next_index] = current.cell;
                open.push(AStarNode{
                    next,
                    tentative,
                    tentative + heuristic(next, goal),
                    current.cell
                });
            }
        }
    }

    return false;
}

bool ExplorerController::planToGoal(const GridCell& start,
                                    const GridCell& goal,
                                    std::vector<geometry_msgs::Point>& waypoints) {
    std::vector<GridCell> cells;
    if (!aStarSearch(start, goal, cells)) {
        return false;
    }

    waypoints.clear();
    waypoints.reserve(cells.size());
    for (const GridCell& cell : cells) {
        geometry_msgs::Point point;
        mapToWorld(cell.x, cell.y, point.x, point.y);
        point.z = 0.0;
        waypoints.push_back(point);
    }
    return true;
}

bool ExplorerController::planAlongRouteToGoal(int start_index,
                                              int goal_index,
                                              std::vector<geometry_msgs::Point>& waypoints) const {
    if (main_route_.empty() ||
        start_index < 0 ||
        goal_index < 0 ||
        start_index >= static_cast<int>(main_route_.size()) ||
        goal_index >= static_cast<int>(main_route_.size())) {
        return false;
    }

    waypoints.clear();
    geometry_msgs::Point current;
    current.x = robot_pose_.position.x;
    current.y = robot_pose_.position.y;
    current.z = 0.0;
    waypoints.push_back(current);

    const int route_size = static_cast<int>(main_route_.size());
    int index = start_index;
    int guard = 0;
    while (guard <= route_size) {
        const GridCell& cell = main_route_[index];
        if (!isInMap(cell.x, cell.y) || isCellBlocked(cell.x, cell.y)) {
            waypoints.clear();
            return false;
        }

        geometry_msgs::Point point;
        mapToWorld(cell.x, cell.y, point.x, point.y);
        point.z = 0.0;
        if (waypoints.empty() ||
            pointDistance(waypoints.back().x, waypoints.back().y, point.x, point.y) > 0.05) {
            waypoints.push_back(point);
        }

        if (index == goal_index) {
            return waypoints.size() >= 2;
        }
        index = (index + 1) % route_size;
        ++guard;
    }

    waypoints.clear();
    return false;
}

bool ExplorerController::findReachableRouteIndex(int robot_x,
                                                 int robot_y,
                                                 int preferred_index,
                                                 int& selected_index,
                                                 std::vector<geometry_msgs::Point>& path) {
    selected_index = -1;
    path.clear();
    if (main_route_.empty() ||
        preferred_index < 0 ||
        preferred_index >= static_cast<int>(main_route_.size())) {
        return false;
    }

    const int route_size = static_cast<int>(main_route_.size());
    const int nearest_route = findRouteIndexWithHeading(robot_x, robot_y);
    const GridCell start{robot_x, robot_y};

    auto tryPlan = [&](int index) -> bool {
        if (index < 0 || index >= route_size) {
            return false;
        }
        const GridCell& candidate = main_route_[index];
        if (!isInMap(candidate.x, candidate.y) || isCellBlocked(candidate.x, candidate.y)) {
            return false;
        }

        double candidate_x = 0.0;
        double candidate_y = 0.0;
        mapToWorld(candidate.x, candidate.y, candidate_x, candidate_y);
        if (pointDistance(robot_pose_.position.x, robot_pose_.position.y,
                          candidate_x, candidate_y) < goal_tol_) {
            if (index >= 0 && index < static_cast<int>(waypoints_.size())) {
                waypoints_[index].visited = true;
                waypoints_[index].times_reached++;
                waypoints_[index].last_visit = ros::Time::now();
                waypoints_[index].last_coverage = getWaypointCov(candidate);
            }
            ROS_INFO("[ROUTE] Waypoint %d is already within %.2fm, marking reached",
                     index, goal_tol_);
            return false;
        }

        std::vector<geometry_msgs::Point> candidate_path;
        if (planAlongRouteToGoal(nearest_route, index, candidate_path) ||
            planToGoal(start, candidate, candidate_path)) {
            selected_index = index;
            path = candidate_path;
            return true;
        }
        return false;
    };

    if (tryPlan(preferred_index)) {
        return true;
    }

    if (use_edge_route_) {
        for (int offset = 1; offset <= edge_reachable_search_window_; ++offset) {
            const int index = (preferred_index + offset) % route_size;
            if (tryPlan(index)) {
                return true;
            }
        }

        const int backward_window = std::min(5, edge_reachable_search_window_);  // 扩大后向搜索窗口，减少因局部阻塞导致遍历中断
        for (int offset = 1; offset <= backward_window; ++offset) {
            const int index = (preferred_index - offset + route_size) % route_size;
            if (tryPlan(index)) {
                return true;
            }
        }

        ROS_WARN("[ROUTE] Safe outer route blocked near waypoint %d, limited search to +%d/-2 instead of jumping across map",
                 preferred_index, edge_reachable_search_window_);

        // Full-route A* fallback: try every unvisited waypoint directly
        for (int offset = 1; offset < route_size; ++offset) {
            const int index = (preferred_index + offset) % route_size;
            const WaypointInfo& info = waypoints_[index];
            if (info.visited || info.times_reached >= MAX_WAYPOINT_VISITS) {
                continue;
            }
            std::vector<geometry_msgs::Point> candidate_path;
            if (planToGoal(start, main_route_[index], candidate_path)) {
                selected_index = index;
                path = candidate_path;
                ROS_WARN("[ROUTE] Full-route A* fallback succeeded at waypoint %d", index);
                return true;
            }
        }
        return false;
    }

    for (int offset = 1; offset < route_size; ++offset) {
        const int index = (preferred_index + offset) % route_size;
        WaypointInfo& info = waypoints_[index];
        info.last_coverage = getWaypointCov(info.cell);
        if ((!use_edge_route_ && info.last_coverage >= WAYPOINT_SKIP_RATIO) ||
            info.visited ||
            info.times_reached >= MAX_WAYPOINT_VISITS) {
            continue;
        }
        if (tryPlan(index)) {
            return true;
        }
    }

    for (int offset = 1; offset < route_size; ++offset) {
        const int index = (preferred_index + offset) % route_size;
        if (tryPlan(index)) {
            return true;
        }
    }

    return false;
}

bool ExplorerController::planToObjectGoal(double object_wx,
                                          double object_wy,
                                          GridCell& approach_cell,
                                          std::vector<geometry_msgs::Point>& path) {
    int start_x = 0;
    int start_y = 0;
    int object_x = 0;
    int object_y = 0;
    if (!worldToMap(robot_pose_.position.x, robot_pose_.position.y, start_x, start_y)) {
        ROS_WARN("[OBJECT] Robot pose outside map while planning approach");
        return false;
    }
    if (!worldToMap(object_wx, object_wy, object_x, object_y)) {
        ROS_WARN("[OBJECT] Object point outside map while planning approach");
        return false;
    }

    EdgeRouteGrid grid;
    grid.width = static_cast<int>(static_map_.info.width);
    grid.height = static_cast<int>(static_map_.info.height);
    grid.resolution = static_map_.info.resolution;
    grid.occupancy.assign(static_map_.data.begin(), static_map_.data.end());

    EdgeRouteConfig config;
    config.object_standoff_m = object_standoff_;

    const std::vector<uint8_t> free_mask =
        buildStaticSafetyMask(route_safety_radius_, min_static_obstacle_cluster_cells_);
    EdgeRouteCell selected{0, 0};
    const bool found = chooseObjectApproachCell(
        grid,
        free_mask,
        EdgeRouteCell{start_x, start_y},
        EdgeRouteCell{object_x, object_y},
        config,
        selected);
    if (!found) {
        ROS_WARN("[OBJECT] No safe approach cell around object map=(%d,%d)", object_x, object_y);
        return false;
    }

    approach_cell = GridCell{selected.x, selected.y};
    if (!planToGoal(GridCell{start_x, start_y}, approach_cell, path)) {
        ROS_WARN("[OBJECT] A* failed to safe approach cell (%d,%d)",
                 approach_cell.x, approach_cell.y);
        return false;
    }

    return true;
}

bool ExplorerController::calcLookAhead(const std::vector<geometry_msgs::Point>& path,
                                       geometry_msgs::Point& look_ahead) {
    if (path.empty()) {
        return false;
    }

    size_t closest_index = 0;
    double closest_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path.size(); ++i) {
        const double dist = pointDistance(path[i].x, path[i].y,
                                          robot_pose_.position.x,
                                          robot_pose_.position.y);
        if (dist < closest_dist) {
            closest_dist = dist;
            closest_index = i;
        }
    }

    double accumulated = 0.0;
    for (size_t i = closest_index; i + 1 < path.size(); ++i) {
        const double segment = pointDistance(path[i], path[i + 1]);
        if (accumulated + segment >= lookahead_) {
            const double ratio = (lookahead_ - accumulated) / std::max(segment, 1e-6);
            look_ahead.x = path[i].x + ratio * (path[i + 1].x - path[i].x);
            look_ahead.y = path[i].y + ratio * (path[i + 1].y - path[i].y);
            look_ahead.z = 0.0;
            return true;
        }
        accumulated += segment;
    }

    look_ahead = path.back();
    return true;
}

bool ExplorerController::isPathBlocked() const {
    if (phase1_static_global_only_) {
        return false;
    }
    if (global_path_.empty()) {
        return false;
    }

    for (const geometry_msgs::Point& point : global_path_) {
        if (pointDistance(point.x, point.y,
                          robot_pose_.position.x,
                          robot_pose_.position.y) > 1.20) {
            continue;
        }
        int mx = 0;
        int my = 0;
        if (!worldToMap(point.x, point.y, mx, my)) {
            continue;
        }
        if (getDynamicCost(mx, my) >= 50) {
            return true;
        }
    }
    return false;
}

void ExplorerController::replanToCurrentGoal(const std::string& reason) {
    if (!has_active_goal_) {
        return;
    }

    int robot_x = 0;
    int robot_y = 0;
    if (!worldToMap(robot_pose_.position.x, robot_pose_.position.y, robot_x, robot_y)) {
        has_active_goal_ = false;
        global_path_.clear();
        setState(RobotState::EXPLORE, "current pose outside map during replan");
        return;
    }

    std::vector<geometry_msgs::Point> path;
    bool planned = false;
    if (!navigating_to_object_ && use_edge_route_ && route_index_ >= 0) {
        const int route_start = findRouteIndexWithHeading(robot_x, robot_y);
        planned = planAlongRouteToGoal(route_start, route_index_, path);
    }
    if (!planned) {
        planned = planToGoal(GridCell{robot_x, robot_y}, current_goal_cell_, path);
    }
    if (!planned) {
        ++path_fail_;
        has_active_goal_ = false;
        global_path_.clear();
        navigating_to_object_ = false;
        setState(RobotState::EXPLORE, "replan failed: " + reason);
        return;
    }

    global_path_ = path;
    visPath(path);
    double goal_world_x = 0.0;
    double goal_world_y = 0.0;
    mapToWorld(current_goal_cell_.x, current_goal_cell_.y, goal_world_x, goal_world_y);
    current_goal_start_distance_ = pointDistance(robot_pose_.position.x, robot_pose_.position.y,
                                                 goal_world_x, goal_world_y);
    last_goal_progress_time_ = ros::Time::now();
    ROS_WARN("[PLAN] Replanned current goal because %s", reason.c_str());
}

bool ExplorerController::xformCloud(const sensor_msgs::PointCloud2& input,
                                    sensor_msgs::PointCloud2& output) {
    try {
        geometry_msgs::TransformStamped transform =
            tf_buffer_.lookupTransform("map", input.header.frame_id,
                                       input.header.stamp, ros::Duration(0.3));
        tf2::doTransform(input, output, transform);
        return true;
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(2.0, "[DYN] Cloud transform failed: %s", ex.what());
        return false;
    }
}

void ExplorerController::updateDynamicLayer(const sensor_msgs::PointCloud2& cloud) {
    if (phase1_static_global_only_) {
        return;
    }
    if (!map_received_ || dynamic_layer_.empty()) {
        return;
    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    const int width = static_map_.info.width;
    const ros::Time now = ros::Time::now();

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
        int mx = 0;
        int my = 0;
        if (!worldToMap(*iter_x, *iter_y, mx, my)) {
            continue;
        }

        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const int nx = mx + dx;
                const int ny = my + dy;
                if (!isInMap(nx, ny)) {
                    continue;
                }
                const int index = ny * width + nx;
                dynamic_layer_[index] = OCCUPIED_CELL;
                dynamic_timestamps_[index] = now;
            }
        }
    }
}

void ExplorerController::decayDynamicLayer() {
    if (phase1_static_global_only_) {
        return;
    }
    if (!map_received_ || dynamic_layer_.empty()) {
        return;
    }

    const ros::Time now = ros::Time::now();
    for (size_t i = 0; i < dynamic_layer_.size(); ++i) {
        if (dynamic_layer_[i] == OCCUPIED_CELL &&
            (now - dynamic_timestamps_[i]).toSec() > dynamic_clear_time_) {
            dynamic_layer_[i] = FREE_CELL;
        }
    }
}

bool ExplorerController::isStuck() {
    if (state_ != RobotState::NAVIGATE) {
        return false;
    }

    const ros::Time now = ros::Time::now();
    if (last_stuck_check_time_.toSec() == 0.0) {
        last_stuck_check_time_ = now;
        last_sx_ = robot_pose_.position.x;
        last_sy_ = robot_pose_.position.y;
        return false;
    }
    if ((now - last_stuck_check_time_).toSec() < 1.0) {
        return false;
    }

    const double moved = pointDistance(robot_pose_.position.x, robot_pose_.position.y,
                                       last_sx_, last_sy_);
    last_sx_ = robot_pose_.position.x;
    last_sy_ = robot_pose_.position.y;
    last_stuck_check_time_ = now;

    if (moved < 0.04) {
        ++stuck_ctr_;
    } else {
        stuck_ctr_ = 0;
    }
    return stuck_ctr_ >= STUCK_THRESHOLD;
}

void ExplorerController::publishCmdVel(double linear, double angular) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    cmd_vel_pub_.publish(cmd);

    ROS_INFO_THROTTLE(1.0,
                      "[CMD] state=%s v=%.3f w=%.3f pose=%s scan_age=%.1fs local_map_age=%.1fs obstacle=%s object=%s",
                      stateName(state_).c_str(),
                      linear,
                      angular,
                      formatPose().c_str(),
                      last_scan_msg_time_.toSec() > 0.0
                          ? (ros::Time::now() - last_scan_msg_time_).toSec()
                          : -1.0,
                      last_lmap_msg_time_.toSec() > 0.0
                          ? (ros::Time::now() - last_lmap_msg_time_).toSec()
                          : -1.0,
                      latest_obstacle_warning_ ? "Y" : "N",
                      navigating_to_object_ ? "Y" : "N");
}

void ExplorerController::eStop() {
    publishCmdVel(0.0, 0.0);
}

void ExplorerController::visPath(const std::vector<geometry_msgs::Point>& path) {
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";
    for (const geometry_msgs::Point& point : path) {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position = point;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }
    global_path_pub_.publish(path_msg);
}

void ExplorerController::visGoal(const GridCell& goal) {
    geometry_msgs::PointStamped point;
    point.header.stamp = ros::Time::now();
    point.header.frame_id = "map";
    mapToWorld(goal.x, goal.y, point.point.x, point.point.y);
    point.point.z = 0.1;
    coverage_goal_pub_.publish(point);
}

void ExplorerController::pubCovMarkers() {
    if (!map_received_ || coverage_grid_.empty()) {
        return;
    }

    visualization_msgs::MarkerArray array;
    const int width = static_map_.info.width;
    const int height = static_map_.info.height;
    const int step = 4;
    const double resolution = static_map_.info.resolution;
    const double origin_x = static_map_.info.origin.position.x;
    const double origin_y = static_map_.info.origin.position.y;

    for (int y = 0; y < height; y += step) {
        for (int x = 0; x < width; x += step) {
            const int index = y * width + x;
            if (static_map_.data[index] != FREE_CELL || coverage_grid_[index] == 1) {
                continue;
            }

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "uncovered";
            marker.id = index;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = origin_x + (x + 0.5) * resolution;
            marker.pose.position.y = origin_y + (y + 0.5) * resolution;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = resolution * step * 0.8;
            marker.scale.y = resolution * step * 0.8;
            marker.scale.z = 0.02;
            marker.color.a = 0.45;
            marker.color.r = 1.0;
            marker.color.g = 0.35;
            marker.color.b = 0.35;
            array.markers.push_back(marker);
        }
    }

    coverage_marker_pub_.publish(array);
}

void ExplorerController::pubRouteMarkers() {
    if (!map_received_ || main_route_.empty()) {
        return;
    }

    visualization_msgs::MarkerArray array;
    for (size_t i = 0; i < main_route_.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "main_route";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        mapToWorld(main_route_[i].x, main_route_[i].y,
                   marker.pose.position.x, marker.pose.position.y);
        marker.pose.position.z = 0.05;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.10;
        marker.scale.y = 0.10;
        marker.scale.z = 0.10;

        const WaypointInfo& info = waypoints_[i];
        if (static_cast<int>(i) == route_index_) {
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.scale.x = 0.16;
            marker.scale.y = 0.16;
            marker.scale.z = 0.16;
        } else if (info.last_coverage >= WAYPOINT_SKIP_RATIO) {
            marker.color.a = 0.65;
            marker.color.r = 0.1;
            marker.color.g = 0.9;
            marker.color.b = 0.1;
        } else if (info.visited) {
            marker.color.a = 0.80;
            marker.color.r = 1.0;
            marker.color.g = 0.9;
            marker.color.b = 0.0;
        } else {
            marker.color.a = 0.70;
            marker.color.r = 0.1;
            marker.color.g = 0.6;
            marker.color.b = 1.0;
        }
        array.markers.push_back(marker);
    }

    coverage_marker_pub_.publish(array);
}

void ExplorerController::dynamicDecayLoop(const ros::TimerEvent&) {
    decayDynamicLayer();
}

void ExplorerController::coverageUpdateLoop(const ros::TimerEvent&) {
    if (!map_received_ || !pose_received_) {
        return;
    }
    markCoverageSector(robot_pose_.position.x, robot_pose_.position.y, robot_yaw_);
}

void ExplorerController::healthReportLoop(const ros::TimerEvent&) {
    if (!map_received_) {
        ROS_INFO_THROTTLE(5.0, "[HEALTH] waiting for static map");
        return;
    }

    const double global_cov = getGlobalCov();
    const double local_cov = pose_received_ ? getRobotLocalCov() : 0.0;
    const double pose_age = last_pose_msg_time_.toSec() > 0.0
                                ? (ros::Time::now() - last_pose_msg_time_).toSec()
                                : -1.0;
    const double scan_rx_age = last_scan_msg_time_.toSec() > 0.0
                                   ? (ros::Time::now() - last_scan_msg_time_).toSec()
                                   : -1.0;
    const double scan_stamp_age = last_scan_stamp_.toSec() > 0.0
                                      ? (ros::Time::now() - last_scan_stamp_).toSec()
                                      : -1.0;
    const double local_map_age = last_lmap_msg_time_.toSec() > 0.0
                                     ? (ros::Time::now() - last_lmap_msg_time_).toSec()
                                     : -1.0;

    ROS_INFO("============================================================");
    ROS_INFO("[HEALTH] state=%s pose=%s",
             stateName(state_).c_str(),
             pose_received_ ? formatPose().c_str() : "N/A");
    ROS_INFO("[HEALTH] coverage global=%.1f%% local=%.1f%% route=%zu idx=%d circuits=%d",
             global_cov * 100.0, local_cov * 100.0,
             main_route_.size(), route_index_, total_circuits_);
    ROS_INFO("[HEALTH] path ok=%d fail=%d replan=%d visited=%d skipped=%d dist=%.2fm",
             path_ok_, path_fail_, replan_ct_, wp_visited_, wp_skipped_, total_dist_);
    if (phase1_static_global_only_) {
        ROS_INFO("[HEALTH] phase1_static_global_only=true msg_age pose=%.1fs scan_rx=%.1fs scan_stamp=%.1fs dynamic_inputs=disabled",
                 pose_age, scan_rx_age, scan_stamp_age);
    } else {
        ROS_INFO("[HEALTH] msg_age pose=%.1fs scan_rx=%.1fs scan_stamp=%.1fs local_map=%.1fs obstacle_warning=%s",
                 pose_age, scan_rx_age, scan_stamp_age, local_map_age,
                 latest_obstacle_warning_ ? "true" : "false");
    }
    ROS_INFO("============================================================");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "explorer_controller");
    ExplorerController controller;
    controller.spin();
    return 0;
}
