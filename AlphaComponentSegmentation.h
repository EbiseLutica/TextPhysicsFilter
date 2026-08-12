#pragma once

#include <algorithm>
#include <cstdint>
#include <deque>
#include <vector>

struct PhysicsAlphaComponent {
    int min_x = 0;
    int min_y = 0;
    int max_x = -1;
    int max_y = -1;
    std::vector<int> pixels;
};

// Opaque cores are segmented first. Antialiased pixels are then assigned to
// the nearest core, so a faint bridge cannot merge separate kanji parts.
inline std::vector<PhysicsAlphaComponent> segmentPhysicsAlphaComponents(
        const std::vector<std::uint8_t>& alpha, int width, int height,
        int threshold, int maximum_components = 512) {
    std::vector<PhysicsAlphaComponent> result;
    if (width <= 0 || height <= 0 ||
        alpha.size() < static_cast<std::size_t>(width) * height) return result;
    threshold = std::clamp(threshold, 1, 255);
    const int core_threshold = std::max(threshold, 96);
    const int count = width * height;
    std::vector<int> labels(static_cast<std::size_t>(count), -1);
    std::deque<int> frontier;
    constexpr int dx[] = {-1, 1, 0, 0};
    constexpr int dy[] = {0, 0, -1, 1};

    for (int start = 0; start < count; ++start) {
        if (labels[start] >= 0 || alpha[start] < core_threshold) continue;
        const int label = static_cast<int>(result.size());
        if (label >= maximum_components) break;
        result.emplace_back();
        std::deque<int> core;
        labels[start] = label;
        core.push_back(start);
        while (!core.empty()) {
            const int index = core.front();
            core.pop_front();
            frontier.push_back(index);
            const int x = index % width;
            const int y = index / width;
            for (int direction = 0; direction < 4; ++direction) {
                const int nx = x + dx[direction];
                const int ny = y + dy[direction];
                if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
                const int next = ny * width + nx;
                if (labels[next] >= 0 || alpha[next] < core_threshold) continue;
                labels[next] = label;
                core.push_back(next);
            }
        }
    }

    while (!frontier.empty()) {
        const int index = frontier.front();
        frontier.pop_front();
        const int x = index % width;
        const int y = index / width;
        for (int direction = 0; direction < 4; ++direction) {
            const int nx = x + dx[direction];
            const int ny = y + dy[direction];
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
            const int next = ny * width + nx;
            if (labels[next] >= 0 || alpha[next] < threshold) continue;
            labels[next] = labels[index];
            frontier.push_back(next);
        }
    }

    // Keep genuinely faint marks that contain no opaque core.
    for (int start = 0; start < count; ++start) {
        if (labels[start] >= 0 || alpha[start] < threshold) continue;
        const int label = static_cast<int>(result.size());
        if (label >= maximum_components) break;
        result.emplace_back();
        labels[start] = label;
        frontier.push_back(start);
        while (!frontier.empty()) {
            const int index = frontier.front();
            frontier.pop_front();
            const int x = index % width;
            const int y = index / width;
            for (int direction = 0; direction < 4; ++direction) {
                const int nx = x + dx[direction];
                const int ny = y + dy[direction];
                if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
                const int next = ny * width + nx;
                if (labels[next] >= 0 || alpha[next] < threshold) continue;
                labels[next] = label;
                frontier.push_back(next);
            }
        }
    }

    for (int index = 0; index < count; ++index) {
        const int label = labels[index];
        if (label < 0 || label >= static_cast<int>(result.size())) continue;
        auto& component = result[label];
        const int x = index % width;
        const int y = index / width;
        if (component.pixels.empty()) {
            component.min_x = component.max_x = x;
            component.min_y = component.max_y = y;
        } else {
            component.min_x = std::min(component.min_x, x);
            component.max_x = std::max(component.max_x, x);
            component.min_y = std::min(component.min_y, y);
            component.max_y = std::max(component.max_y, y);
        }
        component.pixels.push_back(index);
    }
    result.erase(std::remove_if(result.begin(), result.end(),
        [](const PhysicsAlphaComponent& component) { return component.pixels.empty(); }),
        result.end());
    return result;
}
