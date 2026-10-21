// Standalone text decomposition physics filter for AviUtl2 / ExEdit2.
#ifndef UNICODE
#define UNICODE
#endif
#ifndef _UNICODE
#define _UNICODE
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <windows.h>

#include <box2d/box2d.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "plugin2.h"
#include "filter2.h"
#include "AlphaComponentSegmentation.h"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kPixelsPerMeter = 50.0;
constexpr int kMaximumPieces = 256;
constexpr int kMaximumTerrainLayers = 256;
constexpr wchar_t kFilterName[] = L"テキスト分解物理";
constexpr wchar_t kStaticItemName[] = L"静的オブジェクトにする";

FILTER_ITEM_SELECT::ITEM split_modes[] = {
    {L"1文字ずつ", 0},
    {L"文字内のαパーツ", 1},
    {nullptr},
};
auto static_object = FILTER_ITEM_CHECK(L"静的オブジェクトにする", false);
auto split_mode = FILTER_ITEM_SELECT(L"分割単位", 0, split_modes);
auto alpha_threshold = FILTER_ITEM_TRACK(L"αしきい値", 1.0, 1.0, 255.0, 1.0);
auto gravity = FILTER_ITEM_TRACK(L"重力", 1800.0, -100000.0, 100000.0, 1.0);
auto gravity_scale = FILTER_ITEM_TRACK(L"重力倍率", 1.0, -10.0, 10.0, 0.01);
auto friction = FILTER_ITEM_TRACK(L"摩擦", 0.5, 0.0, 1.0, 0.01);
auto restitution = FILTER_ITEM_TRACK(L"反発", 0.0, 0.0, 1.0, 0.01);
auto density = FILTER_ITEM_TRACK(L"密度", 1.0, 0.01, 1000.0, 0.01);
auto initial_speed = FILTER_ITEM_TRACK(L"初速の強さ", 0.0, 0.0, 8000.0, 1.0);
auto initial_angle = FILTER_ITEM_TRACK(L"初速角度 (-360～360)", 0.0, -360.0, 360.0, 1.0);
auto angular_velocity = FILTER_ITEM_TRACK(L"回転速度", 0.0, -2000.0, 2000.0, 1.0);
auto tumble = FILTER_ITEM_CHECK(L"倒れやすくする", true);
auto boundary_collision = FILTER_ITEM_CHECK(L"画面端と衝突", true);
auto region_center_x = FILTER_ITEM_TRACK(L"領域中心X", 0.0, -100000.0, 100000.0, 1.0);
auto region_center_y = FILTER_ITEM_TRACK(L"領域中心Y", 0.0, -100000.0, 100000.0, 1.0);
auto region_width_percent = FILTER_ITEM_TRACK(L"領域幅 (%)", 100.0, 1.0, 1000.0, 0.1);
auto region_height_percent = FILTER_ITEM_TRACK(L"領域高さ (%)", 100.0, 1.0, 1000.0, 0.1);
auto visualize_region = FILTER_ITEM_CHECK(L"物理演算領域を可視化", false);
auto substeps = FILTER_ITEM_TRACK(L"計算精度", 4.0, 1.0, 16.0, 1.0);
auto sleep_speed = FILTER_ITEM_TRACK(L"停止速度", 15.0, 0.0, 500.0, 1.0);

void* items[] = {
    &static_object,
    &split_mode,
    &alpha_threshold,
    &gravity,
    &gravity_scale,
    &friction,
    &restitution,
    &density,
    &initial_speed,
    &initial_angle,
    &angular_velocity,
    &tumble,
    &boundary_collision,
    &region_center_x,
    &region_center_y,
    &region_width_percent,
    &region_height_percent,
    &visualize_region,
    &substeps,
    &sleep_speed,
    nullptr,
};

struct Point {
    double x = 0.0;
    double y = 0.0;
};

struct PhysicsRegion {
    double center_x = 0.0;
    double center_y = 0.0;
    double width = 1.0;
    double height = 1.0;

    double left() const { return center_x - width * 0.5; }
    double right() const { return center_x + width * 0.5; }
    double top() const { return center_y - height * 0.5; }
    double bottom() const { return center_y + height * 0.5; }
};

struct ObjectTransform2D {
    double x = 0.0;
    double y = 0.0;
    double cosine = 1.0;
    double sine = 0.0;
    double scale_x = 1.0;
    double scale_y = 1.0;
};

struct Piece {
    int min_x = 0;
    int min_y = 0;
    int width = 0;
    int height = 0;
    double local_x = 0.0;
    double local_y = 0.0;
    std::vector<PIXEL_RGBA> pixels;
    std::vector<Point> hull;
    b2BodyId body = b2_nullBodyId;
};

// Static text object found on another layer; its pieces become fixed bodies.
struct TerrainSource {
    int layer = 0;
    std::uint64_t signature = 0;
    OBJECT_IMAGE_PARAM pose{};
    std::vector<Piece> pieces;
};

struct Runtime {
    std::mutex mutex;
    bool pieces_valid = false;
    int image_width = 0;
    int image_height = 0;
    int threshold = 0;
    int mode = 0;
    std::wstring text;
    std::wstring font;
    std::vector<Piece> pieces;
    std::vector<TerrainSource> terrains;
    b2WorldId world = b2_nullWorldId;
    bool simulation_valid = false;
    std::uint64_t simulation_key = 0;
    int cached_frame = 0;

    ~Runtime() {
        if (B2_IS_NON_NULL(world)) b2DestroyWorld(world);
    }

    void destroyWorld() {
        if (B2_IS_NON_NULL(world)) b2DestroyWorld(world);
        world = b2_nullWorldId;
        simulation_valid = false;
        cached_frame = 0;
        for (auto& piece : pieces) piece.body = b2_nullBodyId;
    }
};

template<class T>
void hashValue(std::uint64_t& hash, const T& value) {
    const auto* bytes = reinterpret_cast<const unsigned char*>(&value);
    for (std::size_t i = 0; i < sizeof(T); ++i) {
        hash ^= bytes[i];
        hash *= 1099511628211ull;
    }
}

float meters(double pixels) {
    return static_cast<float>(pixels / kPixelsPerMeter);
}

b2Vec2 vectorToBox(double x, double y) {
    return {meters(x), -meters(y)};
}

b2Pos positionToBox(double x, double y) {
    return {meters(x), -meters(y)};
}

float angleToBox(double degrees) {
    return static_cast<float>(-degrees * kPi / 180.0);
}

double angleFromBox(float radians) {
    return -static_cast<double>(radians) * 180.0 / kPi;
}

PhysicsRegion makePhysicsRegion(const SCENE_INFO& scene) {
    const double width_percent = std::clamp(region_width_percent.value, 1.0, 1000.0);
    const double height_percent = std::clamp(region_height_percent.value, 1.0, 1000.0);
    return {
        region_center_x.value,
        region_center_y.value,
        std::max(1.0, scene.width * width_percent / 100.0),
        std::max(1.0, scene.height * height_percent / 100.0),
    };
}

ObjectTransform2D makeObjectTransform(const OBJECT_IMAGE_PARAM& param) {
    const double radians = param.rz * kPi / 180.0;
    return {
        param.x,
        param.y,
        std::cos(radians),
        std::sin(radians),
        std::abs(param.sx) > 1e-6 ? param.sx : 1.0,
        std::abs(param.sy) > 1e-6 ? param.sy : 1.0,
    };
}

Point worldToLocal(const ObjectTransform2D& transform,
                   double world_x, double world_y) {
    const double dx = world_x - transform.x;
    const double dy = world_y - transform.y;
    return {
        (dx * transform.cosine + dy * transform.sine) / transform.scale_x,
        (-dx * transform.sine + dy * transform.cosine) / transform.scale_y,
    };
}

void appendWorldQuad(std::vector<VERTEX_COLOR>& vertices,
                     const ObjectTransform2D& transform,
                     double left, double top, double right, double bottom,
                     float red, float green, float blue, float alpha) {
    const Point corners[] = {
        worldToLocal(transform, left, top),
        worldToLocal(transform, right, top),
        worldToLocal(transform, right, bottom),
        worldToLocal(transform, left, bottom),
    };
    for (const auto& corner : corners) {
        vertices.push_back({
            static_cast<float>(corner.x), static_cast<float>(corner.y), 0.0f,
            red * alpha, green * alpha, blue * alpha, alpha,
        });
    }
}

void drawPhysicsRegion(FILTER_PROC_VIDEO* video, const PhysicsRegion& region,
                       const ObjectTransform2D& transform) {
    if (!video || !video->draw_poly) return;
    constexpr float red = 0.15f;
    constexpr float green = 1.0f;
    constexpr float blue = 0.35f;
    constexpr double border = 2.0;
    const double half_border = border * 0.5;
    const double left = region.left();
    const double right = region.right();
    const double top = region.top();
    const double bottom = region.bottom();
    std::vector<VERTEX_COLOR> vertices;
    vertices.reserve(20);
    appendWorldQuad(vertices, transform, left, top, right, bottom,
                    red, green, blue, 0.08f);
    appendWorldQuad(vertices, transform,
                    left - half_border, top - half_border,
                    right + half_border, top + half_border,
                    red, green, blue, 0.85f);
    appendWorldQuad(vertices, transform,
                    left - half_border, bottom - half_border,
                    right + half_border, bottom + half_border,
                    red, green, blue, 0.85f);
    appendWorldQuad(vertices, transform,
                    left - half_border, top + half_border,
                    left + half_border, bottom - half_border,
                    red, green, blue, 0.85f);
    appendWorldQuad(vertices, transform,
                    right - half_border, top + half_border,
                    right + half_border, bottom - half_border,
                    red, green, blue, 0.85f);
    if (video->set_culling_state) video->set_culling_state(false);
    video->draw_poly(VERTEX_TYPE::QUAD_COLOR, vertices.data(),
                     static_cast<int>(vertices.size()), nullptr);
}

double tumbleAngle(int index) {
    return (index & 1 ? 1.0 : -1.0) * (2.5 + (index % 3) * 1.25);
}

double tumbleVelocity(int index) {
    return (index & 1 ? 1.0 : -1.0) * (24.0 + (index % 4) * 7.0);
}

std::wstring utf8ToWide(const char* text) {
    if (!text || !*text) return {};
    const int length = MultiByteToWideChar(CP_UTF8, 0, text, -1, nullptr, 0);
    if (length <= 1) return {};
    std::wstring result(static_cast<std::size_t>(length), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, text, -1, result.data(), length);
    result.resize(static_cast<std::size_t>(length - 1));
    return result;
}

std::wstring readTextItem(FILTER_PROC_VIDEO* video, LPCWSTR item) {
    if (!video || !video->edit || !video->edit->find_object ||
        !video->edit->get_object_item_value) return {};
    const OBJECT_HANDLE object = video->edit->find_object(video->object->layer,
                                                           video->object->frame_s);
    return object ? utf8ToWide(video->edit->get_object_item_value(
                        object, L"テキスト", item)) : std::wstring{};
}

std::vector<int> lineLengths(const std::wstring& text) {
    std::vector<int> lengths(1, 0);
    for (std::size_t i = 0; i < text.size(); ++i) {
        const wchar_t character = text[i];
        if (character == L'\r') continue;
        if (character == L'\n') {
            lengths.push_back(0);
            continue;
        }
        if (character >= 0xd800 && character <= 0xdbff && i + 1 < text.size() &&
            text[i + 1] >= 0xdc00 && text[i + 1] <= 0xdfff) ++i;
        ++lengths.back();
    }
    return lengths;
}

std::vector<Piece> buildPieces(const std::vector<PIXEL_RGBA>& pixels,
                               int width, int height, int threshold,
                               int mode, const std::wstring& text) {
    std::vector<std::uint8_t> alpha(static_cast<std::size_t>(width) * height);
    for (std::size_t i = 0; i < alpha.size(); ++i) alpha[i] = pixels[i].a;
    auto components = segmentPhysicsAlphaComponents(alpha, width, height, threshold);
    if (components.empty()) return {};

    std::vector<std::vector<int>> groups;
    if (mode == 1 || text.empty()) {
        groups.resize(components.size());
        for (std::size_t i = 0; i < components.size(); ++i) {
            groups[i].push_back(static_cast<int>(i));
        }
    } else {
        const auto lengths = lineLengths(text);
        int left = width, top = height, right = -1, bottom = -1;
        for (const auto& component : components) {
            left = std::min(left, component.min_x);
            top = std::min(top, component.min_y);
            right = std::max(right, component.max_x);
            bottom = std::max(bottom, component.max_y);
        }
        std::vector<int> offsets(lengths.size(), 0);
        int group_count = 0;
        for (std::size_t line = 0; line < lengths.size(); ++line) {
            offsets[line] = group_count;
            group_count += std::max(0, lengths[line]);
        }
        groups.resize(std::max(1, group_count));
        const double occupied_width = std::max(1, right - left + 1);
        const double occupied_height = std::max(1, bottom - top + 1);
        for (std::size_t i = 0; i < components.size(); ++i) {
            const auto& component = components[i];
            const double center_x = (component.min_x + component.max_x + 1) * 0.5;
            const double center_y = (component.min_y + component.max_y + 1) * 0.5;
            int line = std::clamp(static_cast<int>((center_y - top) *
                                  lengths.size() / occupied_height),
                                  0, static_cast<int>(lengths.size()) - 1);
            while (line < static_cast<int>(lengths.size()) && lengths[line] == 0) ++line;
            if (line >= static_cast<int>(lengths.size())) {
                line = static_cast<int>(lengths.size()) - 1;
                while (line > 0 && lengths[line] == 0) --line;
            }
            const int characters = std::max(1, lengths[line]);
            const int column = std::clamp(static_cast<int>((center_x - left) *
                                          characters / occupied_width), 0, characters - 1);
            groups[std::clamp(offsets[line] + column, 0,
                              static_cast<int>(groups.size()) - 1)].push_back(
                                  static_cast<int>(i));
        }
    }

    std::vector<Piece> result;
    for (const auto& group : groups) {
        if (group.empty()) continue;
        int min_x = width, min_y = height, max_x = -1, max_y = -1;
        for (const int component_index : group) {
            const auto& component = components[component_index];
            min_x = std::min(min_x, component.min_x);
            min_y = std::min(min_y, component.min_y);
            max_x = std::max(max_x, component.max_x);
            max_y = std::max(max_y, component.max_y);
        }
        Piece piece{};
        piece.min_x = min_x;
        piece.min_y = min_y;
        piece.width = max_x - min_x + 1;
        piece.height = max_y - min_y + 1;
        piece.local_x = (min_x + max_x + 1) * 0.5 - width * 0.5;
        piece.local_y = (min_y + max_y + 1) * 0.5 - height * 0.5;
        piece.pixels.resize(static_cast<std::size_t>(piece.width) * piece.height);
        std::size_t opaque_count = 0;
        for (const int component_index : group) {
            opaque_count += components[component_index].pixels.size();
            for (const int source_index : components[component_index].pixels) {
                const int source_x = source_index % width;
                const int source_y = source_index / width;
                piece.pixels[static_cast<std::size_t>(source_y - min_y) * piece.width +
                             (source_x - min_x)] = pixels[source_index];
            }
        }
        const std::size_t stride = std::max<std::size_t>(1, opaque_count / 1024);
        std::size_t visited = 0;
        for (const int component_index : group) {
            for (const int source_index : components[component_index].pixels) {
                if ((visited++ % stride) != 0) continue;
                const double x0 = source_index % width - min_x - piece.width * 0.5;
                const double y0 = source_index / width - min_y - piece.height * 0.5;
                piece.hull.push_back({x0, y0});
                piece.hull.push_back({x0 + 1.0, y0});
                piece.hull.push_back({x0 + 1.0, y0 + 1.0});
                piece.hull.push_back({x0, y0 + 1.0});
            }
        }
        result.push_back(std::move(piece));
        if (result.size() >= kMaximumPieces) break;
    }
    return result;
}

std::vector<b2Vec2> outerHull(const std::vector<Point>& points,
                              double scale_x, double scale_y) {
    if (points.empty()) return {};
    std::vector<b2Vec2> source;
    source.reserve(points.size());
    for (const auto& point : points) {
        source.push_back(vectorToBox(point.x * scale_x, point.y * scale_y));
    }
    float min_x = source.front().x, max_x = source.front().x;
    float min_y = source.front().y, max_y = source.front().y;
    float min_sum = source.front().x + source.front().y, max_sum = min_sum;
    float min_diff = source.front().x - source.front().y, max_diff = min_diff;
    for (const auto& point : source) {
        min_x = std::min(min_x, point.x); max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y); max_y = std::max(max_y, point.y);
        min_sum = std::min(min_sum, point.x + point.y);
        max_sum = std::max(max_sum, point.x + point.y);
        min_diff = std::min(min_diff, point.x - point.y);
        max_diff = std::max(max_diff, point.x - point.y);
    }
    std::vector<b2Vec2> polygon = {
        {min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y},
    };
    const auto clip = [](const std::vector<b2Vec2>& input,
                         float a, float b, float limit) {
        std::vector<b2Vec2> output;
        if (input.empty()) return output;
        for (std::size_t i = 0; i < input.size(); ++i) {
            const b2Vec2 current = input[i];
            const b2Vec2 next = input[(i + 1) % input.size()];
            const float dc = a * current.x + b * current.y - limit;
            const float dn = a * next.x + b * next.y - limit;
            const bool current_inside = dc <= 1e-6f;
            const bool next_inside = dn <= 1e-6f;
            if (current_inside) output.push_back(current);
            if (current_inside == next_inside) continue;
            const float denominator = dc - dn;
            if (std::abs(denominator) <= 1e-12f) continue;
            const float t = dc / denominator;
            output.push_back({current.x + (next.x - current.x) * t,
                              current.y + (next.y - current.y) * t});
        }
        return output;
    };
    polygon = clip(polygon, 1.0f, 1.0f, max_sum);
    polygon = clip(polygon, -1.0f, -1.0f, -min_sum);
    polygon = clip(polygon, 1.0f, -1.0f, max_diff);
    polygon = clip(polygon, -1.0f, 1.0f, -min_diff);
    return polygon;
}

b2BodyId createBox(b2WorldId world, double x, double y,
                   double width, double height, bool dynamic,
                   double body_friction, double body_restitution,
                   double body_density) {
    b2BodyDef body_definition = b2DefaultBodyDef();
    body_definition.type = dynamic ? b2_dynamicBody : b2_staticBody;
    body_definition.position = positionToBox(x, y);
    const b2BodyId body = b2CreateBody(world, &body_definition);
    b2ShapeDef shape_definition = b2DefaultShapeDef();
    shape_definition.material.friction = static_cast<float>(body_friction);
    shape_definition.material.restitution = static_cast<float>(body_restitution);
    shape_definition.density = static_cast<float>(body_density);
    const b2Polygon box = b2MakeBox(meters(std::max(1.0, width) * 0.5),
                                    meters(std::max(1.0, height) * 0.5));
    b2CreatePolygonShape(body, &shape_definition, &box);
    if (dynamic) b2Body_UpdateMassFromShapes(body);
    return body;
}

b2Pos pieceWorldPosition(const Piece& piece, const OBJECT_IMAGE_PARAM& pose) {
    const double radians = pose.rz * kPi / 180.0;
    const double cosine = std::cos(radians);
    const double sine = std::sin(radians);
    const double local_x = (piece.local_x - pose.cx) * pose.sx;
    const double local_y = (piece.local_y - pose.cy) * pose.sy;
    return positionToBox(pose.x + local_x * cosine - local_y * sine,
                         pose.y + local_x * sine + local_y * cosine);
}

void createPieceShape(b2BodyId body, const Piece& piece,
                      const b2ShapeDef& shape_definition,
                      const OBJECT_IMAGE_PARAM& pose) {
    const double absolute_scale_x = std::max(1e-6, std::abs(static_cast<double>(pose.sx)));
    const double absolute_scale_y = std::max(1e-6, std::abs(static_cast<double>(pose.sy)));
    const auto points = outerHull(piece.hull, absolute_scale_x, absolute_scale_y);
    if (points.size() >= 3 && points.size() <= B2_MAX_POLYGON_VERTICES) {
        const b2Hull hull = b2ComputeHull(points.data(), static_cast<int>(points.size()));
        if (hull.count >= 3) {
            const b2Polygon polygon = b2MakePolygon(&hull, 0.0f);
            b2CreatePolygonShape(body, &shape_definition, &polygon);
            return;
        }
    }
    const b2Polygon box = b2MakeBox(
        meters(std::max(1.0, piece.width * absolute_scale_x) * 0.5),
        meters(std::max(1.0, piece.height * absolute_scale_y) * 0.5));
    b2CreatePolygonShape(body, &shape_definition, &box);
}

void addTerrainBodies(b2WorldId world, const TerrainSource& terrain, double bounce) {
    for (const auto& piece : terrain.pieces) {
        b2BodyDef body_definition = b2DefaultBodyDef();
        body_definition.type = b2_staticBody;
        body_definition.position = pieceWorldPosition(piece, terrain.pose);
        body_definition.rotation = b2MakeRot(angleToBox(terrain.pose.rz));
        const b2BodyId body = b2CreateBody(world, &body_definition);
        b2ShapeDef shape_definition = b2DefaultShapeDef();
        shape_definition.material.friction = static_cast<float>(friction.value);
        shape_definition.material.restitution = static_cast<float>(bounce);
        createPieceShape(body, piece, shape_definition, terrain.pose);
    }
}

bool isStaticObject(EDIT_SECTION* edit, OBJECT_HANDLE object) {
    if (!edit->count_object_effect || !edit->get_object_item_value ||
        edit->count_object_effect(object, kFilterName) <= 0) return false;
    const char* value = edit->get_object_item_value(object, kFilterName, kStaticItemName);
    return value && *value && std::strcmp(value, "0") != 0 &&
           std::strcmp(value, "false") != 0;
}

bool fetchLayerImage(FILTER_PROC_VIDEO* video, int layer,
                     std::vector<PIXEL_RGBA>& pixels, int& width, int& height) {
    if (!video->copy_image_resource || !video->get_image_resource_size ||
        !video->get_image_resource_data || !video->release_image_resource) return false;
    const std::wstring resource = L"resource:text_physics_terrain_" + std::to_wstring(layer);
    const std::wstring source = L"layer:" + std::to_wstring(layer);
    bool ok = video->copy_image_resource(resource.c_str(), source.c_str()) &&
              video->get_image_resource_size(resource.c_str(), &width, &height) &&
              width > 0 && height > 0;
    if (ok) {
        pixels.resize(static_cast<std::size_t>(width) * height);
        ok = video->get_image_resource_data(resource.c_str(), pixels.data(), width, height,
                                            width * static_cast<int>(sizeof(PIXEL_RGBA)),
                                            OUTPUT_PIXEL_FORMAT::RGBA);
    }
    video->release_image_resource(resource.c_str());
    return ok;
}

// Looks for other layers whose text carries this filter with the static flag on.
// Pixels are read back only when an object's alias or pose changes.
void refreshTerrains(Runtime& runtime, FILTER_PROC_VIDEO* video, int threshold) {
    std::vector<TerrainSource> found;
    EDIT_SECTION* edit = video->edit;
    if (edit && video->get_image_object) {
        const int layer_max = edit->info
            ? std::clamp(edit->info->layer_max, 0, kMaximumTerrainLayers - 1)
            : kMaximumTerrainLayers - 1;
        for (int layer = 0; layer <= layer_max; ++layer) {
            if (layer == video->object->layer) continue;
            const OBJECT_HANDLE object = video->get_image_object(layer, 0.0);
            if (!object || object == video->object->handle || !isStaticObject(edit, object)) continue;
            TerrainSource terrain;
            terrain.layer = layer;
            terrain.pose.sx = terrain.pose.sy = terrain.pose.sz = 1.0f;
            terrain.pose.alpha = 1.0f;
            if (!video->get_output_image_param ||
                !video->get_output_image_param(object, -video->object->time,
                                               &terrain.pose, sizeof(terrain.pose))) {
                if (!video->get_output_image_param ||
                    !video->get_output_image_param(object, 0.0, &terrain.pose,
                                                   sizeof(terrain.pose))) continue;
            }
            std::uint64_t signature = 1469598103934665603ull;
            if (const char* alias = edit->get_object_alias ? edit->get_object_alias(object) : nullptr) {
                for (const char* c = alias; *c; ++c) hashValue(signature, *c);
            }
            hashValue(signature, terrain.pose);
            hashValue(signature, threshold);
            terrain.signature = signature;
            found.push_back(std::move(terrain));
        }
    }
    for (auto& terrain : found) {
        const auto cached = std::find_if(runtime.terrains.begin(), runtime.terrains.end(),
            [&](const TerrainSource& old) {
                return old.layer == terrain.layer && old.signature == terrain.signature;
            });
        if (cached != runtime.terrains.end()) {
            terrain.pieces = std::move(cached->pieces);
            continue;
        }
        std::vector<PIXEL_RGBA> pixels;
        int width = 0, height = 0;
        if (fetchLayerImage(video, terrain.layer, pixels, width, height)) {
            terrain.pieces = buildPieces(pixels, width, height, threshold, 1, {});
            for (auto& piece : terrain.pieces) {
                piece.pixels.clear();
                piece.pixels.shrink_to_fit();
            }
        }
        const std::wstring message = L"[TextPhysicsFilter] terrain layer=" +
            std::to_wstring(terrain.layer) + L" pieces=" +
            std::to_wstring(terrain.pieces.size()) + L"\n";
        OutputDebugStringW(message.c_str());
    }
    runtime.terrains = std::move(found);
}

bool resetSimulation(Runtime& runtime, FILTER_PROC_VIDEO* video,
                     const OBJECT_IMAGE_PARAM& start,
                     const PhysicsRegion& region, std::uint64_t key) {
    runtime.destroyWorld();
    b2WorldDef world_definition = b2DefaultWorldDef();
    world_definition.gravity = {0.0f, -meters(gravity.value)};
    runtime.world = b2CreateWorld(&world_definition);
    if (B2_IS_NULL(runtime.world)) return false;

    const double bounce = std::clamp(restitution.value, 0.0, 1.0);
    if (boundary_collision.value) {
        constexpr double thickness = kPixelsPerMeter;
        createBox(runtime.world, region.center_x,
                  region.bottom() + thickness * 0.5,
                  region.width + thickness * 2.0, thickness,
                  false, friction.value, bounce, 1.0);
        createBox(runtime.world, region.center_x,
                  region.top() - thickness * 0.5,
                  region.width + thickness * 2.0, thickness,
                  false, friction.value, bounce, 1.0);
        createBox(runtime.world, region.left() - thickness * 0.5,
                  region.center_y, thickness, region.height,
                  false, friction.value, bounce, 1.0);
        createBox(runtime.world, region.right() + thickness * 0.5,
                  region.center_y, thickness, region.height,
                  false, friction.value, bounce, 1.0);
    }

    for (const auto& terrain : runtime.terrains) addTerrainBodies(runtime.world, terrain, bounce);

    const double launch_radians = initial_angle.value * kPi / 180.0;
    const double velocity_x = std::sin(launch_radians) * initial_speed.value;
    const double velocity_y = -std::cos(launch_radians) * initial_speed.value;
    for (std::size_t index = 0; index < runtime.pieces.size(); ++index) {
        auto& piece = runtime.pieces[index];
        const bool tumble_enabled = tumble.value;
        b2BodyDef body_definition = b2DefaultBodyDef();
        body_definition.type = b2_dynamicBody;
        body_definition.position = pieceWorldPosition(piece, start);
        const double rotation = start.rz + (tumble_enabled ? tumbleAngle(
            static_cast<int>(index)) : 0.0);
        body_definition.rotation = b2MakeRot(angleToBox(rotation));
        body_definition.linearVelocity = vectorToBox(velocity_x, velocity_y);
        const double configured_angular_velocity = std::abs(angular_velocity.value) >= 0.001
            ? angular_velocity.value
            : (tumble_enabled ? tumbleVelocity(static_cast<int>(index)) : 0.0);
        body_definition.angularVelocity = angleToBox(configured_angular_velocity);
        body_definition.angularDamping = tumble_enabled ? 2.0f : 4.0f;
        body_definition.motionLocks.angularZ = !tumble_enabled &&
                                               std::abs(angular_velocity.value) < 0.001;
        body_definition.gravityScale = static_cast<float>(gravity_scale.value);
        body_definition.sleepThreshold = meters(std::max(0.0, sleep_speed.value));
        const b2BodyId body = b2CreateBody(runtime.world, &body_definition);

        b2ShapeDef shape_definition = b2DefaultShapeDef();
        shape_definition.material.friction = static_cast<float>(friction.value);
        shape_definition.material.restitution = static_cast<float>(bounce);
        shape_definition.density = static_cast<float>(density.value);
        createPieceShape(body, piece, shape_definition, start);
        b2Body_UpdateMassFromShapes(body);
        piece.body = body;
    }
    runtime.simulation_valid = true;
    runtime.simulation_key = key;
    runtime.cached_frame = 0;
    return true;
}

bool processVideo(FILTER_PROC_VIDEO* video) {
    if (!video || !video->scene || !video->object || video->scene->rate <= 0 ||
        !video->get_image_data || !video->create_image_resource || !video->draw_image) return true;
    auto* runtime = static_cast<Runtime*>(video->userdata);
    if (!runtime) return true;
    // Static objects are left untouched; other instances read them as terrain.
    if (static_object.value) return true;
    std::lock_guard<std::mutex> lock(runtime->mutex);
    const int width = video->object->width;
    const int height = video->object->height;
    if (width <= 0 || height <= 0) return true;
    const int threshold = std::clamp(static_cast<int>(std::lround(alpha_threshold.value)), 1, 255);
    const int mode = split_mode.value == 1 ? 1 : 0;
    const std::wstring text = readTextItem(video, L"テキスト");
    const std::wstring font = readTextItem(video, L"フォント");
    if (!runtime->pieces_valid || runtime->image_width != width ||
        runtime->image_height != height || runtime->threshold != threshold ||
        runtime->mode != mode || runtime->text != text || runtime->font != font) {
        std::vector<PIXEL_RGBA> pixels(static_cast<std::size_t>(width) * height);
        video->get_image_data(pixels.data());
        runtime->pieces = buildPieces(pixels, width, height, threshold, mode, text);
        runtime->pieces_valid = true;
        runtime->image_width = width;
        runtime->image_height = height;
        runtime->threshold = threshold;
        runtime->mode = mode;
        runtime->text = text;
        runtime->font = font;
        runtime->destroyWorld();
    }
    const PhysicsRegion region = makePhysicsRegion(*video->scene);
    if (runtime->pieces.empty()) {
        if (visualize_region.value) {
            OBJECT_IMAGE_PARAM current{};
            current.sx = current.sy = current.sz = 1.0f;
            current.alpha = 1.0f;
            if (video->get_output_image_param) {
                video->get_output_image_param(nullptr, 0.0, &current, sizeof(current));
            }
            drawPhysicsRegion(video, region, makeObjectTransform(current));
        }
        if (video->set_default_anchor) video->set_default_anchor(0, 0);
        return false;
    }

    OBJECT_IMAGE_PARAM start{};
    start.sx = start.sy = start.sz = 1.0f;
    start.alpha = 1.0f;
    if (video->get_output_image_param) {
        video->get_output_image_param(nullptr, -video->object->time, &start, sizeof(start));
    }
    refreshTerrains(*runtime, video, threshold);
    std::uint64_t key = 1469598103934665603ull;
    hashValue(key, width); hashValue(key, height);
    hashValue(key, video->scene->width); hashValue(key, video->scene->height);
    hashValue(key, threshold); hashValue(key, mode);
    hashValue(key, gravity.value); hashValue(key, gravity_scale.value);
    hashValue(key, friction.value); hashValue(key, restitution.value); hashValue(key, density.value);
    hashValue(key, initial_speed.value); hashValue(key, initial_angle.value);
    hashValue(key, angular_velocity.value); hashValue(key, tumble.value);
    hashValue(key, boundary_collision.value);
    hashValue(key, region_center_x.value); hashValue(key, region_center_y.value);
    hashValue(key, region_width_percent.value); hashValue(key, region_height_percent.value);
    hashValue(key, substeps.value); hashValue(key, sleep_speed.value);
    hashValue(key, start);
    hashValue(key, runtime->pieces.size());
    for (const auto& terrain : runtime->terrains) {
        hashValue(key, terrain.layer);
        hashValue(key, terrain.signature);
    }
    for (const wchar_t character : text) hashValue(key, character);

    const int frame = std::max(0, video->object->frame);
    if (!runtime->simulation_valid || runtime->simulation_key != key ||
        frame < runtime->cached_frame) {
        if (!resetSimulation(*runtime, video, start, region, key)) return true;
    }
    const double frame_seconds = static_cast<double>(video->scene->scale) / video->scene->rate;
    for (int simulated = runtime->cached_frame; simulated < frame; ++simulated) {
        b2World_Step(runtime->world, static_cast<float>(frame_seconds),
                     std::clamp(static_cast<int>(std::lround(substeps.value)), 1, 16));
    }
    runtime->cached_frame = frame;

    OBJECT_IMAGE_PARAM current{};
    current.sx = current.sy = current.sz = 1.0f;
    current.alpha = 1.0f;
    if (video->get_output_image_param) {
        video->get_output_image_param(nullptr, 0.0, &current, sizeof(current));
    }
    const ObjectTransform2D transform = makeObjectTransform(current);
    for (std::size_t index = 0; index < runtime->pieces.size(); ++index) {
        const auto& piece = runtime->pieces[index];
        if (B2_IS_NULL(piece.body) || !b2Body_IsValid(piece.body)) continue;
        const b2Pos position = b2Body_GetPosition(piece.body);
        const b2Rot rotation = b2Body_GetRotation(piece.body);
        const double world_x = static_cast<double>(position.x) * kPixelsPerMeter;
        const double world_y = -static_cast<double>(position.y) * kPixelsPerMeter;
        const Point local = worldToLocal(transform, world_x, world_y);
        const float local_rotation = static_cast<float>(
            angleFromBox(b2Rot_GetAngle(rotation)) - current.rz);
        const std::wstring resource = L"resource:text_decomposition_" +
            std::to_wstring(video->object->effect_id) + L"_" + std::to_wstring(index);
        video->create_image_resource(resource.c_str(), piece.pixels.data(),
                                     piece.width, piece.height);
        video->draw_image(resource.c_str(), static_cast<float>(local.x),
                          static_cast<float>(local.y), 0.0f,
                          0.0f, 0.0f, local_rotation,
                          1.0f, 1.0f, 1.0f, 1.0f);
    }
    if (visualize_region.value) drawPhysicsRegion(video, region, transform);
    if (video->set_default_anchor) video->set_default_anchor(0, 0);
    return false;
}

void* createRuntime(std::int64_t) {
    return new Runtime();
}

void destroyRuntime(std::int64_t, void* userdata) {
    delete static_cast<Runtime*>(userdata);
}

FILTER_PLUGIN_TABLE filter = {
    FILTER_PLUGIN_TABLE::FLAG_VIDEO | FILTER_PLUGIN_TABLE::FLAG_USERDATA,
    L"テキスト分解物理",
    L"テキスト分解物理",
    L"Standalone Text Decomposition Physics version 0.1.3-terrain (Box2D 3.2.0)",
    items,
    processVideo,
    nullptr,
    createRuntime,
    destroyRuntime,
};

} // namespace

EXTERN_C __declspec(dllexport) DWORD RequiredVersion() {
    return 2003300;
}

EXTERN_C __declspec(dllexport) bool InitializePlugin(DWORD) {
    return true;
}

EXTERN_C __declspec(dllexport) void UninitializePlugin() {
}

EXTERN_C __declspec(dllexport) FILTER_PLUGIN_TABLE* GetFilterPluginTable() {
    return &filter;
}
