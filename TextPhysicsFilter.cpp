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
#include <cstdlib>
#include <cstring>
#include <cwchar>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
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
auto collision_group = FILTER_ITEM_TRACK(L"衝突グループ", 0.0, 0.0, 99.0, 1.0);
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
    &collision_group,
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

// Everything a text object contributes to the physics world. The world level
// fields (gravity, region, substeps, boundary) are taken from the group leader.
struct ObjectParams {
    double gravity = 1800.0;
    double gravity_scale = 1.0;
    double friction = 0.5;
    double restitution = 0.0;
    double density = 1.0;
    double initial_speed = 0.0;
    double initial_angle = 0.0;
    double angular_velocity = 0.0;
    double region_center_x = 0.0;
    double region_center_y = 0.0;
    double region_width_percent = 100.0;
    double region_height_percent = 100.0;
    double substeps = 4.0;
    double sleep_speed = 15.0;
    bool tumble = true;
    bool boundary_collision = true;
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

// Another text object of the same collision group. Its pieces are simulated in
// this instance's world but never drawn here.
struct Neighbor {
    int layer = 0;
    int frame_start = 0;
    int frame_end = 0;
    std::uint64_t signature = 0;
    OBJECT_IMAGE_PARAM start{};
    ObjectParams params;
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
    std::vector<Neighbor> members;
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
        for (auto& member : members) {
            for (auto& piece : member.pieces) piece.body = b2_nullBodyId;
        }
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

ObjectParams ownParams() {
    ObjectParams p;
    p.gravity = gravity.value;
    p.gravity_scale = gravity_scale.value;
    p.friction = friction.value;
    p.restitution = restitution.value;
    p.density = density.value;
    p.initial_speed = initial_speed.value;
    p.initial_angle = initial_angle.value;
    p.angular_velocity = angular_velocity.value;
    p.region_center_x = region_center_x.value;
    p.region_center_y = region_center_y.value;
    p.region_width_percent = region_width_percent.value;
    p.region_height_percent = region_height_percent.value;
    p.substeps = substeps.value;
    p.sleep_speed = sleep_speed.value;
    p.tumble = tumble.value;
    p.boundary_collision = boundary_collision.value;
    return p;
}

void hashParams(std::uint64_t& hash, const ObjectParams& p) {
    hashValue(hash, p.gravity); hashValue(hash, p.gravity_scale);
    hashValue(hash, p.friction); hashValue(hash, p.restitution); hashValue(hash, p.density);
    hashValue(hash, p.initial_speed); hashValue(hash, p.initial_angle);
    hashValue(hash, p.angular_velocity); hashValue(hash, p.tumble);
    hashValue(hash, p.boundary_collision);
    hashValue(hash, p.region_center_x); hashValue(hash, p.region_center_y);
    hashValue(hash, p.region_width_percent); hashValue(hash, p.region_height_percent);
    hashValue(hash, p.substeps); hashValue(hash, p.sleep_speed);
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

PhysicsRegion makePhysicsRegion(const SCENE_INFO& scene, const ObjectParams& params) {
    const double width_percent = std::clamp(params.region_width_percent, 1.0, 1000.0);
    const double height_percent = std::clamp(params.region_height_percent, 1.0, 1000.0);
    return {
        params.region_center_x,
        params.region_center_y,
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

std::wstring readTextItemOf(EDIT_SECTION* edit, OBJECT_HANDLE object, LPCWSTR item) {
    if (!edit || !object || !edit->get_object_item_value) return {};
    return utf8ToWide(edit->get_object_item_value(object, L"テキスト", item));
}

std::wstring readTextItem(FILTER_PROC_VIDEO* video, LPCWSTR item) {
    if (!video || !video->edit || !video->edit->find_object) return {};
    const OBJECT_HANDLE object = video->edit->find_object(video->object->layer,
                                                           video->object->frame_s);
    return readTextItemOf(video->edit, object, item);
}

// Settings of this filter on another object, as strings from its alias data.
double readNumberItem(EDIT_SECTION* edit, OBJECT_HANDLE object, LPCWSTR item, double fallback) {
    const char* value = edit->get_object_item_value(object, kFilterName, item);
    if (!value || !*value) return fallback;
    char* end = nullptr;
    const double parsed = std::strtod(value, &end);
    return end == value ? fallback : parsed;
}

bool readBoolItem(EDIT_SECTION* edit, OBJECT_HANDLE object, LPCWSTR item, bool fallback) {
    const char* value = edit->get_object_item_value(object, kFilterName, item);
    if (!value || !*value) return fallback;
    return std::strcmp(value, "0") != 0 && std::strcmp(value, "false") != 0;
}

int readSplitMode(EDIT_SECTION* edit, OBJECT_HANDLE object, int fallback) {
    const char* value = edit->get_object_item_value(object, kFilterName, split_mode.name);
    if (!value || !*value) return fallback;
    if (std::strcmp(value, "1") == 0) return 1;
    return utf8ToWide(value) == L"文字内のαパーツ" ? 1 : 0;
}

ObjectParams readParams(EDIT_SECTION* edit, OBJECT_HANDLE object, const ObjectParams& base) {
    ObjectParams p = base;
    p.gravity = readNumberItem(edit, object, gravity.name, base.gravity);
    p.gravity_scale = readNumberItem(edit, object, gravity_scale.name, base.gravity_scale);
    p.friction = readNumberItem(edit, object, friction.name, base.friction);
    p.restitution = readNumberItem(edit, object, restitution.name, base.restitution);
    p.density = readNumberItem(edit, object, density.name, base.density);
    p.initial_speed = readNumberItem(edit, object, initial_speed.name, base.initial_speed);
    p.initial_angle = readNumberItem(edit, object, initial_angle.name, base.initial_angle);
    p.angular_velocity = readNumberItem(edit, object, angular_velocity.name, base.angular_velocity);
    p.region_center_x = readNumberItem(edit, object, region_center_x.name, base.region_center_x);
    p.region_center_y = readNumberItem(edit, object, region_center_y.name, base.region_center_y);
    p.region_width_percent = readNumberItem(edit, object, region_width_percent.name,
                                            base.region_width_percent);
    p.region_height_percent = readNumberItem(edit, object, region_height_percent.name,
                                             base.region_height_percent);
    p.substeps = readNumberItem(edit, object, substeps.name, base.substeps);
    p.sleep_speed = readNumberItem(edit, object, sleep_speed.name, base.sleep_speed);
    p.tumble = readBoolItem(edit, object, tumble.name, base.tumble);
    p.boundary_collision = readBoolItem(edit, object, boundary_collision.name,
                                        base.boundary_collision);
    return p;
}

// Rough advance width of a character relative to the font size. Full-width
// characters are 1.0; Latin text is narrower and varies from letter to letter.
double advanceWeight(wchar_t c) {
    if (c == L' ') return 0.3;
    if (c == 0x3000) return 1.0;
    if (c >= 0xff61 && c <= 0xff9f) return 0.5;
    if (c >= 0x1100 && (c <= 0x115f || (c >= 0x2e80 && c <= 0xa4cf) ||
        (c >= 0xac00 && c <= 0xd7a3) || (c >= 0xf900 && c <= 0xfaff) ||
        (c >= 0xfe30 && c <= 0xfe6f) || (c >= 0xff00 && c <= 0xff60) ||
        (c >= 0xffe0 && c <= 0xffe6) || (c >= 0xd800 && c <= 0xdfff))) return 1.0;
    if (std::wcschr(L"iljI.,:;!|'`", c)) return 0.3;
    if (std::wcschr(L"frt()[]{}-/\\", c)) return 0.4;
    if (std::wcschr(L"mwMW@%", c)) return 0.85;
    if (c < 0x80) return 0.55;
    return 1.0;
}

// Expected horizontal layout of one line of text, used to tell which character
// each piece of ink belongs to.
struct LineLayout {
    std::vector<double> weights;
    int first = 0;
    int last = -1;
    double total = 0.0;

    // Index of the character at x, given the ink extent [left, right] of the line.
    int column(double x, double left, double right) const {
        if (weights.empty()) return 0;
        if (last < first || right <= left) return std::max(first, 0);
        const double position = std::clamp((x - left) / (right - left), 0.0, 1.0) * total;
        double cumulative = 0.0;
        for (int i = first; i <= last; ++i) {
            cumulative += weights[i];
            if (position < cumulative) return i;
        }
        return last;
    }
};

std::vector<LineLayout> lineLayouts(const std::wstring& text) {
    std::vector<LineLayout> layouts(1);
    std::vector<std::vector<bool>> blanks(1);
    for (std::size_t i = 0; i < text.size(); ++i) {
        const wchar_t character = text[i];
        if (character == L'\r') continue;
        if (character == L'\n') {
            layouts.emplace_back();
            blanks.emplace_back();
            continue;
        }
        if (character >= 0xd800 && character <= 0xdbff && i + 1 < text.size() &&
            text[i + 1] >= 0xdc00 && text[i + 1] <= 0xdfff) ++i;
        layouts.back().weights.push_back(advanceWeight(character));
        blanks.back().push_back(character == L' ' || character == 0x3000 || character == L'\t');
    }
    for (std::size_t line = 0; line < layouts.size(); ++line) {
        auto& layout = layouts[line];
        const int count = static_cast<int>(layout.weights.size());
        layout.first = 0;
        layout.last = count - 1;
        while (layout.first < count && blanks[line][layout.first]) ++layout.first;
        while (layout.last >= layout.first && blanks[line][layout.last]) --layout.last;
        for (int i = layout.first; i <= layout.last; ++i) layout.total += layout.weights[i];
    }
    return layouts;
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
        const auto layouts = lineLayouts(text);
        std::vector<int> lengths;
        for (const auto& layout : layouts) lengths.push_back(static_cast<int>(layout.weights.size()));
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
        const double occupied_height = std::max(1, bottom - top + 1);
        std::vector<int> line_of(components.size(), 0);
        std::vector<double> line_left(lengths.size(), static_cast<double>(width));
        std::vector<double> line_right(lengths.size(), -1.0);
        for (std::size_t i = 0; i < components.size(); ++i) {
            const auto& component = components[i];
            const double center_y = (component.min_y + component.max_y + 1) * 0.5;
            int line = std::clamp(static_cast<int>((center_y - top) *
                                  lengths.size() / occupied_height),
                                  0, static_cast<int>(lengths.size()) - 1);
            while (line < static_cast<int>(lengths.size()) && lengths[line] == 0) ++line;
            if (line >= static_cast<int>(lengths.size())) {
                line = static_cast<int>(lengths.size()) - 1;
                while (line > 0 && lengths[line] == 0) --line;
            }
            line_of[i] = line;
            line_left[line] = std::min(line_left[line], static_cast<double>(component.min_x));
            line_right[line] = std::max(line_right[line], static_cast<double>(component.max_x + 1));
        }
        // Each line is split by its own ink extent, so lines of different length
        // or alignment do not disturb each other.
        for (std::size_t i = 0; i < components.size(); ++i) {
            const auto& component = components[i];
            const int line = line_of[i];
            const double center_x = (component.min_x + component.max_x + 1) * 0.5;
            const int column = layouts[line].column(center_x, line_left[line], line_right[line]);
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

void addTerrainBodies(b2WorldId world, const TerrainSource& terrain,
                      double body_friction, double bounce) {
    for (const auto& piece : terrain.pieces) {
        b2BodyDef body_definition = b2DefaultBodyDef();
        body_definition.type = b2_staticBody;
        body_definition.position = pieceWorldPosition(piece, terrain.pose);
        body_definition.rotation = b2MakeRot(angleToBox(terrain.pose.rz));
        const b2BodyId body = b2CreateBody(world, &body_definition);
        b2ShapeDef shape_definition = b2DefaultShapeDef();
        shape_definition.material.friction = static_cast<float>(body_friction);
        shape_definition.material.restitution = static_cast<float>(bounce);
        createPieceShape(body, piece, shape_definition, terrain.pose);
    }
}

// Set while this thread reads another layer's image so a nested call of the
// filter passes straight through instead of touching any simulation.
thread_local bool fetching_layer_image = false;

bool fetchLayerImage(FILTER_PROC_VIDEO* video, int layer,
                     std::vector<PIXEL_RGBA>& pixels, int& width, int& height) {
    if (!video->copy_image_resource || !video->get_image_resource_size ||
        !video->get_image_resource_data || !video->release_image_resource) return false;
    struct Guard {
        Guard() { fetching_layer_image = true; }
        ~Guard() { fetching_layer_image = false; }
    } guard;
    const std::wstring resource = L"resource:text_physics_fetch_" +
        std::to_wstring(video->object->effect_id) + L"_" + std::to_wstring(layer);
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

std::uint64_t hashAlias(EDIT_SECTION* edit, OBJECT_HANDLE object) {
    std::uint64_t hash = 1469598103934665603ull;
    const char* alias = edit->get_object_alias ? edit->get_object_alias(object) : nullptr;
    if (alias) {
        for (const char* c = alias; *c; ++c) hashValue(hash, *c);
    }
    return hash;
}

void logNeighbor(const wchar_t* kind, int layer, std::size_t pieces) {
    const std::wstring message = std::wstring(L"[TextPhysicsFilter] ") + kind +
        L" layer=" + std::to_wstring(layer) + L" pieces=" + std::to_wstring(pieces) + L"\n";
    OutputDebugStringW(message.c_str());
}

// Metadata about a text object that carries this filter, found by scanning layers.
struct ScanItem {
    int layer;
    OBJECT_HANDLE handle;
    int frame_start;
    int frame_end;
    std::uint64_t alias_hash;
};

// Settings of such an object. Re-read only when its alias data changes.
struct ObjectInfo {
    bool is_static = false;
    int group = 0;
    int threshold = 1;
    int mode = 0;
    ObjectParams params;
    OBJECT_IMAGE_PARAM start{};
    std::wstring text;
};

struct ObjectKey {
    int layer;
    int frame_start;
    int frame_end;
    std::uint64_t alias_hash;
    bool operator<(const ObjectKey& other) const {
        return std::tie(layer, frame_start, frame_end, alias_hash) <
               std::tie(other.layer, other.frame_start, other.frame_end, other.alias_hash);
    }
};

// Shared by every instance in the process. All instances of one render pass see
// the same scene, so the layer scan is done once per pass instead of once each.
std::mutex shared_mutex;
struct SharedScan {
    std::int64_t rendering_id = 0;
    int scene_id = -1;
    int frame = -1;
    std::vector<ScanItem> items;
} shared_scan;
std::map<ObjectKey, std::shared_ptr<const ObjectInfo>> object_infos;

std::vector<ScanItem> scanObjects(FILTER_PROC_VIDEO* video, int now_frame) {
    const std::int64_t pass = video->scene->rendering_id;
    if (pass != 0) {
        std::lock_guard<std::mutex> lock(shared_mutex);
        if (shared_scan.rendering_id == pass && shared_scan.scene_id == video->scene->id &&
            shared_scan.frame == now_frame) return shared_scan.items;
    }
    std::vector<ScanItem> items;
    EDIT_SECTION* edit = video->edit;
    if (edit && video->get_image_object && edit->get_object_item_value &&
        edit->count_object_effect && edit->get_object_layer_frame) {
        const int layer_max = edit->info
            ? std::clamp(edit->info->layer_max, 0, kMaximumTerrainLayers - 1)
            : kMaximumTerrainLayers - 1;
        for (int layer = 0; layer <= layer_max; ++layer) {
            const OBJECT_HANDLE object = video->get_image_object(layer, 0.0);
            if (!object || edit->count_object_effect(object, kFilterName) <= 0) continue;
            const OBJECT_LAYER_FRAME range = edit->get_object_layer_frame(object);
            items.push_back({layer, object, range.start, range.end, hashAlias(edit, object)});
        }
    }
    if (pass != 0) {
        std::lock_guard<std::mutex> lock(shared_mutex);
        shared_scan = {pass, video->scene->id, now_frame, items};
    }
    return items;
}

std::shared_ptr<const ObjectInfo> getObjectInfo(FILTER_PROC_VIDEO* video, const ScanItem& item,
                                                int now_frame) {
    const ObjectKey key{item.layer, item.frame_start, item.frame_end, item.alias_hash};
    {
        std::lock_guard<std::mutex> lock(shared_mutex);
        const auto found = object_infos.find(key);
        if (found != object_infos.end()) return found->second;
    }
    EDIT_SECTION* edit = video->edit;
    auto info = std::make_shared<ObjectInfo>();
    info->is_static = readBoolItem(edit, item.handle, kStaticItemName, false);
    info->group = static_cast<int>(std::lround(
        readNumberItem(edit, item.handle, collision_group.name, 0.0)));
    info->threshold = std::clamp(static_cast<int>(std::lround(
        readNumberItem(edit, item.handle, alpha_threshold.name, 1.0))), 1, 255);
    info->mode = readSplitMode(edit, item.handle, 0);
    info->params = readParams(edit, item.handle, ObjectParams{});
    info->text = readTextItemOf(edit, item.handle, L"テキスト");
    info->start.sx = info->start.sy = info->start.sz = 1.0f;
    info->start.alpha = 1.0f;
    const double offset = (item.frame_start - now_frame) *
        (static_cast<double>(video->scene->scale) / video->scene->rate);
    if (video->get_output_image_param &&
        !video->get_output_image_param(item.handle, offset, &info->start, sizeof(info->start))) {
        video->get_output_image_param(item.handle, 0.0, &info->start, sizeof(info->start));
    }
    std::lock_guard<std::mutex> lock(shared_mutex);
    if (object_infos.size() > 512) object_infos.clear();
    object_infos[key] = info;
    return info;
}

// Sorts the other text objects into terrain (static ones) and, when this object has
// a collision group, members of the world. Pixels are read back only when an
// object's alias data changes.
void refreshNeighbors(Runtime& runtime, FILTER_PROC_VIDEO* video, int group, int now_frame) {
    std::vector<TerrainSource> terrains;
    std::vector<Neighbor> members;
    const auto fetchPieces = [&](int layer, int piece_threshold, int piece_mode,
                                 const std::wstring& text) {
        std::vector<Piece> pieces;
        std::vector<PIXEL_RGBA> pixels;
        int width = 0, height = 0;
        if (fetchLayerImage(video, layer, pixels, width, height)) {
            pieces = buildPieces(pixels, width, height, piece_threshold, piece_mode, text);
            for (auto& piece : pieces) {
                piece.pixels.clear();
                piece.pixels.shrink_to_fit();
            }
        }
        return pieces;
    };
    for (const auto& item : scanObjects(video, now_frame)) {
        if (item.layer == video->object->layer || item.handle == video->object->handle) continue;
        const auto info = getObjectInfo(video, item, now_frame);
        if (!info->is_static && (group <= 0 || info->group != group)) continue;
        std::uint64_t signature = item.alias_hash;
        hashValue(signature, item.layer);
        hashValue(signature, item.frame_start);
        hashValue(signature, item.frame_end);
        hashValue(signature, info->threshold);
        hashValue(signature, info->mode);
        if (info->is_static) {
            TerrainSource terrain;
            terrain.layer = item.layer;
            terrain.signature = signature;
            terrain.pose = info->start;
            const auto cached = std::find_if(runtime.terrains.begin(), runtime.terrains.end(),
                [&](const TerrainSource& old) {
                    return old.layer == item.layer && old.signature == signature;
                });
            if (cached != runtime.terrains.end()) {
                terrain.pieces = std::move(cached->pieces);
            } else {
                terrain.pieces = fetchPieces(item.layer, info->threshold, 1, {});
                logNeighbor(L"terrain", item.layer, terrain.pieces.size());
            }
            terrains.push_back(std::move(terrain));
            continue;
        }
        Neighbor member;
        member.layer = item.layer;
        member.frame_start = item.frame_start;
        member.frame_end = item.frame_end;
        member.signature = signature;
        member.start = info->start;
        member.params = info->params;
        const auto cached = std::find_if(runtime.members.begin(), runtime.members.end(),
            [&](const Neighbor& old) {
                return old.layer == item.layer && old.signature == signature;
            });
        if (cached != runtime.members.end()) {
            member.pieces = std::move(cached->pieces);
        } else {
            member.pieces = fetchPieces(item.layer, info->threshold, info->mode, info->text);
            logNeighbor(L"member", item.layer, member.pieces.size());
        }
        members.push_back(std::move(member));
    }
    runtime.terrains = std::move(terrains);
    runtime.members = std::move(members);
}

// One text object taking part in the simulation: this instance's own object or a
// member of its collision group.
struct SimEntry {
    int layer;
    int frame_start;
    int frame_end;
    bool self;
    const OBJECT_IMAGE_PARAM* start;
    const ObjectParams* params;
    std::vector<Piece>* pieces;
};

void spawnEntry(b2WorldId world, const SimEntry& entry) {
    const ObjectParams& params = *entry.params;
    const OBJECT_IMAGE_PARAM& start = *entry.start;
    const double bounce = std::clamp(params.restitution, 0.0, 1.0);
    const double launch_radians = params.initial_angle * kPi / 180.0;
    const double velocity_x = std::sin(launch_radians) * params.initial_speed;
    const double velocity_y = -std::cos(launch_radians) * params.initial_speed;
    for (std::size_t index = 0; index < entry.pieces->size(); ++index) {
        auto& piece = (*entry.pieces)[index];
        const bool tumble_enabled = params.tumble;
        b2BodyDef body_definition = b2DefaultBodyDef();
        body_definition.type = b2_dynamicBody;
        body_definition.position = pieceWorldPosition(piece, start);
        const double rotation = start.rz + (tumble_enabled ? tumbleAngle(
            static_cast<int>(index)) : 0.0);
        body_definition.rotation = b2MakeRot(angleToBox(rotation));
        body_definition.linearVelocity = vectorToBox(velocity_x, velocity_y);
        const double configured_angular_velocity = std::abs(params.angular_velocity) >= 0.001
            ? params.angular_velocity
            : (tumble_enabled ? tumbleVelocity(static_cast<int>(index)) : 0.0);
        body_definition.angularVelocity = angleToBox(configured_angular_velocity);
        body_definition.angularDamping = tumble_enabled ? 2.0f : 4.0f;
        body_definition.motionLocks.angularZ = !tumble_enabled &&
                                               std::abs(params.angular_velocity) < 0.001;
        body_definition.gravityScale = static_cast<float>(params.gravity_scale);
        body_definition.sleepThreshold = meters(std::max(0.0, params.sleep_speed));
        const b2BodyId body = b2CreateBody(world, &body_definition);

        b2ShapeDef shape_definition = b2DefaultShapeDef();
        shape_definition.material.friction = static_cast<float>(params.friction);
        shape_definition.material.restitution = static_cast<float>(bounce);
        shape_definition.density = static_cast<float>(params.density);
        createPieceShape(body, piece, shape_definition, start);
        b2Body_UpdateMassFromShapes(body);
        piece.body = body;
    }
}

void removeEntry(const SimEntry& entry) {
    for (auto& piece : *entry.pieces) {
        if (B2_IS_NON_NULL(piece.body) && b2Body_IsValid(piece.body)) b2DestroyBody(piece.body);
        piece.body = b2_nullBodyId;
    }
}

// Entries are in spawn order. The world clock is the scene frame; every entry
// appears at its own start frame and members disappear after their last frame.
bool resetSimulation(Runtime& runtime, const std::vector<SimEntry>& entries,
                     const ObjectParams& world_params, const PhysicsRegion& region,
                     std::uint64_t key, int world_start) {
    runtime.destroyWorld();
    b2WorldDef world_definition = b2DefaultWorldDef();
    world_definition.gravity = {0.0f, -meters(world_params.gravity)};
    runtime.world = b2CreateWorld(&world_definition);
    if (B2_IS_NULL(runtime.world)) return false;

    const double bounce = std::clamp(world_params.restitution, 0.0, 1.0);
    if (world_params.boundary_collision) {
        constexpr double thickness = kPixelsPerMeter;
        createBox(runtime.world, region.center_x,
                  region.bottom() + thickness * 0.5,
                  region.width + thickness * 2.0, thickness,
                  false, world_params.friction, bounce, 1.0);
        createBox(runtime.world, region.center_x,
                  region.top() - thickness * 0.5,
                  region.width + thickness * 2.0, thickness,
                  false, world_params.friction, bounce, 1.0);
        createBox(runtime.world, region.left() - thickness * 0.5,
                  region.center_y, thickness, region.height,
                  false, world_params.friction, bounce, 1.0);
        createBox(runtime.world, region.right() + thickness * 0.5,
                  region.center_y, thickness, region.height,
                  false, world_params.friction, bounce, 1.0);
    }

    for (const auto& terrain : runtime.terrains) {
        addTerrainBodies(runtime.world, terrain, world_params.friction, bounce);
    }
    for (const auto& entry : entries) {
        if (entry.frame_start == world_start) spawnEntry(runtime.world, entry);
    }
    runtime.simulation_valid = true;
    runtime.simulation_key = key;
    runtime.cached_frame = world_start;
    return true;
}

void advanceSimulation(Runtime& runtime, const std::vector<SimEntry>& entries,
                       const ObjectParams& world_params, int target_frame,
                       double frame_seconds) {
    const int sub_steps = std::clamp(static_cast<int>(std::lround(world_params.substeps)), 1, 16);
    for (int frame = runtime.cached_frame; frame < target_frame; ++frame) {
        b2World_Step(runtime.world, static_cast<float>(frame_seconds), sub_steps);
        const int next = frame + 1;
        for (const auto& entry : entries) {
            if (entry.frame_start == next) spawnEntry(runtime.world, entry);
            else if (!entry.self && entry.frame_end + 1 == next) removeEntry(entry);
        }
    }
    runtime.cached_frame = std::max(runtime.cached_frame, target_frame);
}

bool processVideo(FILTER_PROC_VIDEO* video) {
    if (!video || !video->scene || !video->object || video->scene->rate <= 0 ||
        !video->get_image_data || !video->create_image_resource || !video->draw_image) return true;
    auto* runtime = static_cast<Runtime*>(video->userdata);
    if (!runtime || fetching_layer_image) return true;
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
        const std::wstring built = std::wstring(L"[TextPhysicsFilter] pieces mode=") +
            std::to_wstring(mode) + L" text_length=" + std::to_wstring(text.size()) +
            L" pieces=" + std::to_wstring(runtime->pieces.size()) + L"\n";
        OutputDebugStringW(built.c_str());
        runtime->pieces_valid = true;
        runtime->image_width = width;
        runtime->image_height = height;
        runtime->threshold = threshold;
        runtime->mode = mode;
        runtime->text = text;
        runtime->font = font;
        runtime->destroyWorld();
    }
    const ObjectParams own = ownParams();
    if (runtime->pieces.empty()) {
        if (visualize_region.value) {
            OBJECT_IMAGE_PARAM current{};
            current.sx = current.sy = current.sz = 1.0f;
            current.alpha = 1.0f;
            if (video->get_output_image_param) {
                video->get_output_image_param(nullptr, 0.0, &current, sizeof(current));
            }
            drawPhysicsRegion(video, makePhysicsRegion(*video->scene, own),
                              makeObjectTransform(current));
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
    const int group = std::clamp(static_cast<int>(std::lround(collision_group.value)), 0, 99);
    const int frame = std::max(0, video->object->frame);
    const int now_frame = video->object->frame_s + frame;
    refreshNeighbors(*runtime, video, group, now_frame);

    std::vector<SimEntry> entries;
    entries.push_back({video->object->layer, video->object->frame_s, video->object->frame_e,
                       true, &start, &own, &runtime->pieces});
    for (auto& member : runtime->members) {
        entries.push_back({member.layer, member.frame_start, member.frame_end,
                           false, &member.start, &member.params, &member.pieces});
    }
    std::sort(entries.begin(), entries.end(), [](const SimEntry& a, const SimEntry& b) {
        return a.frame_start != b.frame_start ? a.frame_start < b.frame_start : a.layer < b.layer;
    });
    // The group leader (lowest layer) decides the world level settings.
    const SimEntry* leader = &entries.front();
    for (const auto& entry : entries) {
        if (entry.layer < leader->layer) leader = &entry;
    }
    const ObjectParams& world_params = *leader->params;
    const PhysicsRegion region = makePhysicsRegion(*video->scene, world_params);
    const int world_start = entries.front().frame_start;

    std::uint64_t key = 1469598103934665603ull;
    hashValue(key, width); hashValue(key, height);
    hashValue(key, video->scene->width); hashValue(key, video->scene->height);
    hashValue(key, threshold); hashValue(key, mode);
    hashParams(key, world_params);
    for (const auto& entry : entries) {
        hashValue(key, entry.layer);
        hashValue(key, entry.frame_start);
        hashValue(key, entry.frame_end);
        hashValue(key, entry.self);
        hashValue(key, *entry.start);
        hashParams(key, *entry.params);
        hashValue(key, entry.pieces->size());
    }
    for (const auto& member : runtime->members) hashValue(key, member.signature);
    for (const auto& terrain : runtime->terrains) {
        hashValue(key, terrain.layer);
        hashValue(key, terrain.signature);
    }
    for (const wchar_t character : text) hashValue(key, character);

    if (!runtime->simulation_valid || runtime->simulation_key != key ||
        now_frame < runtime->cached_frame) {
        if (!resetSimulation(*runtime, entries, world_params, region, key, world_start)) {
            return true;
        }
    }
    advanceSimulation(*runtime, entries, world_params, now_frame,
                      static_cast<double>(video->scene->scale) / video->scene->rate);

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
    L"Standalone Text Decomposition Physics version 0.1.4-group (Box2D 3.2.0)",
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
