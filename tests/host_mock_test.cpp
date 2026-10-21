// Headless test: mocks the AviUtl2 host so the simulation logic runs without AviUtl2.
// Linux only; tests/stub/windows.h stands in for the Windows headers.
#include <cstdio>
#include <map>
#include <string>
#include <vector>

#include "../TextPhysicsFilter.cpp"

int MultiByteToWideChar(unsigned, unsigned long, const char* s, int n, wchar_t* out, int outn) {
    std::wstring w;
    for (const unsigned char* c = reinterpret_cast<const unsigned char*>(s); *c; ++c) w.push_back(*c);
    if (out && outn > 0) { for (size_t i = 0; i < w.size() && (int)i < outn; ++i) out[i] = w[i]; out[w.size()] = 0; }
    return (int)w.size() + 1;
}
void OutputDebugStringW(const wchar_t* m) { std::wprintf(L"%ls", m); }

struct MockObject {
    int layer, frame_s, frame_e, w, h;
    float x, y;
    std::map<std::wstring, std::string> items;
    std::vector<PIXEL_RGBA> image;
    void* runtime = nullptr;
};

static std::vector<MockObject*> g_objects;
static MockObject* g_current = nullptr;
static int g_frame = 0;
static std::map<std::wstring, std::pair<std::vector<PIXEL_RGBA>, std::pair<int, int>>> g_resources;
static std::vector<std::pair<float, float>> g_draws;
static std::string g_string_buffer;
static EDIT_SECTION g_edit;
static EDIT_INFO g_info;
static SCENE_INFO g_scene;
static OBJECT_INFO g_object_info;

static MockObject* objectAtLayer(int layer) {
    for (auto* o : g_objects) if (o->layer == layer && g_frame >= o->frame_s && g_frame <= o->frame_e) return o;
    return nullptr;
}

static void fillParam(const MockObject* o, OBJECT_IMAGE_PARAM* p) {
    p->x = o->x; p->y = o->y; p->z = 0; p->rx = p->ry = p->rz = 0;
    p->sx = p->sy = p->sz = 1; p->cx = p->cy = p->cz = 0; p->alpha = 1;
}

static FILTER_PROC_VIDEO makeVideo(MockObject* o) {
    g_current = o;
    FILTER_PROC_VIDEO v{};
    g_scene = {1920, 1080, 60, 1, 48000, 0, 0, 0, 0};
    g_object_info = {};
    g_object_info.id = o->layer; g_object_info.frame = g_frame - o->frame_s;
    g_object_info.time = (g_frame - o->frame_s) / 60.0;
    g_object_info.width = o->w; g_object_info.height = o->h;
    g_object_info.effect_id = 100 + o->layer; g_object_info.layer = o->layer;
    g_object_info.frame_s = o->frame_s; g_object_info.frame_e = o->frame_e;
    g_object_info.handle = o;
    v.scene = &g_scene; v.object = &g_object_info; v.userdata = o->runtime; v.edit = &g_edit;
    v.get_image_data = [](PIXEL_RGBA* b) { std::copy(g_current->image.begin(), g_current->image.end(), b); };
    v.create_image_resource = [](LPCWSTR, const PIXEL_RGBA*, int, int) {};
    v.draw_image = [](LPCWSTR, float x, float y, float, float, float, float, float, float, float, float) {
        g_draws.push_back({x, y}); return true; };
    v.set_default_anchor = [](int, int) {};
    v.get_output_image_param = [](OBJECT_HANDLE h, double, OBJECT_IMAGE_PARAM* p, int) {
        fillParam(h ? (MockObject*)h : g_current, p); return true; };
    v.get_image_object = [](int layer, double) -> OBJECT_HANDLE { return objectAtLayer(layer); };
    v.copy_image_resource = [](LPCWSTR dst, LPCWSTR src) {
        std::wstring s = src; if (s.rfind(L"layer:", 0) != 0) return false;
        MockObject* o = objectAtLayer(std::stoi(s.substr(6)));
        if (!o) return false;
        g_resources[dst] = {o->image, {o->w, o->h}}; return true; };
    v.get_image_resource_size = [](LPCWSTR r, int* w, int* h) {
        auto it = g_resources.find(r); if (it == g_resources.end()) return false;
        *w = it->second.second.first; *h = it->second.second.second; return true; };
    v.get_image_resource_data = [](LPCWSTR r, void* b, int, int, int, OUTPUT_PIXEL_FORMAT) {
        auto& px = g_resources[r].first; std::memcpy(b, px.data(), px.size() * sizeof(PIXEL_RGBA)); return true; };
    v.release_image_resource = [](LPCWSTR r) { g_resources.erase(r); return true; };
    return v;
}

static void setupEdit() {
    g_info = {}; g_info.layer_max = 3;
    g_edit.info = &g_info;
    g_edit.find_object = [](int layer, int) -> OBJECT_HANDLE { return objectAtLayer(layer); };
    g_edit.count_object_effect = [](OBJECT_HANDLE, LPCWSTR e) { return std::wstring(e) == kFilterName ? 1 : 0; };
    g_edit.get_object_layer_frame = [](OBJECT_HANDLE h) {
        auto* o = (MockObject*)h; return OBJECT_LAYER_FRAME{o->layer, o->frame_s, o->frame_e}; };
    g_edit.get_object_alias = [](OBJECT_HANDLE h) -> LPCSTR {
        static std::string a; a = "alias" + std::to_string(((MockObject*)h)->layer); return a.c_str(); };
    g_edit.get_object_item_value = [](OBJECT_HANDLE h, LPCWSTR, LPCWSTR item) -> LPCSTR {
        auto* o = (MockObject*)h; auto it = o->items.find(item);
        if (it == o->items.end()) return nullptr; g_string_buffer = it->second; return g_string_buffer.c_str(); };
}

static MockObject* makeBox(int layer, float x, float y, int group, bool is_static, int frame_s = 0) {
    auto* o = new MockObject{layer, frame_s, 100000, 100, 100, x, y};
    o->image.assign(100 * 100, PIXEL_RGBA{255, 255, 255, 255});
    o->items[collision_group.name] = std::to_string(group);
    o->items[static_object.name] = is_static ? "1" : "0";
    o->runtime = createRuntime(0);
    g_objects.push_back(o);
    return o;
}

// Renders one object at g_frame the way the host would and returns its world y (centre).
static float render(MockObject* o) {
    g_draws.clear();
    FILTER_PROC_VIDEO v = makeVideo(o);
    static_object.value = o->items[static_object.name] == "1";
    collision_group.value = std::stod(o->items[collision_group.name]);
    tumble.value = false;
    processVideo(&v);
    return g_draws.empty() ? -9999.0f : g_draws[0].second + o->y;
}

static void cleanup() {
    for (auto* o : g_objects) { destroyRuntime(0, o->runtime); delete o; }
    g_objects.clear();
}

int main() {
    setupEdit();
    int failures = 0;
    auto check = [&](const char* name, bool ok, float a, float b) {
        std::printf("%s %-46s (%.1f, %.1f)\n", ok ? "PASS" : "FAIL", name, a, b); if (!ok) ++failures; };

    // floor top is at y = 540. A box (100px) resting on the floor has centre y = 490.
    for (int pass = 0; pass < 2; ++pass) {
        const int group = pass == 0 ? 0 : 1;
        MockObject* a = makeBox(0, 0, 400, group, false);
        MockObject* b = makeBox(1, 0, 0, group, false);
        float ya = 0, yb = 0;
        for (g_frame = 0; g_frame <= 300; ++g_frame) { ya = render(a); yb = render(b); }
        if (group == 0) check("group 0: both fall to the floor independently", std::abs(ya - 490) < 3 && std::abs(yb - 490) < 3, ya, yb);
        else check("group 1: B lands on top of A", std::abs(ya - 490) < 3 && std::abs(yb - 390) < 3, ya, yb);
        cleanup();
    }

    // Two renders of the same frame by different instances must agree (A rendered first vs second).
    {
        MockObject* a = makeBox(0, 0, 400, 1, false);
        MockObject* b = makeBox(1, 0, 0, 1, false);
        float ya = 0, yb = 0;
        for (g_frame = 0; g_frame <= 200; ++g_frame) { yb = render(b); ya = render(a); }
        check("group 1: render order B then A", std::abs(ya - 490) < 3 && std::abs(yb - 390) < 3, ya, yb);
        // seek backwards and forwards again
        g_frame = 40; render(a); render(b);
        g_frame = 200; ya = render(a); yb = render(b);
        check("group 1: seek back then forward", std::abs(ya - 490) < 3 && std::abs(yb - 390) < 3, ya, yb);
        cleanup();
    }

    // Different start frames: B appears at frame 60 above A and still lands on it.
    {
        MockObject* a = makeBox(0, 0, 400, 1, false, 0);
        MockObject* b = makeBox(1, 0, 0, 1, false, 60);
        float ya = 0, yb = 0;
        for (g_frame = 0; g_frame <= 300; ++g_frame) { ya = render(a); if (g_frame >= 60) yb = render(b); }
        check("group 1: staggered start frames", std::abs(ya - 490) < 3 && std::abs(yb - 390) < 3, ya, yb);
        cleanup();
    }

    // Static terrain still works: static A at y=400, dynamic B falls on it.
    {
        MockObject* a = makeBox(0, 0, 400, 0, true);
        MockObject* b = makeBox(1, 0, 0, 0, false);
        float yb = 0;
        for (g_frame = 0; g_frame <= 300; ++g_frame) { render(a); yb = render(b); }
        check("static terrain: B rests on static A", std::abs(yb - 300) < 3, yb, 300.0f);
        cleanup();
    }
    std::printf(failures ? "FAILED\n" : "ALL PASSED\n");
    return failures ? 1 : 0;
}
