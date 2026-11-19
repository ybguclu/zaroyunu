#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

// ============================================================
// 0. MOTOR SABITLERI (Engine::Config) - FULL HD (1920x1080)
// ============================================================
namespace Engine::Config {
    constexpr float PI = 3.14159265358979f;
    constexpr int SCREEN_W = 1920;
    constexpr int SCREEN_H = 1080;
}

// ============================================================
// 1. CIFT CEKIRDEK PARALEL ISLEM MOTORU (Engine::Parallel)
// ============================================================
namespace Engine::Parallel {
    class DualCoreWorker {
        std::thread worker;
        std::mutex mtx;
        std::condition_variable cvStart;
        std::condition_variable cvDone;
        std::function<void()> currentTask;
        bool hasWork = false;
        bool stopWorker = false;

        void threadLoop() {
            while (true) {
                std::function<void()> task;
                {
                    std::unique_lock<std::mutex> lock(mtx);
                    cvStart.wait(lock, [&] { return hasWork || stopWorker; });
                    if (stopWorker) return;
                    task = currentTask;
                }

                task();

                {
                    std::lock_guard<std::mutex> lock(mtx);
                    hasWork = false;
                }
                cvDone.notify_one();
            }
        }

    public:
        DualCoreWorker() {
            worker = std::thread(&DualCoreWorker::threadLoop, this);
        }

        ~DualCoreWorker() {
            shutdown();
        }

        template <typename F1, typename F2>
        void runParallel(F1&& core1Task, F2&& core2Task) {
            {
                std::lock_guard<std::mutex> lock(mtx);
                currentTask = std::forward<F2>(core2Task);
                hasWork = true;
            }
            cvStart.notify_one();

            core1Task();

            std::unique_lock<std::mutex> lock(mtx);
            cvDone.wait(lock, [&] { return !hasWork; });
        }

        void shutdown() {
            {
                std::lock_guard<std::mutex> lock(mtx);
                if (stopWorker) return;
                stopWorker = true;
            }
            cvStart.notify_one();
            if (worker.joinable()) worker.join();
        }
    };
}

// ============================================================
// 2. 3D MATEMATIK MOTORU (Engine::Math)
// ============================================================
namespace Engine::Math {
    using std::sqrt;
    using std::sin;
    using std::cos;

    struct Vec3 {
        float x = 0, y = 0, z = 0;

        Vec3 operator+(const Vec3& o) const { return { x + o.x, y + o.y, z + o.z }; }
        Vec3 operator-(const Vec3& o) const { return { x - o.x, y - o.y, z - o.z }; }
        Vec3 operator*(float s) const { return { x * s, y * s, z * s }; }
        Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }

        float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
        Vec3 cross(const Vec3& o) const {
            return { y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x };
        }
        float lengthSq() const { return x * x + y * y + z * z; }
        float length() const { return sqrt(lengthSq()); }
        Vec3 normalized() const {
            float l = length();
            return (l > 1e-5f) ? Vec3{ x / l, y / l, z / l } : Vec3{ 0, 0, 0 };
        }
    };

    struct Mat3 {
        float m[3][3]{};
        Vec3 operator*(const Vec3& v) const {
            return {
                m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
            };
        }
    };

    struct Quat {
        float w = 1, x = 0, y = 0, z = 0;

        Quat normalized() const {
            float l = sqrt(w * w + x * x + y * y + z * z);
            return (l > 1e-6f) ? Quat{ w / l, x / l, y / l, z / l } : Quat{ 1, 0, 0, 0 };
        }

        Quat operator*(const Quat& o) const {
            return {
                w * o.w - x * o.x - y * o.y - z * o.z,
                w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x,
                w * o.z + x * o.y - y * o.x + z * o.w
            };
        }

        static Quat nlerp(const Quat& a, Quat b, float t) {
            float d = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
            if (d < 0.0f) { b.w = -b.w; b.x = -b.x; b.y = -b.y; b.z = -b.z; }
            return Quat{
                a.w + (b.w - a.w) * t,
                a.x + (b.x - a.x) * t,
                a.y + (b.y - a.y) * t,
                a.z + (b.z - a.z) * t
            }.normalized();
        }

        Mat3 toMat3() const {
            Mat3 r;
            float xx = x * x, yy = y * y, zz = z * z;
            float xy = x * y, xz = x * z, yz = y * z;
            float wx = w * x, wy = w * y, wz = w * z;

            r.m[0][0] = 1 - 2 * (yy + zz); r.m[0][1] = 2 * (xy - wz);     r.m[0][2] = 2 * (xz + wy);
            r.m[1][0] = 2 * (xy + wz);     r.m[1][1] = 1 - 2 * (xx + zz); r.m[1][2] = 2 * (yz - wx);
            r.m[2][0] = 2 * (xz - wy);     r.m[2][1] = 2 * (yz + wx);     r.m[2][2] = 1 - 2 * (xx + yy);
            return r;
        }
    };
}

// ============================================================
// 3. 3D GRAFIK & CIFT CEKIRDEK RASTERIZER (Engine::Graphics)
// ============================================================
namespace Engine::Graphics {
    using namespace Engine::Config;
    using namespace Engine::Math;
    using namespace Engine::Parallel;
    using std::vector;
    using std::clamp;
    using std::min;
    using std::max;
    using std::abs;
    using std::sin;
    using std::sqrt;
    using std::pow;
    using std::floor;
    using std::ceil;
    using std::swap;

    inline uint32_t makeRGB(int r, int g, int b) {
        return (uint32_t(clamp(r, 0, 255)) << 16) |
            (uint32_t(clamp(g, 0, 255)) << 8) |
            uint32_t(clamp(b, 0, 255));
    }

    inline uint32_t scaleRGB(uint32_t col, float s) {
        return makeRGB(int(((col >> 16) & 0xFF) * s),
            int(((col >> 8) & 0xFF) * s),
            int((col & 0xFF) * s));
    }

    struct Texture {
        int size = 256;
        vector<uint32_t> pixels;

        void generateDiceFace(int value) {
            pixels.resize(size_t(size) * size);
            struct Pip { float u, v; };
            vector<Pip> pips;

            switch (value) {
            case 1: pips = { { 0.5f, 0.5f } }; break;
            case 2: pips = { { 0.26f, 0.26f },{ 0.74f, 0.74f } }; break;
            case 3: pips = { { 0.24f, 0.24f },{ 0.50f, 0.50f },{ 0.76f, 0.76f } }; break;
            case 4: pips = { { 0.26f, 0.26f },{ 0.74f, 0.26f },{ 0.26f, 0.74f },{ 0.74f, 0.74f } }; break;
            case 5: pips = { { 0.25f, 0.25f },{ 0.75f, 0.25f },{ 0.50f, 0.50f },{ 0.25f, 0.75f },{ 0.75f, 0.75f } }; break;
            case 6: pips = { { 0.27f, 0.23f },{ 0.27f, 0.50f },{ 0.27f, 0.77f },
                            { 0.73f, 0.23f },{ 0.73f, 0.50f },{ 0.73f, 0.77f } }; break;
            }

            float pipRadius = (value == 1) ? 0.145f : 0.11f;

            for (int y = 0; y < size; ++y) {
                float v = (y + 0.5f) / size;
                for (int x = 0; x < size; ++x) {
                    float u = (x + 0.5f) / size;

                    float edgeDist = min({ u, 1.0f - u, v, 1.0f - v });
                    float bevel = clamp(edgeDist / 0.09f, 0.76f, 1.0f);
                    int r = int(246 * bevel), g = int(238 * bevel), b = int(218 * bevel);

                    for (const auto& p : pips) {
                        float dx = u - p.u, dy = v - p.v;
                        float d = sqrt(dx * dx + dy * dy);
                        if (d < pipRadius) {
                            float edge = clamp((pipRadius - d) / 0.018f, 0.0f, 1.0f);
                            int pr = (value == 1) ? 165 : 26;
                            int pg = (value == 1) ? 26 : 24;
                            int pb = (value == 1) ? 26 : 26;
                            float shade = 0.75f + 0.35f * clamp((dy / pipRadius + 1.0f) * 0.5f, 0.0f, 1.0f);
                            r = int(r * (1.0f - edge) + pr * shade * edge);
                            g = int(g * (1.0f - edge) + pg * shade * edge);
                            b = int(b * (1.0f - edge) + pb * shade * edge);
                        }
                    }
                    pixels[size_t(y) * size + x] = makeRGB(r, g, b);
                }
            }
        }

        // Bilinear Filtering (Cift Dogrusal Filtreleme)
        uint32_t sample(float u, float v) const {
            float tx = clamp(u, 0.0f, 1.0f) * float(size) - 0.5f;
            float ty = clamp(v, 0.0f, 1.0f) * float(size) - 0.5f;

            int ix = int(floor(tx));
            int iy = int(floor(ty));

            float fx = tx - float(ix);
            float fy = ty - float(iy);

            int x0 = clamp(ix, 0, size - 1);
            int x1 = clamp(ix + 1, 0, size - 1);
            int y0 = clamp(iy, 0, size - 1);
            int y1 = clamp(iy + 1, 0, size - 1);

            uint32_t p00 = pixels[size_t(y0) * size + x0];
            uint32_t p10 = pixels[size_t(y0) * size + x1];
            uint32_t p01 = pixels[size_t(y1) * size + x0];
            uint32_t p11 = pixels[size_t(y1) * size + x1];

            auto bilerpChannel = [fx, fy](uint32_t c00, uint32_t c10, uint32_t c01, uint32_t c11, int shift) -> int {
                float v00 = float((c00 >> shift) & 0xFF);
                float v10 = float((c10 >> shift) & 0xFF);
                float v01 = float((c01 >> shift) & 0xFF);
                float v11 = float((c11 >> shift) & 0xFF);

                float top = v00 + (v10 - v00) * fx;
                float bottom = v01 + (v11 - v01) * fx;
                return int(top + (bottom - top) * fy + 0.5f);
                };

            int r = bilerpChannel(p00, p10, p01, p11, 16);
            int g = bilerpChannel(p00, p10, p01, p11, 8);
            int b = bilerpChannel(p00, p10, p01, p11, 0);

            return makeRGB(r, g, b);
        }
    };

    struct Camera {
        Vec3 pos{ 0.0f, 11.8f, -8.2f };
        Vec3 target{ 0.0f, 0.0f, 0.3f };
        Vec3 forward, right, up;
        float fov = 1150.0f;

        Camera() {
            forward = (target - pos).normalized();
            right = Vec3{ 0, 1, 0 }.cross(forward).normalized();
            up = forward.cross(right).normalized();
        }

        bool project(const Vec3& world, float& sx, float& sy, float& cz) const {
            Vec3 rel = world - pos;
            float cx = rel.dot(right);
            float cy = rel.dot(up);
            cz = rel.dot(forward);
            if (cz <= 0.2f) return false;
            sx = SCREEN_W * 0.5f + (cx / cz) * fov;
            sy = SCREEN_H * 0.5f - (cy / cz) * fov;
            return true;
        }

        // 2D Fare koordinatlarini (mouseX, mouseY) 3D dunyada birim isin yonune (Ray) donusturur
        Vec3 screenToRay(int mouseX, int mouseY) const {
            float cx = (float(mouseX) - SCREEN_W * 0.5f) / fov;
            float cy = (SCREEN_H * 0.5f - float(mouseY)) / fov;
            return (right * cx + up * cy + forward).normalized();
        }

        // Fare isininin belirli bir Y yuksekligindeki yatay duzlemle kesistigi 3D noktayi bulur
        bool screenToPlaneY(int mouseX, int mouseY, float planeY, Vec3& outWorldPos) const {
            Vec3 dir = screenToRay(mouseX, mouseY);
            if (abs(dir.y) < 1e-4f) return false;
            float t = (planeY - pos.y) / dir.y;
            if (t <= 0.0f) return false;
            outWorldPos = pos + dir * t;
            return true;
        }
    };

    struct ScreenVert {
        float sx, sy, invZ, uOverZ, vOverZ;
    };

    class Renderer3D {
    public:
        Camera cam;
        Vec3 lightDir = Vec3{ 0.42f, 0.82f, -0.38f }.normalized();
        vector<uint32_t> backgroundBuffer;
        vector<float> backgroundZBuffer;
        vector<float> zBuffer;
        Texture diceTextures[7];

        struct AABB {
            Vec3 bmin, bmax;
            bool isHorizontal;
        };

        Renderer3D(DualCoreWorker& dualCore)
            : backgroundBuffer(size_t(SCREEN_W)* SCREEN_H),
            backgroundZBuffer(size_t(SCREEN_W)* SCREEN_H, 0.0f),
            zBuffer(size_t(SCREEN_W)* SCREEN_H, 0.0f) {
            for (int i = 1; i <= 6; ++i) diceTextures[i].generateDiceFace(i);

            constexpr int midY = SCREEN_H / 2;
            dualCore.runParallel(
                [&]() { bakeBoardSlice(0, midY); },
                [&]() { bakeBoardSlice(midY, SCREEN_H); }
            );
        }

        static bool intersectAABB(const Vec3& orig, const Vec3& dir, const AABB& box,
            float& outT, Vec3& outNormal) {
            float tmin = -1e9f, tmax = 1e9f;
            Vec3 nMin{ 0, 0, 0 };

            auto slab = [&](float o, float d, float mn, float mx, Vec3 negN, Vec3 posN) -> bool {
                if (abs(d) < 1e-6f) return (o >= mn && o <= mx);
                float invD = 1.0f / d;
                float t1 = (mn - o) * invD, t2 = (mx - o) * invD;
                Vec3 n = negN;
                if (t1 > t2) { swap(t1, t2); n = posN; }
                if (t1 > tmin) { tmin = t1; nMin = n; }
                if (t2 < tmax) { tmax = t2; }
                return tmin <= tmax;
                };

            if (!slab(orig.x, dir.x, box.bmin.x, box.bmax.x, { -1, 0, 0 }, { 1, 0, 0 })) return false;
            if (!slab(orig.y, dir.y, box.bmin.y, box.bmax.y, { 0, -1, 0 }, { 0, 1, 0 })) return false;
            if (!slab(orig.z, dir.z, box.bmin.z, box.bmax.z, { 0, 0, -1 }, { 0, 0, 1 })) return false;

            if (tmax < 0.0f || tmin < 0.0f) return false;
            outT = tmin;
            outNormal = nMin;
            return true;
        }

        void bakeBoardSlice(int yStart, int yEnd) {
            constexpr float wallTop = 0.78f;
            constexpr float boxBottom = -0.34f;

            const AABB walls[4] = {
                { { -6.85f, boxBottom,  3.95f },{ 6.85f, wallTop,  4.75f }, true },
                { { -6.85f, boxBottom, -4.75f },{ 6.85f, wallTop, -3.95f }, true },
                { { -6.85f, boxBottom, -3.95f },{ -6.00f, wallTop,  3.95f }, false },
                { { 6.00f, boxBottom, -3.95f },{ 6.85f, wallTop,  3.95f }, false }
            };
            const AABB fullBox{ { -6.85f, boxBottom, -4.75f },{ 6.85f, wallTop, 4.75f }, true };

            for (int py = yStart; py < yEnd; ++py) {
                float cy = (SCREEN_H * 0.5f - py) / cam.fov;
                for (int px = 0; px < SCREEN_W; ++px) {
                    float cx = (px - SCREEN_W * 0.5f) / cam.fov;
                    Vec3 dir = (cam.right * cx + cam.up * cy + cam.forward).normalized();

                    float bestT = 1e9f;
                    int hitWallIdx = -1;
                    Vec3 hitNormal{ 0, 1, 0 };

                    for (int w = 0; w < 4; ++w) {
                        float t; Vec3 n;
                        if (intersectAABB(cam.pos, dir, walls[w], t, n) && t < bestT) {
                            bestT = t; hitWallIdx = w; hitNormal = n;
                        }
                    }

                    bool hitFloor = false;
                    if (dir.y < -0.001f) {
                        float tFloor = -cam.pos.y / dir.y;
                        if (tFloor > 0.0f && tFloor < bestT) {
                            Vec3 pf = cam.pos + dir * tFloor;
                            if (abs(pf.x) <= 6.01f && abs(pf.z) <= 3.96f) {
                                bestT = tFloor; hitWallIdx = -1; hitFloor = true; hitNormal = { 0, 1, 0 };
                            }
                        }
                    }

                    uint32_t col = makeRGB(22, 18, 16);
                    float bakedInvZ = 0.0f;

                    if (hitWallIdx != -1) {
                        Vec3 p = cam.pos + dir * bestT;
                        const AABB& w = walls[hitWallIdx];

                        float gCoord = w.isHorizontal
                            ? (p.z * 14.0f + p.y * 12.0f + sin(p.x * 2.2f) * 1.4f)
                            : (p.x * 14.0f + p.y * 12.0f + sin(p.z * 2.2f) * 1.4f);
                        float woodMod = 0.84f + 0.20f * (sin(gCoord) * 0.5f + 0.5f);
                        float light = 0.34f + 0.72f * max(0.0f, hitNormal.dot(lightDir));

                        float edgeHighlight = 1.0f;
                        if (hitNormal.y > 0.5f) {
                            float dEdge = min({ abs(p.x - w.bmin.x), abs(p.x - w.bmax.x),
                                               abs(p.z - w.bmin.z), abs(p.z - w.bmax.z) });
                            if (dEdge < 0.07f) edgeHighlight = 1.15f;
                        }
                        else if (abs(p.y - wallTop) < 0.06f) {
                            edgeHighlight = 1.14f;
                        }

                        float ao = clamp(0.76f + (p.y - boxBottom) * 0.30f, 0.76f, 1.0f);
                        int r = int(106 * woodMod * light * edgeHighlight * ao);
                        int g = int(60 * woodMod * light * edgeHighlight * ao);
                        int b = int(32 * woodMod * light * edgeHighlight * ao);

                        if (w.isHorizontal && hitNormal.y > 0.5f && abs(p.x) < 0.30f &&
                            abs(p.z) > 4.10f && abs(p.z) < 4.60f) {
                            float shine = 0.85f + 0.25f * sin(p.x * 18.0f);
                            r = int(205 * shine * light);
                            g = int(165 * shine * light);
                            b = int(75 * shine * light);
                        }

                        col = makeRGB(r, g, b);
                        float cz = bestT * dir.dot(cam.forward);
                        if (cz > 0.2f) bakedInvZ = 1.0f / cz;
                    }
                    else if (hitFloor) {
                        Vec3 p = cam.pos + dir * bestT;
                        float grain = sin(p.x * 9.0f + sin(p.z * 2.5f) * 1.8f) * 0.5f + 0.5f;
                        float woodMod = 0.88f + 0.16f * grain + sin(p.z * 32.0f + p.x * 4.0f) * 0.04f;

                        int br = int(176 * woodMod), bg = int(136 * woodMod), bb = int(90 * woodMod);

                        if (abs(p.x) < 0.38f) {
                            float bevel = (abs(p.x) > 0.31f) ? 0.72f : 1.0f;
                            br = int(92 * woodMod * bevel);
                            bg = int(54 * woodMod * bevel);
                            bb = int(30 * woodMod * bevel);
                        }
                        else {
                            float halfStart = (p.x < 0) ? -6.0f : 0.38f;
                            float slotW = (6.0f - 0.38f) / 6.0f;
                            float relX = p.x - halfStart;
                            int slotIdx = clamp(int(relX / slotW), 0, 5);
                            float u = (relX - slotIdx * slotW) / slotW;

                            float distFromEdge = 3.95f - abs(p.z);
                            float triLen = 3.05f;
                            if (distFromEdge >= 0.0f && distFromEdge < triLen) {
                                float halfWidthAtZ = 0.47f * (1.0f - distFromEdge / triLen);
                                if (abs(u - 0.5f) < halfWidthAtZ) {
                                    bool isDark = ((slotIdx + (p.z > 0 ? 1 : 0)) % 2 == 0);
                                    br = int((isDark ? 118 : 224) * woodMod);
                                    bg = int((isDark ? 34 : 204) * woodMod);
                                    bb = int((isDark ? 26 : 164) * woodMod);
                                }
                            }
                        }

                        Vec3 shadowOrig = p + Vec3{ 0, 0.01f, 0 };
                        bool inShadow = false;
                        for (int w = 0; w < 4; ++w) {
                            float st; Vec3 sn;
                            if (intersectAABB(shadowOrig, lightDir, walls[w], st, sn)) {
                                inShadow = true; break;
                            }
                        }

                        float wallAO = min({ 1.0f, (6.0f - abs(p.x)) * 1.15f + 0.58f,
                                                  (3.95f - abs(p.z)) * 1.15f + 0.58f });
                        float sm = inShadow ? 0.56f : 1.0f;
                        col = makeRGB(int(br * wallAO * sm), int(bg * wallAO * sm), int(bb * wallAO * sm));
                    }
                    else if (dir.y < -0.001f) {
                        float tTable = (boxBottom - cam.pos.y) / dir.y;
                        if (tTable > 0.0f) {
                            Vec3 pt = cam.pos + dir * tTable;
                            float vig = clamp(1.12f - sqrt(pt.x * pt.x + pt.z * pt.z) * 0.052f, 0.25f, 1.0f);
                            float st; Vec3 sn;
                            bool boxShadow = intersectAABB(pt + Vec3{ 0, 0.01f, 0 }, lightDir, fullBox, st, sn);
                            float sm = boxShadow ? 0.42f : 1.0f;
                            col = makeRGB(int(48 * vig * sm), int(38 * vig * sm), int(32 * vig * sm));
                        }
                    }

                    size_t idx = size_t(py) * SCREEN_W + px;
                    backgroundBuffer[idx] = col;
                    backgroundZBuffer[idx] = bakedInvZ;
                }
            }
        }

        void beginFrameSlice(uint32_t* frameBuffer, int yStart, int yEnd) {
            size_t offset = size_t(yStart) * SCREEN_W;
            size_t count = size_t(yEnd - yStart) * SCREEN_W;
            std::memcpy(frameBuffer + offset, backgroundBuffer.data() + offset, count * sizeof(uint32_t));
            std::memcpy(zBuffer.data() + offset, backgroundZBuffer.data() + offset, count * sizeof(float));
        }

        void drawShadowSlice(uint32_t* frameBuffer, const Vec3& diePos, float halfSize, int yStart, int yEnd) {
            Vec3 shadowWorld{ diePos.x - lightDir.x * diePos.y * 0.65f, 0.01f, diePos.z - lightDir.z * diePos.y * 0.65f };
            float sx, sy, cz;
            if (!cam.project(shadowWorld, sx, sy, cz)) return;

            float rx = ((halfSize * (1.30f + diePos.y * 0.08f)) / cz) * cam.fov;
            float ry = rx * 0.72f;
            float strength = clamp(0.48f - diePos.y * 0.06f, 0.10f, 0.48f);

            int x0 = max(0, int(sx - rx)), x1 = min(SCREEN_W - 1, int(sx + rx));
            int y0 = max(yStart, int(sy - ry)), y1 = min(yEnd - 1, int(sy + ry));

            for (int y = y0; y <= y1; ++y) {
                float dy = (y - sy) / ry;
                for (int x = x0; x <= x1; ++x) {
                    size_t idx = size_t(y) * SCREEN_W + x;
                    if (zBuffer[idx] > 0.0f) continue;
                    float dx = (x - sx) / rx;
                    float d2 = dx * dx + dy * dy;
                    if (d2 < 1.0f) {
                        frameBuffer[idx] = scaleRGB(frameBuffer[idx], 1.0f - (1.0f - d2) * strength);
                    }
                }
            }
        }

        static float edgeFunc(float ax, float ay, float bx, float by, float cx, float cy) {
            return (cx - ax) * (by - ay) - (cy - ay) * (bx - ax);
        }

        void drawTriangleSlice(uint32_t* fb, ScreenVert v0, ScreenVert v1, ScreenVert v2,
            const Texture& tex, float lighting, int yStart, int yEnd) {
            float area = edgeFunc(v0.sx, v0.sy, v1.sx, v1.sy, v2.sx, v2.sy);
            if (abs(area) < 0.01f) return;
            if (area < 0.0f) { swap(v1, v2); area = -area; }
            float invArea = 1.0f / area;

            int minX = max(0, int(floor(min({ v0.sx, v1.sx, v2.sx }))));
            int maxX = min(SCREEN_W - 1, int(ceil(max({ v0.sx, v1.sx, v2.sx }))));
            int minY = max(yStart, int(floor(min({ v0.sy, v1.sy, v2.sy }))));
            int maxY = min(yEnd - 1, int(ceil(max({ v0.sy, v1.sy, v2.sy }))));

            for (int y = minY; y <= maxY; ++y) {
                float py = y + 0.5f;
                for (int x = minX; x <= maxX; ++x) {
                    float px = x + 0.5f;
                    float w0 = edgeFunc(v1.sx, v1.sy, v2.sx, v2.sy, px, py);
                    float w1 = edgeFunc(v2.sx, v2.sy, v0.sx, v0.sy, px, py);
                    float w2 = edgeFunc(v0.sx, v0.sy, v1.sx, v1.sy, px, py);

                    if (w0 >= 0 && w1 >= 0 && w2 >= 0) {
                        w0 *= invArea; w1 *= invArea; w2 *= invArea;
                        float invZ = w0 * v0.invZ + w1 * v1.invZ + w2 * v2.invZ;
                        size_t idx = size_t(y) * SCREEN_W + x;

                        if (invZ > zBuffer[idx]) {
                            zBuffer[idx] = invZ;
                            float z = 1.0f / invZ;
                            float u = (w0 * v0.uOverZ + w1 * v1.uOverZ + w2 * v2.uOverZ) * z;
                            float v = (w0 * v0.vOverZ + w1 * v1.vOverZ + w2 * v2.vOverZ) * z;
                            fb[idx] = scaleRGB(tex.sample(u, v), lighting);
                        }
                    }
                }
            }
        }

        void drawDiceSlice(uint32_t* fb, const Vec3& pos, const Quat& rot, float halfSize, int yStart, int yEnd) {
            Mat3 R = rot.toMat3();
            struct FaceDef { int val; Vec3 n, uAxis, vAxis; };
            static const FaceDef faces[6] = {
                { 1,{ 0,  1,  0 },{ 1,  0,  0 },{ 0,  0,  1 } },
                { 6,{ 0, -1,  0 },{ 1,  0,  0 },{ 0,  0, -1 } },
                { 2,{ 0,  0,  1 },{ 1,  0,  0 },{ 0, -1,  0 } },
                { 5,{ 0,  0, -1 },{ -1,  0,  0 },{ 0, -1,  0 } },
                { 3,{ 1,  0,  0 },{ 0,  0, -1 },{ 0, -1,  0 } },
                { 4,{ -1,  0,  0 },{ 0,  0,  1 },{ 0, -1,  0 } }
            };

            for (const auto& f : faces) {
                Vec3 worldN = R * f.n;
                Vec3 toCam = (cam.pos - (pos + worldN * halfSize)).normalized();
                if (worldN.dot(toCam) <= 0.0f) continue;

                float diffuse = max(0.0f, worldN.dot(lightDir));
                float spec = pow(max(0.0f, worldN.dot((lightDir + toCam).normalized())), 24.0f) * 0.25f;
                float lighting = clamp(0.42f + 0.62f * diffuse + spec, 0.35f, 1.18f);

                Vec3 corners[4] = {
                    pos + (R * (f.n - f.uAxis - f.vAxis)) * halfSize,
                    pos + (R * (f.n + f.uAxis - f.vAxis)) * halfSize,
                    pos + (R * (f.n + f.uAxis + f.vAxis)) * halfSize,
                    pos + (R * (f.n - f.uAxis + f.vAxis)) * halfSize
                };
                float uvs[4][2] = { { 0, 0 },{ 1, 0 },{ 1, 1 },{ 0, 1 } };

                ScreenVert sv[4];
                bool ok = true;
                for (int i = 0; i < 4; ++i) {
                    float cz;
                    if (!cam.project(corners[i], sv[i].sx, sv[i].sy, cz)) { ok = false; break; }
                    sv[i].invZ = 1.0f / cz;
                    sv[i].uOverZ = uvs[i][0] * sv[i].invZ;
                    sv[i].vOverZ = uvs[i][1] * sv[i].invZ;
                }
                if (!ok) continue;

                drawTriangleSlice(fb, sv[0], sv[1], sv[2], diceTextures[f.val], lighting, yStart, yEnd);
                drawTriangleSlice(fb, sv[0], sv[2], sv[3], diceTextures[f.val], lighting, yStart, yEnd);
            }
        }

        void renderSlice(uint32_t* fb, int yStart, int yEnd,
            const Vec3& p1, const Quat& r1,
            const Vec3& p2, const Quat& r2, float halfSize) {
            beginFrameSlice(fb, yStart, yEnd);
            drawShadowSlice(fb, p1, halfSize, yStart, yEnd);
            drawShadowSlice(fb, p2, halfSize, yStart, yEnd);
            drawDiceSlice(fb, p1, r1, halfSize, yStart, yEnd);
            drawDiceSlice(fb, p2, r2, halfSize, yStart, yEnd);
        }
    };
}

// ============================================================
// 4. 3D ZAR FIZIK MOTORU (Engine::Physics)
// ============================================================
namespace Engine::Physics {
    using namespace Engine::Math;
    using std::max;
    using std::min;
    using std::abs;
    using std::pow;

    struct DiceBody {
        static constexpr float HALF = 0.34f;
        Vec3 pos{ 0, HALF, 0 };
        Vec3 vel{ 0, 0, 0 };
        Vec3 angVel{ 0, 0, 0 };
        Quat rot{ 1, 0, 0, 0 };
        bool settled = true;
        bool hitFloorOnce = false;
        bool isHeld = false; // Fare ile tutuluyor mu?

        void throwDice(std::mt19937& rng, float startX, float startZ, float dirX) {
            std::uniform_real_distribution<float> f01(0.0f, 1.0f);
            pos = { startX, 2.8f + f01(rng) * 1.2f, startZ };
            vel = { dirX * (5.8f + f01(rng) * 4.2f), 1.8f + f01(rng) * 2.2f, (f01(rng) - 0.5f) * 6.0f };
            angVel = { (f01(rng) - 0.5f) * 28.0f, (f01(rng) - 0.5f) * 20.0f, (f01(rng) - 0.5f) * 28.0f };
            rot = Quat{ f01(rng) - 0.5f, f01(rng) - 0.5f, f01(rng) - 0.5f, f01(rng) - 0.5f }.normalized();
            settled = false;
            hitFloorOnce = false;
            isHeld = false;
        }

        int getTopValue(Vec3* outTopWorldNormal = nullptr) const {
            Mat3 R = rot.toMat3();
            static const std::pair<int, Vec3> normals[6] = {
                { 1,{ 0,  1,  0 } },{ 6,{ 0, -1,  0 } },
                { 2,{ 0,  0,  1 } },{ 5,{ 0,  0, -1 } },
                { 3,{ 1,  0,  0 } },{ 4,{ -1,  0,  0 } }
            };
            int bestVal = 1;
            float bestY = -2.0f;
            Vec3 bestN{ 0, 1, 0 };
            for (const auto& [val, n] : normals) {
                Vec3 wn = R * n;
                if (wn.y > bestY) { bestY = wn.y; bestVal = val; bestN = wn; }
            }
            if (outTopWorldNormal) *outTopWorldNormal = bestN;
            return bestVal;
        }

        void update(float dt) {
            // Zar fare ile tutuluyorsa yercekimi ve zemin fizigini devre disi birak!
            if (isHeld) {
                Quat dq{ 0.0f, angVel.x * 0.5f * dt, angVel.y * 0.5f * dt, angVel.z * 0.5f * dt };
                Quat qDot = dq * rot;
                rot = Quat{ rot.w + qDot.w, rot.x + qDot.x, rot.y + qDot.y, rot.z + qDot.z }.normalized();
                angVel = angVel * pow(0.05f, dt);
                return;
            }

            if (settled) return;

            constexpr float gravity = 26.0f;
            vel.y -= gravity * dt;
            pos += vel * dt;

            Quat dq{ 0.0f, angVel.x * 0.5f * dt, angVel.y * 0.5f * dt, angVel.z * 0.5f * dt };
            Quat qDot = dq * rot;
            rot = Quat{ rot.w + qDot.w, rot.x + qDot.x, rot.y + qDot.y, rot.z + qDot.z }.normalized();

            Mat3 R = rot.toMat3();
            Vec3 corners[8];
            float minCornerY = 100.0f;
            int idx = 0;
            for (int sx : {-1, 1}) {
                for (int sy : {-1, 1}) {
                    for (int sz : {-1, 1}) {
                        corners[idx] = R * Vec3{ sx * HALF, sy * HALF, sz * HALF };
                        if (corners[idx].y < minCornerY) minCornerY = corners[idx].y;
                        idx++;
                    }
                }
            }

            constexpr float inertia = (2.0f / 3.0f) * HALF * HALF;
            constexpr float invI = 1.0f / inertia;

            bool onFloor = false;
            if (pos.y + minCornerY <= 0.0f) {
                pos.y = -minCornerY;
                onFloor = true;
                hitFloorOnce = true;

                Vec3 r{ 0, 0, 0 };
                int contactCount = 0;
                for (int i = 0; i < 8; ++i) {
                    if (corners[i].y <= minCornerY + 0.025f) {
                        r += corners[i];
                        contactCount++;
                    }
                }
                r = r * (1.0f / float(contactCount));

                Vec3 vContact = vel + angVel.cross(r);

                float jn = 0.0f;
                if (vContact.y < 0.0f) {
                    Vec3 rCrossN = r.cross(Vec3{ 0, 1, 0 });
                    float denom = 1.0f + rCrossN.dot(rCrossN) * invI;
                    float restitution = (vContact.y < -1.2f) ? 0.44f : 0.0f;
                    jn = -(1.0f + restitution) * vContact.y / denom;

                    vel.y += jn;
                    angVel += r.cross(Vec3{ 0, jn, 0 }) * invI;
                }

                vContact = vel + angVel.cross(r);
                Vec3 vTan{ vContact.x, 0.0f, vContact.z };
                float vTanLen = vTan.length();
                if (vTanLen > 1e-4f) {
                    Vec3 t = vTan * (1.0f / vTanLen);
                    Vec3 rCrossT = r.cross(t);
                    float denomT = 1.0f + rCrossT.dot(rCrossT) * invI;

                    float jt = max(-vTanLen / denomT, -0.92f * max(jn, gravity * dt));
                    Vec3 impT = t * jt;
                    vel += impT;
                    angVel += r.cross(impT) * invI;
                }

                angVel.y *= pow(0.10f, dt);

                if (vel.lengthSq() < 0.55f && angVel.lengthSq() < 8.0f && abs(vel.y) < 0.4f) {
                    Vec3 topN;
                    getTopValue(&topN);
                    Quat qAlign = Quat{ 1.0f + topN.y, -topN.z, 0.0f, topN.x }.normalized();
                    Quat qTarget = (qAlign * rot).normalized();

                    float settleStrength = (contactCount >= 4) ? 14.0f : 6.0f;
                    rot = Quat::nlerp(rot, qTarget, min(1.0f, dt * settleStrength));
                    vel.x *= pow(0.08f, dt);
                    vel.z *= pow(0.08f, dt);
                    angVel = angVel * pow(0.06f, dt);

                    if (vel.lengthSq() < 0.015f && angVel.lengthSq() < 0.12f && topN.y > 0.994f) {
                        rot = qTarget;
                        pos.y = HALF;
                        vel = { 0, 0, 0 };
                        angVel = { 0, 0, 0 };
                        settled = true;
                    }
                }
            }

            constexpr float wallX = 5.52f;
            constexpr float wallZ = 3.48f;
            if (pos.x < -wallX) { pos.x = -wallX; vel.x = abs(vel.x) * 0.58f; angVel.z -= vel.x * 2.2f; }
            if (pos.x > wallX) { pos.x = wallX; vel.x = -abs(vel.x) * 0.58f; angVel.z -= vel.x * 2.2f; }
            if (pos.z < -wallZ) { pos.z = -wallZ; vel.z = abs(vel.z) * 0.58f; angVel.x += vel.z * 2.2f; }
            if (pos.z > wallZ) { pos.z = wallZ; vel.z = -abs(vel.z) * 0.58f; angVel.x += vel.z * 2.2f; }

            if (!onFloor) {
                angVel = angVel * pow(0.92f, dt);
            }
        }

        static void resolveCollision(DiceBody& a, DiceBody& b) {
            if (a.isHeld || b.isHeld) return;

            Vec3 diff = b.pos - a.pos;
            float dist = diff.length();
            float minDist = HALF * 2.25f;
            if (dist < minDist && dist > 1e-4f) {
                Vec3 n = diff * (1.0f / dist);
                float overlap = (minDist - dist) * 0.5f;
                a.pos = a.pos - n * overlap;
                b.pos = b.pos + n * overlap;

                float vn = (b.vel - a.vel).dot(n);
                if (vn < 0.0f) {
                    float imp = -1.65f * vn * 0.5f;
                    a.vel = a.vel - n * imp;
                    b.vel = b.vel + n * imp;
                    a.angVel += Vec3{ n.z * 6.0f, 3.0f, -n.x * 6.0f };
                    b.angVel += Vec3{ -n.z * 6.0f, -3.0f, n.x * 6.0f };
                    a.settled = false;
                    b.settled = false;
                }
            }
        }
    };
}

// ============================================================
// 5. WIN32 MCI MP3 SES MOTORU (Engine::Audio)
// ============================================================
namespace Engine::Audio {
    using std::wstring;

    class Win32Audio {
    public:
        bool loaded = false;

        bool tryOpen(const wstring& fullPath) {
            wstring cmd = L"open \"" + fullPath + L"\" type mpegvideo alias diceSfx";
            if (mciSendStringW(cmd.c_str(), nullptr, 0, nullptr) == 0) {
                loaded = true;
                mciSendStringW(L"setaudio diceSfx volume to 900", nullptr, 0, nullptr);
                return true;
            }
            return false;
        }

        bool load(const wstring& fileName) {
            if (tryOpen(fileName)) return true;

            wchar_t exePath[MAX_PATH]{};
            GetModuleFileNameW(nullptr, exePath, MAX_PATH);
            wstring dir(exePath);
            size_t slash = dir.find_last_of(L"\\/");
            if (slash != wstring::npos) {
                return tryOpen(dir.substr(0, slash + 1) + fileName);
            }
            return false;
        }

        void play() {
            if (loaded) mciSendStringW(L"play diceSfx from 0", nullptr, 0, nullptr);
        }

        void stop() {
            if (loaded) mciSendStringW(L"stop diceSfx", nullptr, 0, nullptr);
        }

        void shutdown() {
            if (loaded) {
                mciSendStringW(L"close diceSfx", nullptr, 0, nullptr);
                loaded = false;
            }
        }
    };
}

// ============================================================
// 6. TAVLA OYUN MANTIGI & FARE ILE TUTUP FIRLATMA (Tavla)
// ============================================================
namespace Tavla {
    using namespace Engine::Math;
    using namespace Engine::Graphics;
    using namespace Engine::Physics;
    using namespace Engine::Audio;
    using std::wstring;
    using std::to_wstring;
    using std::swap;
    using std::clamp;

    wstring getTavlaNameW(int d1, int d2) {
        if (d1 < d2) swap(d1, d2);
        if (d1 == d2) {
            switch (d1) {
            case 1: return L"HEPYEK (1-1)";
            case 2: return L"D\u00DCBARA (2-2)";
            case 3: return L"D\u00DCSE (3-3)";
            case 4: return L"D\u00D6RT C\u0130HAR (4-4)";
            case 5: return L"D\u00DCBE\u015E (5-5)";
            case 6: return L"D\u00DC\u015EE\u015E (6-6)";
            }
        }
        static const wchar_t* names[7] = {
            L"", L"Yek", L"D\u00FC", L"Se", L"Cihar", L"Pen\u00E7", L"\u015Ee\u015F"
        };
        return wstring(names[d1]) + L"-\u00FC " + names[d2] +
            L" (" + to_wstring(d1) + L"-" + to_wstring(d2) + L")";
    }

    int evaluateTavlaPower(int d1, int d2) {
        return (d1 == d2) ? (100 + d1 * 10) : (d1 + d2);
    }

    enum class GameState { READY, PLAYER_ROLLING, WAIT_FOR_CPU, CPU_ROLLING, ROUND_OVER };

    struct TavlaGame {
        static constexpr float TUR_BEKLEME_SURESI = 2.0f;
        static constexpr float SES_GECIKMESI = 0.0f;
        static constexpr float HOLD_PLANE_Y = 1.45f; // Zarin havada tutuldugu duzlem yuksekligi
        static constexpr float THROW_FORCE = 0.42f; // Fare firlatma kuvvet carpani

        std::mt19937 rng{ std::random_device{}() };
        DiceBody die1, die2;
        Win32Audio audio;
        GameState state = GameState::READY;

        // Fare ile zar tutma & firlatma degiskenleri
        DiceBody* heldDie = nullptr;
        int lastMouseX = 0, lastMouseY = 0;
        float deltaX = 0.0f, deltaY = 0.0f;

        int playerScore = 0, cpuScore = 0, round = 1;
        int p1 = 0, p2 = 0, c1 = 0, c2 = 0;

        float stateTimer = 0.0f;
        float roundCooldown = 0.0f;
        bool soundPlayed = true;
        float soundDelayTimer = 0.0f;

        wstring statusText = L"Zara sol t\u0131klay\u0131p tut ve f\u0131rlat! (veya [BO\u015ELUK] tu\u015Funa bas)";
        wstring subText = L"\u00C7ift atan kazan\u0131r  |  [R] Skoru S\u0131f\u0131rla  |  [ESC] \u00C7\u0131k\u0131\u015F";

        TavlaGame() {
            die1.pos = { -0.8f, DiceBody::HALF,  0.15f };
            die2.pos = { 0.8f, DiceBody::HALF, -0.15f };
            audio.load(L"zar.mp3");
        }

        void triggerRoll() {
            if (heldDie) return;
            if (state == GameState::ROUND_OVER && roundCooldown > 0.0f) return;

            if (state == GameState::READY || state == GameState::ROUND_OVER) {
                state = GameState::PLAYER_ROLLING;
                die1.throwDice(rng, -4.8f, -1.1f, 1.0f);
                die2.throwDice(rng, -4.8f, 1.1f, 1.0f);
                soundPlayed = false;
                soundDelayTimer = 0.0f;
                statusText = L"SEN zar at\u0131yorsun...";
                subText = L"Zarlar fizik motoruyla yuvarlan\u0131yor...";
            }
        }

        // 1. WM_LBUTTONDOWN: Fareden 3D isin gonder, zara carptiysa tut (isHeld = true)
        bool onMouseDown(int mouseX, int mouseY, const Camera& cam) {
            if (state == GameState::CPU_ROLLING || state == GameState::WAIT_FOR_CPU) return false;
            if (state == GameState::ROUND_OVER && roundCooldown > 0.0f) return false;

            Vec3 rayDir = cam.screenToRay(mouseX, mouseY);
            constexpr float pickExtent = DiceBody::HALF * 1.45f; // Rahat yakalama sinir kutusu

            DiceBody* candidates[2] = { &die1, &die2 };
            float bestT = 1e9f;
            DiceBody* picked = nullptr;

            for (DiceBody* d : candidates) {
                Renderer3D::AABB box{
                    d->pos - Vec3{ pickExtent, pickExtent, pickExtent },
                    d->pos + Vec3{ pickExtent, pickExtent, pickExtent },
                    true
                };
                float t; Vec3 n;
                if (Renderer3D::intersectAABB(cam.pos, rayDir, box, t, n) && t < bestT) {
                    bestT = t;
                    picked = d;
                }
            }

            if (picked) {
                heldDie = picked;
                heldDie->isHeld = true;
                heldDie->settled = false;
                heldDie->hitFloorOnce = false;
                heldDie->vel = { 0, 0, 0 };
                heldDie->angVel = { 0, 0, 0 };

                lastMouseX = mouseX;
                lastMouseY = mouseY;
                deltaX = 0.0f;
                deltaY = 0.0f;

                Vec3 planePos;
                if (cam.screenToPlaneY(mouseX, mouseY, HOLD_PLANE_Y, planePos)) {
                    heldDie->pos.x = clamp(planePos.x, -5.35f, 5.35f);
                    heldDie->pos.y = HOLD_PLANE_Y;
                    heldDie->pos.z = clamp(planePos.z, -3.30f, 3.30f);
                }

                statusText = L"Zar tutuldu! Fareyi h\u0131zl\u0131ca s\u00FCr\u00FCkleyip b\u0131rakarak f\u0131rlat!";
                subText = L"Fırlatma h\u0131z\u0131n zar\u0131n h\u0131z\u0131n\u0131 ve d\u00F6n\u00FC\u015F\u00FCn\u00FC belirler.";
                return true;
            }
            return false;
        }

        // 2. WM_MOUSEMOVE: Tutulan zari 3D duzlemde tasi ve fare hizini (deltaX, deltaY) kaydet
        void onMouseMove(int mouseX, int mouseY, const Camera& cam) {
            if (!heldDie || !heldDie->isHeld) return;

            float rawDx = float(mouseX - lastMouseX);
            float rawDy = float(mouseY - lastMouseY);
            lastMouseX = mouseX;
            lastMouseY = mouseY;

            // Yumusatilmis anlik fare hizi (firlatma aninda titresimi onler)
            deltaX = deltaX * 0.5f + rawDx * 0.5f;
            deltaY = deltaY * 0.5f + rawDy * 0.5f;

            Vec3 planePos;
            if (cam.screenToPlaneY(mouseX, mouseY, HOLD_PLANE_Y, planePos)) {
                heldDie->pos.x = clamp(planePos.x, -5.35f, 5.35f);
                heldDie->pos.y = HOLD_PLANE_Y;
                heldDie->pos.z = clamp(planePos.z, -3.30f, 3.30f);
            }

            // Elde suruklerken fare hareketine bagli canli donus
            heldDie->angVel = { -deltaY * 0.45f, (deltaX - deltaY) * 0.20f, -deltaX * 0.45f };
        }

        // 3. WM_LBUTTONUP: Zari birak (isHeld = false), fare hizini vel ve angVel'e aktararak firlat!
        void onMouseUp() {
            if (!heldDie) return;

            DiceBody* thrown = heldDie;
            heldDie = nullptr;
            thrown->isHeld = false;
            thrown->settled = false;
            thrown->hitFloorOnce = false;

            // Fare hizini (deltaX, deltaY) 3D dunya hizina (vel) aktar
            float vx = clamp(deltaX * THROW_FORCE, -14.5f, 14.5f);
            float vz = clamp(-deltaY * THROW_FORCE, -14.5f, 14.5f); // Ekran Y asagi, Dunya Z yukari
            float speed = std::sqrt(vx * vx + vz * vz);

            std::uniform_real_distribution<float> jitter(-5.0f, 5.0f);
            if (speed < 0.8f) {
                // Fare hareket ettirilmeden birakildiysa hafif rastgele atis ver
                vx = jitter(rng) * 0.7f;
                vz = jitter(rng) * 0.7f;
                speed = 2.5f;
            }

            float vy = clamp(1.2f + speed * 0.18f, 1.2f, 4.2f);
            thrown->vel = { vx, vy, vz };

            // Firlatma hizina bagli donme hizi (angVel) ver
            thrown->angVel = {
                 vz * 2.8f + jitter(rng),
                (vx - vz) * 1.4f + jitter(rng),
                -vx * 2.8f + jitter(rng)
            };

            state = GameState::PLAYER_ROLLING;
            soundPlayed = false;
            soundDelayTimer = 0.0f;
            statusText = L"SEN zar\u0131 f\u0131rlatt\u0131n!";
            subText = L"Zar fizik motoruyla yuvarlan\u0131yor...";
        }

        void resetScores() {
            if (heldDie) { heldDie->isHeld = false; heldDie = nullptr; }
            playerScore = cpuScore = 0;
            round = 1;
            state = GameState::READY;
            roundCooldown = 0.0f;
            soundPlayed = true;
            audio.stop();
            statusText = L"Skor s\u0131f\u0131rland\u0131! Zara t\u0131klay\u0131p tut ve f\u0131rlat (veya [BO\u015ELUK])";
            subText = L"\u00C7ift atan kazan\u0131r  |  [R] Skoru S\u0131f\u0131rla  |  [ESC] \u00C7\u0131k\u0131\u015F";
        }

        void update(float dt) {
            if (heldDie && heldDie->isHeld) {
                // Fare sabit bekletilirse birikmis firlatma hizini yavasca sifirla
                deltaX *= std::pow(0.02f, dt);
                deltaY *= std::pow(0.02f, dt);
            }

            const int subSteps = 6;
            float subDt = dt / subSteps;
            for (int s = 0; s < subSteps; ++s) {
                die1.update(subDt);
                die2.update(subDt);
                DiceBody::resolveCollision(die1, die2);
            }

            if ((state == GameState::PLAYER_ROLLING || state == GameState::CPU_ROLLING) &&
                !soundPlayed && (die1.hitFloorOnce || die2.hitFloorOnce)) {
                soundDelayTimer += dt;
                if (soundDelayTimer >= SES_GECIKMESI) {
                    audio.play();
                    soundPlayed = true;
                }
            }

            if (state == GameState::PLAYER_ROLLING &&
                !die1.isHeld && !die2.isHeld && die1.settled && die2.settled) {
                p1 = die1.getTopValue();
                p2 = die2.getTopValue();
                statusText = L"SEN ATTIN: " + getTavlaNameW(p1, p2);
                subText = L"S\u0131ra bilgisayarda... (bekleniyor)";
                state = GameState::WAIT_FOR_CPU;
                stateTimer = TUR_BEKLEME_SURESI;
            }
            else if (state == GameState::WAIT_FOR_CPU) {
                stateTimer -= dt;
                if (stateTimer <= 0.0f) {
                    state = GameState::CPU_ROLLING;
                    die1.throwDice(rng, 4.8f, -1.1f, -1.0f);
                    die2.throwDice(rng, 4.8f, 1.1f, -1.0f);
                    soundPlayed = false;
                    soundDelayTimer = 0.0f;
                    statusText = L"B\u0130LG\u0130SAYAR zar at\u0131yor...";
                    subText = L"Senin zar\u0131n: " + getTavlaNameW(p1, p2);
                }
            }
            else if (state == GameState::CPU_ROLLING && die1.settled && die2.settled) {
                c1 = die1.getTopValue();
                c2 = die2.getTopValue();
                int pPow = evaluateTavlaPower(p1, p2);
                int cPow = evaluateTavlaPower(c1, c2);

                if (pPow > cPow) {
                    playerScore++;
                    statusText = L"BU EL SEN\u0130N!  (" + getTavlaNameW(p1, p2) + L"  >  " + getTavlaNameW(c1, c2) + L")";
                }
                else if (cPow > pPow) {
                    cpuScore++;
                    statusText = L"BU EL B\u0130LG\u0130SAYARIN!  (" + getTavlaNameW(p1, p2) + L"  <  " + getTavlaNameW(c1, c2) + L")";
                }
                else {
                    statusText = L"BERABERE!  (" + getTavlaNameW(p1, p2) + L"  =  " + getTavlaNameW(c1, c2) + L")";
                }
                subText = L"Sonu\u00E7 de\u011Ferlendiriliyor...";
                round++;
                roundCooldown = TUR_BEKLEME_SURESI;
                state = GameState::ROUND_OVER;
            }
            else if (state == GameState::ROUND_OVER && roundCooldown > 0.0f) {
                roundCooldown -= dt;
                if (roundCooldown <= 0.0f) {
                    roundCooldown = 0.0f;
                    subText = L"Yeni el i\u00E7in zar\u0131 fareyle tutup f\u0131rlat veya [BO\u015ELUK] tu\u015Funa bas  |  [R] S\u0131f\u0131rla";
                }
            }
        }
    };
}

// ============================================================
// 7. WIN32 PLATFORM & FULL HD CIFT CEKIRDEKLI ANA OYUN DONGUSU
// ============================================================
using namespace Engine::Config;
using namespace Engine::Parallel;
using namespace Engine::Graphics;
using namespace Engine::Physics;
using namespace Tavla;

struct Win32App {
    bool running = true;
    TavlaGame* game = nullptr;
    Renderer3D* renderer = nullptr;
};

LRESULT CALLBACK WindowProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    Win32App* app = reinterpret_cast<Win32App*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    switch (msg) {
    case WM_CLOSE:
    case WM_DESTROY:
        if (app) app->running = false;
        PostQuitMessage(0);
        return 0;
    case WM_KEYDOWN:
        if (app && app->game) {
            if (wParam == VK_SPACE || wParam == VK_RETURN) app->game->triggerRoll();
            if (wParam == 'R') app->game->resetScores();
            if (wParam == VK_ESCAPE) { app->running = false; PostQuitMessage(0); }
        }
        return 0;
    case WM_LBUTTONDOWN:
        if (app && app->game && app->renderer) {
            int mx = int(short(LOWORD(lParam)));
            int my = int(short(HIWORD(lParam)));
            if (app->game->onMouseDown(mx, my, app->renderer->cam)) {
                SetCapture(hwnd); // Fare pencere disina ciksa bile birakma olayini yakala
            }
        }
        return 0;
    case WM_MOUSEMOVE:
        if (app && app->game && app->renderer) {
            int mx = int(short(LOWORD(lParam)));
            int my = int(short(HIWORD(lParam)));
            app->game->onMouseMove(mx, my, app->renderer->cam);
        }
        return 0;
    case WM_LBUTTONUP:
        if (app && app->game) {
            ReleaseCapture();
            app->game->onMouseUp();
        }
        return 0;
    }
    return DefWindowProcW(hwnd, msg, wParam, lParam);
}

void drawHudText(HDC dc, HFONT font, int x, int y, const std::wstring& text, COLORREF color) {
    SelectObject(dc, font);
    SetBkMode(dc, TRANSPARENT);
    SetTextColor(dc, RGB(15, 10, 8));
    TextOutW(dc, x + 2, y + 2, text.c_str(), int(text.size()));
    SetTextColor(dc, color);
    TextOutW(dc, x, y, text.c_str(), int(text.size()));
}

int main() {
    SetProcessDPIAware();

    HINSTANCE hInst = GetModuleHandleW(nullptr);

    WNDCLASSW wc{};
    wc.lpfnWndProc = WindowProc;
    wc.hInstance = hInst;
    wc.lpszClassName = L"VanillaWin32TavlaEngineFHD";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    RegisterClassW(&wc);

    RECT wr{ 0, 0, SCREEN_W, SCREEN_H };
    DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
    AdjustWindowRect(&wr, style, FALSE);

    int winW = wr.right - wr.left;
    int winH = wr.bottom - wr.top;
    int posX = std::max(0, (GetSystemMetrics(SM_CXSCREEN) - winW) / 2);
    int posY = std::max(0, (GetSystemMetrics(SM_CYSCREEN) - winH) / 2);

    HWND hwnd = CreateWindowExW(
        0, wc.lpszClassName, L"Tavla Zar\u0131 - \u00C7ift \u00C7ekirdekli 3D Oyun Motoru (1920x1080 Full HD)",
        style | WS_VISIBLE, posX, posY, winW, winH,
        nullptr, nullptr, hInst, nullptr
    );
    if (!hwnd) return 1;

    HDC windowDC = GetDC(hwnd);
    HDC memDC = CreateCompatibleDC(windowDC);

    BITMAPINFO bmi{};
    bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bmi.bmiHeader.biWidth = SCREEN_W;
    bmi.bmiHeader.biHeight = -SCREEN_H;
    bmi.bmiHeader.biPlanes = 1;
    bmi.bmiHeader.biBitCount = 32;
    bmi.bmiHeader.biCompression = BI_RGB;
    bmi.bmiHeader.biSizeImage = DWORD(size_t(SCREEN_W) * SCREEN_H * sizeof(uint32_t));

    uint32_t* frameBuffer = nullptr;
    HBITMAP dibBitmap = CreateDIBSection(memDC, &bmi, DIB_RGB_COLORS, (void**)&frameBuffer, nullptr, 0);
    if (!dibBitmap || !frameBuffer) return 1;
    SelectObject(memDC, dibBitmap);

    HFONT titleFont = CreateFontW(36, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, 0, 0, CLEARTYPE_QUALITY, 0, L"Segoe UI");
    HFONT subFont = CreateFontW(24, 0, 0, 0, FW_SEMIBOLD, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, 0, 0, CLEARTYPE_QUALITY, 0, L"Segoe UI");

    DualCoreWorker dualCore;
    Renderer3D renderer(dualCore);
    TavlaGame game;
    Win32App app{ true, &game, &renderer };
    SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(&app));

    LARGE_INTEGER freq, prevTime, currTime;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&prevTime);

    static_assert(SCREEN_H % 2 == 0, "SCREEN_H cift sayi olmalidir");
    constexpr int midY = SCREEN_H / 2;

    while (app.running) {
        MSG msg;
        while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) app.running = false;
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }

        QueryPerformanceCounter(&currTime);
        float dt = float(currTime.QuadPart - prevTime.QuadPart) / float(freq.QuadPart);
        prevTime = currTime;
        dt = std::min(dt, 0.04f);

        game.update(dt);

        dualCore.runParallel(
            [&]() {
                renderer.renderSlice(frameBuffer, 0, midY,
                    game.die1.pos, game.die1.rot,
                    game.die2.pos, game.die2.rot, DiceBody::HALF);
            },
            [&]() {
                renderer.renderSlice(frameBuffer, midY, SCREEN_H,
                    game.die1.pos, game.die1.rot,
                    game.die2.pos, game.die2.rot, DiceBody::HALF);
            }
        );

        std::wstring scoreLine = L"TUR " + std::to_wstring(game.round) +
            L"    |    SEN: " + std::to_wstring(game.playerScore) +
            L"   -   B\u0130LG\u0130SAYAR: " + std::to_wstring(game.cpuScore);
        std::wstring infoLine = L"1920x1080  |  2 \u00C7ekirdek: Aktif  |  " +
            std::wstring(game.audio.loaded ? L"Ses: A\u00E7\u0131k" : L"Ses: zar.mp3 yok");

        drawHudText(memDC, titleFont, 36, 24, scoreLine, RGB(248, 224, 168));
        drawHudText(memDC, subFont, SCREEN_W - 560, 30, infoLine,
            game.audio.loaded ? RGB(170, 235, 160) : RGB(240, 180, 140));
        drawHudText(memDC, titleFont, 36, SCREEN_H - 98, game.statusText, RGB(255, 245, 220));
        drawHudText(memDC, subFont, 36, SCREEN_H - 52, game.subText, RGB(210, 195, 170));

        BitBlt(windowDC, 0, 0, SCREEN_W, SCREEN_H, memDC, 0, 0, SRCCOPY);
        Sleep(1);
    }

    dualCore.shutdown();
    game.audio.shutdown();
    DeleteObject(titleFont);
    DeleteObject(subFont);
    DeleteObject(dibBitmap);
    DeleteDC(memDC);
    ReleaseDC(hwnd, windowDC);
    return 0;
}