#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <string>
#include <vector>

constexpr float PI = 3.14159265358979f;
constexpr int SCREEN_W = 960;
constexpr int SCREEN_H = 640;

// ============================================================
// 1. 3D MATEMATIK MOTORU (Vec3, Mat3, Quaternion)
// ============================================================
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
    float length() const { return std::sqrt(lengthSq()); }
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
        float l = std::sqrt(w * w + x * x + y * y + z * z);
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

// ============================================================
// 2. RENK & PROSEDUREL DOKU MOTORU
// ============================================================
inline uint32_t makeRGB(int r, int g, int b) {
    r = std::clamp(r, 0, 255);
    g = std::clamp(g, 0, 255);
    b = std::clamp(b, 0, 255);
    return (uint32_t(r) << 16) | (uint32_t(g) << 8) | uint32_t(b);
}

inline uint32_t scaleRGB(uint32_t col, float s) {
    int r = int(((col >> 16) & 0xFF) * s);
    int g = int(((col >> 8) & 0xFF) * s);
    int b = int((col & 0xFF) * s);
    return makeRGB(r, g, b);
}

struct Texture {
    int size = 128;
    std::vector<uint32_t> pixels;

    void generateDiceFace(int value) {
        pixels.resize(size * size);
        struct Pip { float u, v; };
        std::vector<Pip> pips;

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

                float edgeDist = std::min({ u, 1.0f - u, v, 1.0f - v });
                float bevel = std::clamp(edgeDist / 0.09f, 0.76f, 1.0f);
                int r = int(246 * bevel);
                int g = int(238 * bevel);
                int b = int(218 * bevel);

                for (const auto& p : pips) {
                    float dx = u - p.u;
                    float dy = v - p.v;
                    float d = std::sqrt(dx * dx + dy * dy);
                    if (d < pipRadius) {
                        float edge = std::clamp((pipRadius - d) / 0.018f, 0.0f, 1.0f);
                        int pr = (value == 1) ? 165 : 26;
                        int pg = (value == 1) ? 26 : 24;
                        int pb = (value == 1) ? 26 : 26;
                        float shade = 0.75f + 0.35f * std::clamp((dy / pipRadius + 1.0f) * 0.5f, 0.0f, 1.0f);
                        r = int(r * (1.0f - edge) + pr * shade * edge);
                        g = int(g * (1.0f - edge) + pg * shade * edge);
                        b = int(b * (1.0f - edge) + pb * shade * edge);
                    }
                }
                pixels[y * size + x] = makeRGB(r, g, b);
            }
        }
    }

    uint32_t sample(float u, float v) const {
        int ix = std::clamp(int(u * size), 0, size - 1);
        int iy = std::clamp(int(v * size), 0, size - 1);
        return pixels[iy * size + ix];
    }
};

// ============================================================
// 3. YAZILIMSAL 3D RENDER MOTORU (3D Kenarlar + Z-Buffer)
// ============================================================
struct Camera {
    Vec3 pos{ 0.0f, 11.8f, -8.2f };
    Vec3 target{ 0.0f, 0.0f, 0.3f };
    Vec3 forward, right, up;
    float fov = 680.0f;

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
};

struct ScreenVert {
    float sx, sy, invZ, uOverZ, vOverZ;
};

class Renderer3D {
public:
    Camera cam;
    Vec3 lightDir = Vec3{ 0.42f, 0.82f, -0.38f }.normalized();
    std::vector<uint32_t> backgroundBuffer;
    std::vector<float> backgroundZBuffer;
    std::vector<float> zBuffer;
    Texture diceTextures[7];

    struct AABB {
        Vec3 bmin, bmax;
        bool isHorizontal;
    };

    Renderer3D()
        : backgroundBuffer(SCREEN_W* SCREEN_H),
        backgroundZBuffer(SCREEN_W* SCREEN_H, 0.0f),
        zBuffer(SCREEN_W* SCREEN_H, 0.0f) {
        for (int i = 1; i <= 6; ++i) diceTextures[i].generateDiceFace(i);
        bakeBackgammonBoard();
    }

    static bool intersectAABB(const Vec3& orig, const Vec3& dir, const AABB& box,
        float& outT, Vec3& outNormal) {
        float tmin = -1e9f, tmax = 1e9f;
        Vec3 nMin{ 0, 0, 0 };

        auto slab = [&](float o, float d, float mn, float mx, Vec3 negN, Vec3 posN) -> bool {
            if (std::abs(d) < 1e-6f) return (o >= mn && o <= mx);
            float invD = 1.0f / d;
            float t1 = (mn - o) * invD;
            float t2 = (mx - o) * invD;
            Vec3 n = negN;
            if (t1 > t2) { std::swap(t1, t2); n = posN; }
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

    void bakeBackgammonBoard() {
        constexpr float wallTop = 0.78f;
        constexpr float boxBottom = -0.34f;

        const AABB walls[4] = {
            { { -6.85f, boxBottom,  3.95f },{ 6.85f, wallTop,  4.75f }, true },
            { { -6.85f, boxBottom, -4.75f },{ 6.85f, wallTop, -3.95f }, true },
            { { -6.85f, boxBottom, -3.95f },{ -6.00f, wallTop,  3.95f }, false },
            { { 6.00f, boxBottom, -3.95f },{ 6.85f, wallTop,  3.95f }, false }
        };
        const AABB fullBox{ { -6.85f, boxBottom, -4.75f },{ 6.85f, wallTop, 4.75f }, true };

        for (int py = 0; py < SCREEN_H; ++py) {
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
                        bestT = t;
                        hitWallIdx = w;
                        hitNormal = n;
                    }
                }

                bool hitFloor = false;
                if (dir.y < -0.001f) {
                    float tFloor = -cam.pos.y / dir.y;
                    if (tFloor > 0.0f && tFloor < bestT) {
                        Vec3 pf = cam.pos + dir * tFloor;
                        if (std::abs(pf.x) <= 6.01f && std::abs(pf.z) <= 3.96f) {
                            bestT = tFloor;
                            hitWallIdx = -1;
                            hitFloor = true;
                            hitNormal = { 0, 1, 0 };
                        }
                    }
                }

                uint32_t col = makeRGB(22, 18, 16);
                float bakedInvZ = 0.0f;

                if (hitWallIdx != -1) {
                    Vec3 p = cam.pos + dir * bestT;
                    const AABB& w = walls[hitWallIdx];

                    float gCoord = w.isHorizontal
                        ? (p.z * 14.0f + p.y * 12.0f + std::sin(p.x * 2.2f) * 1.4f)
                        : (p.x * 14.0f + p.y * 12.0f + std::sin(p.z * 2.2f) * 1.4f);
                    float grain = std::sin(gCoord) * 0.5f + 0.5f;
                    float woodMod = 0.84f + 0.20f * grain;

                    float diff = std::max(0.0f, hitNormal.dot(lightDir));
                    float light = 0.34f + 0.72f * diff;

                    float distTop = std::abs(p.y - wallTop);
                    float edgeHighlight = 1.0f;
                    if (hitNormal.y > 0.5f) {
                        float dEdge = std::min({
                            std::abs(p.x - w.bmin.x), std::abs(p.x - w.bmax.x),
                            std::abs(p.z - w.bmin.z), std::abs(p.z - w.bmax.z)
                            });
                        if (dEdge < 0.07f) edgeHighlight = 1.15f;
                    }
                    else if (distTop < 0.06f) {
                        edgeHighlight = 1.14f;
                    }

                    float ao = std::clamp(0.76f + (p.y - boxBottom) * 0.30f, 0.76f, 1.0f);
                    int r = int(106 * woodMod * light * edgeHighlight * ao);
                    int g = int(60 * woodMod * light * edgeHighlight * ao);
                    int b = int(32 * woodMod * light * edgeHighlight * ao);

                    // Pirinc mentese detayi
                    if (w.isHorizontal && hitNormal.y > 0.5f && std::abs(p.x) < 0.30f &&
                        std::abs(p.z) > 4.10f && std::abs(p.z) < 4.60f) {
                        float shine = 0.85f + 0.25f * std::sin(p.x * 18.0f);
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
                    float grain = std::sin(p.x * 9.0f + std::sin(p.z * 2.5f) * 1.8f) * 0.5f + 0.5f;
                    float fine = std::sin(p.z * 32.0f + p.x * 4.0f) * 0.04f;
                    float woodMod = 0.88f + 0.16f * grain + fine;

                    int br = int(176 * woodMod);
                    int bg = int(136 * woodMod);
                    int bb = int(90 * woodMod);

                    if (std::abs(p.x) < 0.38f) {
                        float bevel = (std::abs(p.x) > 0.31f) ? 0.72f : 1.0f;
                        br = int(92 * woodMod * bevel);
                        bg = int(54 * woodMod * bevel);
                        bb = int(30 * woodMod * bevel);
                    }
                    else {
                        float halfStart = (p.x < 0) ? -6.0f : 0.38f;
                        float slotW = (6.0f - 0.38f) / 6.0f;
                        float relX = p.x - halfStart;
                        int slotIdx = std::clamp(int(relX / slotW), 0, 5);
                        float u = (relX - slotIdx * slotW) / slotW;

                        float distFromEdge = 3.95f - std::abs(p.z);
                        float triLen = 3.05f;
                        if (distFromEdge >= 0.0f && distFromEdge < triLen) {
                            float halfWidthAtZ = 0.47f * (1.0f - distFromEdge / triLen);
                            if (std::abs(u - 0.5f) < halfWidthAtZ) {
                                bool isDark = ((slotIdx + (p.z > 0 ? 1 : 0)) % 2 == 0);
                                if (isDark) {
                                    br = int(118 * woodMod); bg = int(34 * woodMod); bb = int(26 * woodMod);
                                }
                                else {
                                    br = int(224 * woodMod); bg = int(204 * woodMod); bb = int(164 * woodMod);
                                }
                            }
                        }
                    }

                    Vec3 shadowOrig = p + Vec3{ 0, 0.01f, 0 };
                    bool inShadow = false;
                    for (int w = 0; w < 4; ++w) {
                        float st; Vec3 sn;
                        if (intersectAABB(shadowOrig, lightDir, walls[w], st, sn)) {
                            inShadow = true;
                            break;
                        }
                    }

                    float wallAO = std::min({
                        1.0f,
                        (6.0f - std::abs(p.x)) * 1.15f + 0.58f,
                        (3.95f - std::abs(p.z)) * 1.15f + 0.58f
                        });
                    float shadowMul = inShadow ? 0.56f : 1.0f;
                    col = makeRGB(int(br * wallAO * shadowMul),
                        int(bg * wallAO * shadowMul),
                        int(bb * wallAO * shadowMul));
                }
                else if (dir.y < -0.001f) {
                    float tTable = (boxBottom - cam.pos.y) / dir.y;
                    if (tTable > 0.0f) {
                        Vec3 pt = cam.pos + dir * tTable;
                        float dist = std::sqrt(pt.x * pt.x + pt.z * pt.z);
                        float vig = std::clamp(1.12f - dist * 0.052f, 0.25f, 1.0f);

                        float st; Vec3 sn;
                        bool boxShadow = intersectAABB(pt + Vec3{ 0, 0.01f, 0 }, lightDir, fullBox, st, sn);
                        float sm = boxShadow ? 0.42f : 1.0f;
                        col = makeRGB(int(48 * vig * sm), int(38 * vig * sm), int(32 * vig * sm));
                    }
                }

                int idx = py * SCREEN_W + px;
                backgroundBuffer[idx] = col;
                backgroundZBuffer[idx] = bakedInvZ;
            }
        }
    }

    void beginFrame(uint32_t* frameBuffer) {
        std::memcpy(frameBuffer, backgroundBuffer.data(), SCREEN_W * SCREEN_H * sizeof(uint32_t));
        std::memcpy(zBuffer.data(), backgroundZBuffer.data(), SCREEN_W * SCREEN_H * sizeof(float));
    }

    void drawShadow(uint32_t* frameBuffer, const Vec3& diePos, float halfSize) {
        Vec3 shadowWorld{ diePos.x - lightDir.x * diePos.y * 0.65f, 0.01f, diePos.z - lightDir.z * diePos.y * 0.65f };
        float sx, sy, cz;
        if (!cam.project(shadowWorld, sx, sy, cz)) return;

        float radiusWorld = halfSize * (1.30f + diePos.y * 0.08f);
        float rx = (radiusWorld / cz) * cam.fov;
        float ry = rx * 0.72f;
        float strength = std::clamp(0.48f - diePos.y * 0.06f, 0.10f, 0.48f);

        int x0 = std::max(0, int(sx - rx));
        int x1 = std::min(SCREEN_W - 1, int(sx + rx));
        int y0 = std::max(0, int(sy - ry));
        int y1 = std::min(SCREEN_H - 1, int(sy + ry));

        for (int y = y0; y <= y1; ++y) {
            float dy = (y - sy) / ry;
            for (int x = x0; x <= x1; ++x) {
                int idx = y * SCREEN_W + x;
                if (zBuffer[idx] > 0.0f) continue;
                float dx = (x - sx) / rx;
                float d2 = dx * dx + dy * dy;
                if (d2 < 1.0f) {
                    float falloff = (1.0f - d2) * strength;
                    frameBuffer[idx] = scaleRGB(frameBuffer[idx], 1.0f - falloff);
                }
            }
        }
    }

    static float edgeFunc(float ax, float ay, float bx, float by, float cx, float cy) {
        return (cx - ax) * (by - ay) - (cy - ay) * (bx - ax);
    }

    void drawTriangle(uint32_t* fb, ScreenVert v0, ScreenVert v1, ScreenVert v2,
        const Texture& tex, float lighting) {
        float area = edgeFunc(v0.sx, v0.sy, v1.sx, v1.sy, v2.sx, v2.sy);
        if (std::abs(area) < 0.01f) return;
        if (area < 0.0f) { std::swap(v1, v2); area = -area; }
        float invArea = 1.0f / area;

        int minX = std::max(0, int(std::floor(std::min({ v0.sx, v1.sx, v2.sx }))));
        int maxX = std::min(SCREEN_W - 1, int(std::ceil(std::max({ v0.sx, v1.sx, v2.sx }))));
        int minY = std::max(0, int(std::floor(std::min({ v0.sy, v1.sy, v2.sy }))));
        int maxY = std::min(SCREEN_H - 1, int(std::ceil(std::max({ v0.sy, v1.sy, v2.sy }))));

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
                    int idx = y * SCREEN_W + x;

                    if (invZ > zBuffer[idx]) {
                        zBuffer[idx] = invZ;
                        float z = 1.0f / invZ;
                        float u = (w0 * v0.uOverZ + w1 * v1.uOverZ + w2 * v2.uOverZ) * z;
                        float v = (w0 * v0.vOverZ + w1 * v1.vOverZ + w2 * v2.vOverZ) * z;
                        uint32_t texel = tex.sample(u, v);
                        fb[idx] = scaleRGB(texel, lighting);
                    }
                }
            }
        }
    }

    void drawDice(uint32_t* fb, const Vec3& pos, const Quat& rot, float halfSize) {
        Mat3 R = rot.toMat3();

        struct FaceDef {
            int val;
            Vec3 n, uAxis, vAxis;
        };
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
            Vec3 faceCenter = pos + worldN * halfSize;
            Vec3 toCam = (cam.pos - faceCenter).normalized();

            if (worldN.dot(toCam) <= 0.0f) continue;

            float diffuse = std::max(0.0f, worldN.dot(lightDir));
            Vec3 halfVec = (lightDir + toCam).normalized();
            float spec = std::pow(std::max(0.0f, worldN.dot(halfVec)), 24.0f) * 0.25f;
            float lighting = std::clamp(0.42f + 0.62f * diffuse + spec, 0.35f, 1.18f);

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

            drawTriangle(fb, sv[0], sv[1], sv[2], diceTextures[f.val], lighting);
            drawTriangle(fb, sv[0], sv[2], sv[3], diceTextures[f.val], lighting);
        }
    }
};

// ============================================================
// 4. 3D ZAR FIZIK MOTORU (Gercek Yuvarlanma & Carpisma)
// ============================================================
struct DiceBody {
    static constexpr float HALF = 0.34f;
    Vec3 pos{ 0, HALF, 0 };
    Vec3 vel{ 0, 0, 0 };
    Vec3 angVel{ 0, 0, 0 };
    Quat rot{ 1, 0, 0, 0 };
    bool settled = true;
    bool hitFloorOnce = false; // <--- 1) EKLENDI

    void throwDice(std::mt19937& rng, float startX, float startZ, float dirX) {
        std::uniform_real_distribution<float> f01(0.0f, 1.0f);
        pos = { startX, 2.8f + f01(rng) * 1.2f, startZ };
        vel = { dirX * (5.8f + f01(rng) * 4.2f), 1.8f + f01(rng) * 2.2f, (f01(rng) - 0.5f) * 6.0f };
        angVel = { (f01(rng) - 0.5f) * 28.0f, (f01(rng) - 0.5f) * 20.0f, (f01(rng) - 0.5f) * 28.0f };
        rot = Quat{ f01(rng) - 0.5f, f01(rng) - 0.5f, f01(rng) - 0.5f, f01(rng) - 0.5f }.normalized();
        settled = false;
        hitFloorOnce = false;
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
            if (wn.y > bestY) {
                bestY = wn.y;
                bestVal = val;
                bestN = wn;
            }
        }
        if (outTopWorldNormal) *outTopWorldNormal = bestN;
        return bestVal;
    }

    void update(float dt) {
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

            // Normal itki (sekme + kose devrilme torku)
            float jn = 0.0f;
            if (vContact.y < 0.0f) {
                Vec3 n{ 0, 1, 0 };
                Vec3 rCrossN = r.cross(n);
                float denom = 1.0f + rCrossN.dot(rCrossN) * invI;
                float restitution = (vContact.y < -1.2f) ? 0.44f : 0.0f;
                jn = -(1.0f + restitution) * vContact.y / denom;

                vel.y += jn;
                angVel += r.cross(Vec3{ 0, jn, 0 }) * invI;
            }

            // Tegetsel surtunme itkisi (kaymayi keser, hizi yuvarlanmaya cevirir)
            vContact = vel + angVel.cross(r);
            Vec3 vTan{ vContact.x, 0.0f, vContact.z };
            float vTanLen = vTan.length();
            if (vTanLen > 1e-4f) {
                Vec3 t = vTan * (1.0f / vTanLen);
                Vec3 rCrossT = r.cross(t);
                float denomT = 1.0f + rCrossT.dot(rCrossT) * invI;

                float jt = -vTanLen / denomT;
                float normalLoad = std::max(jn, gravity * dt);
                float maxFriction = 0.92f * normalLoad;
                jt = std::max(jt, -maxFriction);

                Vec3 impT = t * jt;
                vel += impT;
                angVel += r.cross(impT) * invI;
            }

            angVel.y *= std::pow(0.10f, dt);

            if (vel.lengthSq() < 0.55f && angVel.lengthSq() < 8.0f && std::abs(vel.y) < 0.4f) {
                Vec3 topN;
                getTopValue(&topN);
                Vec3 c{ -topN.z, 0.0f, topN.x };
                Quat qAlign = Quat{ 1.0f + topN.y, c.x, c.y, c.z }.normalized();
                Quat qTarget = (qAlign * rot).normalized();

                float settleStrength = (contactCount >= 4) ? 14.0f : 6.0f;
                rot = Quat::nlerp(rot, qTarget, std::min(1.0f, dt * settleStrength));
                vel.x *= std::pow(0.08f, dt);
                vel.z *= std::pow(0.08f, dt);
                angVel = angVel * std::pow(0.06f, dt);

                if (vel.lengthSq() < 0.015f && angVel.lengthSq() < 0.12f && topN.y > 0.994f) {
                    rot = qTarget;
                    pos.y = HALF;
                    vel = { 0, 0, 0 };
                    angVel = { 0, 0, 0 };
                    settled = true;
                }
            }
        }

        // 3D Ahsap duvarlardan sekme
        constexpr float wallX = 5.52f;
        constexpr float wallZ = 3.48f;
        if (pos.x < -wallX) { pos.x = -wallX; vel.x = std::abs(vel.x) * 0.58f; angVel.z -= vel.x * 2.2f; }
        if (pos.x > wallX) { pos.x = wallX; vel.x = -std::abs(vel.x) * 0.58f; angVel.z -= vel.x * 2.2f; }
        if (pos.z < -wallZ) { pos.z = -wallZ; vel.z = std::abs(vel.z) * 0.58f; angVel.x += vel.z * 2.2f; }
        if (pos.z > wallZ) { pos.z = wallZ; vel.z = -std::abs(vel.z) * 0.58f; angVel.x += vel.z * 2.2f; }

        if (!onFloor) {
            angVel = angVel * std::pow(0.92f, dt);
        }
    }

    static void resolveCollision(DiceBody& a, DiceBody& b) {
        Vec3 diff = b.pos - a.pos;
        float dist = diff.length();
        float minDist = HALF * 2.25f;
        if (dist < minDist && dist > 1e-4f) {
            Vec3 n = diff * (1.0f / dist);
            float overlap = (minDist - dist) * 0.5f;
            a.pos = a.pos - n * overlap;
            b.pos = b.pos + n * overlap;

            Vec3 relVel = b.vel - a.vel;
            float vn = relVel.dot(n);
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

// ============================================================
// 5. WIN32 MCI MP3 SES MOTORU (Harici kutuphanesiz)
// ============================================================
class Win32Audio {
public:
    bool loaded = false;

    bool tryOpen(const std::wstring& fullPath) {
        std::wstring cmd = L"open \"" + fullPath + L"\" type mpegvideo alias diceSfx";
        if (mciSendStringW(cmd.c_str(), nullptr, 0, nullptr) == 0) {
            loaded = true;
            mciSendStringW(L"setaudio diceSfx volume to 900", nullptr, 0, nullptr);
            return true;
        }
        return false;
    }

    bool load(const std::wstring& fileName) {
        if (tryOpen(fileName)) return true;

        wchar_t exePath[MAX_PATH]{};
        GetModuleFileNameW(nullptr, exePath, MAX_PATH);
        std::wstring dir(exePath);
        size_t slash = dir.find_last_of(L"\\/");
        if (slash != std::wstring::npos) {
            return tryOpen(dir.substr(0, slash + 1) + fileName);
        }
        return false;
    }

    void play() {
        if (!loaded) return;
        mciSendStringW(L"play diceSfx from 0", nullptr, 0, nullptr);
    }

    void stop() {
        if (!loaded) return;
        mciSendStringW(L"stop diceSfx", nullptr, 0, nullptr);
    }

    void shutdown() {
        if (loaded) {
            mciSendStringW(L"close diceSfx", nullptr, 0, nullptr);
            loaded = false;
        }
    }
};

// ============================================================
// 6. OYUN MANTIGI & GELENEKSEL TURKCE TAVLA ISIMLERI
// ============================================================
std::wstring getTavlaNameW(int d1, int d2) {
    if (d1 < d2) std::swap(d1, d2);
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
    return std::wstring(names[d1]) + L"-\u00FC " + names[d2] +
        L" (" + std::to_wstring(d1) + L"-" + std::to_wstring(d2) + L")";
}

int evaluateTavlaPower(int d1, int d2) {
    if (d1 == d2) return 100 + d1 * 10;
    return d1 + d2;
}

enum class GameState { READY, PLAYER_ROLLING, WAIT_FOR_CPU, CPU_ROLLING, ROUND_OVER };

struct TavlaGame {
    // --- SURE AYARLARI (Saniye cinsinden istedigin gibi degistirebilirsin) ---
    static constexpr float TUR_BEKLEME_SURESI = 2.0f; // Oyuncu ve CPU atislari sonrasi bekleme (sn)
    static constexpr float SES_GECIKMESI = 0.0f; // Yere degdikten kac sn sonra ses calsin (0.0 = tam temas ani)

    std::mt19937 rng{ std::random_device{}() };
    DiceBody die1, die2;
    Win32Audio audio;
    GameState state = GameState::READY;

    int playerScore = 0, cpuScore = 0, round = 1;
    int p1 = 0, p2 = 0, c1 = 0, c2 = 0;

    float stateTimer = 0.0f;
    float roundCooldown = 0.0f;
    bool soundPlayed = true;
    float soundDelayTimer = 0.0f;

    std::wstring statusText = L"Zar atmak i\u00E7in [BO\u015ELUK] tu\u015Funa bas veya fareyle t\u0131kla!";
    std::wstring subText = L"\u00C7ift atan kazan\u0131r  |  [R] Skoru S\u0131f\u0131rla  |  [ESC] \u00C7\u0131k\u0131\u015F";

    TavlaGame() {
        die1.pos = { -0.8f, DiceBody::HALF,  0.15f };
        die2.pos = { 0.8f, DiceBody::HALF, -0.15f };
        audio.load(L"zar.mp3");
    }

    void triggerRoll() {
        // Bilgisayar atisindan sonraki bekleme suresi bitmeden yeni el baslamasin
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

    void resetScores() {
        playerScore = cpuScore = 0;
        round = 1;
        state = GameState::READY;
        roundCooldown = 0.0f;
        soundPlayed = true;
        audio.stop();
        statusText = L"Skor s\u0131f\u0131rland\u0131! Zar atmak i\u00E7in [BO\u015ELUK] veya T\u0131kla";
        subText = L"\u00C7ift atan kazan\u0131r  |  [R] Skoru S\u0131f\u0131rla  |  [ESC] \u00C7\u0131k\u0131\u015F";
    }

    void update(float dt) {
        const int subSteps = 6;
        float subDt = dt / subSteps;
        for (int s = 0; s < subSteps; ++s) {
            die1.update(subDt);
            die2.update(subDt);
            DiceBody::resolveCollision(die1, die2);
        }

        // Zarlar havadayken ses calma -> Ilk zar ahsaba degdigi an (+ varsa ek gecikmeyle) cal!
        if ((state == GameState::PLAYER_ROLLING || state == GameState::CPU_ROLLING) &&
            !soundPlayed && (die1.hitFloorOnce || die2.hitFloorOnce)) {
            soundDelayTimer += dt;
            if (soundDelayTimer >= SES_GECIKMESI) {
                audio.play();
                soundPlayed = true;
            }
        }

        if (state == GameState::PLAYER_ROLLING && die1.settled && die2.settled) {
            p1 = die1.getTopValue();
            p2 = die2.getTopValue();
            statusText = L"SEN ATTIN: " + getTavlaNameW(p1, p2);
            subText = L"S\u0131ra bilgisayarda... (bekleniyor)";
            state = GameState::WAIT_FOR_CPU;
            stateTimer = TUR_BEKLEME_SURESI; // Oyuncu sonrasi bekleme
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
            roundCooldown = TUR_BEKLEME_SURESI; // CPU sonrasi bekleme
            state = GameState::ROUND_OVER;
        }
        else if (state == GameState::ROUND_OVER && roundCooldown > 0.0f) {
            roundCooldown -= dt;
            if (roundCooldown <= 0.0f) {
                roundCooldown = 0.0f;
                subText = L"Yeni el i\u00E7in [BO\u015ELUK] tu\u015Funa bas veya t\u0131kla  |  [R] S\u0131f\u0131rla";
            }
        }
    }
};

// ============================================================
// 7. WIN32 PENCERE & GDI OYUN MOTORU KATMANI
// ============================================================
struct Win32App {
    bool running = true;
    TavlaGame* game = nullptr;
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
        if (app && app->game) app->game->triggerRoll();
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
    HINSTANCE hInst = GetModuleHandleW(nullptr);

    WNDCLASSW wc{};
    wc.lpfnWndProc = WindowProc;
    wc.hInstance = hInst;
    wc.lpszClassName = L"VanillaWin32TavlaEngine";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    RegisterClassW(&wc);

    RECT wr{ 0, 0, SCREEN_W, SCREEN_H };
    DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
    AdjustWindowRect(&wr, style, FALSE);

    HWND hwnd = CreateWindowExW(
        0, wc.lpszClassName, L"Tavla Zar\u0131 - 3D Oyun Motoru",
        style | WS_VISIBLE, CW_USEDEFAULT, CW_USEDEFAULT,
        wr.right - wr.left, wr.bottom - wr.top,
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

    uint32_t* frameBuffer = nullptr;
    HBITMAP dibBitmap = CreateDIBSection(memDC, &bmi, DIB_RGB_COLORS, (void**)&frameBuffer, nullptr, 0);
    SelectObject(memDC, dibBitmap);

    HFONT titleFont = CreateFontW(26, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, 0, 0, CLEARTYPE_QUALITY, 0, L"Segoe UI");
    HFONT subFont = CreateFontW(19, 0, 0, 0, FW_SEMIBOLD, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, 0, 0, CLEARTYPE_QUALITY, 0, L"Segoe UI");

    Renderer3D renderer;
    TavlaGame game;
    Win32App app{ true, &game };
    SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(&app));

    LARGE_INTEGER freq, prevTime, currTime;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&prevTime);

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

        renderer.beginFrame(frameBuffer);
        renderer.drawShadow(frameBuffer, game.die1.pos, DiceBody::HALF);
        renderer.drawShadow(frameBuffer, game.die2.pos, DiceBody::HALF);
        renderer.drawDice(frameBuffer, game.die1.pos, game.die1.rot, DiceBody::HALF);
        renderer.drawDice(frameBuffer, game.die2.pos, game.die2.rot, DiceBody::HALF);

        std::wstring scoreLine = L"TUR " + std::to_wstring(game.round) +
            L"    |    SEN: " + std::to_wstring(game.playerScore) +
            L"   -   B\u0130LG\u0130SAYAR: " + std::to_wstring(game.cpuScore);
        std::wstring audioLine = game.audio.loaded
            ? L"Ses: A\u00E7\u0131k"
            : L"Ses: zar.mp3 bulunamad\u0131";

        drawHudText(memDC, titleFont, 24, 16, scoreLine, RGB(248, 224, 168));
        drawHudText(memDC, subFont, SCREEN_W - 230, 20, audioLine,
            game.audio.loaded ? RGB(170, 235, 160) : RGB(240, 150, 140));
        drawHudText(memDC, titleFont, 24, SCREEN_H - 68, game.statusText, RGB(255, 245, 220));
        drawHudText(memDC, subFont, 24, SCREEN_H - 36, game.subText, RGB(210, 195, 170));

        BitBlt(windowDC, 0, 0, SCREEN_W, SCREEN_H, memDC, 0, 0, SRCCOPY);
        Sleep(1);
    }

    game.audio.shutdown();
    DeleteObject(titleFont);
    DeleteObject(subFont);
    DeleteObject(dibBitmap);
    DeleteDC(memDC);
    ReleaseDC(hwnd, windowDC);
    return 0;
}