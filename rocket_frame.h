#pragma once

#include <Arduino.h>
#include <math.h>

// Preserve the existing DISPLAY/TELEMETRY frame S (not the physical body R).
//   X_S <- Y_BNO, Y_S <- Z_BNO, Z_S <- X_BNO.
// User's bench observation: rotation about screen Z is axial (nose/tail).
// Therefore use Z_S as the longitudinal axis for tilt. The physical +nose
// sign and transverse mounting signs still require validation before flight.
// Body naming convention: X_R=Z_S, Y_R=X_S, Z_R=Y_S (right-handed).
// BNO_ROCKET_* identifiers below are retained for compatibility; they map to S.
enum RocketAxisSource : uint8_t {
    BNO_AXIS_X = 0x00,
    BNO_AXIS_Y = 0x01,
    BNO_AXIS_Z = 0x02,
};

constexpr uint8_t bnoAxisMapByte(
    RocketAxisSource rocketXSource,
    RocketAxisSource rocketYSource,
    RocketAxisSource rocketZSource) {
    return static_cast<uint8_t>(rocketXSource) |
           (static_cast<uint8_t>(rocketYSource) << 2) |
           (static_cast<uint8_t>(rocketZSource) << 4);
}

constexpr uint8_t BNO_ROCKET_AXIS_MAP_CONFIG =
    bnoAxisMapByte(BNO_AXIS_Y, BNO_AXIS_Z, BNO_AXIS_X);
constexpr uint8_t BNO_ROCKET_AXIS_MAP_SIGN = 0x00;

constexpr int rocketMapSource(int axis) {
    return (BNO_ROCKET_AXIS_MAP_CONFIG >> (axis * 2)) & 3;
}
constexpr int rocketMapSign(int axis) {
    // Bosch AXIS_MAP_SIGN: bit 2=X, bit 1=Y, bit 0=Z.
    return (BNO_ROCKET_AXIS_MAP_SIGN >> (2 - axis)) & 1;
}
static_assert(rocketMapSource(0) < 3 && rocketMapSource(1) < 3 && rocketMapSource(2) < 3 &&
              rocketMapSource(0) != rocketMapSource(1) &&
              rocketMapSource(0) != rocketMapSource(2) &&
              rocketMapSource(1) != rocketMapSource(2), "Each sensor axis must occur once");
static_assert((((rocketMapSource(0) > rocketMapSource(1)) +
                (rocketMapSource(0) > rocketMapSource(2)) +
                (rocketMapSource(1) > rocketMapSource(2)) +
                rocketMapSign(0) + rocketMapSign(1) + rocketMapSign(2)) % 2) == 0,
              "Mapping must preserve the right-handed frame (determinant +1)");

inline void rocketMapVector(const float sensor[3], float body[3]) {
    for (int i = 0; i < 3; ++i)
        body[i] = (rocketMapSign(i) ? -1.0f : 1.0f) * sensor[rocketMapSource(i)];
}

struct RocketQuaternion { float w, x, y, z; };
struct RocketAngles { float roll, pitch, yaw, tilt; };

inline bool rocketNormalize(RocketQuaternion& q) {
    float n = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
    if (!isfinite(n) || n < 1.0e-8f) return false;
    float s = 1.0f / sqrtf(n);
    q.w *= s; q.x *= s; q.y *= s; q.z *= s;
    return true;
}
inline RocketQuaternion rocketMultiply(const RocketQuaternion& a, const RocketQuaternion& b) {
    return {a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
            a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
            a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
            a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};
}
inline float rocketClampUnit(float x) { return x < -1 ? -1 : (x > 1 ? 1 : x); }

// q maps local sensor/display vectors to the fusion world frame. Reference is
// captured ONLY while the rocket is stationary, nose up. Not in-flight zeroing.
// Hardware remains identity-mapped: change basis by M*Rrelative*M^T.
// For proper rotations this maps the relative quaternion's vector part by M.
inline bool rocketRelativeAngles(RocketQuaternion q, RocketQuaternion reference,
                                 bool sensorFrame, RocketAngles& out) {
    out = {};
    if (!rocketNormalize(q) || !rocketNormalize(reference)) return false;
    reference.x = -reference.x; reference.y = -reference.y; reference.z = -reference.z;
    RocketQuaternion d = rocketMultiply(reference, q);
    if (sensorFrame) {
        const float v[3] = {d.x,d.y,d.z}; float mapped[3];
        rocketMapVector(v,mapped);
        d.x=mapped[0]; d.y=mapped[1]; d.z=mapped[2];
    }
    if (!rocketNormalize(d)) return false;
    const float degrees = 57.2957795131f;
    // Existing DISPLAY X/Y/Z values stay exactly unchanged (ZYX decomposition).
    // roll/pitch/yaw here name Euler components, not physical axial roll.
    out.roll = atan2f(2*(d.w*d.x+d.y*d.z),1-2*(d.x*d.x+d.y*d.y))*degrees;
    out.pitch = asinf(rocketClampUnit(2*(d.w*d.y-d.z*d.x)))*degrees;
    out.yaw = atan2f(2*(d.w*d.z+d.x*d.y),1-2*(d.y*d.y+d.z*d.z))*degrees;
    // dot(reference Z_S, current Z_S) = Rrelative[2][2]. Range 0..180 deg.
    // Z_S is the observed longitudinal axis (= body X_R). A local Z_S spin
    // leaves this dot product unchanged, even when already tilted.
    // Reversing the chosen nose axis in BOTH poses does not change this angle.
    out.tilt = acosf(rocketClampUnit(1-2*(d.x*d.x+d.y*d.y)))*degrees;
    return true;
}

struct RocketQuaternionMean {
    RocketQuaternion sum = {0,0,0,0};
    RocketQuaternion anchor = {1,0,0,0};
    uint8_t count = 0;
    void reset() { sum = {0,0,0,0}; count = 0; }
    bool add(RocketQuaternion q) {
        if (!rocketNormalize(q) || count == 255) return false;
        if (!count) anchor = q;
        float sign = (anchor.w*q.w+anchor.x*q.x+anchor.y*q.y+anchor.z*q.z < 0) ? -1 : 1;
        sum.w += sign*q.w; sum.x += sign*q.x; sum.y += sign*q.y; sum.z += sign*q.z;
        ++count; return true;
    }
    bool finish(RocketQuaternion& result) const {
        result = sum;
        return count > 0 && rocketNormalize(result);
    }
};

/* BODY FRAME (choose and physically mark the viewing side before tests):
                  +X_R  NOSE
                     ^
                     |
                   [UKB] ----> +Y_R (right in this chosen front view)
                     |
                     v
                  -X_R  TAIL
   +Z_R points INTO this drawing: X_R cross Y_R = Z_R.
   DISPLAY frame is separate: X_R=Z_S, Y_R=X_S, Z_R=Y_S.
   Display X/Y are tilt axes; display Z is axial rotation. Physical signs
   are provisional; verify with axis markings and the six-face test.
   A: nose up, hold still while reference is captured: tilt near 0.
   B/C: rotate about display X or Y by 30/60/90 degrees: tilt follows.
   D: spin about nose/tail (display Z): Z changes, tilt stays unchanged.
   At pitch +/-90 Euler decomposition is singular; use tilt, NOT Euler axes,
   to judge the result. Test signs near 30-60 degrees as well.

   Hardware uses MAP=0x24, SIGN=0 (native axes).
   Software X/Y/Z retain the DISPLAY mapping; no diagnostic/visualizer service.
   STATIC SIX-FACE TEST: point each marked +body axis vertically up then down;
   note which native ACC axis reads approximately +9.81 then -9.81 m/s^2.
   Put that source in the corresponding map slot; invert its sign if needed.
   Confirm signs with the chip/breakout axis marks; linear accel is NOT useful
   for this test because it removes gravity. Cross-check A-D and handedness.
   Never remap Euler triplets as if they were Cartesian vectors.
   Normal flight and SIT/SUT loops are active. Disconnect energetic loads
   for bench validation; this header is not an arming/safety interlock.
*/

inline float signedAngleDelta(float current, float reference) {
    float delta = fmodf(current - reference + 180.0f, 360.0f);
    if (delta < 0.0f) delta += 360.0f;
    return delta - 180.0f;
}
