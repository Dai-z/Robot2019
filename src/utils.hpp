#include <algorithm>

template <typename T>
inline T DegreeToRadian(T angle) {
    return angle / 180.f * M_PI;
}

template <typename T>
inline T RadianToDegree(T angle) {
    return angle / M_PI * 180.f;
}

template <typename T>
inline void CorrectAngleDegree180(T& deg) {
    T x_mod = std::fmod(deg + 180, 360);
    if(x_mod < 0) {
        x_mod += 360;
    }
    deg =  x_mod - 180;
}

template <typename T>
inline T GetSlope(T x1, T y1, T x2, T y2) {
    auto res = std::atan2(y2 - y1, x2 - x1);
    return RadianToDegree(res);
}
