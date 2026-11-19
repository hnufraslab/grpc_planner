#ifndef LINK_LENGTH_H
#define LINK_LENGTH_H

#include <cmath>

namespace device_geometry {
constexpr double kDefaultLinkLength = 0.7835;
constexpr double kMinLinkLength = 0.75;
constexpr double kMaxLinkLength = 1.5;

inline bool validLinkLength(double length) {
    return std::isfinite(length) && length >= kMinLinkLength && length <= kMaxLinkLength;
}
}  // namespace device_geometry

#endif
