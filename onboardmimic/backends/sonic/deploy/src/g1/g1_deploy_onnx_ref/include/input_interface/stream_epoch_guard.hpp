#pragma once
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string_view>

// Shared host CLOCK_MONOTONIC boundary, checked before protocol establishment.
// Access is serialized by the endpoint data mutex. An old packet cannot become
// the first packet of a new joint/SMPL stream, even if its callback is delayed.
class StreamEpochGuard {
 public:
  static bool IsLocalHost(std::string_view host) {
    return host == "localhost" || host == "127.0.0.1" || host == "::1";
  }
  static double Now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void Reset(double now = Now()) { boundary_ = now; }
  template<class Header, class Buffers>
  bool Accepts(const Header& header, const Buffers& buffers, double now = Now()) const {
    for (size_t i = 0; i < header.fields.size(); ++i) {
      const auto& field = header.fields[i];
      if (field.name != "timestamp_monotonic") continue;
      if (i >= buffers.size() || field.dtype != "f64" || buffers[i].size != sizeof(double) ||
          field.shape.size() != 1 || field.shape[0] != 1) return false;
      double stamp;
      std::memcpy(&stamp, buffers[i].data, sizeof(stamp));
      uint64_t bits;
      std::memcpy(&bits, &stamp, sizeof(bits));
      const bool finite = (bits & 0x7ff0000000000000ULL) != 0x7ff0000000000000ULL;
      return finite && stamp >= boundary_ && stamp <= now + .05;
    }
    return false; // local producers must stamp at production, never at receipt
  }
 private:
  double boundary_ = 0.;
};
