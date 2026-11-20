#include <cassert>
#include <string>
#include <vector>
#include <limits>
#include "input_interface/stream_epoch_guard.hpp"
struct Field { std::string name, dtype; std::vector<int> shape; };
struct Header { int version; std::vector<Field> fields; };
struct Buffer { const void* data; size_t size; };
int main() {
 assert(StreamEpochGuard::IsLocalHost("localhost"));
 assert(StreamEpochGuard::IsLocalHost("127.0.0.1"));
 assert(StreamEpochGuard::IsLocalHost("::1"));
 assert(!StreamEpochGuard::IsLocalHost("198.51.100.27"));
 StreamEpochGuard guard;
 for (int from : {1,3}) for (int to : {1,3}) {
  Header old{from, {{"timestamp_monotonic", "f64", {1}}}};
  Header next{to, old.fields};
  double old_time=10., new_time=11.01;
  std::vector<Buffer> delayed{{&old_time,8}}, fresh{{&new_time,8}};
  guard.Reset(11.);
  assert(!guard.Accepts(old,delayed,11.02));
  assert(guard.Accepts(next,fresh,11.02));
  // A delayed packet must remain rejected even after the first new packet.
  assert(!guard.Accepts(old,delayed,11.03));
 }
 Header h{3, {{"timestamp_monotonic","f64",{1}}}};
 double value=std::numeric_limits<double>::quiet_NaN();
 std::vector<Buffer> b{{&value,8}};
 assert(!guard.Accepts(h,b,12.));
 value=20.; assert(!guard.Accepts(h,b,12.));
 value=12.; b[0].size=4; assert(!guard.Accepts(h,b,12.));
 b[0].size=8; h.fields[0].shape={2}; assert(!guard.Accepts(h,b,12.));
 h.fields.clear(); assert(!guard.Accepts(h,b,12.));
}
