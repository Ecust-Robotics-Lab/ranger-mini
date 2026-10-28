#pragma once
#include "types.hpp"
#include <list>
#include <map>
namespace ranger_backend {
std::string sha256(const fs::path& path);
void write_json(const fs::path& path, const Json& value);
Keyframe store_keyframe(const fs::path& session_directory, Keyframe record, const Cloud& full, const Cloud& match, const std::vector<uint8_t>& cdr);
class CloudCache {
public:
  explicit CloudCache(size_t budget) : budget_(budget){
  }
  Cloud::ConstPtr get(const Keyframe& frame);
  size_t bytes()const {
    return bytes_;
  }
private:
  struct Item {
    Cloud::ConstPtr cloud;
    std::list<uint64_t>::iterator position;
  };
  size_t budget_, bytes_ = 0;
  std::list<uint64_t> order_;
  std::map<uint64_t, Item> items_;
};
struct ExportResult {
  bool success = false;
  std::string reason;
  fs::path manifest;
};
ExportResult export_snapshot(const Snapshot& snapshot, const fs::path& destination);
}
