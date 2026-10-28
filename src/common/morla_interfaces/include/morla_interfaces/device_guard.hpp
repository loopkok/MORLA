#pragma once
// Same host-local legacy lane as morla_core.device_guard; POSIX Linux/macOS.
#include <sys/file.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace morla_interfaces {
class LegacyDeviceGuard {
  int descriptor_ = -1;
 public:
  LegacyDeviceGuard() {
    const char* root_env = std::getenv("MORLA_RESOURCE_ROOT");
    const std::string root = root_env ? root_env : "/tmp/morla-runtime-locks";
    ::mkdir(root.c_str(), 0700);
    struct stat dir_info{};
    if (::stat(root.c_str(), &dir_info) || dir_info.st_uid != ::getuid() || (dir_info.st_mode & 0022))
      throw std::runtime_error("UNSAFE_RESOURCE_LOCK_DIRECTORY");
    const auto path = root + "/active-system.lock";
    const char* inherited = std::getenv("MORLA_RUNTIME_LOCK_FD");
    if (inherited) {
      char* end = nullptr;
      const long delegated = std::strtol(inherited, &end, 10);
      struct stat fd_info{}, path_info{};
      if (!*inherited || *end || delegated < 0 || ::fstat(static_cast<int>(delegated), &fd_info) ||
          ::stat(path.c_str(), &path_info) || fd_info.st_dev != path_info.st_dev || fd_info.st_ino != path_info.st_ino ||
          ::flock(static_cast<int>(delegated), LOCK_EX | LOCK_NB))
        throw std::runtime_error("INVALID_RUNTIME_LOCK_DELEGATION");
      descriptor_ = ::dup(static_cast<int>(delegated));
    } else {
      descriptor_ = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
      if (descriptor_ < 0 || ::flock(descriptor_, LOCK_SH | LOCK_NB)) {
        if (descriptor_ >= 0) ::close(descriptor_);
        descriptor_ = -1;
        throw std::runtime_error("COMPOSABLE_RUNTIME_OWNS_DEVICES");
      }
    }
  }
  ~LegacyDeviceGuard() { if (descriptor_ >= 0) ::close(descriptor_); }
  LegacyDeviceGuard(const LegacyDeviceGuard&) = delete;
  LegacyDeviceGuard& operator=(const LegacyDeviceGuard&) = delete;
};
}
