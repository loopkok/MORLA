#ifndef _IP_UTIL_HPP__
#define _IP_UTIL_HPP__

#include <string>
#include <memory>
#include "visibility_control.h"

namespace xhand_control {

class IpUtil {
public:
    XHAND_CONTROL_PUBLIC static std::string get_local_ip();
    XHAND_CONTROL_PUBLIC static std::string get_gateway_ip();
    XHAND_CONTROL_PUBLIC static std::string get_public_ip();
    XHAND_CONTROL_PUBLIC static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp);

private:
    static std::string local_ip;
    static std::string public_ip;
};

} // namespace xhand_control

#endif // _IP_UTIL_HPP__ 