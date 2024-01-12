/**
 * @file client.cpp
 * @brief API for accessing ioboard via sDSP
 * @author Parker Lusk <plusk@mit.edu>
 * @date 15 April 2021
 */

#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <sstream>

#include "snap_ipc/client.h"

#include "ipc/common.h"

namespace acl {
namespace snapipc {

Client::Client(const std::string& vehname)
: vehname_(vehname)
{
  // unique key to access the same shmem location
  const size_t esckey = acl::ipc::createKeyFromStr(vehname_, "esc");
  escserver_.reset(new acl::ipc::Server<throttle_commands>(esckey));
}

// ----------------------------------------------------------------------------

void Client::set_motors(const std::array<float, 6>& throttle)
{
  throttle_commands msg;
  memcpy(&msg.throttle, throttle.data(), sizeof(throttle));
  escserver_->send(msg);
}

} // ns snapipc
} // ns acl
