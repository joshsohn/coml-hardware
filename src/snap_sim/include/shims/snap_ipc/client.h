/**
 * @file client.h
 * @brief API for accessing ioboard via sDSP
 * @author Parker Lusk <plusk@mit.edu>
 * @date 15 April 2021
 */

#pragma once

#include <atomic>
#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "ipc/server.h"

struct throttle_commands {
  float throttle[6];
};

namespace acl {
namespace snapipc {

  class Client
  {
  public:
    Client(const std::string& vehname);
    ~Client() = default;

    void set_motors(const std::array<float, 6>& throttle);

  private:
    const std::string vehname_;

    // IPC server to send esc commands
    std::unique_ptr<acl::ipc::Server<throttle_commands>> escserver_;

  };

} // ns snapipc
} // ns acl
