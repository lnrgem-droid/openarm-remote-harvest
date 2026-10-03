// Copyright 2026 OpenArm Contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <array>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <cerrno>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>

namespace openarm_gravity_pd_control {
// Passive second CAN receiver: uses no command frames and never enables motors.
// Required only for the opt-in tracking candidate. The upstream CAN library
// caches positions without exposing a per-drive receive time/enable status.
class DriveFeedbackGuard {
public:
  using Clock = std::chrono::steady_clock;
  ~DriveFeedbackGuard() { if (fd_ >= 0) ::close(fd_); }
  DriveFeedbackGuard() = default;
  DriveFeedbackGuard(const DriveFeedbackGuard&) = delete;
  DriveFeedbackGuard& operator=(const DriveFeedbackGuard&) = delete;
  void open(const std::string& interface) {
    if (fd_ >= 0) throw std::runtime_error("drive guard already open");
    fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
    if (fd_ < 0) throw std::runtime_error("cannot open passive drive guard");
    int enabled=1;
    std::array<can_filter,8> filters{};
    for (size_t j=0; j<8; ++j) filters[j] = {static_cast<canid_t>(0x11+j), CAN_EFF_FLAG|CAN_RTR_FLAG|CAN_SFF_MASK};
    sockaddr_can address{};
    address.can_family=AF_CAN;
    address.can_ifindex=static_cast<int>(if_nametoindex(interface.c_str()));
    if (!address.can_ifindex ||
        setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPNS, &enabled, sizeof(enabled)) < 0 ||
        setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enabled, sizeof(enabled)) < 0 ||
        setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), sizeof(filters)) < 0 ||
        ::bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
      throw std::runtime_error("cannot configure passive drive guard: " + interface);
  }
  // Public ingestion permits testing real classic/FD framing without any CAN.
  void observe(const canfd_frame& frame, ssize_t bytes, Clock::time_point now) {
    if ((bytes!=CAN_MTU && bytes!=CANFD_MTU) || frame.len!=8 ||
        frame.can_id<0x11 || frame.can_id>0x18) return;
    const size_t j=frame.can_id-0x11;
    if ((frame.data[0]&0xf) != j+1) return;
    seen_[j]=true; last_[j]=now; states_[j]=frame.data[0]>>4;
  }
  bool healthy(Clock::time_point now) const {
    if (io_fault_) return false;
    for (size_t j=0;j<7;++j)
      if (!seen_[j] || states_[j]!=1 || now<last_[j] ||
          now-last_[j]>std::chrono::milliseconds(50)) return false;
    return true;
  }
  bool gripperHealthy(Clock::time_point now) const {
    return !io_fault_ && seen_[7] && states_[7]==1 && now>=last_[7] &&
      now-last_[7]<=std::chrono::milliseconds(50);
  }
  bool poll() {
    if (fd_<0) return false;
    // Bounded work. If a delayed thread encounters a backlog, require a fresh
    // empty-to-live cycle, not arbitrarily old packets marked with "now".
    for (size_t count=0; count<128; ++count) {
      canfd_frame frame{};
      iovec data{&frame, sizeof(frame)};
      alignas(cmsghdr) char ancillary[CMSG_SPACE(sizeof(timespec))]{};
      msghdr message{};
      message.msg_iov=&data; message.msg_iovlen=1;
      message.msg_control=ancillary; message.msg_controllen=sizeof(ancillary);
      ssize_t bytes=::recvmsg(fd_, &message, MSG_DONTWAIT);
      if (bytes<0) {
        if (errno==EAGAIN || errno==EWOULDBLOCK) return healthy(Clock::now());
        io_fault_=true; return false;
      }
      bool timestamp_valid=false;
      timespec timestamp{};
      for (auto* c=CMSG_FIRSTHDR(&message); c; c=CMSG_NXTHDR(&message,c)) {
        if (c->cmsg_level==SOL_SOCKET && c->cmsg_type==SO_TIMESTAMPNS &&
            c->cmsg_len>=CMSG_LEN(sizeof(timestamp))) {
          std::memcpy(&timestamp,CMSG_DATA(c),sizeof(timestamp)); timestamp_valid=true;
        }
      }
      if (!timestamp_valid || (message.msg_flags&(MSG_CTRUNC|MSG_TRUNC))) continue;
      const auto received=std::chrono::seconds(timestamp.tv_sec)+std::chrono::nanoseconds(timestamp.tv_nsec);
      const auto age=std::chrono::system_clock::now().time_since_epoch()-received;
      if (age<std::chrono::nanoseconds(0) || age>std::chrono::milliseconds(50)) continue;
      observe(frame, bytes, Clock::now()-std::chrono::duration_cast<Clock::duration>(age));
    }
    seen_.fill(false);
    return false;
  }
private:
  int fd_=-1;
  bool io_fault_=false;
  std::array<bool,8> seen_{};
  std::array<unsigned,8> states_{};
  std::array<Clock::time_point,8> last_{};
};
}  // namespace openarm_gravity_pd_control
