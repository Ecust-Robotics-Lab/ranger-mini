// Exercise the real SDK transport using AF_UNIX datagrams, never SocketCAN.
#include <asio.hpp>
#include <atomic>
#include <memory>
#include <thread>
#include <functional>
#include <mutex>
#define private public
#include "ugv_sdk/details/async_port/async_can.hpp"
#undef private
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <cassert>
#include <cstring>
#include <iostream>
#include <chrono>

using westonrobot::AsyncCAN;

static std::shared_ptr<AsyncCAN> attach(int pair[2]) {
  assert(socketpair(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK, 0, pair) == 0);
  auto port = std::make_shared<AsyncCAN>("offline-only");
  port->can_fd_ = pair[0];
  port->socketcan_stream_.assign(pair[0]);
  port->port_opened_ = true;
  return port;
}

template<class Predicate> static void wait_for(Predicate ready) {
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!ready() && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  assert(ready());
}

int main() {
  int pair[2];
  auto port = attach(pair);
  for (unsigned i = 0; i < 128; ++i) {
    can_frame sent{};
    sent.can_id = 0x111;
    sent.can_dlc = 8;
    std::memset(sent.data, i, 8);
    port->SendFrame(sent);
    std::memset(&sent, 0xa5, sizeof(sent));
    can_frame received{};
    assert(recv(pair[1], &received, sizeof(received), 0) == sizeof(received));
    assert(received.can_id == 0x111 && received.can_dlc == 8);
    for (auto byte : received.data) assert(byte == i);
  }
  std::cout << "PASS: immediate send retains frame bytes/order after caller overwrite\n";

  can_frame frame{};
  while (send(pair[0], &frame, sizeof(frame), MSG_DONTWAIT) == sizeof(frame)) {}
  auto start = std::chrono::steady_clock::now();
  port->SendFrame(frame);  // deliberately full socket; must return, not enqueue.
  assert(std::chrono::steady_clock::now() - start < std::chrono::milliseconds(100));
  std::cout << "PASS: full socket does not block or retain commands\n";
  port->Close();
  int reused = open("/dev/null", O_RDONLY);
  assert(reused >= 0);
  port.reset();
  assert(fcntl(reused, F_GETFD) >= 0);
  close(reused);
  close(pair[1]);
  std::cout << "PASS: repeated Close/destructor does not close a reused descriptor\n";

  port = attach(pair);
  std::atomic<int> count{0};
  port->SetReceiveCallback([&](can_frame* f) { assert(f->can_id == 0x211); ++count; });
  asio::post(port->io_context_, [&] {
    port->ReadFromPort(port->rcv_frame_, port->socketcan_stream_);
  });
  port->io_thread_ = std::thread([&] { port->io_context_.run(); });
  frame.can_id = 0x211;
  assert(send(pair[1], &frame, sizeof(frame), 0) == sizeof(frame));
  wait_for([&] { return count.load() == 1; });
  asio::post(port->io_context_, [&] { port->socketcan_stream_.cancel(); });
  wait_for([&] { return !port->IsOpened(); });
  port->Close();
  port.reset();
  close(pair[1]);
  std::cout << "PASS: receive error exits IO thread without self-join\n";

  port = attach(pair);
  asio::post(port->io_context_, [&] {
    port->ReadFromPort(port->rcv_frame_, port->socketcan_stream_);
  });
  port->io_thread_ = std::thread([&] { port->io_context_.run(); });
  port->Close();
  port.reset();
  close(pair[1]);
  std::cout << "PASS: close with pending receive; no CAN device was opened\n";
}
