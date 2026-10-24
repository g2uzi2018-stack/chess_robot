// Loopback-only fault fixtures. Never talks to a controller.
#include "net.hpp"
#include <array>
#include <exception>
#include <functional>
#include <iostream>
#include <thread>

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void rejected(const std::function<void()>& action, const std::string& reason) {
  try { action(); }
  catch (const std::runtime_error& e) {
    require(std::string(e.what()).find(reason) != std::string::npos, "unexpected failure reason");
    return;
  }
  throw std::runtime_error("invalid/truncated/stale input accepted");
}

void fixture(const std::function<void(int)>& serve, const std::function<void(Socket&)>& check) {
  const int listener = socket(AF_INET, SOCK_STREAM, 0);
  require(listener >= 0, "fixture socket failed");
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (bind(listener, reinterpret_cast<sockaddr*>(&a), sizeof(a)) || listen(listener, 1)) {
    close(listener);
    throw std::runtime_error("fixture listen failed");
  }
  socklen_t len = sizeof(a);
  if (getsockname(listener, reinterpret_cast<sockaddr*>(&a), &len)) {
    close(listener);
    throw std::runtime_error("fixture getsockname failed");
  }
  std::exception_ptr server_error;
  std::thread worker([&] {
    int peer = -1;
    try {
      pollfd p{listener, POLLIN, 0};
      require(poll(&p, 1, 1500) > 0, "fixture accept timed out");
      peer = accept(listener, nullptr, nullptr);
      require(peer >= 0, "fixture accept failed");
      timeval timeout{1, 0};
      require(setsockopt(peer, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == 0,
              "fixture receive timeout failed");
      serve(peer);
    } catch (...) { server_error = std::current_exception(); }
    if (peer >= 0) close(peer);
  });
  std::exception_ptr client_error;
  try { Socket client("127.0.0.1", ntohs(a.sin_port)); check(client); }
  catch (...) { client_error = std::current_exception(); }
  worker.join();
  close(listener);
  if (client_error) std::rethrow_exception(client_error);
  if (server_error) std::rethrow_exception(server_error);
}

void receive_query(int peer) {
  std::array<char, 4> b{};
  require(recv(peer, b.data(), b.size(), MSG_WAITALL) == 4, "fixture query missing");
  require(std::string(b.data(), b.size()) == "ping", "fixture query corrupted");
}

void send_all(int peer, const std::string& data) {
  size_t done = 0;
  while (done < data.size()) {
    const auto n = send(peer, data.data() + done, data.size() - done, MSG_NOSIGNAL);
    require(n > 0, "fixture send failed");
    done += static_cast<size_t>(n);
  }
}

int main() {
  try {
    rejected([] { Socket s("localhost", 29999); }, "IPv4 literal");
    for (const int port : {-1, 0, 65536})
      rejected([port] { Socket s("127.0.0.1", port); }, "port must");
    for (const int timeout : {-1, 0})
      rejected([timeout] { Socket s("127.0.0.1", 29999, timeout); }, "timeout must");

    fixture([](int peer) {
      for (unsigned char c = 0; c < 12; ++c) {
        require(send(peer, &c, 1, MSG_NOSIGNAL) == 1, "fragment send failed");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }, [](Socket& s) {
      std::array<unsigned char, 12> b{};
      s.exact(b.data(), b.size(), 500);
      for (size_t i = 0; i < b.size(); ++i) require(b[i] == i, "fragmented data corrupted");
    });

    fixture([](int peer) { send_all(peer, "x"); }, [](Socket& s) {
      char b[2]; rejected([&] { s.exact(b, sizeof(b), 500); }, "connection closed");
    });
    fixture([](int) { std::this_thread::sleep_for(std::chrono::milliseconds(160)); }, [](Socket& s) {
      char b; rejected([&] { s.exact(&b, 1, 60); }, "deadline exceeded");
      rejected([&] { s.exact(&b, 1, 0); }, "timeout must");
      rejected([&] { s.query("ping", -1); }, "timeout must");
    });

    // Continuous incoming bytes must not reset the complete-read/query budget.
    const auto trickle = [](int peer) {
      for (int i = 0; i < 40; ++i) {
        if (send(peer, "x", 1, MSG_NOSIGNAL) != 1) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    };
    fixture(trickle, [](Socket& s) {
      char b[40]; rejected([&] { s.exact(b, sizeof(b), 80); }, "deadline exceeded");
    });
    fixture([&](int peer) { receive_query(peer); trickle(peer); }, [](Socket& s) {
      rejected([&] { s.query("ping", 80); }, "deadline exceeded");
    });

    fixture([](int peer) {
      receive_query(peer);
      for (const char c : std::string("0,{1},ping;")) {
        require(send(peer, &c, 1, MSG_NOSIGNAL) == 1, "reply fragment send failed");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }, [](Socket& s) { require(s.query("ping", 500) == "0,{1},ping;", "reply corrupted"); });

    fixture([](int peer) { receive_query(peer); send_all(peer, "incomplete"); }, [](Socket& s) {
      bool failed=false;
      try { s.query("ping",500); }
      catch(const QueryFailure& e) {
        require(e.partial_reply=="incomplete", "partial query reply lost");
        failed=true;
      }
      require(failed,"incomplete query accepted");
    });
    fixture([](int peer) { receive_query(peer); send_all(peer, std::string(16384, 'x')); }, [](Socket& s) {
      rejected([&] { s.query("ping", 1500); }, "oversized reply");
    });
    fixture([](int peer) { receive_query(peer); send_all(peer, std::string(16383, 'x') + ';'); }, [](Socket& s) {
      const auto response = s.query("ping", 1500);
      require(response.size() == 16384 && response.back() == ';', "boundary reply rejected");
    });
    std::cout << "PASS loopback transport: invalid arguments, fragments, EOF, silence, "
                 "whole-read/query deadlines, reply framing and size boundary; no controller accessed\n";
    return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 2; }
}
