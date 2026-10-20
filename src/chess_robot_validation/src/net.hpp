#pragma once
#include <arpa/inet.h>
#include <sys/socket.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>

using Steady = std::chrono::steady_clock;
inline long long steady_ns() { return std::chrono::duration_cast<std::chrono::nanoseconds>(Steady::now().time_since_epoch()).count(); }
inline long long utc_ns() { return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count(); }
class Socket {
  int fd_ = -1;
public:
  Socket(const std::string& ip, int port, int timeout_ms=1500) {
    sockaddr_in addr{}; addr.sin_family=AF_INET; addr.sin_port=htons(port);
    if (inet_pton(AF_INET,ip.c_str(),&addr.sin_addr)!=1) throw std::runtime_error("IPv4 literal required");
    fd_=socket(AF_INET,SOCK_STREAM,0);
    if(fd_<0) throw std::runtime_error("socket failed");
    fcntl(fd_,F_SETFL,O_NONBLOCK);
    int result=connect(fd_,reinterpret_cast<sockaddr*>(&addr),sizeof(addr));
    if(result<0 && errno!=EINPROGRESS) fail(std::strerror(errno));
    pollfd p{fd_,POLLOUT,0};
    if(result<0 && poll(&p,1,timeout_ms)<=0) fail("connect timeout");
    int error=0; socklen_t len=sizeof(error);
    if(getsockopt(fd_,SOL_SOCKET,SO_ERROR,&error,&len)<0 || error) fail(error?std::strerror(error):"SO_ERROR failed");
  }
  Socket(const Socket&)=delete; Socket& operator=(const Socket&)=delete;
  ~Socket(){if(fd_>=0) close(fd_);}
  [[noreturn]] void fail(const std::string& msg){ if(fd_>=0) close(fd_); fd_=-1; throw std::runtime_error(msg); }
  void exact(void* buffer,size_t n,int timeout_ms=1500) {
    auto end=Steady::now()+std::chrono::milliseconds(timeout_ms); size_t done=0;
    while(done<n){
      auto remaining=std::chrono::duration_cast<std::chrono::milliseconds>(end-Steady::now()).count();
      if(remaining<=0) throw std::runtime_error("read timeout/stale feedback");
      pollfd p{fd_,POLLIN,0};
      int r=poll(&p,1,static_cast<int>(remaining));
      if(r<0 && errno==EINTR) continue;
      if(r<=0) throw std::runtime_error("read timeout/error");
      auto got=recv(fd_,static_cast<char*>(buffer)+done,n-done,0);
      if(got<0 && (errno==EAGAIN || errno==EINTR)) continue;
      if(got<=0) throw std::runtime_error("connection closed/read error");
      done+=static_cast<size_t>(got);
    }
  }
  std::string query(const std::string& command) {
    size_t sent=0; auto end=Steady::now()+std::chrono::seconds(2);
    while(sent<command.size()){
      if(Steady::now()>end) throw std::runtime_error("send timeout");
      pollfd p{fd_,POLLOUT,0}; if(poll(&p,1,100)<=0) continue;
      auto n=send(fd_,command.data()+sent,command.size()-sent,MSG_NOSIGNAL);
      if(n<0 && (errno==EAGAIN || errno==EINTR)) continue;
      if(n<=0) throw std::runtime_error("send failed");
      sent+=static_cast<size_t>(n);
    }
    std::string response;
    auto deadline=Steady::now()+std::chrono::seconds(3);
    while(response.size()<16384 && Steady::now()<deadline){char c; exact(&c,1,500);response+=c;if(c==';') return response;}
    throw std::runtime_error("unterminated/oversized reply");
  }
};
