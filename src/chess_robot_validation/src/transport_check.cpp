// Loopback-only socket fault fixtures. Never talks to a controller.
#include "net.hpp"
#include <thread>
#include <iostream>
#include <array>

void fixture(int mode){
  int listener=socket(AF_INET,SOCK_STREAM,0);if(listener<0)throw std::runtime_error("fixture socket failed");
  sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);a.sin_port=0;
  if(bind(listener,reinterpret_cast<sockaddr*>(&a),sizeof(a)) || listen(listener,1)){close(listener);throw std::runtime_error("fixture listen failed");}
  socklen_t len=sizeof(a);getsockname(listener,reinterpret_cast<sockaddr*>(&a),&len);
  std::thread worker([&]{int peer=accept(listener,nullptr,nullptr);if(peer<0)return;
    if(mode==0){for(int i=0;i<12;i++){unsigned char c=static_cast<unsigned char>(i);send(peer,&c,1,MSG_NOSIGNAL);std::this_thread::sleep_for(std::chrono::milliseconds(2));}}
    else if(mode==1){char c='x';send(peer,&c,1,MSG_NOSIGNAL);}
    else {std::this_thread::sleep_for(std::chrono::milliseconds(180));}
    close(peer);
  });
  bool failed=false;std::array<unsigned char,12> b{};
  try{Socket s("127.0.0.1",ntohs(a.sin_port));s.exact(b.data(),b.size(),100);}catch(...){failed=true;}
  worker.join();close(listener);
  if(mode==0){if(failed)throw std::runtime_error("fragmented packet rejected");for(int i=0;i<12;i++)if(b[i]!=i)throw std::runtime_error("fragmented packet corrupted");}
  else if(!failed)throw std::runtime_error("truncated/stale stream accepted");
}
int main(){try{fixture(0);fixture(1);fixture(2);std::cout<<"PASS loopback only: fragmented receive, truncated EOF, read deadline; no controller accessed\n";return 0;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
