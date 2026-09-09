#include "gimbal_client.h"
#include <bit>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
namespace {
using namespace quest_newton;using Bytes=std::array<std::byte,96>;
void Check(bool c,const char* why){if(!c)throw std::runtime_error(why);}
template<class T>void Put(Bytes& b,std::size_t at,T value){static_assert(std::endian::native==std::endian::little);std::memcpy(b.data()+at,&value,sizeof(T));}
Bytes Feedback(){Bytes b{};std::memcpy(b.data(),"QGCF",4);Put<std::uint16_t>(b,4,1);Put<std::uint16_t>(b,6,96);
    for(std::size_t at:{8U,16U,32U,48U,64U})Put<std::uint64_t>(b,at,1);
    Put<std::uint32_t>(b,56,5);Put(b,72,-1.F);Put(b,76,1.F);Put(b,80,-.5F);Put(b,84,.5F);Put(b,88,2.F);return b;}
std::uint64_t U(std::span<const std::byte>b,std::size_t at,std::size_t count=4){std::uint64_t n=0;for(std::size_t i=0;i<count;++i)n|=std::uint64_t(std::to_integer<unsigned char>(b[at+i]))<<(i*8);return n;}
Bytes Reply(std::uint64_t sequence,bool armed=false){auto b=Feedback();Put(b,16,sequence+1);Put(b,24,sequence);Put(b,32,sequence+1);
    if(armed){Put<std::uint32_t>(b,56,15);Put<std::uint32_t>(b,60,100);}return b;}
template<class F>void Throws(F&& f){bool thrown=false;try{f();}catch(const std::exception&){thrown=true;}Check(thrown,"invalid client command state accepted");}
}
int main(){try{GimbalClientPolicy p;std::string e;auto b=Feedback();Put(b,40,2.F);
    using namespace std::chrono_literals;const auto now=GimbalTime{}+1s;
    Check(!p.Accept(b,now,e),"feedback position outside advertised limits accepted");
    for(std::uint32_t flags:{4U,7U,9U,11U,16U}){GimbalClientPolicy bad;b=Feedback();Put(b,56,flags);Check(!bad.Accept(b,now,e),"inconsistent simulated/armed/lease flags accepted");}
    b=Feedback();Put<std::uint32_t>(b,60,1);Check(!p.Accept(b,now,e),"nonzero lease while unarmed accepted");
    Check(p.Accept(Feedback(),now,e),"initial simulated feedback rejected");
    GimbalIntent aim{GimbalOperation::Aim,true,.5F,.2F,now};
    auto command=p.Command(aim,false,now,1);Check(U(command,40)==0,"connection must Hold until explicit release acknowledgement");
    Throws([&]{p.Command(aim,false,now,2);});auto same_tick=Reply(1);Put<std::uint64_t>(same_tick,32,1);
    Check(p.Accept(same_tick,now+1ms,e),"same-tick server timestamp with new challenge/ack must be accepted");
    command=p.Command(aim,false,now+2ms,2);Check(U(command,40)==0,"forced Hold must not rearm held input");
    Check(p.Accept(Reply(2),now+3ms,e),"Hold response rejected");
    GimbalIntent release;release.sampled=now+4ms;command=p.Command(release,false,now+4ms,3);Check(U(command,40)==0,"release must produce Hold");
    Check(p.Accept(Reply(3),now+5ms,e),"release acknowledgement rejected");
    aim.sampled=now+6ms;command=p.Command(aim,false,now+6ms,4);Check(U(command,40)==1&&U(command,44)==1&&U(command,56)==100,"fresh released clutch did not request bounded Aim lease");
    Check(p.Accept(Reply(4,true),now+7ms,e),"armed Aim reply rejected");
    command=p.Command(aim,false,now+200ms,5);Check(U(command,40)==0&&U(command,44)==0,"stale intent must Hold and consume clutch");
    Check(p.Accept(Reply(5),now+201ms,e),"stale Hold reply rejected");
    aim.sampled=now+202ms;command=p.Command(aim,false,now+202ms,6);Check(U(command,40)==0,"fresh Aim cannot silently rearm after stale gap");
    Check(p.Accept(Reply(6),now+203ms,e),"post-gap Hold reply rejected");
    release.sampled=now+204ms;command=p.Command(release,false,now+204ms,7);Check(p.Accept(Reply(7),now+205ms,e),"second release rejected");
    aim.sampled=now+206ms;command=p.Command(aim,true,now+206ms,8);Check(U(command,40)==2&&U(command,44)==0,"map reset must disarm and use a fresh challenge");
    Check(p.Accept(Reply(8),now+207ms,e),"map-reset reply rejected");
    command=p.Command(aim,false,now+208ms,9);Check(U(command,40)==0,"map reset requires release again");
    const auto good=Reply(9);
    for(std::size_t at:{8U,16U,24U,32U,48U,64U}){auto bad=good;Put<std::uint64_t>(bad,at,at==16?9:at==32?8:999);
        if(at==64)Put<std::uint64_t>(bad,at,0);Check(!p.Accept(bad,now+209ms,e)&&p.Feedback().ack==8,"changed session/source/ack/challenge/time accepted");}
    auto changed=good;Put(changed,76,1.1F);Check(!p.Accept(changed,now+209ms,e),"changed advertised limits accepted");
    Check(!p.Accept(Reply(9,true),now+209ms,e),"armed response to Hold accepted");
    Check(p.Accept(good,now+209ms,e),"valid retry fixture reply rejected");
    Throws([&]{p.Command(aim,false,now+210ms,9);});
    aim.sampled=now+500ms;command=p.Command(aim,false,now+211ms,10);Check(U(command,40)==0,"future input stamp accepted");
    Check(p.Accept(Reply(10),now+212ms,e),"future-input Hold reply rejected");
    aim.pan=std::numeric_limits<float>::quiet_NaN();aim.sampled=now+213ms;command=p.Command(aim,false,now+213ms,11);Check(U(command,40)==0,"nonfinite Aim input accepted");
    std::cout<<"Gimbal client policy checks passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
