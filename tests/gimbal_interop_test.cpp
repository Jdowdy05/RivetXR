#include "gimbal_client.h"
#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>
namespace {
void Check(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
std::uint64_t U(std::span<const std::byte>b,std::size_t at,std::size_t count){
    std::uint64_t n=0;for(std::size_t i=0;i<count;++i)n|=std::uint64_t(std::to_integer<unsigned char>(b[at+i]))<<(i*8);return n;
}
quest_newton::GimbalTime Time(std::uint64_t n){
    Check(n&&n<=static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()),"invalid fixture monotonic time");
    return quest_newton::GimbalTime{}+std::chrono::nanoseconds(static_cast<std::int64_t>(n));
}
}
int main(int argc,char** argv){try{
    using namespace quest_newton;Check(argc==2,"expected a Java/Python control transcript");std::ifstream input(argv[1],std::ios::binary|std::ios::ate);
    Check(static_cast<bool>(input),"cannot open control transcript");const auto length=input.tellg();Check(length==1404,"control transcript length mismatch");
    std::vector<std::byte> storage(static_cast<std::size_t>(length));input.seekg(0);input.read(reinterpret_cast<char*>(storage.data()),length);Check(static_cast<bool>(input),"transcript read failed");
    const std::span<const std::byte> bytes(storage);Check(U(bytes,0,4)==0x54494751&&U(bytes,4,4)==1&&U(bytes,8,4)==7,"transcript header mismatch");
    GimbalClientPolicy policy;std::string error;Check(policy.Accept(bytes.subspan(20,96),Time(U(bytes,12,8)),error),error.c_str());
    std::size_t at=116;
    for(unsigned sequence=1;sequence<=7;++sequence){
        const auto sample=Time(U(bytes,at,8)),sent=Time(U(bytes,at+8,8));const auto command=bytes.subspan(at+16,64);
        const bool aim=sequence==2||sequence==3||sequence==4||sequence==6;
        GimbalIntent intent{aim?GimbalOperation::Aim:GimbalOperation::Hold,aim,.25F,.1F,sample};
        const auto encoded=policy.Command(intent,false,sent,U(command,32,8));
        Check(std::equal(encoded.begin(),encoded.end(),command.begin()),"native policy command differs from loopback Java command");
        Check(policy.Accept(bytes.subspan(at+88,96),Time(U(bytes,at+80,8)),error),error.c_str());at+=184;
    }
    Check(policy.Feedback().ack==7&&!(policy.Feedback().flags&2U),"final control state is not unarmed");
    std::cout<<"native policy accepted 8 Python feedback frames and matched 7 Java commands\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
