#include "interaction_trace.h"
#include <iomanip>
#include <iostream>
#include <sstream>

namespace {
std::string JsonString(const std::string& value){
    std::ostringstream out;out<<'"';
    for(unsigned char c:value){
        if(c=='"'||c=='\\')out<<'\\'<<static_cast<char>(c);
        else if(c<32)out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<static_cast<unsigned>(c)<<std::dec;
        else out<<static_cast<char>(c);
    }
    out<<'"';return out.str();
}
}

int main(int argc,char** argv){
    if(argc!=2){std::cerr<<"Usage: interaction_replay RECORDING.qitr\n";return 2;}
    const auto result=quest_newton::ReplayInteractionFile(argv[1]);
    std::cout<<"{\"schema_version\":1,\"scope\":\"native control replay; scheduler, physics and XR are external observations\",\"comparison\":\"bit_exact_f32\","
             <<"\"passed\":"<<(result.passed?"true":"false")<<",\"run_id\":"<<JsonString(result.run_id)
             <<",\"events\":"<<result.event_count<<",\"control_operations\":"<<result.control_count
             <<",\"raw_inputs\":"<<result.input_count<<",\"external_observations\":"<<result.external_observations
             <<",\"error\":"<<JsonString(result.error)<<"}\n";
    return result.passed?0:1;
}
