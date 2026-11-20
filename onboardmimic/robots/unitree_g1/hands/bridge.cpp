// Only BrainCo hand topics. No LowCmd, motion-switcher, serial-port or policy API.
#include "core.hpp"
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <nlohmann/json.hpp>
#include <zmq.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <sys/file.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
using J=nlohmann::json;
using Cmd=unitree_go::msg::dds_::MotorCmds_;
using State=unitree_go::msg::dds_::MotorStates_;
using namespace pico_hands;
std::atomic<bool> running{true};
uint64_t ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
void signal_handler(int){running=false;}
std::string identity(){std::ifstream f("/proc/sys/kernel/random/uuid");std::string s;f>>s;s.erase(std::remove(s.begin(),s.end(),'-'),s.end());if(s.size()!=32)throw std::runtime_error("worker identity unavailable");return s;}
struct Hardware {
 std::mutex mutex;std::array<Feedback,2> feedback{};std::array<uint64_t,2> other_at{},received{};
 std::array<std::shared_ptr<unitree::robot::ChannelSubscriber<State>>,2> states;
 std::array<std::shared_ptr<unitree::robot::ChannelSubscriber<Cmd>>,2> commands;
 std::array<std::shared_ptr<unitree::robot::ChannelPublisher<Cmd>>,2> publishers;
 std::array<bool,2> publisher_created{};
 Hardware(const std::string& interface){
  unitree::robot::ChannelFactory::Instance()->Init(0,interface);
  for(int side=0;side<2;++side){
   const std::string name=side==0?"left":"right";
   states[side]=std::make_shared<unitree::robot::ChannelSubscriber<State>>("rt/brainco/"+name+"/state");
   states[side]->InitChannel([this,side](const void* raw){
    const auto& v=*static_cast<const State*>(raw);Feedback f;f.at=ns();f.valid=v.states().size()==6;
    if(f.valid){for(size_t k=0;k<6;++k)f.q[k]=v.states()[k].q();f.valid=valid_q(f.q);}
    std::lock_guard<std::mutex> lock(mutex);f.count=feedback[side].count+1;feedback[side]=f;
   },1);
   commands[side]=std::make_shared<unitree::robot::ChannelSubscriber<Cmd>>("rt/brainco/"+name+"/cmd");
   commands[side]->InitChannel([this,side](const void*){
    std::lock_guard<std::mutex> lock(mutex);++received[side];
    // Startup admission observes foreign traffic before creating a publisher.
    // This cannot promise exclusion of a remote writer started afterwards.
    if(!publisher_created[side])other_at[side]=ns();
   },1);
  }
 }
 ~Hardware(){
  for(int side=0;side<2;++side)if(publishers[side]){
   try{Feedback f;{std::lock_guard<std::mutex> lock(mutex);f=feedback[side];}
    if(fresh(f,ns()))send(side,{true,f.q,0.});
   }catch(...){} // A dead DDS link cannot guarantee delivery of the final hold.
  }
 }
 bool send(int side,const Output& out){
  if(!publishers[side]){
   const std::string name=side==0?"left":"right";
   {std::lock_guard<std::mutex> lock(mutex);publisher_created[side]=true;}
   publishers[side]=std::make_shared<unitree::robot::ChannelPublisher<Cmd>>("rt/brainco/"+name+"/cmd");publishers[side]->InitChannel();
  }
  Cmd cmd;cmd.cmds().resize(6);
  for(size_t k=0;k<6;++k){cmd.cmds()[k].q()=std::clamp(out.q[k],0.,1.);cmd.cmds()[k].dq()=out.speed;}
  return publishers[side]->Write(cmd);
 }
};
int main(int argc,char** argv){try{
 std::string interface,status;bool physical=false,observe=false;int port=16686;double duration=0;
 for(int i=1;i<argc;++i){std::string a=argv[i];
  if(a=="--physical")physical=true;else if(a=="--observe-only")observe=true;
  else if(a=="--interface"&&i+1<argc)interface=argv[++i];
  else if(a=="--status"&&i+1<argc)status=argv[++i];
  else if(a=="--port"&&i+1<argc)port=std::stoi(argv[++i]);
  else if(a=="--duration"&&i+1<argc)duration=std::stod(argv[++i]);
  else throw std::runtime_error("invalid arguments");
 }
 if(interface.empty()||status.empty()||physical==observe||port<1024||port>65535||!std::isfinite(duration)||duration<0||duration>3600)
  throw std::runtime_error("Require interface, status, and exactly one of --physical/--observe-only");
 int lockfd=-1;
 if(physical){
  auto path="/tmp/g1-brainco-hand-"+std::to_string(getuid())+".lock";
  lockfd=::open(path.c_str(),O_RDWR|O_CREAT|O_CLOEXEC|O_NOFOLLOW,0600);struct stat st{};
  if(lockfd<0||fstat(lockfd,&st)||!S_ISREG(st.st_mode)||st.st_nlink!=1||flock(lockfd,LOCK_EX|LOCK_NB))throw std::runtime_error("hand owner lock unavailable");
 }
 signal(SIGINT,signal_handler);signal(SIGTERM,signal_handler);
 Hardware hardware(interface);const auto started=ns();Gate gate(identity());std::array<Side,2> sides;
 void* ctx=zmq_ctx_new();void* sub=zmq_socket(ctx,ZMQ_SUB);
 int linger=0,conflate=1;zmq_setsockopt(sub,ZMQ_LINGER,&linger,sizeof(linger));zmq_setsockopt(sub,ZMQ_CONFLATE,&conflate,sizeof(conflate));
 constexpr char topic[]="pico_hands";zmq_setsockopt(sub,ZMQ_SUBSCRIBE,topic,sizeof(topic)-1);
 const auto endpoint="tcp://127.0.0.1:"+std::to_string(port);if(zmq_connect(sub,endpoint.c_str()))throw std::runtime_error("hand intent connection failed");
 Intent latest;uint64_t last_report=0,last_step=0;std::array<uint64_t,2> sent{};uint64_t malformed=0;J report;
 auto save=[&](bool alive){
  // Copy/inspect feedback before sampling its age: callbacks may run at any time.
  std::lock_guard<std::mutex> lock(hardware.mutex);
  const auto now=ns();
  report={{"protocol",1},{"running",alive},{"observe_only",observe},{"bridge_id",gate.bridge},{"updated_monotonic_ns",now},{"active",gate.active},{"session",gate.active?gate.session:""},{"last_session",gate.session},{"error",gate.error},{"intent_protocols",{1,2}},{"continuous_sides",{"left","right"}},{"finish_open_supported",true},{"malformed",malformed},{"sides",J::object()}};
  for(int side=0;side<2;++side){const auto& f=hardware.feedback[side];
   report["sides"][side==0?"left":"right"]={{"ready",fresh(f,now)},{"samples",f.count},{"age_ms",f.at&&now>=f.at?J(double(now-f.at)/1e6):J(nullptr)},
    {"admitted",sides[side].admitted&&gate.active},{"commanding",sides[side].driving&&(gate.active||sides[side].finish_stage=="opening")},{"finish_session",sides[side].finish_session},{"finish_stage",sides[side].finish_stage},{"finish_error",sides[side].finish_error},{"error",sides[side].error},{"commands_sent",sent[side]},{"q",f.q},{"target",sides[side].target}};
  }
  std::filesystem::create_directories(std::filesystem::path(status).parent_path());std::ofstream out(status+".tmp");out<<report.dump()<<'\n';out.close();std::filesystem::rename(status+".tmp",status);
 };
 while(running&&(duration==0||double(ns()-started)/1e9<duration)){
  const auto now=ns();char buffer[2048];const int size=zmq_recv(sub,buffer,sizeof(buffer),ZMQ_DONTWAIT);
  if(size>0){try{
   if(size>int(sizeof(buffer))||size<int(sizeof(topic)-1)||memcmp(buffer,topic,sizeof(topic)-1))throw std::runtime_error("wire");
   J j=J::parse(buffer+sizeof(topic)-1,buffer+size);
   for(const auto* key:{"active"})if(!j.at(key).is_boolean())throw std::runtime_error("boolean");
   if((j.at("protocol")!=1&&j.at("protocol")!=2)||!j.at("at_ns").is_number_unsigned()||!j.at("sequence").is_number_unsigned())throw std::runtime_error("protocol");
   auto side_intent=[&](const char* key){if(j.at(key).is_null())return -1;if(!j.at(key).is_boolean())throw std::runtime_error("hand boolean");return j.at(key).get<bool>()?1:0;};
   const bool continuous=j.at("protocol")==2;
   if(continuous&&j.at("source")!="pnlink")throw std::runtime_error("hand source");
   Intent i; i.bridge=j.at("bridge_id").get<std::string>();i.session=j.at("session").get<std::string>();
   i.at=j.at("at_ns").get<uint64_t>();i.sequence=j.at("sequence").get<uint64_t>();i.active=j.at("active").get<bool>();
   i.continuous=continuous;
   if(j.contains("finish_open")){
    if(!j.at("finish_open").is_boolean())throw std::runtime_error("finish boolean");
    i.finish_open=j.at("finish_open").get<bool>();
   }
   if(continuous){
    for(const auto* side:{"left","right"}){
     if(j.at(side).is_null())continue;
     const auto& q=j.at(side);if(!q.is_array()||q.size()!=6)throw std::runtime_error("hand target");
     auto& goal=std::string(side)=="left"?i.left_goal:i.right_goal;
     for(size_t k=0;k<6;++k){if(!q[k].is_number())throw std::runtime_error("hand number");goal[k]=q[k].get<double>();}
     (std::string(side)=="left"?i.left:i.right)=1;
    }
   }else{i.left=side_intent("left");i.right=side_intent("right");}
   if(j.contains("finish_sides")){
    if(!i.finish_open||!j.at("finish_sides").is_array()||j.at("finish_sides").empty()||j.at("finish_sides").size()>2)throw std::runtime_error("finish sides");
    i.finish_left=i.finish_right=false;
    for(const auto& side:j.at("finish_sides")){
     if(side=="left"&&!i.finish_left)i.finish_left=true;
     else if(side=="right"&&!i.finish_right)i.finish_right=true;
     else throw std::runtime_error("finish side");
    }
   }
   if(gate.observe(i,ns()))latest=i;
  }catch(const std::exception&){++malformed;}}
  gate.expire(ns());
  if(now-last_step>=50000000){
   last_step=now;
   for(int side=0;side<2;++side){Feedback feedback;uint64_t other;
    {std::lock_guard<std::mutex> lock(hardware.mutex);feedback=hardware.feedback[side];other=hardware.other_at[side];}
    const auto step_now=ns();
    const bool quiet=step_now-started>=5000000000ULL&&(!other||step_now-other>=5000000000ULL);
    auto out=sides[side].step(gate,side==0?latest.left:latest.right,feedback,step_now,quiet,
      latest.continuous?(side==0?&latest.left_goal:&latest.right_goal):nullptr,
      latest.finish_open&&(side==0?latest.finish_left:latest.finish_right));
    if(physical&&out.send){if(!hardware.send(side,out)){sides[side].admitted=false;if(sides[side].finish_stage=="opening"||sides[side].finish_stage=="succeeded"){sides[side].finish_stage="failed";sides[side].finish_error="清零命令发送失败";}sides[side].error="手部命令发送失败，需退出并重新进入遥操";}else ++sent[side];}
   }
  }
  if(now-last_report>=100000000){last_report=now;save(true);}
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
 }
 gate.active=false;
 for(int side=0;side<2;++side){Feedback f;{std::lock_guard<std::mutex> lock(hardware.mutex);f=hardware.feedback[side];}
  auto out=sides[side].step(gate,-1,f,ns(),true);if(physical&&out.send)hardware.send(side,out);
 }
 save(false);std::cout<<report.dump(2)<<std::endl;zmq_close(sub);zmq_ctx_term(ctx);if(lockfd>=0)close(lockfd);return 0;
}catch(const std::exception& e){std::cerr<<"BrainCo hands: "<<e.what()<<std::endl;return 1;}}
