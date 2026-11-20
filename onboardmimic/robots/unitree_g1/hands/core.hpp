#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <set>
#include <string>

namespace pico_hands {
using Q=std::array<double,6>;
constexpr Q open{};
// Verified physical Revo2 grasp in host_setup/g1_hands/control.py, not Dex3 radians.
constexpr Q closed{.6,.75,1.,1.,1.,1.};
constexpr Q fingers{0.,.75,1.,1.,1.,1.};
constexpr uint64_t input_timeout=250000000,feedback_timeout=500000000;
inline bool valid_q(const Q& q){for(double x:q)if(!std::isfinite(x)||x<-.03||x>1.03)return false;return true;}
struct Feedback {Q q{};uint64_t at=0,count=0;bool valid=false;};
inline bool fresh(const Feedback& f,uint64_t now){return f.valid&&f.count>=10&&now>=f.at&&now-f.at<=feedback_timeout;}
struct Intent {std::string bridge,session;uint64_t at=0,sequence=0;bool active=false;int left=-1,right=-1;bool continuous=false;Q right_goal{};bool finish_open=false;Q left_goal{};bool finish_left=false,finish_right=true;};
class Gate {
public:
 std::string bridge,session,error;bool active=false,idle_seen=false;uint64_t last_at=0,last_sequence=0;
 std::set<std::string> blocked;
 explicit Gate(std::string id):bridge(std::move(id)){}
 bool observe(const Intent& i,uint64_t now){
  if(i.finish_open&&(!i.continuous||i.active||i.session.size()!=32||i.session!=session||i.left!=-1||i.right!=-1||(!i.finish_left&&!i.finish_right)))return false;
  if(i.continuous){
   for(const auto* q:{&i.left_goal,&i.right_goal})
    for(size_t k=0;k<6;++k)if(!std::isfinite((*q)[k])||(*q)[k]<0||(*q)[k]>closed[k])return false;
   if(i.active&&i.left==-1&&i.right==-1)return false;
  }
  if(i.bridge!=bridge||i.at>now||now-i.at>input_timeout||i.at<=last_at||i.sequence==0||
     i.left< -1||i.left>1||i.right< -1||i.right>1)return false;
  if(i.active&&(i.session.size()!=32||!idle_seen||blocked.count(i.session)))return false;
  last_at=i.at;last_sequence=i.sequence;
  if(!i.active){if(i.finish_open)blocked.insert(session);active=false;idle_seen=true;return true;}
  if(!session.empty()&&session!=i.session)blocked.insert(session);
  session=i.session;active=true;error.clear();return true;
 }
 void expire(uint64_t now){if(active&&(now<last_at||now-last_at>input_timeout)){blocked.insert(session);active=false;idle_seen=false;error="手部意图超过250ms未更新，需退出并重新进入遥操";}}
};
struct Output {bool send=false;Q q{};double speed=0;};
class Side {
public:
 std::string session,error;bool admitted=false,driving=false,thumb_phase=false;Q target{};
 uint64_t last_step=0,finish_started=0;bool ever_admitted=false;
 std::string finish_session,finish_stage,finish_error;
 Output step(const Gate& gate,int intent,const Feedback& fb,uint64_t now,bool quiet,const Q* continuous_goal=nullptr,bool finish_open=false){
  const bool ready=fresh(fb,now);
  if(!gate.active&&finish_open){
   if(finish_session!=gate.session){
    finish_session=gate.session;finish_started=now;finish_stage="opening";finish_error.clear();target=fb.q;last_step=now;
    if(session!=gate.session||!ever_admitted){finish_stage="failed";finish_error="没有对应的已接管手部会话";}
   }
   if(finish_stage!="opening")return {};
   const bool timed_out=now<finish_started||now-finish_started>6000000000ULL;
   const bool intent_lost=now<gate.last_at||now-gate.last_at>input_timeout;
   if(!ready||timed_out||intent_lost){
    finish_stage="failed";finish_error=!ready?"清零时手部反馈失效":intent_lost?"清零意图断流":"清零超时";
    Output out=driving&&ready?Output{true,fb.q,0.}:Output{};driving=false;return out;
   }
   const double dt=last_step&&now>=last_step?std::min(.1,double(now-last_step)/1e9):0.;last_step=now;
   bool settled=true;
   for(size_t i=0;i<6;++i){
    target[i]=std::clamp(target[i]+std::clamp(-target[i]*(1.-std::exp(-dt/.12)),-.8*dt,.8*dt),0.,1.);
    settled&=target[i]<.003&&std::abs(fb.q[i])<=.02;
   }
   if(settled){target=open;finish_stage="succeeded";driving=false;return {true,open,0.};}
   driving=true;return {true,target,.3};
  }
  if(finish_stage=="opening"){
   finish_stage="failed";finish_error="清零已被新意图终止";
  }
  if(!gate.active||intent<0){
   Output out;
   if(driving&&ready){out={true,fb.q,0.};target=fb.q;}
   driving=false;last_step=0;
   return out;
  }
  if(session!=gate.session){
   session=gate.session;admitted=ready&&quiet;ever_admitted=admitted;thumb_phase=false;target=fb.q;last_step=now;
   error=!ready?"手部反馈缺失或过期":!quiet?"检测到其他手部命令或尚未完成静默观察":"";
  }
  if(!admitted)return {};
  if(!ready){admitted=false;driving=false;error="手部反馈断流，需退出并重新进入遥操";return {};}
  Q goal=open;
  if(continuous_goal)goal=*continuous_goal;
  else if(intent){
   bool reached=true;
   for(size_t i=1;i<6;++i)reached&=std::abs(fb.q[i]-fingers[i])<=.08;
   thumb_phase=thumb_phase||reached;
   goal=thumb_phase?closed:fingers;
  }else thumb_phase=false;
  const double dt=last_step&&now>=last_step?std::min(.1,double(now-last_step)/1e9):0.;last_step=now;
  for(size_t i=0;i<6;++i){
   const double delta=(goal[i]-target[i])*(continuous_goal?(1.-std::exp(-dt/.12)):1.);
   target[i]=std::clamp(target[i]+std::clamp(delta,-.8*dt,.8*dt),0.,1.);
  }
  driving=true;return {true,target,.3};
 }
};
}
