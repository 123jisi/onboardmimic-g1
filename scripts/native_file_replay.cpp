// Offline only: no Unitree, DDS, sockets, hand bridge or controller construction.
// Production MotionSequence/merger/math and gather method bodies are compiled in.
#include <sstream>
#include <memory>
#include <nlohmann/json.hpp>
#include "motion_data_reader.hpp"
#include "input_interface/streamed_motion_merger.hpp"
#define main benchmark_unused_main
#include "native_benchmark.cpp"
#undef main
using json=nlohmann::json;
struct ReplayHistory {
 struct Entry {std::array<double,4> base_quat{1,0,0,0};};
 std::vector<Entry> GetLatest(size_t n,double){return std::vector<Entry>(n);}
};
struct HeadingState {std::array<double,4> init_base_quat{1,0,0,0};double delta_heading=0;
 HeadingState()=default;HeadingState(std::array<double,4> q,double d):init_base_quat(q),delta_heading(d){}};
struct HeadingBuffer {std::shared_ptr<HeadingState> value;void SetData(HeadingState s){value=std::make_shared<HeadingState>(s);}struct Value {std::shared_ptr<HeadingState> data;};Value GetDataWithTime(){return {value};}};
struct Gather {
 std::shared_ptr<MotionSequence> current_motion_;int current_frame_=0,saved_frame_for_observation_window_=0;
 struct {bool play=true;} operator_state;double control_dt_=.02;bool reinitialize_heading_=true;
 std::shared_ptr<ReplayHistory> state_logger_=std::make_shared<ReplayHistory>();HeadingBuffer heading_state_buffer_;
 std::array<double,4> init_ref_data_root_rot_array_{1,0,0,0};
 std::array<double,29> upper_body_joint_positions_buffer_{},upper_body_joint_velocities_buffer_{};
 bool ShouldApplyUpperBodyObservationOverride(){return false;}bool ShouldOverrideUpperBodyIndex(size_t){return false;}
#include "replay_gather_generated.hpp"
};
StreamedMotionMerger::IncomingData decode(const json& x){
 StreamedMotionMerger::IncomingData d;d.frame_indices=x.at("frame_index").get<std::vector<int64_t>>();
 d.joint_pos=x.at("joint_pos").get<std::vector<std::vector<double>>>();d.joint_vel=x.at("joint_vel").get<std::vector<std::vector<double>>>();
 d.num_frames=d.frame_indices.size();d.num_joints=29;d.num_quat_bodies=1;d.protocol_version=x.at("version");
 d.body_quat=x.at("body_quat").get<std::vector<std::vector<std::array<double,4>>>>();
 if(d.protocol_version==3){d.smpl_joints=x.at("smpl_joints").get<std::vector<std::vector<std::array<double,3>>>>();d.smpl_pose=x.at("smpl_pose").get<std::vector<std::vector<std::array<double,3>>>>();d.num_smpl_joints=24;d.num_smpl_poses=21;}
 return d;
}
std::vector<double> observe(Gather& g,const json& layout,int mode){
 std::vector<double> out(layout.at("dimension").get<int>(),0.);g.UpdateHeadingState();
 for(const auto& item:layout.at("fields")){
  const auto name=item.at("name").get<std::string>();const int offset=item.at("offset");bool ok=true;
  if(name=="encoder_mode_4")out[offset]=mode;
  if(mode==0){
   if(name=="motion_joint_positions_10frame_step5")ok=g.GatherMotionJointPositionsMultiFrame(out,offset,10,5);
   if(name=="motion_joint_velocities_10frame_step5")ok=g.GatherMotionJointVelocitiesMultiFrame(out,offset,10,5);
   if(name=="motion_anchor_orientation_10frame_step5")ok=g.GatherMotionAnchorOrientationMutiFrame(out,offset,10,5);
  }else if(mode==2){
   if(name=="smpl_joints_10frame_step1")ok=g.GatherMotionSmplJointsMultiFrame(out,offset,10,1);
   if(name=="smpl_anchor_orientation_10frame_step1")ok=g.GatherMotionAnchorOrientationMutiFrame(out,offset,10,1);
   if(name=="motion_joint_positions_wrists_10frame_step1")ok=g.GatherMotionJointPositionsMultiFrame(out,offset,10,1,wrist_joint_isaaclab_order_in_isaaclab_index);
  }else throw std::runtime_error("unsupported explicit encoder mode");
  if(!ok)throw std::runtime_error("observation unavailable; no mode fallback: "+name);
 }
 return out;
}
int main(int argc,char**argv){try{
 if(argc!=3)throw std::runtime_error("usage: native_file_replay input.json output.json");
 json input;std::ifstream(argv[1])>>input;
 // Suppress production verbose text, preserving stdout for the result summary.
 std::ostringstream debug;auto* saved=std::cout.rdbuf(debug.rdbuf());
 struct RestoreOutput {std::streambuf* original;~RestoreOutput(){std::cout.rdbuf(original);}} restore{saved};
 std::unique_ptr<Model> enc,dec;if(input.value("gpu",false)){
  const auto path=input.at("engines").get<std::string>();enc=std::make_unique<Model>(path+"/encoder_fp32.engine");dec=std::make_unique<Model>(path+"/decoder_fp32.engine");}
 StreamedMotionMerger merger;Gather g;int window=0;json traces=json::array();
 std::vector<float> actions(29,0.);int unexpected_resets=0;
 for(const auto& tick:input.at("ticks")){
  if(tick.value("reset",false)){merger.Reset();g=Gather();window=0;std::fill(actions.begin(),actions.end(),0.);}
  if(tick.contains("cursor"))g.current_frame_=tick.at("cursor");
  bool reset=false;
  if(!tick.at("packet").is_null()){
   auto d=decode(tick.at("packet"));auto result=merger.MergeIncomingData(d,g.current_frame_);
   if(!result.motion)throw std::runtime_error("merge failed");
   reset=result.did_catchup_reset;if(reset){if(g.current_motion_)++unexpected_resets;g.current_frame_=0;g.reinitialize_heading_=true;}
   else g.current_frame_=std::max(0,std::min(g.current_frame_-result.frame_offset_adjustment,result.motion->timesteps-1));
   g.current_motion_=result.motion;g.current_motion_->SetEncodeMode(d.protocol_version==3?2:0);window=result.window_start;
  }
  if(!g.current_motion_)continue;
  const int mode=g.current_motion_->GetEncodeMode();auto obs=observe(g,input.at("encoder_layout"),mode);
  json record={{"tick",tick.at("tick")},{"window_start",window},{"window_frames",g.current_motion_->timesteps},{"consumed_frame",window+g.current_frame_},{"mode",mode},{"catchup",reset},{"encoder_observation",obs}};
  if(enc){
   if(enc->sizes[0]!=obs.size()*4)throw std::runtime_error("encoder binding dimension differs from production config");
   auto ep=static_cast<float*>(enc->host[0]);for(size_t k=0;k<obs.size();++k)ep[k]=obs[k];enc->run();
   int token_index=-1;for(int k=0;k<enc->engine->getNbBindings();++k)if(!enc->engine->bindingIsInput(k))token_index=k;
   if(token_index<0||enc->sizes[token_index]!=64*4)throw std::runtime_error("token binding");
   std::vector<float> token(64);memcpy(token.data(),enc->host[token_index],64*4);
   // Fixed nominal histories: zero joint deviation/velocity/gyro, upright gravity.
   // last_action is intentionally fixed zero, so comparisons share identical history.
   auto policy=static_cast<float*>(dec->ptr("obs_dict"));std::fill_n(policy,dec->sizes[0]/4,0.f);memcpy(policy,token.data(),64*4);
   const int expected_policy_dim=64+30+290+290+290+30;
   if(dec->sizes[0]!=expected_policy_dim*4)throw std::runtime_error("decoder config dimension");
   for(int k=0;k<10;++k)policy[64+30+290+290+290+k*3+2]=-1;
   dec->run();memcpy(actions.data(),dec->ptr("action"),29*4);
   std::vector<double> target(29);for(int i=0;i<29;++i)target[i]=default_angles[i]+double(actions[isaaclab_to_mujoco[i]])*g1_action_scale[i];
   record["token"]=token;record["policy_observation"]=std::vector<float>(policy,policy+expected_policy_dim);record["raw_action"]=actions;record["unrestricted_hardware_q_target"]=target;
  }
  traces.push_back(std::move(record));
  const int lookahead=std::max(1,g.saved_frame_for_observation_window_);
  g.current_frame_=std::min(g.current_frame_+1,std::max(0,g.current_motion_->timesteps-lookahead));
  debug.str("");debug.clear();
 }
 std::cout.rdbuf(saved);
 json out={{"motor_output",false},{"hand_output",false},{"dds_linked",false},{"gpu",bool(enc)},{"synthetic_fixed_robot_history",true},{"final_output_safety_filters_exercised",false},{"unexpected_catchup_resets",unexpected_resets},{"traces",traces}};
 std::ofstream(argv[2])<<out.dump();std::cout<<json({{"frames",traces.size()},{"unexpected_resets",unexpected_resets},{"gpu",bool(enc)}}).dump()<<'\n';
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
