#include "core.hpp"
#include <cassert>
#include <limits>
using namespace pico_hands;
int main(){
 const std::string id(32,'a'),sid(32,'b'),sid2(32,'c');uint64_t now=1000000000;
 Gate g(id);Intent i{id,sid,now,1,true,1,0};
 assert(!g.observe(i,now));i.active=false;assert(g.observe(i,now));
 i.active=true;i.at+=1;i.sequence++;assert(g.observe(i,now+1));
 assert(!g.observe(i,now+1));
 g.expire(now+input_timeout+2);assert(!g.active);assert(!g.error.empty());
 i.active=false;i.at=now+input_timeout+3;i.sequence++;assert(g.observe(i,i.at));
 i.active=true;i.at++;i.sequence++;assert(!g.observe(i,i.at));
 i.session=sid2;i.at++;i.sequence++;assert(g.observe(i,i.at));assert(g.error.empty());
 auto bad=i;bad.bridge=std::string(32,'d');bad.at++;bad.sequence++;assert(!g.observe(bad,bad.at));
 Feedback f;f.q={.1,.2,.3,.4,.5,.6};f.count=10;f.at=i.at;f.valid=true;
 Side s;auto out=s.step(g,1,f,i.at,true);assert(out.send&&out.speed==.3);
 f.at+=50000000;out=s.step(g,1,f,f.at,true);assert(out.q[0]<.1);assert(out.q[2]>.3); // curl four fingers before thumb
 f.q=fingers;f.at+=50000000;out=s.step(g,1,f,f.at,true);assert(s.thumb_phase&&out.q[0]>.06);
 g.active=false;out=s.step(g,-1,f,f.at,true);assert(out.send&&out.speed==0&&out.q==f.q);
 assert(!s.step(g,-1,f,f.at,true).send);
 g.active=true;f.at+=feedback_timeout+1;assert(!s.step(g,1,Feedback{},f.at,true).send);assert(!s.admitted);
 f.valid=true;f.count=20;assert(!s.step(g,1,f,f.at,true).send); // same session cannot revive
 g.session=std::string(32,'e');assert(s.step(g,0,f,f.at,true).send);
 Side missing;assert(!missing.step(g,1,Feedback{},f.at,true).send);
 Side busy;assert(!busy.step(g,1,f,f.at,false).send);
 Q nan{};nan[0]=std::numeric_limits<double>::quiet_NaN();assert(!valid_q(nan));
 assert(!fresh(f,f.at+feedback_timeout+1));
 // A feedback callback can arrive after the loop timestamp. Age must use a
 // clock sample taken after reading feedback, never the earlier loop clock.
 Feedback arriving;arriving.valid=true;arriving.count=10;arriving.at=now+1000;
 assert(!fresh(arriving,now));assert(fresh(arriving,arriving.at+1));
 assert(!fresh(arriving,arriving.at+feedback_timeout+1));
 // PN continuous targets use host smoothing and slew, never the PICO fist stages.
 Gate cg(id);Intent ci{id,sid,now,1,false,-1,-1};assert(cg.observe(ci,now));
 ci.active=true;ci.at++;ci.sequence++;ci.continuous=true;ci.right=1;ci.right_goal={.15,.1875,.25,.25,.25,.25};
 assert(cg.observe(ci,ci.at));Side cs;Feedback cf;cf.valid=true;cf.count=10;cf.at=ci.at;
 auto co=cs.step(cg,1,cf,cf.at,true,&ci.right_goal);assert(co.send&&co.q==Q{});
 cf.at+=50000000;co=cs.step(cg,1,cf,cf.at,true,&ci.right_goal);
 assert(co.q[0]>0&&co.q[0]<=.0400001);assert(co.q[2]>.0&&co.q[2]<=.0400001);
 Q low{.01,.01,.01,.01,.01,.01};cf.at+=50000000;co=cs.step(cg,1,cf,cf.at,true,&low);
 assert(co.q[0]<.04&&co.q[0]>.01); // smooth decay, not binary open
 auto invalid=ci;invalid.at=cf.at+1;invalid.sequence++;invalid.right_goal[0]=.61;
 assert(!cg.observe(invalid,invalid.at));invalid.right_goal[0]=NAN;assert(!cg.observe(invalid,invalid.at));
 invalid.right_goal[0]=0;invalid.left=1;invalid.left_goal[0]=.61;assert(!cg.observe(invalid,invalid.at));
 invalid.left_goal[0]=NAN;assert(!cg.observe(invalid,invalid.at));
 invalid.left_goal=open;assert(cg.observe(invalid,invalid.at));
 cg.expire(cf.at+input_timeout+1);cf.at+=input_timeout+1;
 co=cs.step(cg,-1,cf,cf.at,true,&low);assert(co.send&&co.speed==0&&co.q==cf.q);

 // Only a fresh, explicit finish of the current PN session can open the hand.
 Gate fg(id);Intent fi{id,sid,now,1,false,-1,-1};assert(fg.observe(fi,now));
 fi.active=true;fi.continuous=true;fi.right=1;fi.right_goal=closed;fi.at++;fi.sequence++;
 assert(fg.observe(fi,fi.at));Side fs;Feedback ff;ff.valid=true;ff.count=10;ff.at=fi.at;ff.q=closed;
 assert(fs.step(fg,1,ff,ff.at,true,&fi.right_goal).send);
 auto wrong=fi;wrong.active=false;wrong.right=-1;wrong.finish_open=true;wrong.session=sid2;wrong.at++;wrong.sequence++;
 assert(!fg.observe(wrong,wrong.at));
 fi.active=false;fi.right=-1;fi.finish_open=true;fi.at++;fi.sequence++;
 assert(fg.observe(fi,fi.at));assert(fg.blocked.count(sid));
 Output zeroing;
 for(int n=0;n<120&&fs.finish_stage!="succeeded";++n){
  fi.at+=50000000;fi.sequence++;assert(fg.observe(fi,fi.at));ff.at=fi.at;
  zeroing=fs.step(fg,-1,ff,ff.at,true,nullptr,true);
  if(zeroing.send){for(size_t k=0;k<6;++k)assert(zeroing.q[k]<=ff.q[k]+1e-12);ff.q=zeroing.q;}
 }
 assert(fs.finish_stage=="succeeded"&&zeroing.speed==0&&ff.q==open);
 assert(!fs.step(fg,-1,ff,ff.at,true,nullptr,true).send); // duplicate finish cannot replay
 fi.active=true;fi.finish_open=false;fi.right=1;fi.at++;fi.sequence++;assert(!fg.observe(fi,fi.at));
 // Real feedback/input loss during zeroing stops and cannot auto-resume.
 Gate lg(id);Intent li{id,sid2,now,1,false,-1,-1};assert(lg.observe(li,now));
 li.active=true;li.continuous=true;li.right=1;li.at++;li.sequence++;assert(lg.observe(li,li.at));
 Side ls;ff.q=closed;ff.at=li.at;assert(ls.step(lg,1,ff,ff.at,true,&closed).send);
 li.active=false;li.finish_open=true;li.right=-1;li.at++;li.sequence++;assert(lg.observe(li,li.at));
 ff.at=li.at;assert(ls.step(lg,-1,ff,ff.at,true,nullptr,true).send);
 auto lost=ls.step(lg,-1,ff,ff.at+input_timeout+1,true,nullptr,true);
 assert(ls.finish_stage=="failed"&&lost.send&&lost.speed==0);
 ff.at+=input_timeout+2;li.at=ff.at;li.sequence++;assert(lg.observe(li,li.at));
 assert(!ls.step(lg,-1,ff,ff.at,true,nullptr,true).send);

 // Bilateral continuous goals share a session but have independent smoothing.
 Gate bg(id);Intent bi{id,sid,now,1,false,-1,-1};assert(bg.observe(bi,now));
 bi.active=true;bi.continuous=true;bi.left=bi.right=1;bi.at++;bi.sequence++;
 bi.left_goal={0.,0.,.25,0.,0.,0.};bi.right_goal={0.,0.,0.,0.,0.,.25};
 assert(bg.observe(bi,bi.at));std::array<Side,2> bs;Feedback bf;bf.valid=true;bf.count=10;bf.at=bi.at;
 assert(bs[0].step(bg,1,bf,bf.at,true,&bi.left_goal).send);
 assert(bs[1].step(bg,1,bf,bf.at,true,&bi.right_goal).send);
 bf.at+=50000000;
 auto bl=bs[0].step(bg,1,bf,bf.at,true,&bi.left_goal),br=bs[1].step(bg,1,bf,bf.at,true,&bi.right_goal);
 assert(bl.q[2]>0&&bl.q[5]==0&&br.q[5]>0&&br.q[2]==0);
 bi.active=false;bi.left=bi.right=-1;bi.finish_open=true;bi.finish_left=bi.finish_right=true;bi.at=bf.at+1;bi.sequence++;
 assert(bg.observe(bi,bi.at));
 for(int side=0;side<2;++side){
  bf.q=side==0?bl.q:br.q;bf.at=bi.at;
  assert(bs[side].step(bg,-1,bf,bf.at,true,nullptr,true).send);
  assert(bs[side].finish_stage=="opening");
 }
 // A finish never opens an unadmitted side, and inactive/no-goal streams hold.
 Side never_owned;assert(!never_owned.step(bg,-1,bf,bf.at,true,nullptr,true).send);
 assert(never_owned.finish_stage=="failed");
 auto empty=bi;empty.finish_left=empty.finish_right=false;empty.at++;empty.sequence++;
 assert(!bg.observe(empty,empty.at));

}
