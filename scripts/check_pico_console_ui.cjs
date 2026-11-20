// Isolated browser acceptance. The fake HTTP server cannot operate a robot.
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const http=require('http'),fs=require('fs'),path=require('path'),assert=require('assert');
const root=path.resolve(__dirname,'../onboardmimic/console/static');
const artifacts=fs.mkdtempSync(path.join(require('os').tmpdir(),'onboard-pico-ui-'));
let state={state:'stopped',mode:'onboard_real',robot:'G1 mock',robot_id:'unitree_g1',backend:'sonic',backend_label:'SONIC',capabilities:['planner','pose','joint_playback','smpl_playback'],pids:[],input_mode:'planner',input_source:'planner',stationary:false,planner_control_allowed:false,input_mode_switch_available:false,pose_adapter_ready:true,pose_input_ready:true,pico_ready:true,mode_switch_pending:false,telemetry:{fresh:true,valid:true,faults:[]},operator:{owner:null,generation:0},revision:0,pico_session:{controllers_ready:true,buttons_released:false,locomotion_mode:0},playback:{state:'loaded',position_seconds:0,clip:{name:'sample',duration_seconds:3}}};
const posts=[];
const server=http.createServer(async(req,res)=>{
 res.setHeader('Content-Type','application/json');
 if(req.url==='/api/session')return res.end(JSON.stringify({token:'offline-only'}));
 if(req.url==='/api/platform')return res.end(JSON.stringify({schema_version:1,robots:[{id:'unitree_g1',label:'G1'}],backends:[{id:'sonic',label:'SONIC'}],pairs:[{robot_id:'unitree_g1',backend_id:'sonic'}],selected:{robot_id:'unitree_g1',backend_id:'sonic'},switching:{supported:false}}));
 if(req.method==='POST'){
  let raw='';for await(const c of req)raw+=c;const x=JSON.parse(raw);posts.push({route:req.url,...x});
  if(req.url==='/api/operator'&&['acquire','takeover'].includes(x.action))state.operator={owner:x.actor,generation:++state.revision,remaining_ms:5000};
  if(req.url==='/api/controller'&&x.action==='select_pico'){
   assert.equal(x.alignment_confirmed,true);state.input_source='pico';state.input_mode=x.pico_mode==='pose'?'pose':'planner';state.pico_mode=x.pico_mode;state.pico_session.active=true;
  }
  if(req.url==='/api/controller'&&x.action==='select_input_mode'){
   state.input_mode='planner';state.input_source='planner';state.pico_mode=null;state.pico_session.active=false;
  }
  if(req.url==='/api/playback'&&x.action==='switch'){state.input_source='file_joint';state.input_mode='joint_playback';state.pico_session.active=false;}
  return res.end(JSON.stringify(state));
 }
 if(req.url==='/api/controller')return res.end(JSON.stringify(state));
 if(req.url==='/api/motions')return res.end(JSON.stringify({state:'ready',entries:[{id:'cache/demo',name:'demo',kind:'cache',motion:'demo',status:'ready'}]}));
 if(req.url.includes('/skeleton'))return res.end(JSON.stringify({available:false,joints:[]}));
 const name=req.url==='/'?'index.html':req.url==='/remote'?'mobile.html':req.url.slice(1);
 if(!/^[a-z0-9.-]+$/.test(name)||!fs.existsSync(path.join(root,name))){res.statusCode=404;return res.end('{}');}
 res.setHeader('Content-Type',name.endsWith('.js')?'text/javascript':name.endsWith('.css')?'text/css':'text/html');res.end(fs.readFileSync(path.join(root,name)));
});
(async()=>{
 await new Promise(r=>server.listen(0,'127.0.0.1',r));let browser;
 try{
  browser=await chromium.launch({executablePath:process.env.CHROMIUM_EXECUTABLE||undefined,headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--no-proxy-server']});
  const errors=[],page=await browser.newPage({viewport:{width:1280,height:1000},locale:'zh-CN'});page.on('pageerror',e=>errors.push(e.message));
  const url='http://127.0.0.1:'+server.address().port;await page.goto(url);
  await page.waitForFunction(()=>!document.getElementById('workspace').hidden);
  await page.locator('#pose-tab').click();assert(await page.locator('#pico-mode').isVisible());assert(await page.locator('#select-pose').isDisabled());
  assert.equal(posts.length,0,'GET/preview must not send commands');
  state.pico_hands={enabled:true,fresh:true,sides:{left:{ready:false},right:{ready:true,error:''}}};
  await page.waitForFunction(()=>document.getElementById('pico-hands-state').textContent.includes('右手：就绪'));
  await page.locator('#operator-acquire').click();
  state.setup_ready=true;state.setup_state='active';state.setup_message='控制服务已连接';
  state.prepare_available=false;state.telemetry.faults=[5];
  state.prepare_blockers=[{code:'motor_fault',message:'电机 5（左踝横滚）报告故障：512（0x200）；温度反馈 [42, 42] °C'},{code:'feedback_continuity',message:'机器人反馈链路未就绪'}];
  const faultPollStart=posts.length;let unexpectedDialog=false;
  const rejectDialog=async d=>{unexpectedDialog=true;await d.dismiss()};page.on('dialog',rejectDialog);
  await page.waitForFunction(()=>document.getElementById('message').textContent.includes('0x200')&&document.getElementById('message').textContent.includes('机器人反馈链路未就绪'));
  assert((await page.locator('#connection').textContent()).includes('电机故障'));
  assert(!(await page.locator('#connection').textContent()).includes('状态反馈正常'));
  assert(await page.locator('#prepare').isDisabled());
  assert(!unexpectedDialog,'preparation errors must be inline, without a dialog');
  assert(!posts.slice(faultPollStart).some(x=>x.route!=='/api/operator'),'displaying self-check errors must never send motion commands');
  await page.locator('#language').selectOption('en');
  await page.waitForFunction(()=>document.getElementById('message').textContent.includes('Preparation checks failed'));
  await page.locator('#language').selectOption('zh-CN');
  state.prepare_blockers=[];state.telemetry.faults=[];state.prepare_available=true;
  await page.waitForFunction(()=>document.getElementById('message').textContent==='控制服务已连接');
  assert(!await page.locator('#prepare').isDisabled());page.off('dialog',rejectDialog);
  state.setup_message='';
  state.state='active';state.pids=[123];state.input_mode='joint_playback';state.input_source='file_joint';state.input_mode_switch_available=true;
  await page.locator('#pico-alignment').check();await page.locator('#pico-mode').selectOption('vr3pt');
  await page.waitForFunction(()=>!document.getElementById('select-pose').disabled);
  page.on('dialog',d=>d.accept());await page.locator('#select-pose').click();
  await page.waitForFunction(()=>document.getElementById('pico-session-state').textContent.includes('摇杆行走＋三点跟随'));
  assert(posts.some(x=>x.action==='select_pico'&&x.pico_mode==='vr3pt'));
  assert(!await page.locator('#pose-stop').isDisabled());
  const beforeHoldPolls=posts.length;
  state.pico_session.recovering=true;state.pico_session.recovery_remaining_ms=2500;
  state.pico_session.error='PICO 断流保持中，剩余 2.5 秒：等待新鲜身体和双手柄数据';
  state.pico_session.controllers_ready=false;state.pico_ready=false;state.pose_input_ready=false;
  await page.waitForFunction(()=>document.getElementById('pico-session-state').textContent.includes('断流保持中，剩余 2.5 秒'));
  assert(!await page.locator('#pose-stop').isDisabled(),'holding PICO must still permit explicit stop');
  assert(!posts.slice(beforeHoldPolls).some(x=>x.route!=='/api/operator'),'hold display must never request a mode switch');
  state.pico_session.recovering=false;state.pico_session.error='';
  state.pico_session.controllers_ready=true;state.pico_ready=true;state.pose_input_ready=true;
  state.pico_hands.error='手部状态不可用，本体不受影响';state.pico_hands.fresh=false;
  await page.waitForFunction(()=>document.getElementById('pico-hands-state').textContent==='手部状态不可用，本体不受影响');
  assert(!await page.locator('#select-pose').isDisabled(),'optional hand loss must not disable body PICO entry');
  delete state.pico_hands.error;state.pico_hands.fresh=true;
  // Real gateway clears its progress reason after acknowledged handoff.
  // The page must clear the old status text on subsequent read-only polls.
  const progress='PICO 切换：等待零速 Planner 确认并站稳',beforeStatusPolls=posts.length;
  state.motion_task={kind:'pico',stage:'waiting_planner',target:'vr3pt'};
  state.mode_switch_pending=true;state.mode_switch_error=progress;
  await page.waitForFunction(text=>document.getElementById('message').textContent===text,progress);
  state.motion_task.stage='finished';state.mode_switch_pending=false;state.mode_switch_error='';
  await page.waitForFunction(()=>document.getElementById('message').textContent==='PICO 模式切换已完成',{},{timeout:4000});
  state.workflow_error='真实状态反馈故障';
  await page.waitForFunction(()=>document.getElementById('message').textContent==='真实状态反馈故障');
  state.workflow_error='';
  await page.waitForFunction(()=>document.getElementById('message').textContent==='PICO 模式切换已完成');
  assert(!posts.slice(beforeStatusPolls).some(x=>x.route!=='/api/operator'),'status updates must not send control requests');
  await page.locator('#planner-tab').click();assert(!await page.locator('#select-planner').isDisabled(),'PICO planner must allow explicit exit');
  await page.locator('#select-planner').click();await page.waitForFunction(()=>document.getElementById('pose-stop').disabled);
  await page.locator('#pose-tab').click();await page.locator('#pico-mode').selectOption('pose');await page.locator('#select-pose').click();
  await page.waitForFunction(()=>!document.getElementById('motion-play').disabled);
  const switched=page.waitForResponse(r=>r.url().endsWith('/api/playback')&&r.request().method()==='POST');await page.locator('#motion-play').click();await switched;
  assert(posts.some(x=>x.route==='/api/playback'&&x.action==='switch'));
  // Stopped/newly prepared generations carry no retired switch notice.
  const completedFile='文件播放结束，已请求零速 Planner';
  state.input_source='planner';state.input_mode='planner';state.mode_switch_error=completedFile;
  await page.waitForFunction(text=>document.getElementById('message').textContent===text,completedFile);
  state.state='stopped';state.pids=[];state.mode_switch_error='';state.mode_switch_pending=false;
  await page.waitForFunction(()=>document.getElementById('message').textContent==='');
  state.state='prepared';state.pids=[456];state.pose_ready=true;
  await page.waitForFunction(()=>document.getElementById('mode-state').textContent==='姿态已就绪');
  assert.equal(await page.locator('#message').textContent(),'');
  state.workflow_error='真实状态反馈故障';
  await page.waitForFunction(()=>document.getElementById('message').textContent==='真实状态反馈故障');
  state.workflow_error='';
  await page.waitForFunction(()=>document.getElementById('message').textContent==='');
  const mobile=await browser.newPage({viewport:{width:390,height:844},isMobile:true});mobile.on('pageerror',e=>errors.push(e.message));await mobile.goto(url+'/remote');await mobile.locator('#pose-tab').click();
  assert(await mobile.locator('#pico-mode').isVisible());assert(!await mobile.evaluate(()=>document.documentElement.scrollWidth>innerWidth));
  assert(await mobile.locator('#pico-hands-state').isVisible());
  state.state='stopped';state.pids=[];state.prepare_available=false;state.setup_message='控制服务已连接';state.telemetry.faults=[5];
  state.prepare_blockers=[{code:'motor_fault',message:'电机 5（左踝横滚）报告故障：512（0x200）；温度反馈 [42, 42] °C'}];
  await mobile.waitForFunction(()=>document.getElementById('message').textContent.includes('0x200'));
  assert(await mobile.locator('#prepare').isDisabled());
  assert(!await mobile.evaluate(()=>document.documentElement.scrollWidth>innerWidth));
  state.prepare_blockers=[];state.telemetry.faults=[];state.prepare_available=true;state.setup_message='';
  await mobile.evaluate(()=>window.scrollTo(0,0));await page.evaluate(()=>window.scrollTo(0,0));
  await mobile.screenshot({path:path.join(artifacts,'mobile.png'),fullPage:true});await page.screenshot({path:path.join(artifacts,'desktop.png'),fullPage:true});
  await page.locator('#language').selectOption('en');await page.waitForFunction(()=>document.querySelector('option[value="vr3pt"]').textContent==='Walk with three-point tracking');
  await page.waitForFunction(()=>document.getElementById('pico-hands-state').textContent.includes('Hold the index trigger to close'));
  assert.deepEqual(errors,[]);assert(!posts.some(x=>['prepare_pose','start_policy','estop','connect_services'].includes(x.action)));
  console.log(JSON.stringify({preview_before_start:true,file_to_pico:true,pico_planner_exit:true,pico_to_file:true,explicit_alignment:true,cleared_completed_switch_progress:true,preserved_live_errors:true,cleared_retired_controller_notices:true,mobile:true,english:true,errors,artifacts}));
 }finally{if(browser)await browser.close();server.close();}
})().catch(e=>{console.error(e);process.exitCode=1});
