// Isolated browser fixture; it has no SDK, ROS, robot credentials or motor output.
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const http=require('http'),fs=require('fs'),path=require('path'),assert=require('assert');
const root=path.resolve(__dirname,'../onboardmimic/console/static'),posts=[];
let rejectHandSample=false;
let state={state:'stopped',mode:'onboard_real',robot_id:'unitree_g1',backend:'sonic',capabilities:['planner','pose'],pids:[],input_mode:'planner',input_source:'planner',mode_switch_pending:false,telemetry:{fresh:true},operator:{owner:null,generation:0},revision:0,pnlink_session:{active:false,paused:false,ready:true,calibration_confirmed:false},tracking_inputs:{pnlink:{connected:true,capture_state:'capturing',ready:true,receiver_id:'a',body_age_ms:5,command:{}}}};
state.pnlink_hands={enabled:true,mode:'off',closure:.25,input_ready:true,input_age_ms:5,calibration_present:false,calibration_confirmed:false,capture:{stage:'idle'},worker:{fresh:true,sides:{right:{ready:true}}},preview:{fingers:{ready:true,target:[.1,.12,.2,.15,.05,.1]},grasp:{ready:true,target:[.15,.1875,.25,.25,.25,.25]}}};
state.pnlink_hands.hand='right';
state.pnlink_hands.sides={};
for(const side of ['left','right'])state.pnlink_hands.sides[side]={input_ready:true,input_age_ms:5,calibration_present:false,calibration_confirmed:false,capture:{stage:'idle'},preview:JSON.parse(JSON.stringify(state.pnlink_hands.preview))};
state.pnlink_hands.sides.left.preview.fingers.target=[.03,.04,.05,.06,.07,.08];
state.pnlink_hands.worker.sides.left={ready:true};
state.pnlink_hands.sides.left.preview.grasp.target=[.03,.04,.05,.06,.07,.08];
const server=http.createServer(async(req,res)=>{
 res.setHeader('Content-Type','application/json');
 if(req.url==='/api/session')return res.end(JSON.stringify({token:'offline'}));
 if(req.url==='/api/platform')return res.end(JSON.stringify({robots:[{id:'unitree_g1',label:'G1'}],backends:[{id:'sonic',label:'SONIC'}],selected:{robot_id:'unitree_g1',backend_id:'sonic'},switching:{supported:false}}));
 if(req.url==='/api/motions')return res.end(JSON.stringify({entries:[],state:'ready'}));
 if(req.url.includes('/skeleton'))return res.end(JSON.stringify({available:false,joints:[]}));
 if(req.method==='POST'){
  let raw='';for await(const part of req)raw+=part;const x=JSON.parse(raw);posts.push({path:req.url,...x});
  if(rejectHandSample&&x.action==='capture_hand_open'){res.statusCode=409;return res.end(JSON.stringify({error:'采样测试错误：请检查手部数据'}));}
  if(x.action==='takeover')state.operator={owner:x.actor,generation:2,remaining_ms:5000};
  if(x.action==='configure_hands'){assert(['off','fingers','grasp'].includes(x.hand_mode));state.pnlink_hands.mode=x.hand_mode;state.pnlink_hands.closure=x.hand_closure;if(x.hand)state.pnlink_hands.hand=x.hand;if(x.hand_mode==='off'&&state.pnlink_session.active){state.pnlink_hands.finish={stage:'opening'};setTimeout(()=>state.pnlink_hands.finish={stage:'succeeded'},800);}}
  if(x.action?.startsWith('capture_hand_')){
   const sides=x.hand==='both'?['left','right']:[x.hand||'right'];
   for(const side of sides){const h=state.pnlink_hands.sides[side];h.capture={stage:'running',kind:x.action.slice(13),seconds:0,frames:0};setTimeout(()=>{h.capture={stage:'succeeded'};h.calibration_present=true;h.calibration_confirmed=true;},650);}
  }
  if(x.action==='confirm_hand_calibration'){assert(x.calibration_confirmed);state.pnlink_hands.sides[x.hand||'right'].calibration_confirmed=true;}
  if(x.action==='acquire')state.operator={owner:x.actor,generation:1,remaining_ms:5000};
  if(x.action==='start_receiver'){state.tracking_inputs.pnlink.service={active_state:'active'};state.tracking_inputs.pnlink.sdk_open=true;state.tracking_inputs.pnlink.connected=true;state.pnlink_session.ready=true;}
  if(x.action==='stop_receiver'){state.tracking_inputs.pnlink.service={active_state:'inactive'};state.tracking_inputs.pnlink.sdk_open=false;state.tracking_inputs.pnlink.connected=false;state.pnlink_session.ready=false;}
  if(x.action==='confirm_calibration'){assert.equal(x.calibration_confirmed,true);state.pnlink_session.calibration_confirmed=true}
  if(x.action==='select_pnlink'){assert.equal(x.alignment_confirmed,true);state.input_source='pnlink';state.input_mode='pose';state.pnlink_session.active=true}
  if(x.action==='pause_pnlink')state.pnlink_session.paused=true;
  if(x.action==='resume_pnlink')state.pnlink_session.paused=false;
  if(x.stop||x.action==='calibrate'){state.input_source='planner';state.input_mode='planner';state.pnlink_session.active=false}
  if(x.action==='calibrate')state.tracking_inputs.pnlink.command={action:'calibrate',stage:'running',pose:'T-pose',progress:3};
  return res.end(JSON.stringify(state));
 }
 if(req.url==='/api/controller')return res.end(JSON.stringify(state));
 const name=req.url==='/'?'index.html':req.url==='/remote'?'mobile.html':req.url.slice(1);
 if(!/^[a-z0-9.-]+$/.test(name)||!fs.existsSync(path.join(root,name))){res.statusCode=404;return res.end('{}')}
 res.setHeader('Content-Type',name.endsWith('.js')?'text/javascript':name.endsWith('.css')?'text/css':'text/html');res.end(fs.readFileSync(path.join(root,name)));
});
(async()=>{
 await new Promise(r=>server.listen(0,'127.0.0.1',r));let browser;
 try{
  browser=await chromium.launch({executablePath:process.env.CHROMIUM_EXECUTABLE||undefined,headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--no-proxy-server']});
  const page=await browser.newPage({viewport:{width:1280,height:1000}}),errors=[];
  page.on('pageerror',e=>errors.push(e.message));const url='http://127.0.0.1:'+server.address().port;
  await page.goto(url);await page.waitForFunction(()=>document.getElementById('pnlink-state').textContent.includes('设备已应答'));
  assert.equal(posts.length,0,'loading and GET must not issue commands');assert(await page.locator('#pnlink-enter').isDisabled());
  await page.locator('#operator-acquire').click();await page.locator('#pnlink-calibration-check').check();
  await page.waitForFunction(()=>!document.getElementById('pnlink-confirm').disabled);await page.locator('#pnlink-confirm').click();
  await page.waitForFunction(()=>document.getElementById('pnlink-calibration-state').textContent.includes('操作者已确认'));
  await page.locator('#pnlink-hand-mode').selectOption('fingers');await page.locator('#pnlink-hand-closure').fill('25');
  await page.locator('#pnlink-hand-save').click();await page.waitForFunction(()=>document.getElementById('pnlink-hand-state').textContent.includes('逐手指'));
  rejectHandSample=true;await page.locator('#pnlink-hand-open').click();
  await page.waitForFunction(()=>document.getElementById('pnlink-hand-request').textContent.includes('采样测试错误'));
  assert.equal(state.pnlink_hands.sides.right.capture.stage,'idle');assert(await page.locator('#pnlink-hand-closed').isDisabled());
  rejectHandSample=false;
  await page.locator('#pnlink-hand-open').click();await page.waitForFunction(()=>document.getElementById('pnlink-hand-capture-state').textContent.includes('采样中'));
  assert(await page.locator('#pnlink-enter').isDisabled());await page.waitForFunction(()=>document.getElementById('pnlink-hand-capture-state').textContent.includes('采样成功'));
  await page.locator('#pnlink-hand-closed').click();await page.waitForFunction(()=>document.getElementById('pnlink-hand-capture-state').textContent.includes('采样中'));await page.waitForFunction(()=>document.getElementById('pnlink-hand-capture-state').textContent.includes('采样成功'));
  state.pnlink_hands.sides.right.error='PN Link 校准已改变，请重新采样张手和握拳';
  await page.waitForFunction(()=>!document.getElementById('pnlink-hand-state').textContent.includes('请重新采样'));
  await page.locator('#pnlink-hand-finger').selectOption('pinky');await page.locator('#pnlink-hand-capture-finger').click();await page.waitForFunction(()=>document.getElementById('pnlink-hand-capture-state').textContent.includes('采样中'));await page.waitForFunction(()=>document.getElementById('pnlink-hand-capture-state').textContent.includes('采样成功'));
  await page.locator('#pnlink-hand-mode').selectOption('grasp');await page.locator('#pnlink-hand-closure').fill('30');await page.locator('#pnlink-hand-save').click();await page.waitForFunction(()=>document.getElementById('pnlink-hand-state').textContent.includes('30%'));
  // Left requests and previews must stay on the left, and batch capture targets both.
  assert.equal(await page.locator('#pnlink-left-hand-value-2').textContent(),' 0.050');
  assert.equal(await page.locator('#pnlink-hand-value-2').textContent(),' 0.250');
  await page.locator('#pnlink-left-hand-open').click();
  await page.waitForFunction(()=>document.getElementById('pnlink-left-hand-capture-state').textContent.includes('采样中'));
  await page.waitForFunction(()=>document.getElementById('pnlink-left-hand-capture-state').textContent.includes('采样成功'));
  await page.locator('#pnlink-left-hand-closed').click();
  await page.waitForFunction(()=>document.getElementById('pnlink-left-hand-capture-state').textContent.includes('采样中'));
  await page.waitForFunction(()=>document.getElementById('pnlink-left-hand-capture-state').textContent.includes('采样成功'));
  await page.locator('#pnlink-left-hand-finger').selectOption('thumb');await page.locator('#pnlink-left-hand-capture-finger').click();
  await page.waitForFunction(()=>document.getElementById('pnlink-left-hand-capture-state').textContent.includes('采样中'));
  await page.waitForFunction(()=>document.getElementById('pnlink-left-hand-capture-state').textContent.includes('采样成功'));
  await page.locator('#pnlink-left-hand-check').check();await page.locator('#pnlink-left-hand-confirm').click();
  await page.waitForFunction(()=>!document.getElementById('pnlink-hands-open').disabled);
  await page.locator('#pnlink-hands-open').click();
  await page.waitForFunction(()=>['pnlink-left-hand-capture-state','pnlink-hand-capture-state'].every(id=>document.getElementById(id).textContent.includes('采样中')));
  await page.waitForFunction(()=>['pnlink-left-hand-capture-state','pnlink-hand-capture-state'].every(id=>document.getElementById(id).textContent.includes('采样成功')));
  await page.locator('#pnlink-hands-closed').click();
  await page.waitForFunction(()=>['pnlink-left-hand-capture-state','pnlink-hand-capture-state'].every(id=>document.getElementById(id).textContent.includes('采样中')));
  await page.waitForFunction(()=>['pnlink-left-hand-capture-state','pnlink-hand-capture-state'].every(id=>document.getElementById(id).textContent.includes('采样成功')));
  await page.locator('#pnlink-hand-selection').selectOption('both');await page.locator('#pnlink-hand-save').click();
  await page.waitForFunction(()=>document.getElementById('pnlink-hands-state').textContent.includes('双手'));
  assert(posts.some(x=>x.action==='configure_hands'&&x.hand==='both'));
  assert(posts.some(x=>x.action==='capture_hand_closed'&&x.hand==='left'));
  assert(posts.some(x=>x.action==='capture_hand_closed'&&x.hand==='both'));
  assert(posts.some(x=>x.action==='confirm_hand_calibration'&&x.hand==='left'));
  state.state='active';state.pids=[123];await page.locator('#pnlink-alignment').check();
  await page.waitForFunction(()=>!document.getElementById('pnlink-enter').disabled);await page.locator('#pnlink-enter').click();
  await page.waitForFunction(()=>document.getElementById('pnlink-hand-save').disabled&&!document.getElementById('pnlink-hand-off').disabled);
  assert(await page.locator('#pnlink-hand-selection').isDisabled());
  assert(await page.locator('#pnlink-left-hand-open').isDisabled());
  state.pnlink_hands.last_fault={reason:'手部反馈测试故障'};
  await page.locator('#pnlink-hand-off').click();await page.waitForFunction(()=>document.getElementById('pnlink-hand-off').disabled);assert(state.pnlink_session.active,'disabling hands cannot exit body');
  await page.waitForFunction(()=>document.getElementById('pnlink-hand-finish').textContent.includes('清零中'));
  await page.waitForFunction(()=>document.getElementById('pnlink-hand-finish').textContent.includes('自然张开'));
  await page.waitForFunction(()=>document.getElementById('pnlink-hand-fault').textContent.includes('手部反馈测试故障'));
  await page.waitForFunction(()=>!document.getElementById('pnlink-pause').disabled);await page.locator('#pnlink-pause').click();
  await page.waitForFunction(()=>!document.getElementById('pnlink-resume').disabled);await page.locator('#pnlink-resume').click();
  await page.waitForFunction(()=>!document.getElementById('pnlink-calibrate').disabled);await page.locator('#pnlink-calibrate').click();
  await page.waitForFunction(()=>document.getElementById('pnlink-command').textContent.includes('T-pose'));
  assert(await page.locator('#pnlink-enter').isDisabled());
  state.tracking_inputs.pnlink.command={stage:'unknown',error:'结果未知'};
  await page.waitForFunction(()=>document.getElementById('pnlink-command').textContent.includes('结果未知'));
  assert(await page.locator('#pnlink-calibrate').isDisabled());
  state.tracking_inputs.pnlink.command={};state.tracking_inputs.pnlink.receiver_id='b';state.pnlink_session.calibration_confirmed=false;
  await page.waitForFunction(()=>!document.getElementById('pnlink-calibration-check').checked);
  const before=posts.length;await page.reload();await page.waitForFunction(()=>document.getElementById('pnlink-state').textContent.includes('设备已应答'));
  assert(!posts.slice(before).some(x=>x.path!=='/api/operator'),'reload must not restore teleoperation');
  state.tracking_inputs.pnlink.service={active_state:'inactive'};state.tracking_inputs.pnlink.sdk_open=false;state.tracking_inputs.pnlink.connected=false;state.pnlink_session.ready=false;
  await page.waitForFunction(()=>!document.getElementById('pnlink-receiver-start').disabled);
  await page.locator('#pnlink-receiver-start').click();
  await page.waitForFunction(()=>document.getElementById('pnlink-receiver-start').disabled&&!document.getElementById('pnlink-receiver-stop').disabled);
  const receiverStopped=page.waitForResponse(r=>r.url().endsWith('/api/inputs/pnlink')&&r.request().method()==='POST'&&r.request().postDataJSON().action==='stop_receiver');
  await page.locator('#pnlink-receiver-stop').click();await receiverStopped;
  await page.waitForFunction(()=>document.getElementById('pnlink-receiver-stop').disabled&&!document.getElementById('pnlink-receiver-start').disabled);
  assert(posts.some(x=>x.path==='/api/inputs/pnlink'&&x.action==='start_receiver'));
  assert(posts.some(x=>x.path==='/api/inputs/pnlink'&&x.action==='stop_receiver'));
  const mobile=await browser.newPage({viewport:{width:390,height:844},isMobile:true});mobile.on('pageerror',e=>errors.push(e.message));
  await mobile.goto(url+'/remote');assert(await mobile.locator('#pnlink-calibrate').isVisible());
  assert(await mobile.locator('#pnlink-hand-controls').isVisible());
  assert(await mobile.locator('#pnlink-left-hand-panel').isVisible());
  assert(await mobile.locator('#pnlink-hand-panel').isVisible());
  await mobile.locator('#pnlink-hand-controls').screenshot({path:'/tmp/pnlink-hands-mobile.png'});
  await page.locator('#pnlink-hand-controls').screenshot({path:'/tmp/pnlink-hands-desktop.png'});
  await mobile.locator('#language').selectOption('en');await mobile.waitForFunction(()=>document.getElementById('pnlink-hand-save').textContent.includes('Save hand'));
  assert((await mobile.locator('#pnlink-hand-mode option[value="fingers"]').textContent()).includes('Individual'));
  assert(!await mobile.evaluate(()=>document.documentElement.scrollWidth>innerWidth),'mobile must not overflow');
  assert.deepEqual(errors,[]);console.log(JSON.stringify({desktop:true,mobile:true,no_auto_commands:true,operator_calibration_pause:true,hand_settings_sampling_and_off:true,bilateral_sampling_and_independent_preview:true,english:true,errors,robot_output_enabled:false}));
 }finally{if(browser)await browser.close();await new Promise(r=>server.close(r))}
})().catch(e=>{console.error(e);process.exit(1)});
