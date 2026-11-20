/* PN Link controls use the same operator, revision and console authentication. */
(()=>{
 const $=id=>document.getElementById(id),t=x=>window.onboardMimicI18n.t(x);
 if(!$('pnlink-panel'))return;
 let state={},token='',working=false,lastReceiver=null,lastFrame=null;
 const terminal=new Set(['finished','error','cancelled']);
 const calibrationCard=$('pnlink-calibration-card');
 let calibration=null,observedInput={};
 async function refresh(){const r=await fetch('/api/controller',{cache:'no-store',signal:AbortSignal.timeout(2000)});if(!r.ok)throw Error(t('机载接口未就绪'));state=await r.json();window.onboardMimicOperator?.update(state);observeCalibration(true);render()}
 async function post(path,data){
  if(working)return;
  const isHand=data.action?.startsWith('capture_hand_')||['configure_hands','confirm_hand_calibration'].includes(data.action);
  const notice=$(isHand?(data.hand==='left'?'pnlink-left-hand-request':data.hand==='both'?'pnlink-hands-request':'pnlink-hand-request'):'pnlink-message');
  notice.textContent=t('请求发送中');working=true;render();
  let sent=false,response;
  try{
   await refresh();const op=window.onboardMimicOperator;
   if(!op?.owns())throw Error(t('请先取得操作权'));
   if(!token){const r=await fetch('/api/session',{cache:'no-store'});token=(await r.json()).token}
   sent=true;
   response=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json','X-Control-Token':token},body:JSON.stringify({...op.authority(),...data}),signal:AbortSignal.timeout(5000)});
   const x=await response.json();if(!response.ok){if(response.status===403)token='';throw Error(x.error||t('操作未确认'))}
   state=x;op.update(x);observeCalibration(false);notice.textContent=t('请求已接受，请观察任务状态');
  }catch(e){
   notice.textContent=e.message;
   if(data.action==='calibrate'&&calibration?.phase==='sending'){
    calibration.phase=!sent||response&&!response.ok?'failed':'unknown';
    calibration.error=e.message;
   }
  }finally{working=false;render()}
 }
 let handDirty=false;
 const handNames=['拇指屈伸','拇指辅助','食指','中指','无名指','小指'];
 const handPanels=[{side:'left',prefix:'pnlink-left-hand',label:'左手'},{side:'right',prefix:'pnlink-hand',label:'右手'}];
 for(const {side,prefix} of handPanels){
  for(let i=0;i<6;i++){
   const row=document.createElement('label');row.className='hand-meter';
   const name=document.createElement('span');name.textContent=t(handNames[i])+' ';
   const bar=document.createElement('progress');bar.id=prefix+'-bar-'+i;bar.max=1;bar.value=0;
   const value=document.createElement('span');value.id=prefix+'-value-'+i;value.textContent=' —';
   row.append(name,bar,value);$(prefix+'-bars').append(row);
  }
  $(prefix+'-open').onclick=()=>post('/api/inputs/pnlink',{action:'capture_hand_open',hand:side});
  $(prefix+'-closed').onclick=()=>post('/api/inputs/pnlink',{action:'capture_hand_closed',hand:side});
  $(prefix+'-capture-finger').onclick=()=>post('/api/inputs/pnlink',{action:'capture_hand_finger',hand:side,finger:$(prefix+'-finger').value});
  $(prefix+'-confirm').onclick=()=>post('/api/inputs/pnlink',{action:'confirm_hand_calibration',hand:side,calibration_confirmed:$(prefix+'-check').checked});
  $(prefix+'-check').onchange=render;
 }
 for(const kind of ['open','closed'])$('pnlink-hands-'+kind).onclick=()=>post('/api/inputs/pnlink',{action:'capture_hand_'+kind,hand:'both'});
 for(const id of ['pnlink-hand-mode','pnlink-hand-closure','pnlink-hand-selection'])$(id).oninput=()=>{handDirty=true};
 $('pnlink-hand-save').onclick=async()=>{
  await post('/api/inputs/pnlink',{action:'configure_hands',hand:$('pnlink-hand-selection').value,hand_mode:$('pnlink-hand-mode').value,hand_closure:Number($('pnlink-hand-closure').value)/100});
  handDirty=false;render();
 };
 $('pnlink-hand-off').onclick=async()=>{await post('/api/inputs/pnlink',{action:'configure_hands',hand_mode:'off',hand_closure:state.pnlink_hands?.closure??.25});handDirty=false;render()};
 const device=action=>post('/api/inputs/pnlink',{action});
 $('pnlink-receiver-start').onclick=()=>device('start_receiver');
 $('pnlink-receiver-stop').onclick=()=>device('stop_receiver');
 $('pnlink-capture').onclick=()=>device('start_capture');
 $('pnlink-stop-capture').onclick=()=>device('stop_capture');
 $('pnlink-calibrate').onclick=()=>{
  if(working)return;
  if(calibrationCard){
   calibration={phase:'sending',receiver:observedInput.receiver_id,
    previousRequests:new Set([state.pnlink_command?.request_id,observedInput.command?.request_id]),
    previousTask:state.motion_task?.id};
   clearPreview();
  }
  device('calibrate');
 };
 $('pnlink-confirm').onclick=()=>post('/api/inputs/pnlink',{action:'confirm_calibration',calibration_confirmed:$('pnlink-calibration-check').checked});
 $('pnlink-enter').onclick=()=>post('/api/controller',{action:'select_pnlink',alignment_confirmed:$('pnlink-alignment').checked});
 $('pnlink-pause').onclick=()=>post('/api/controller',{action:'pause_pnlink'});
 $('pnlink-resume').onclick=()=>post('/api/controller',{action:'resume_pnlink'});
 $('pnlink-exit').onclick=()=>post('/api/control',{stop:true,client:'pnlink-web-stop',seq:Date.now()});
 for(const id of ['pnlink-calibration-check','pnlink-alignment'])$(id).onchange=render;
 function render(){
  const p=state.pnlink_session||{},input=state.tracking_inputs?.pnlink||{},own=window.onboardMimicOperator?.owns();
  if(input.receiver_id!==lastReceiver){lastReceiver=input.receiver_id;$('pnlink-calibration-check').checked=false;$('pnlink-alignment').checked=false;for(const {prefix} of handPanels)$(prefix+'-check').checked=false}
  const task=state.motion_task&&!terminal.has(state.motion_task.stage),cmd=input.command||{},busy=Boolean(state.pnlink_command_pending)||cmd.stage==='running'||cmd.stage==='unknown'||state.pnlink_command?.stage==='sending';
  const blocked=working||!own||!state.pnlink_session||task||state.mode_switch_pending;
  const service=input.service?.active_state||'unknown';
  $('pnlink-receiver-start').disabled=blocked||busy||service==='active'||service==='activating';
  $('pnlink-receiver-stop').disabled=blocked||busy||service==='inactive'||service==='failed';
  for(const id of ['pnlink-capture','pnlink-stop-capture','pnlink-calibrate'])$(id).disabled=blocked||busy||!(input.sdk_open??input.connected);
  $('pnlink-calibrate').disabled||=!input.connected||input.capture_state!=='capturing';
  $('pnlink-confirm').disabled=blocked||busy||!p.ready||!$('pnlink-calibration-check').checked;
  $('pnlink-enter').disabled=blocked||busy||state.state!=='active'||!p.ready||!p.calibration_confirmed||!$('pnlink-alignment').checked||p.active;
  $('pnlink-pause').disabled=blocked||!p.active||p.paused;
  $('pnlink-resume').disabled=blocked||!p.active||!p.paused;
  $('pnlink-exit').disabled=working||!p.active&&!task;
  const capture={capturing:'采集中',starting:'等待人体帧',stopped:'采集已停止',unknown:'采集状态未知'};
  $('pnlink-state').textContent=[input.connected?t('设备已应答'):t('等待设备应答'),t(capture[input.capture_state]||'—'),input.body_age_ms!=null?t('接收年龄')+': '+input.body_age_ms+' ms':'',p.active?t(p.paused?'已暂停':'跟随中'):'',input.reason||p.error||''].filter(Boolean).join(' · ');
  $('pnlink-calibration-state').textContent=p.calibration_confirmed?t('复用校准：操作者已确认'):t('设备校准状态：SDK 未提供查询');
  if(input.calibration?.evidence==='sdk_command_succeeded')$('pnlink-calibration-state').textContent+=' · '+t('本次SDK校准命令成功');
  const actions={start_capture:'开始采集',stop_capture:'停止采集',calibrate:'手动校准'},stages={running:'进行中',succeeded:'成功',failed:'失败',unknown:'结果未知'};
  const progress=cmd.progress!=null?(cmd.step===1?t('倒计时')+': '+cmd.progress:t('进度')+': '+cmd.progress+'%'):'';
  $('pnlink-command').textContent=[t(actions[cmd.action]||''),t(stages[cmd.stage]||cmd.stage||''),cmd.pose,progress,cmd.error,state.pnlink_command?.error].filter(Boolean).join(' · ');
  const activeActions=$('pnlink-active-actions');
  if(activeActions){
   const trackingTask=task&&['pnlink','pnlink_device'].includes(state.motion_task.kind);
   activeActions.classList.toggle('is-active',Boolean(p.active||trackingTask));
   $('pnlink-active-status').textContent=p.active?t(p.paused?'已暂停':'跟随中'):trackingTask?t('切换中'):'';
  }
  renderCalibration();
  renderHands(p,blocked,busy);
  if(!p.ready)clearPreview();
 }
 function renderHands(p,blocked,busy){
  const h=state.pnlink_hands||{},available=h.enabled===true,finish=h.finish||{},finishing=['sending','opening'].includes(finish.stage);
  const sampling=h.sampling===true||handPanels.some(({side})=>(h.sides?.[side]||(side==='right'?h:{})).capture?.stage==='running');
  const frozen=blocked||busy||!available||sampling||finishing;
  if(!handDirty){$('pnlink-hand-mode').value=h.mode||'off';$('pnlink-hand-closure').value=Math.round((h.closure??.25)*100);$('pnlink-hand-selection').value=h.hand||'right'}
  for(const id of ['pnlink-hand-mode','pnlink-hand-closure','pnlink-hand-selection','pnlink-hand-save'])$(id).disabled=frozen||p.active;
  $('pnlink-hand-off').disabled=working||!window.onboardMimicOperator?.owns()||!available||h.mode==='off';
  const modes={off:'关闭',fingers:'逐手指',grasp:'整体张握'},handLabels={left:'左手',right:'右手',both:'双手'};
  $('pnlink-hands-state').textContent=[t(handLabels[h.hand||'right']),t(modes[h.mode]||'关闭'),Math.round((h.closure??.25)*100)+'%',h.error].filter(Boolean).join(' · ');
  const bothReady=handPanels.every(({side})=>h.sides?.[side]?.input_ready);
  $('pnlink-hands-open').disabled=frozen||p.active||!bothReady;
  $('pnlink-hands-closed').disabled=frozen||p.active||!bothReady||!handPanels.every(({side})=>h.sides?.[side]?.calibration_present);
  for(const {side,prefix,label} of handPanels){
   const local=h.sides?.[side]||(side==='right'?h:{}),capture=local.capture||{},capturing=capture.stage==='running',fb=h.worker?.sides?.[side]||{};
   for(const suffix of ['open','closed','capture-finger','finger'])$(prefix+'-'+suffix).disabled=frozen||p.active||!local.input_ready;
   $(prefix+'-closed').disabled||=!local.calibration_present;
   $(prefix+'-capture-finger').disabled||=!local.calibration_present;
   $(prefix+'-confirm').disabled=frozen||p.active||!local.input_ready||!local.calibration_present||!$(prefix+'-check').checked;
   const oldWarning='PN Link 校准已改变，请重新采样张手和握拳',error=local.calibration_confirmed&&local.error===oldWarning?'':local.error;
   $(prefix+'-state').textContent=!available?t('端侧手部功能未配置'):[t(label),t(modes[h.mode]||'关闭'),Math.round((h.closure??.25)*100)+'%',local.input_ready?t('手指数据就绪'):t('等待'+label+'指数据'),local.input_age_ms!=null?local.input_age_ms+' ms':'',fb.ready?t(label+'反馈就绪'):t('等待'+label+'反馈'),local.calibration_confirmed?t('手部标定已确认'):t('手部标定待确认'),fb.commanding&&p.active?t(label+'命令发送中'):'',error||h.worker?.error||fb.error].filter(Boolean).join(' · ');
   const stages={idle:'未采样',running:'采样中',succeeded:'采样成功',failed:'采样失败'},kinds={open:'张手',closed:'握拳',finger:'单指'};
   $(prefix+'-capture-state').textContent=[t(kinds[capture.kind]||''),t(stages[capture.stage]||''),capturing?(capture.seconds??0)+' / 2 s · '+(capture.frames??0)+' '+t('帧'):'',capture.error].filter(Boolean).join(' · ');
   const mode=h.mode==='off'?'fingers':h.mode,preview=local.preview?.[mode]||{};
   $(prefix+'-preview').textContent=[t('映射目标预览（不驱动手）'),t(modes[mode]||''),preview.error,h.mode==='off'?t('关闭时预览逐手指目标'):''].filter(Boolean).join(' · ');
   for(let i=0;i<6;i++){
    const q=preview.target?.[i];$(prefix+'-bar-'+i).value=q??0;$(prefix+'-value-'+i).textContent=q==null?' —':' '+q.toFixed(3);
   }
  }
  const lastFault=h.last_fault?.reason,oldWarning='PN Link 校准已改变，请重新采样张手和握拳';
  $('pnlink-hand-fault').textContent=lastFault&&!(h.calibration_confirmed&&lastFault===oldWarning)?t('上次手控停用原因')+': '+lastFault:'';
  const finishStages={sending:'请求结束清零',opening:'手部清零中',succeeded:'手部已自然张开',failed:'手部清零失败'};
  $('pnlink-hand-finish').textContent=[t(finishStages[finish.stage]||''),...(finish.sides||[]).map(s=>t(handLabels[s])),finish.error].filter(Boolean).join(' · ');
  if(finishing)$('pnlink-enter').disabled=true;
  if(sampling)for(const id of ['pnlink-receiver-start','pnlink-receiver-stop','pnlink-capture','pnlink-stop-capture','pnlink-calibrate','pnlink-confirm','pnlink-enter'])$(id).disabled=true;
 }
 const canvas=$('pnlink-preview'),ctx=canvas.getContext('2d'),parents=[-1,0,0,0,1,2,3,4,5,6,7,8,9,9,9,12,13,14,16,17,18,19,20,21];
 let previewAt=0;
 const previewStatus=$('pnlink-preview-status');
 function setText(element,value){if(element&&element.textContent!==value)element.textContent=value}
 function observeCalibration(fromPoll){
  if(!calibrationCard)return;
  if(fromPoll)observedInput=state.tracking_inputs?.pnlink||{};
  const {receiver_id:receiver,command:sdk,receiver_running:live,frame_id:frame}=observedInput;
  const gateway=state.pnlink_command;
  const handoff=state.motion_task?.kind==='pnlink_device'&&state.motion_task.target==='calibrate'?state.motion_task:null;
  const gatewayRequest=gateway?.action==='calibrate'&&gateway.request_id?gateway:null;
  const sdkCommand=fromPoll&&live!==false&&receiver&&sdk?.action==='calibrate'&&sdk.request_id?sdk:null;
  const activeGateway=gatewayRequest&&(gatewayRequest.stage==='sending'||
   state.pnlink_command_pending&&['accepted','unknown'].includes(gatewayRequest.stage))&&
   !(sdkCommand?.request_id===gatewayRequest.request_id&&['succeeded','failed'].includes(sdkCommand.stage));
  const activeSdk=sdkCommand&&['running','unknown'].includes(sdkCommand.stage);
  const activeTask=handoff?.stage==='waiting_planner';
  const newRequest=activeGateway?gatewayRequest.request_id:activeSdk?sdkCommand.request_id:null;
  const newReceiver=activeGateway?gatewayRequest.receiver_id:receiver;
  const ended=calibration&&['succeeded','failed','unknown'].includes(calibration.phase);
  const newAttempt=ended&&(newRequest&&newRequest!==calibration.request&&
   !calibration.previousRequests?.has(newRequest)||activeTask&&handoff.id!==calibration.task&&handoff.id!==calibration.previousTask);
  if((!calibration||newAttempt)&&(newRequest||activeTask)){
   calibration={phase:'waiting',receiver:newReceiver,request:newRequest,task:activeTask?handoff.id:null};
   clearPreview();
  }
  const attempt=calibration;
  if(!attempt)return;
  if(attempt.phase==='succeeded'){
   if(fromPoll&&receiver&&receiver!==attempt.receiver&&!attempt.previewReady&&!attempt.previewInterrupted){
    attempt.previewInterrupted=true;
    clearPreview();
   }
   return;
  }
  if(attempt.receiver&&receiver&&attempt.receiver!==receiver&&fromPoll){
   attempt.phase='unknown';attempt.error='接收会话已变化，无法确认校准结果';return;
  }
  attempt.offline=fromPoll?live===false:attempt.offline;
  if(handoff?.id&&handoff.id!==attempt.previousTask&&!attempt.task&&!attempt.request)attempt.task=handoff.id;
  if(!attempt.request&&gatewayRequest&&gatewayRequest.receiver_id&&
    (!attempt.receiver||attempt.receiver===gatewayRequest.receiver_id)&&!attempt.previousRequests?.has(gatewayRequest.request_id)){
   attempt.request=gatewayRequest.request_id;
   attempt.receiver=gatewayRequest.receiver_id;
  }
  // The GET includes the receiver snapshot. A Planner task finishing only means the SDK command was dispatched.
  if(attempt.request&&sdkCommand?.request_id===attempt.request&&receiver===attempt.receiver){
   attempt.sdk=sdkCommand;attempt.phase=sdkCommand.stage;attempt.error=sdkCommand.error;
   if(attempt.phase==='succeeded'){
    attempt.watermark=frame;
    $('pnlink-panel').open=true;
    clearPreview();
    requestAnimationFrame(()=>{
     if(!document.hidden&&calibration===attempt)canvas.closest('.pnlink-preview-panel')?.scrollIntoView({block:'nearest'});
    });
   }
   return;
  }
  if(attempt.request&&gatewayRequest?.request_id===attempt.request){
   if(!attempt.sdk){
    const gatewayPhases={sending:'sending',error:'failed',unknown:'unknown',accepted:'waiting'};
    attempt.phase=gatewayPhases[gatewayRequest.stage]||'waiting';
    attempt.error=gatewayRequest.error;
   }
  }else if(attempt.task&&handoff?.id===attempt.task){
   attempt.phase=handoff.stage==='waiting_planner'?'handoff':handoff.stage==='finished'?'waiting':'failed';
   attempt.error=handoff.error;
  }
 }
 function renderCalibration(){
  if(!calibrationCard||!calibration)return;
  const {phase,sdk,error,dismissed,offline}=calibration;
  const problem=phase==='failed'||phase==='unknown'||offline;
  const labels={sending:'正在发送校准请求',handoff:'等待零速 Planner 确认并站稳',waiting:'等待设备开始校准',
   failed:'校准失败',unknown:'校准结果未知',succeeded:'校准完成'};
  const detail=offline&&!['failed','succeeded'].includes(phase)?'暂时无法读取校准状态':labels[phase]||
   (sdk?.step===1?'请就位并准备保持姿势':sdk?.step===2?(sdk.progress>=100?'等待设备返回结果':'保持当前姿势'):'请按提示准备当前姿势');
  const message=[t(detail),problem&&error?t(error):''].filter(Boolean).join(' · ');
  setText($('pnlink-calibration-result'),calibration.previewInterrupted?t('接收会话已变化，无法核验校准后的骨架'):
   problem||phase==='succeeded'?message:'');
  calibrationCard.hidden=phase==='succeeded'||Boolean(problem&&dismissed===message);
  calibrationCard.classList.toggle('is-error',Boolean(problem));
  const instructing=phase==='running'&&!offline;
  const countdown=instructing&&sdk?.step===1&&Number.isFinite(sdk.progress);
  const percent=instructing&&sdk?.step===2&&Number.isFinite(sdk.progress);
  setText($('pnlink-calibration-phase'),message);
  $('pnlink-calibration-pose').hidden=!instructing||!sdk?.pose;
  setText($('pnlink-calibration-pose'),instructing?sdk?.pose||'':'');
  $('pnlink-calibration-countdown').hidden=!countdown;
  setText($('pnlink-calibration-countdown'),countdown?sdk.progress+' s':'');
  $('pnlink-calibration-meter').hidden=!percent;
  if(percent){
   $('pnlink-calibration-progress').value=sdk.progress;
   setText($('pnlink-calibration-percent'),sdk.progress+'%');
  }
  $('pnlink-calibration-dismiss').hidden=!problem;
 }
 if(calibrationCard)$('pnlink-calibration-dismiss').onclick=()=>{
  calibration.dismissed=$('pnlink-calibration-phase').textContent;
  renderCalibration();
  $('pnlink-panel').querySelector('summary')?.focus({preventScroll:true});
 };
 function awaitingCalibratedFrame(){
  return calibration?.phase==='succeeded'&&!calibration.previewInterrupted&&!calibration.previewReady;
 }
 function clearPreview(){
  if(lastFrame!==null)ctx.clearRect(0,0,canvas.width,canvas.height);
  lastFrame=null;
  if(previewStatus){
   previewStatus.hidden=false;
   setText(previewStatus,t(awaitingCalibratedFrame()?'等待校准后的骨架':'等待 PN Link 身体数据'));
  }
 }
 async function preview(){
  if(document.hidden||!$('pnlink-panel').open)return;
  const receiver=observedInput.receiver_id;
  try{const r=await fetch('/api/input/skeleton?source=pnlink',{cache:'no-store',signal:AbortSignal.timeout(700)});if(!r.ok)throw Error();let x=await r.json();
   if(!x.available){clearPreview();return}
   if(awaitingCalibratedFrame()&&receiver!==calibration.receiver){clearPreview();return}
   if(calibration&&receiver===calibration.receiver){
    if(calibration.phase!=='failed'&&(calibration.phase!=='succeeded'||observedInput.receiver_id!==receiver||
       !Number.isInteger(calibration.watermark)||!Number.isInteger(x.frame_id)||x.frame_id<=calibration.watermark)){clearPreview();return}
    if(awaitingCalibratedFrame()){
     // The skeleton endpoint has no receiver ID. Confirm the candidate in the next controller snapshot.
     const candidate=calibration.previewCandidate;
     calibration.previewCandidate=x;
     if(!candidate||!Number.isInteger(observedInput.frame_id)||observedInput.frame_id<candidate.frame_id){clearPreview();return}
     x=candidate;
    }
   }
   if(x.frame_id===lastFrame)return;lastFrame=x.frame_id;previewAt=performance.now();
   const joints=x.joints,w=canvas.width,h=canvas.height,origin=joints[0],pelvisY=h*.6;
   const centered=joints.map(p=>[p[1]-origin[1],p[2]-origin[2]]);
   const scale=centered.reduce((limit,[side,up])=>{
    const verticalRoom=up<0?h-pelvisY:pelvisY;
    return Math.min(limit,(w/2-14)/Math.abs(side),(verticalRoom-14)/Math.abs(up));
   },Math.min(w/2.2,h/2.2));
   ctx.clearRect(0,0,w,h);ctx.lineCap='round';ctx.lineWidth=4;
   const points=centered.map(([side,up])=>[w/2-side*scale,pelvisY-up*scale]);
   for(let i=1;i<24;i++){const a=points[parents[i]],b=points[i];ctx.strokeStyle=i%2?'#68d9ce':'#79a9f7';ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke()}
   if(calibration?.phase==='succeeded'&&receiver===calibration.receiver)calibration.previewReady=true;
   if(previewStatus)previewStatus.hidden=true;
  }catch{clearPreview()}
 }
 async function poll(){try{await refresh();await preview()}catch(e){state={};if(calibration&&calibration.phase!=='succeeded')calibration.offline=true;render();clearPreview();$('pnlink-message').textContent=e.message}setTimeout(poll,300)}
 setInterval(()=>{if(performance.now()-previewAt>600)clearPreview()},100);
 document.addEventListener('visibilitychange',()=>{if(document.hidden)clearPreview()});poll();
})();
