(()=>{
 const $=id=>document.getElementById(id), t=x=>window.onboardMimicI18n.t(x);
 // getRandomValues is available on ordinary HTTP LAN origins too. randomUUID
 // is secure-context-only, so it cannot be used for IP-based robot consoles.
 const newId=()=>Array.from(crypto.getRandomValues(new Uint8Array(16)),byte=>byte.toString(16).padStart(2,'0')).join('');
 let actor;try{actor=sessionStorage.getItem('onboard-operator')}catch{}
 if(!actor){actor=newId();try{sessionStorage.setItem('onboard-operator',actor)}catch{}}
 let state={},catalog={entries:[]},token='',selected='',sourceSelected='',working=false,lastTask='',lastConversion='',lastCatalog='';
 const nameDrafts=new Map();
 const terminal=new Set(['finished','error','cancelled']);
 function owns(){return state.operator?.owner===actor&&state.operator.remaining_ms>0}
 function authority(){return {actor,operator_generation:state.operator?.generation,expected_revision:state.revision}}
 window.onboardMimicOperator={owns,authority,update};
 async function post(path,data){
  if(!token){const r=await fetch('/api/session',{cache:'no-store'});token=(await r.json()).token}
  // Use the latest runtime revision, while retaining this tab's operator identity.
  if(path!=='/api/operator'){
   const r=await fetch('/api/controller',{cache:'no-store'});if(!r.ok)throw Error('机载接口未就绪');update(await r.json());
  }
  const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json','X-Control-Token':token},body:JSON.stringify({...authority(),...data}),signal:AbortSignal.timeout(5000)});
  const x=await r.json();if(!r.ok){if(r.status===403)token='';throw Error(x.error||'操作未确认')}
  update(x);return x;
 }
 async function perform(fn){if(working)return;working=true;render();try{await fn();$('motion-message').textContent='请求已接受，请观察任务状态'}catch(e){$('motion-message').textContent=e.message}finally{working=false;render()}}
 function update(s){if(s.sampled_at!=null&&state.sampled_at!=null&&s.sampled_at<state.sampled_at)return;state=s;render()}
 const labels={ready:'可加载',needs_conversion:'需转换',needs_reimport:'需重新转换',needs_confirmation:'参数待确认',invalid:'格式无效',idle:'等待刷新',scanning:'扫描中',checking:'检查目标动作',waiting_planner:'等待 Planner 确认并站稳',starting:'等待播放接管',finished:'已完成',error:'失败',cancelled:'已取消',running:'转换中',unloaded:'未加载',loaded:'已加载',playing:'播放中',paused:'已暂停'};
 const label=x=>t(labels[x]||x||'—');
 const cacheEntry=()=>catalog.entries.find(e=>e.id===selected&&e.kind==='cache'&&e.status==='ready');
 const sourceEntry=()=>catalog.entries.find(e=>e.id===sourceSelected);
 const variants=row=>(row?.variants||[]).map(id=>catalog.entries.find(e=>e.id===id)).filter(Boolean);
 const staleVariants=row=>variants(row).filter(e=>e.status==='needs_reimport');
 function original(row){const kind={gmr_pkl:'gmr',smpl_pt:'smpl',kimodo_amass_npz:'kimodo'}[row?.metadata?.source_format];return kind&&row.metadata?.source_sha256?catalog.entries.find(e=>e.kind===kind&&e.source_sha256===row.metadata.source_sha256):null}
 function sourceRows(){return catalog.entries.filter(e=>e.kind!=='cache'||e.status==='needs_confirmation'||e.status==='needs_reimport'&&!original(e))}
 function sourceMeta(row){return row?.kind==='cache'?row.metadata||{}:(staleVariants(row)[0]||variants(row).find(e=>e.status==='ready'))?.metadata||{}}
 function sourceStatus(row){return staleVariants(row).length?'needs_reimport':row.status}
 function selectCache(){
  selected=$('motion-select').value;const row=cacheEntry(),meta=row?.metadata||{};
  $('motion-details').textContent=row?[meta.duration_seconds!=null?meta.duration_seconds.toFixed(2)+' s':'',meta.fps?meta.fps+' Hz':'',meta.source_format||'CSV',meta.source_name||''].filter(Boolean).join(' · '):t('暂无可加载动作，请先转换或登记');
  render();
 }
 function defaultName(row){
  const prefix={smpl:'genmo',kimodo:'kimodo',gmr:'gmr'}[row?.kind];
  if(!prefix)return '';
  let stem=row.name.replace(/\.[^.]+$/,'').replace(/(?:_kimodo)?_amass$/i,'').replace(/[^\p{L}\p{N}_. -]/gu,'_').slice(0,60);
  return stem.toLowerCase().startsWith(prefix+'_')?stem:prefix+'_'+stem;
 }
 function selectSource(){
  sourceSelected=$('source-select').value;const row=sourceEntry(),meta=sourceMeta(row),isCSV=row?.kind==='cache'&&row.status==='needs_confirmation';
  $('motion-name-wrap').hidden=!row||row.kind==='cache';
  $('motion-name').value=nameDrafts.get(sourceSelected)??defaultName(row);
  $('motion-options').hidden=!row||row.kind==='cache'&&!isCSV;
  $('motion-fps-wrap').hidden=!(isCSV||['smpl','kimodo'].includes(row?.kind));
  $('motion-group-wrap').hidden=row?.kind!=='smpl';$('motion-order-wrap').hidden=!(isCSV||row?.kind==='gmr');$('motion-root-wrap').hidden=!isCSV;
  $('motion-order').replaceChildren();
  for(const value of isCSV?['','sonic_reference']:['','mujoco','isaaclab']){const o=document.createElement('option');o.value=value;o.textContent=value||t('请选择关节顺序');$('motion-order').append(o)}
  let saved={};try{saved=JSON.parse(localStorage.getItem('motion-options:'+row?.source_sha256)||'{}')}catch{}
  $('motion-fps').value=saved.fps??(isCSV?50:meta.source_fps??'');$('motion-order').value=saved.joint_order||meta.options?.input_joint_order||'';
  $('motion-group').value=saved.group||meta.options?.selected_group||meta.options?.group||'auto';$('motion-root').value=saved.root_body_index??0;
  $('source-details').textContent=row?[({gmr:'GMR PKL',smpl:'GENMO PT',kimodo:'Kimodo AMASS',cache:'CSV'}[row.kind]||row.kind),(row.kind!=='cache'&&sourceStatus(row)==='ready'?t('已有结果，可再次转换'):label(sourceStatus(row))),row.error||''].filter(Boolean).join(' · '):t('暂无源文件，请先传文件到 G1');
  $('source-results').textContent=variants(row).length?t('再次转换将替换以下旧结果')+': '+variants(row).map(e=>e.name+' · '+label(e.status)).join('；'):'';
  render();
 }
 function render(){
  if(!$('motion-select'))return;
  const own=owns(),active=state.state==='active',stopped=['stopped','error'].includes(state.state)&&!state.pids?.length;
  const job=state.motion_task,task=job&&!terminal.has(job.stage),converting=state.conversion?.stage==='running',cache=cacheEntry(),source=sourceEntry();
  $('operator-state').textContent=own?t('本端持有操作权'):state.operator?.owner?t('另一端持有操作权'):t('尚无操作端');
  $('operator-acquire').disabled=working||own||Boolean(state.operator?.owner);$('operator-takeover').disabled=working||((own||!state.operator?.owner)&&!['pico','pnlink'].includes(state.input_source));$('operator-release').disabled=working||!own;
  $('motion-refresh').disabled=working||!own||catalog.state==='scanning';
  $('motion-load').disabled=working||!own||!cache||task||converting||!(active||stopped)||state.input_mode==='pose'||state.mode_switch_pending;
  $('motion-play').disabled=working||!own||!cache||task||converting||!active||state.mode_switch_pending;
  const possible=source&&(source.kind!=='cache'&&source.status!=='invalid'||source.status==='needs_confirmation');
  $('motion-name').disabled=working||converting;
  $('motion-convert').disabled=working||!own||!stopped||converting||task||!possible||source?.kind!=='cache'&&!$('motion-name').value.trim();
  $('motion-convert').textContent=source?.status==='needs_confirmation'?t('登记 CSV'):source&&sourceStatus(source)==='needs_reimport'?t('重新转换'):t('转换');
  $('conversion-note').textContent=!source?t('选择需要处理的原始文件'):source.kind==='cache'&&source.status==='needs_reimport'?t('缺少匹配的原始文件，请上传原文件后刷新动作库'):!own?t('请先取得操作权'):!stopped?t('转换前请停止控制器；已转换动作仍可加载播放'):converting?t('转换中'):source.status==='invalid'?t('文件格式无效，请检查源文件'):variants(source).length?t('新结果校验成功后删除此原文件的旧结果，只保留新动作'):source.status==='needs_confirmation'?t('确认参数后登记为可加载动作，不修改轨迹'):t('转换结果写入动作库，完成后在加载列表选择');
  $('motion-pause').disabled=working||!own||task||state.playback?.state!=='playing'||state.mode_switch_pending;
  $('motion-resume').disabled=working||!own||task||state.playback?.state!=='paused'||state.mode_switch_pending;
  $('motion-cancel').disabled=working||!own||!task;$('import-cancel').disabled=working||!own||!converting;
  $('motion-entry-checks').disabled=working||!own;
  const pb=state.playback||{},clip=pb.clip||{};$('motion-progress').max=clip.duration_seconds||1;$('motion-progress').value=pb.position_seconds||0;
  $('motion-position').textContent=(clip.name||'—')+' · '+label(pb.state)+' · '+(pb.position_seconds||0).toFixed(2)+' / '+(clip.duration_seconds||0).toFixed(2)+' s';
  const currentSource={planner:'Planner',pico:'PICO',pnlink:'PN Link',file_joint:'CSV / joint',file_smpl:'SMPL'}[state.input_source]||'—';
  $('motion-source').textContent=t('实际来源')+': '+currentSource+' · '+(state.mode_switch_pending?t('等待后端确认'):t('后端模式'))+': '+(state.backend_feedback?.mode||'—');
  $('motion-task').textContent=job?[t('动作任务'),job.target,label(job.stage),job.entry_checks!=null?t('首帧检查')+': '+job.entry_checks:'',job.error||''].filter(Boolean).join(' · '):'';
  const converted=state.conversion;$('import-task').textContent=converted?[t('转换任务'),converted.motion_id,label(converted.stage),converted.result?.motion||'',converted.result?.replaced_motions?.length?t('已替换旧结果')+': '+converted.result.replaced_motions.join('、'):'',converted.result?.warning||'',converted.error||''].filter(Boolean).join(' · '):'';
 }
 function fillSelect(select,rows,value,empty){
  select.replaceChildren();for(const row of rows){const o=document.createElement('option');o.value=row.id;o.textContent=row.name+' · '+(row.kind!=='cache'&&sourceStatus(row)==='ready'?t('已有结果，可再次转换'):label(row.kind==='cache'?row.status:sourceStatus(row)));select.append(o)}
  if(!rows.length){const o=document.createElement('option');o.value='';o.textContent=t(empty);select.append(o)}
  if(rows.some(e=>e.id===value))select.value=value;
 }
 async function readCatalog(){
  const r=await fetch('/api/motions',{cache:'no-store',signal:AbortSignal.timeout(3000)});if(!r.ok)throw Error('动作库未启用');const next=await r.json(),signature=JSON.stringify(next);catalog=next;
  if(signature!==lastCatalog){
   lastCatalog=signature;const caches=catalog.entries.filter(e=>e.kind==='cache'&&e.status==='ready');
   fillSelect($('motion-select'),caches,selected||'cache/'+state.playback?.clip?.name,'暂无可加载动作，请先转换或登记');selectCache();
   const sources=sourceRows().sort((a,b)=>Number(sourceStatus(b)!=='ready')-Number(sourceStatus(a)!=='ready'));
   fillSelect($('source-select'),sources,sourceSelected,'暂无源文件，请先传文件到 G1');selectSource();
  }
  const completed=state.conversion;
  if(completed?.stage==='finished'&&completed.id!==lastConversion&&completed.result?.motion){
   const target='cache/'+completed.result.motion;
   if(catalog.entries.some(e=>e.id===target&&e.status==='ready')){lastConversion=completed.id;$('motion-select').value=target;selectCache()}
  }
  $('catalog-state').textContent=label(catalog.state)+(catalog.error?' · '+catalog.error:'');render();
 }
 async function playback(action,extra={}){return post('/api/playback',{action,...extra})}
 $('operator-acquire').onclick=()=>perform(()=>post('/api/operator',{action:'acquire'}));
 $('operator-takeover').onclick=()=>{if(confirm(t('接管会取消尚未开始的动作任务，已播放动作继续。确认接管？')))perform(()=>post('/api/operator',{action:'takeover'}))};
 $('operator-release').onclick=()=>perform(()=>post('/api/operator',{action:'release'}));
 $('motion-name').oninput=()=>{nameDrafts.set(sourceSelected,$('motion-name').value);render()};
 $('motion-select').onchange=selectCache;$('source-select').onchange=selectSource;
 $('motion-refresh').onclick=()=>perform(async()=>{await post('/api/motions',{action:'refresh'});await readCatalog()});
 $('motion-load').onclick=()=>perform(()=>playback('load_async',{motion_id:cacheEntry().id,request_id:newId()}));
 $('motion-play').onclick=()=>perform(async()=>{await playback('switch',{motion_id:cacheEntry().id,entry_checks:$('motion-entry-checks').checked?'on':'off',request_id:newId()});$('motion-entry-checks').checked=true});
 for(const action of ['pause','resume','stop','cancel'])$('motion-'+action).onclick=()=>perform(()=>playback(action));
 $('import-cancel').onclick=()=>perform(()=>post('/api/imports',{action:'cancel'}));
 $('motion-convert').onclick=()=>perform(async()=>{
  const row=sourceEntry(),raw=$('motion-fps').value;
  const options={fps:raw===''?null:Number(raw),joint_order:$('motion-order').value,group:$('motion-group').value,root_body_index:Number($('motion-root').value)};
  if(row.kind!=='cache')options.output_name=$('motion-name').value.trim();
  await post('/api/imports',{action:row.kind==='cache'&&row.status==='needs_confirmation'?'register':'convert',motion_id:row.id,...options});
  try{localStorage.setItem('motion-options:'+row.source_sha256,JSON.stringify(options))}catch{}
 });
 setInterval(async()=>{if(!owns())return;try{await post('/api/operator',{action:'heartbeat'})}catch{}},1000);
 async function poll(){try{const r=await fetch('/api/controller',{cache:'no-store',signal:AbortSignal.timeout(2000)});if(r.ok)update(await r.json());else update({});await readCatalog()}catch(e){update({});$('motion-message').textContent=e.message}setTimeout(poll,1000)}poll();
})();
