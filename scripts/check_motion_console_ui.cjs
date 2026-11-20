// Offline UI acceptance against a mock HTTP service. No robot or motor I/O.
const { chromium }=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const http=require('http'),fs=require('fs'),path=require('path'),assert=require('assert');
const root=path.resolve(__dirname,'../onboardmimic/console/static');
const artifacts=fs.mkdtempSync(path.join(require('os').tmpdir(),'onboard-motion-ui-'));
const testHost=process.env.MOTION_UI_TEST_HOST||'127.0.0.1';
const lanHttp=!['127.0.0.1','localhost'].includes(testHost);
const requests=[];
let s={state:'active',mode:'onboard_real',robot:'G1 mock / NO MOTOR I/O',robot_id:'unitree_g1',backend:'sonic',backend_label:'SONIC',capabilities:['planner','pose','joint_playback','smpl_playback'],pids:[123],input_mode:'planner',input_source:'planner',stationary:true,planner_control_allowed:true,mode_switch_pending:false,telemetry:{fresh:true,valid:true,faults:[]},backend_feedback:{mode:'planner'},operator:{owner:null,generation:0,remaining_ms:0},revision:0,playback:{state:'loaded',position_seconds:0,clip:{name:'test-motion',duration_seconds:3}}};
s.pnlink_hands={enabled:false,mode:'off',hand:'right',closure:.25};
const catalog={state:'ready',entries:[{id:'cache/demo',name:'demo CSV',kind:'cache',motion:'demo',status:'ready',metadata:{fps:50,duration_seconds:3}},{id:'smpl/a.pt',name:'a.pt',kind:'smpl',status:'needs_conversion',source_sha256:'aaaa'}]};
catalog.entries.push(
 {id:'kimodo/wave_amass.npz',name:'wave_amass.npz',kind:'kimodo',status:'needs_conversion',source_sha256:'kkkk'},
 {id:'cache/legacy',name:'legacy CSV',kind:'cache',motion:'legacy',status:'needs_reimport',error:'旧版或未知 GMR 关节映射，必须重新转换',metadata:{source_format:'gmr_pkl',source_sha256:'bbbb',source_fps:30,options:{input_joint_order:'mujoco'}}},
 {id:'gmr/source.pkl',name:'source.pkl',kind:'gmr',status:'needs_reimport',source_sha256:'bbbb',variants:['cache/legacy']},
 {id:'cache/no-source',name:'source missing',kind:'cache',motion:'no-source',status:'needs_reimport',metadata:{source_format:'gmr_pkl',source_sha256:'cccc'}});
const server=http.createServer(async(req,res)=>{res.setHeader('Content-Type','application/json');const route=req.url;
 if(route==='/api/session')return res.end(JSON.stringify({token:'test'}));
 if(route==='/api/platform')return res.end(JSON.stringify({schema_version:1,robots:[{id:'unitree_g1',label:'G1'}],backends:[{id:'sonic',label:'SONIC'}],pairs:[{robot_id:'unitree_g1',backend_id:'sonic'}],selected:{robot_id:'unitree_g1',backend_id:'sonic'},switching:{supported:false,reason:'service_configuration_required'}}));
 if(req.method==='POST'){let body='';for await(const c of req)body+=c;const x=JSON.parse(body);requests.push({route,...x});
  if(route==='/api/operator'){if(['acquire','takeover'].includes(x.action)){s.operator={owner:x.actor,generation:++s.revision,remaining_ms:5000}}else if(x.action==='heartbeat')s.operator.remaining_ms=5000;else if(x.action==='release')s.operator.owner=null;}
  if(route==='/api/playback'&&x.action==='switch'){s.input_mode='joint_playback';s.input_source='file_joint';s.playback.state='playing';s.motion_task={id:'test',target:'demo',stage:'finished',entry_checks:x.entry_checks};}
  if(route==='/api/imports'&&x.action==='convert'){
   s.conversion={id:'conversion-1',stage:'finished',motion_id:x.motion_id,result:{motion:'reconverted'}};
   catalog.entries=catalog.entries.filter(e=>e.id!=='cache/legacy');
   catalog.entries.push({id:'cache/reconverted',name:'reconverted',motion:'reconverted',kind:'cache',status:'ready',metadata:{source_format:'gmr_pkl',source_sha256:'bbbb',source_fps:30,fps:50,duration_seconds:3,options:{input_joint_order:'mujoco'}}});
   Object.assign(catalog.entries.find(e=>e.id==='gmr/source.pkl'),{status:'ready',variants:['cache/reconverted']});
  }
  return res.end(JSON.stringify(s));
 }
 if(route==='/api/controller')return res.end(JSON.stringify(s));
 if(route==='/api/motions')return res.end(JSON.stringify(catalog));
 if(route.includes('/skeleton'))return res.end(JSON.stringify({available:false,joints:[]}));
 const file=route==='/'?'index.html':route==='/remote'?'mobile.html':route.slice(1);
 if(!/^[a-z0-9.-]+$/.test(file)||!fs.existsSync(path.join(root,file))){res.statusCode=404;return res.end('{}')}
 res.setHeader('Content-Type',file.endsWith('.js')?'text/javascript':file.endsWith('.css')?'text/css':'text/html');res.end(fs.readFileSync(path.join(root,file)));
});
(async()=>{await new Promise(r=>server.listen(0,testHost,r));let browser;try{
 browser=await chromium.launch({executablePath:process.env.CHROMIUM_EXECUTABLE||undefined,headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--no-proxy-server']});
 const errors=[];const context=await browser.newContext({viewport:{width:1400,height:1100},locale:'zh-CN'});const page=await context.newPage();page.on('pageerror',e=>errors.push(e.stack||e.message));
 const url='http://'+testHost+':'+server.address().port;await page.goto(url);await page.locator('#motion-select option').first().waitFor({state:'attached'});
 const cryptoContext=await page.evaluate(()=>({secure:isSecureContext,uuid:typeof crypto.randomUUID,random:typeof crypto.getRandomValues}));
 assert.equal(cryptoContext.secure,!lanHttp);if(lanHttp)assert.equal(cryptoContext.uuid,'undefined');assert.equal(cryptoContext.random,'function');
 await page.locator('#operator-acquire').click();await page.waitForFunction(()=>!document.getElementById('motion-play').disabled);
 assert.match(s.operator.owner,/^[0-9a-f]{32}$/);
 await page.locator('#motion-refresh').click();await page.waitForFunction(()=>!document.getElementById('motion-load').disabled);
 await page.locator('#motion-load').click();await page.waitForFunction(()=>!document.getElementById('motion-play').disabled);
 await page.locator('#motion-entry-checks').uncheck();await page.locator('#motion-play').click();await page.waitForFunction(()=>document.getElementById('motion-entry-checks').checked);assert.equal(s.motion_task.entry_checks,'off');
 await page.reload();await page.waitForFunction(()=>document.getElementById('operator-state').textContent.includes('本端'));assert.equal(s.playback.state,'playing');
 await page.screenshot({path:path.join(artifacts,'desktop.png'),fullPage:true});
 const mobileContext=await browser.newContext({viewport:{width:390,height:844},isMobile:true,locale:'zh-CN'});const mobile=await mobileContext.newPage();mobile.on('pageerror',e=>errors.push(e.stack||e.message));await mobile.goto(url+'/remote');await mobile.locator('#motion-select option').first().waitFor({state:'attached'});assert(await mobile.locator('#motion-play').isDisabled());assert(await mobile.locator('#motion-refresh').isVisible());
 const overflow=await mobile.evaluate(()=>document.documentElement.scrollWidth>innerWidth);assert(!overflow,'mobile horizontal overflow');
 await mobile.screenshot({path:path.join(artifacts,'mobile.png'),fullPage:true});
 await page.locator('#language').selectOption('en');await page.waitForTimeout(150);assert.equal(await page.locator('#motion-library h2').textContent(),'Local motion library');
 mobile.once('dialog',dialog=>dialog.accept());await mobile.locator('#operator-takeover').click();await mobile.waitForFunction(()=>!document.getElementById('motion-load').disabled);
 await mobile.locator('#motion-load').click();await mobile.waitForFunction(()=>!document.getElementById('motion-play').disabled);
 await mobile.locator('#motion-play').click();await mobile.waitForFunction(()=>!document.getElementById('motion-play').disabled);
 const ids=requests.filter(x=>x.route==='/api/playback'&&['load_async','switch'].includes(x.action)).map(x=>x.request_id);
 assert.equal(ids.length,4);assert(ids.every(x=>/^[0-9a-f]{32}$/.test(x)));assert.equal(new Set(ids).size,ids.length);
 // Load and conversion choices are disjoint and remain independent.
 const loadIDs=await mobile.locator('#motion-select option').evaluateAll(es=>es.map(e=>e.value));
 assert.deepEqual(loadIDs,['cache/demo']);
 const sourceIDs=await mobile.locator('#source-select option').evaluateAll(es=>es.map(e=>e.value));
 assert(sourceIDs.includes('gmr/source.pkl')&&sourceIDs.includes('smpl/a.pt')&&sourceIDs.includes('cache/no-source'));
 assert(!sourceIDs.includes('cache/demo'));assert(!sourceIDs.includes('cache/legacy'));
 await mobile.locator('#source-select').selectOption('gmr/source.pkl');
 assert.equal(await mobile.locator('#motion-select').inputValue(),'cache/demo');
 assert(await mobile.locator('#motion-convert').isDisabled());
 assert.equal(await mobile.locator('#motion-convert').textContent(),'重新转换');
 assert.equal(await mobile.locator('#motion-name').inputValue(),'gmr_source');
 await mobile.locator('#motion-name').fill('我的挥手');
 await mobile.locator('#source-select').selectOption('smpl/a.pt');assert.equal(await mobile.locator('#motion-name').inputValue(),'genmo_a');
 await mobile.locator('#source-select').selectOption('kimodo/wave_amass.npz');assert.equal(await mobile.locator('#motion-name').inputValue(),'kimodo_wave');
 await mobile.locator('#source-select').selectOption('gmr/source.pkl');assert.equal(await mobile.locator('#motion-name').inputValue(),'我的挥手');
 assert.equal(await mobile.locator('#motion-order').inputValue(),'mujoco');assert(await mobile.locator('#motion-order').isVisible());
 s.state='stopped';s.pids=[];s.playback.state='loaded';
 await mobile.waitForFunction(()=>!document.getElementById('motion-convert').disabled);
 await mobile.locator('#motion-name').fill('');assert(await mobile.locator('#motion-convert').isDisabled());
 await mobile.locator('#motion-name').fill('我的挥手');
 await mobile.locator('#motion-select').selectOption('cache/demo');
 assert.equal(await mobile.locator('#source-select').inputValue(),'gmr/source.pkl');
 const playbackBefore=requests.filter(x=>x.route==='/api/playback').length;
 await mobile.locator('#motion-convert').click();
 await mobile.waitForFunction(()=>document.getElementById('motion-select').value==='cache/reconverted');
 assert.equal(requests.filter(x=>x.route==='/api/playback').length,playbackBefore,'conversion must not load or play');
 const imported=requests.find(x=>x.route==='/api/imports'&&x.action==='convert');
 assert(imported);assert.equal(imported.motion_id,'gmr/source.pkl');assert.equal(imported.joint_order,'mujoco');assert.equal(imported.output_name,'我的挥手');
 assert.equal(await mobile.locator('#source-select').inputValue(),'gmr/source.pkl');
 assert.equal(await mobile.locator('#motion-convert').textContent(),'转换');
 s.state='active';s.pids=[123];await mobile.waitForFunction(()=>document.getElementById('motion-convert').disabled);
 s.state='stopped';s.pids=[];
 await mobile.locator('#source-select').selectOption('cache/no-source');
 assert(await mobile.locator('#motion-convert').isDisabled());assert(await mobile.locator('#motion-name-wrap').isHidden());
 assert((await mobile.locator('#conversion-note').textContent()).includes('缺少匹配的原始文件'));
 s.motion_task={id:'pico-finished',kind:'pico',stage:'finished',target:'planner'};
 await mobile.waitForTimeout(1200);assert(!(await mobile.locator('#motion-task').textContent()).includes('undefined'));
 await mobile.locator('#source-select').selectOption('smpl/a.pt');assert(await mobile.locator('#motion-fps').isVisible());
 assert(!(await mobile.locator('#motion-order').isVisible()));assert(await mobile.locator('#motion-group').isVisible());
 await mobile.locator('#source-select').selectOption('gmr/source.pkl');
 await page.waitForFunction(()=>document.getElementById('motion-select').value==='cache/reconverted');
 await page.locator('#source-select').selectOption('cache/no-source');assert.equal(await page.locator('#motion-convert').textContent(),'Reconvert');
 await page.locator('#source-select').selectOption('gmr/source.pkl');
 const desktopBoxes=await page.locator('.motion-card').evaluateAll(es=>es.map(e=>{let r=e.getBoundingClientRect();return {x:r.x,y:r.y,width:r.width}}));
 assert(desktopBoxes[1].x>desktopBoxes[0].x&&Math.abs(desktopBoxes[1].y-desktopBoxes[0].y)<2,'desktop cards side by side');
 const mobileBoxes=await mobile.locator('.motion-card').evaluateAll(es=>es.map(e=>{let r=e.getBoundingClientRect();return {x:r.x,y:r.y,width:r.width}}));
 assert(mobileBoxes[1].y>mobileBoxes[0].y&&Math.abs(mobileBoxes[1].x-mobileBoxes[0].x)<2,'mobile cards stacked');
 assert(!(await mobile.evaluate(()=>document.documentElement.scrollWidth>innerWidth)),'mobile overflow after conversion');
 await page.locator('#language').selectOption('zh-CN');await page.reload();await page.waitForFunction(()=>document.getElementById('motion-select').value==='cache/reconverted');await page.locator('#source-select').selectOption('gmr/source.pkl');
 await page.locator('#motion-library').screenshot({path:path.join(artifacts,'desktop-motion-library.png')});
 await mobile.locator('#motion-library').screenshot({path:path.join(artifacts,'mobile-motion-library.png')});
 assert(requests.some(x=>x.route==='/api/motions'&&x.action==='refresh'));
 assert.deepEqual(errors,[]);console.log(JSON.stringify({origin:url,cryptoContext,desktop:true,mobile:true,refresh_owner:true,second_client_readonly:true,load_and_switch_ids:true,entry_checks_once:true,manual_name_and_source_defaults:true,name_draft_preserved:true,split_independent_selects:true,conversion_does_not_load_or_play:true,responsive_cards:true,stale_cache_reconvert:true,missing_source_explained:true,running_conversion_blocked:true,english:true,js_errors:errors,artifacts}));
 }finally{if(browser)await browser.close();server.close()}})().catch(e=>{console.error(e);process.exitCode=1});
