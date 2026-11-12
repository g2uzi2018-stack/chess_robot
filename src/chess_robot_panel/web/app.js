'use strict';
(() => {
  const $ = id => document.getElementById(id);
  const phases = {idle:'暂无指令',sending:'等待接受',accepted:'执行中',succeeded:'执行完成',failed:'驱动返回异常',rejected:'请求被拒绝',unknown:'结果未知'};
  let snapshot = null, receivedAt = -Infinity, posting = false, needsState = false;
  let configured = false, transportNote = '', observedRequest = '', generation = 0;
  let jog = null, suppressClickUntil = 0;
  let activeDevice = "arm";
  function requestId() {
    if (typeof crypto.randomUUID === 'function') return crypto.randomUUID();
    return Array.from(crypto.getRandomValues(new Uint8Array(16)), x => x.toString(16).padStart(2,'0')).join('');
  }
  function integer(id,min,max) {
    const text = $(id).value.trim(), n = Number(text);
    if (!/^\d+$/.test(text) || !Number.isSafeInteger(n) || n < min || n > max) throw new Error(`请输入 ${min}–${max} 范围内的整数`);
    return n;
  }
  function live() { return snapshot && performance.now() - receivedAt < 2000; }
  function canSend() { return activeDevice === "gripper" && live() && !posting && !needsState && snapshot.can_move; }
  function stopJog(message) {
    const old = jog;
    if (!old) return;
    jog = null; clearTimeout(old.timer); suppressClickUntil = performance.now() + 700;
    old.element.classList.remove('holding'); old.element.setAttribute('aria-pressed','false');
    if (old.pointerId !== undefined && old.element.hasPointerCapture(old.pointerId)) old.element.releasePointerCapture(old.pointerId);
    $('jog-status').textContent = message || '已松开，不再追加动作；已发出的这一小步可能仍会走完。';
  }
  function updateControls() {
    updateArmControls();
    if (jog && !live()) stopJog('连接中断，连续点动已结束；恢复后需重新按住。');
    for (const id of ['open','close']) $(id).disabled = jog && jog.element.id === id ? false : !canSend() || !!jog;
    $('send').disabled = !canSend() || !!jog;
    $('initialize').disabled = activeDevice !== 'gripper' || !live() || posting || needsState || !!jog || !snapshot.can_initialize;
    $('step').disabled = $('force').disabled = !!jog;
    if (!live()) {
      $('connection').textContent = '面板连接已断开'; $('connection-dot').className = 'dot';
      $('blocked').textContent = '连接中断，操作已禁用。连续点动不会自动恢复。'; $('blocked').className = 'notice error';
      $('position').textContent = '—'; $('position-bar').style.width = '0%';
      ['initialization','device-status','contact','age'].forEach(id => { $(id).textContent = '—'; });
    }
  }
  function render(data) {
    snapshot = data; receivedAt = performance.now(); needsState = false;
    const s = data.state, c = data.command;
    if (!configured) {
      $('force').value = data.config.default_force_percent; $('force').max = data.config.max_force_percent;
      $('step').value = Math.min(100,data.config.step_raw);
      $('target').value = $('slider').value = data.config.open_position_raw;
      $('force-help').textContent = `范围 20–${data.config.max_force_percent}%，不是实测力`; configured = true;
    }
    if (observedRequest !== c.request_id) { transportNote = ''; observedRequest = c.request_id; }
    if (jog) {
      if (jog.serverId !== data.server_id) stopJog('服务已重启，连续点动已结束。');
      else if (!s.fresh || !s.connected || !s.commands_enabled || s.initialization !== 1) stopJog('设备状态不可用，连续点动已结束；不会自动恢复。');
      else if (c.request_id !== jog.requestId) stopJog('检测到其他控制请求，连续点动已结束。');
      else if (['failed','rejected','unknown'].includes(c.phase) || s.object_dropped) stopJog('接触、异常或结果不确定，连续点动已结束；请查看最近指令。');
    }
    $('connection').textContent = !s.received ? '等待设备' : s.connected && s.fresh ? '设备已连接' : '设备反馈不可用';
    $('connection-dot').className = s.connected && s.fresh ? 'dot online' : 'dot';
    $('mode').textContent = !s.received ? '等待设备' : s.simulated ? '模拟设备' : '真实设备';
    $('mode').className = `badge ${!s.received ? 'neutral' : s.simulated ? '' : 'real'}`;
    $('blocked').textContent = `夹爪 · ${data.blocked_reason || '可以点动 · 点一下走一步，按住连续，松开不再追加'}`;
    $('blocked').className = data.can_move ? 'notice ready' : 'notice';
    $('position').textContent = s.fresh && s.position_raw <= 1000 ? s.position_raw : '—';
    $('position-bar').style.width = s.fresh && s.position_raw <= 1000 ? `${s.position_raw / 10}%` : '0%';
    $('initialization').textContent = !s.fresh ? '—' : ({0:'未初始化',1:'已初始化',2:'初始化中'})[s.initialization] || '未知';
    $('device-status').textContent = !s.fresh ? '—' : s.busy ? '执行中' : ({0:'运动中',1:'已到位',2:'检测到接触',3:'检测到掉落'})[s.grip_status] || '未知';
    $('contact').textContent = !s.fresh ? '—' : s.object_dropped ? '掉落信号，请检查' : s.object_detected ? '检测到接触，请目视确认' : '未检测到接触';
    $('age').textContent = !s.received || s.feedback_age_sec < 0 ? '—' : `${s.feedback_age_sec.toFixed(2)} s${s.fresh ? '' : ' · 过期'}`;
    $('device-detail').textContent = s.detail || ''; $('phase').textContent = phases[c.phase] || c.phase;
    $('phase').className = `badge ${['failed','rejected','unknown'].includes(c.phase) ? 'real' : c.phase === 'idle' ? 'neutral' : ''}`;
    $('result').textContent = transportNote || c.detail || '点击按钮后，这里显示驱动接受与执行结果。';
    $('request-id').textContent = c.request_id ? `ID ${c.request_id}${c.code === null ? '' : ` · 返回码 ${c.code}`}` : '等待操作';
    $('update-time').textContent = `状态更新 ${new Date().toLocaleTimeString('zh-CN',{hour12:false})}`;
    renderArm(data); updateControls(); maybeJog();
  }
  async function poll() {
    const requestGeneration = generation;
    const abort = new AbortController(), timer = setTimeout(() => abort.abort(),1500);
    try {
      const res = await fetch('/api/state',{cache:'no-store',signal:abort.signal});
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      const data = await res.json();
      if (requestGeneration === generation && !posting) render(data);
    } catch (_) { stopJog('状态读取失败，连续点动已结束；恢复后需重新按住。'); updateControls(); }
    finally { clearTimeout(timer); setTimeout(poll,jog ? 100 : 400); }
  }
  async function send(initialize,position,session = null) {
    if (activeDevice !== "gripper") return;
    if (session && jog !== session) return;
    if (!live() || posting || needsState || !(initialize ? snapshot.can_initialize : snapshot.can_move)) return;
    let body;
    try {
      body = {request_id:requestId(),server_id:snapshot.server_id};
      if (!initialize) {
        if (!Number.isInteger(position) || position < 0 || position > 1000) throw new Error('开度范围为 0–1000');
        body.position_raw = position; body.force_percent = session ? session.force : integer('force',20,snapshot.config.max_force_percent);
      }
    } catch (e) { stopJog(e.message); $('result').textContent = e.message; return; }
    if (session) { session.requestId = body.request_id; session.target = position; }
    posting = true; needsState = true; generation++; updateControls();
    const abort = new AbortController(), timer = setTimeout(() => abort.abort(),2500);
    try {
      const res = await fetch(`/api/gripper/${initialize ? 'initialize' : 'move'}`,{method:'POST',headers:{'Content-Type':'application/json','X-Panel-Request':'2'},body:JSON.stringify(body),signal:abort.signal});
      const data = await res.json();
      if (!res.ok) { transportNote = data.error || '请求被拒绝'; stopJog('请求未接受，连续点动已结束。'); }
      else transportNote = '请求已登记，等待驱动执行结果。';
    } catch (_) { transportNote = '请求结果未知，请检查设备及最近指令；不会自动重发。'; stopJog('发送结果不确定，连续点动已结束；请检查设备。'); }
    finally { clearTimeout(timer); posting = false; generation++; $('result').textContent = transportNote; updateControls(); }
  }
  function jogStep(session) {
    if (jog !== session || !canSend()) return;
    const current = snapshot.state.position_raw, target = Math.max(0,Math.min(1000,current + session.sign * session.step));
    if (!snapshot.state.fresh || current > 1000) { stopJog('等待新鲜开度反馈。'); return; }
    if (target === current) { stopJog(session.sign < 0 ? '已到闭合端点。' : '已到张开端点。'); updateControls(); return; }
    $('jog-status').textContent = session.repeating ? '连续点动中 · 松开不再追加' : `单步 ${session.step} raw · 继续按住可连续点动`;
    send(false,target,session);
  }
  function maybeJog() {
    const session = jog;
    if (session && session.repeating && canSend() && session.requestId && snapshot.command.request_id === session.requestId && snapshot.command.phase === 'succeeded' && snapshot.command.code === 1 && snapshot.state.grip_status === 1 && Math.abs(snapshot.state.position_raw - session.target) <= 5) jogStep(session);
  }
  function startJog(element,sign,pointerId,key) {
    if (jog || !canSend() || document.hidden) return;
    try {
      const session = {element,sign,pointerId,key,step:integer('step',1,100),force:integer('force',20,snapshot.config.max_force_percent),serverId:snapshot.server_id,requestId:'',repeating:false};
      jog = session; suppressClickUntil = performance.now() + 700;
      element.classList.add('holding'); element.setAttribute('aria-pressed','true');
      if (pointerId !== undefined) element.setPointerCapture(pointerId);
      session.timer = setTimeout(() => { if (jog === session) { session.repeating = true; $('jog-status').textContent = '连续点动中 · 松开不再追加'; maybeJog(); } },350);
      jogStep(session);
    } catch (e) { stopJog(e.message); $('result').textContent = e.message; }
  }
  for (const [id,sign] of [['close',-1],['open',1]]) {
    const button = $(id);
    button.addEventListener('pointerdown',e => { if (e.button === 0 && e.isPrimary) { e.preventDefault(); startJog(button,sign,e.pointerId); } });
    button.addEventListener('lostpointercapture',e => { if (jog && jog.element === button && jog.pointerId === e.pointerId) { stopJog(); updateControls(); } });
    button.addEventListener('contextmenu',e => e.preventDefault());
    button.addEventListener('keydown',e => { if (e.key === ' ' || e.key === 'Enter') { e.preventDefault(); if (!e.repeat) startJog(button,sign,undefined,e.key); } });
    button.addEventListener('keyup',e => { if (jog && jog.element === button && jog.key === e.key) { e.preventDefault(); stopJog(); updateControls(); } });
    button.addEventListener('click',e => { e.preventDefault(); if (e.detail === 0 && performance.now() > suppressClickUntil) { startJog(button,sign); stopJog(); updateControls(); } });
  }
  window.addEventListener('pointerup',e => { if (jog && jog.pointerId === e.pointerId) { stopJog(); updateControls(); } },true);
  window.addEventListener('pointercancel',e => { if (jog && jog.pointerId === e.pointerId) { stopJog('触控已取消，不再追加动作。'); updateControls(); } },true);
  window.addEventListener('pointermove',e => {
    if (!jog || jog.pointerId !== e.pointerId) return;
    const r = jog.element.getBoundingClientRect();
    if (e.clientX < r.left || e.clientX > r.right || e.clientY < r.top || e.clientY > r.bottom) { stopJog('已移出按钮，不再追加动作。'); updateControls(); }
  },true);
  window.addEventListener('blur',() => { stopJog('窗口失去焦点，不再追加动作。'); updateControls(); });
  window.addEventListener('pagehide',() => stopJog());
  document.addEventListener('visibilitychange',() => { if (document.hidden) { stopJog('页面已隐藏，不再追加动作。'); updateControls(); } });
  $('initialize').addEventListener('click',() => { stopJog(); send(true); });
  $('send').addEventListener('click',() => { stopJog(); try { send(false,integer('target',0,1000)); } catch (e) { $('result').textContent = e.message; } });
  $('target').addEventListener('input',() => { const n = Number($('target').value); if (Number.isInteger(n) && n >= 0 && n <= 1000) $('slider').value = n; });
  $('slider').addEventListener('input',() => { $('target').value = $('slider').value; });
  let armTransportNote = '', armObserved = '', armServer = '';
  const armInitialized = new Set();
  function armCanSend() {
    return activeDevice === "arm" && live() && !posting && !needsState && !jog && snapshot.arm && snapshot.arm.can_move;
  }
  function updateArmControls() {
    const enabled = armCanSend();
    for (let i=0;i<6;i++) for (const op of ['minus','plus','execute']) $(`joint-${op}-${i}`).disabled = !enabled;
    $('joint-step').disabled = posting || !!jog;
    if (!live()) {
      $('arm-blocked').textContent = '面板连接已断开，操作已禁用。恢复后需重新点击。';
      $('arm-blocked').className = 'notice error';
      for(let i=0;i<6;i++) $(`joint-measured-${i}`).textContent = '—';
    }
  }
  function renderArm(data) {
    const a=data.arm, c=data.arm_command;
    if (!a || !c) {
      $('arm-blocked').textContent = '当前面板后端尚未支持关节控制。';
      return;
    }
    if(armServer && armServer!==data.server_id) {
      armInitialized.clear(); armTransportNote='服务已重启，请核对实测角度后重新操作。';
    }
    armServer=data.server_id;
    if(armObserved!==c.request_id) {armObserved=c.request_id;armTransportNote='';}
    const valid=a.fresh && a.joints_valid && a.matched && Array.isArray(a.positions_deg) && a.positions_deg.length===6 && a.positions_deg.every(Number.isFinite);
    for(let i=0;i<6;i++) {
      $(`joint-measured-${i}`).textContent=valid ? `${a.positions_deg[i].toFixed(2)}°` : '—';
      if(valid && !armInitialized.has(i)) {
        if(!$(`joint-target-${i}`).value.trim()) $(`joint-target-${i}`).value=a.positions_deg[i].toFixed(2);
        armInitialized.add(i);
      }
    }
    $('arm-mode').textContent=!a.received?'等待机械臂':a.simulated?'模拟设备':'真实设备';
    $('arm-mode').className=`badge ${a.simulated?'':'real'}`;
    $('arm-blocked').textContent=a.blocked_reason || '可以微调 · 点击走一步，输入目标后手动执行';
    $('arm-blocked').className=a.can_move?'notice ready':'notice';
    $('arm-phase').textContent=phases[c.phase] || c.phase;
    $('arm-phase').className=`badge ${['failed','unknown','rejected'].includes(c.phase)?'real':c.phase==='idle'?'neutral':''}`;
    $('arm-result').textContent=armTransportNote || c.detail || '等待操作';
    $('arm-request-id').textContent=c.request_id?`ID ${c.request_id}${c.code===null?'':` · 返回码 ${c.code}`}`:'';
    updateArmControls();
  }
  function armNumber(input) {
    const raw=input.value.trim(),value=Number(raw);
    if(!raw || !Number.isFinite(value) || !/^-?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?$/.test(raw)) throw new Error('请输入有效的目标角度。');
    return value;
  }
  async function sendArm(index,sign) {
    if(!armCanSend() || document.hidden) return;
    let target;
    try {
      const measured=snapshot.arm.positions_deg[index];
      if(!Number.isFinite(measured) || !snapshot.arm.fresh) throw new Error('等待新鲜关节反馈。');
      if(sign) {
        const step=armNumber($('joint-step'));
        if(step<.1 || step>5) throw new Error('点击步长范围为 0.1–5°。');
        target=Number((measured+sign*step).toFixed(6));
      } else target=armNumber($(`joint-target-${index}`));
      const limit=snapshot.arm_config.max_delta_deg;
      if(Math.abs(target-measured)>limit+1e-8) throw new Error(`单次调整不能超过 ${limit}°。`);
      if(Math.abs(target-measured)<.00001) throw new Error('目标与当前角度相同，请先调整目标。');
    } catch(e) {$('arm-result').textContent=e.message;return;}
    const body={request_id:requestId(),server_id:snapshot.server_id,joint_index:index,target_deg:target};
    posting=true;needsState=true;generation++;updateControls();
    const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),2500);
    try {
      const res=await fetch('/api/arm/joint',{method:'POST',headers:{'Content-Type':'application/json','X-Panel-Request':'2'},body:JSON.stringify(body),signal:abort.signal});
      const data=await res.json();
      armTransportNote=res.ok?'请求已登记，等待机械臂实测到位。':data.error || '关节请求被拒绝。';
      if(res.ok) {$(`joint-target-${index}`).value=String(target);armObserved=body.request_id;}
    } catch(_) {armTransportNote='请求结果未知，请检查设备及最近指令；不会自动重发。';}
    finally {clearTimeout(timer);posting=false;generation++;$('arm-result').textContent=armTransportNote;updateControls();}
  }
  for(let i=0;i<6;i++) {
    $(`joint-minus-${i}`).addEventListener('click',()=>sendArm(i,-1));
    $(`joint-plus-${i}`).addEventListener('click',()=>sendArm(i,1));
    $(`joint-execute-${i}`).addEventListener('click',()=>sendArm(i,0));
    $(`joint-target-${i}`).addEventListener('input',()=>armInitialized.add(i));
    for(const op of ['minus','plus','execute']) $(`joint-${op}-${i}`).addEventListener('keydown',e=>{if(e.repeat && (e.key==='Enter' || e.key===' '))e.preventDefault();});
  }

  function selectDevice(device) {
    stopJog('设备已切换，不再追加夹爪动作。');
    activeDevice=device;
    for(const name of ['arm','gripper']) {
      const selected=name===device;
      $(`device-${name}`).classList.toggle('is-active',selected);
      $(`device-${name}`).setAttribute('aria-selected',String(selected));
      $(`${name}-device`).hidden=!selected;
    }
    updateControls();
  }
  for(const name of ['arm','gripper']) $(`device-${name}`).addEventListener('click',()=>selectDevice(name));
  $('device-arm').addEventListener('keydown',e=>{if(e.key==='ArrowRight'){e.preventDefault();selectDevice('gripper');$('device-gripper').focus();}});
  $('device-gripper').addEventListener('keydown',e=>{if(e.key==='ArrowLeft'){e.preventDefault();selectDevice('arm');$('device-arm').focus();}});

  setInterval(updateControls,100); poll();
})();
