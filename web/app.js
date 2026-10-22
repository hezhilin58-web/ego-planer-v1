'use strict';
let draft, lastSaved, state = {}, token = '', busy = false, initialized = false, toastTimer;
const $ = id => document.getElementById(id);
const activeStates = ['TRAVEL', 'ALIGN', 'HOVER', 'RECOVER', 'RETURN_HOME', 'DESCEND'];
const labels = {WAIT_READY:'等待开始', TRAVEL:'飞往目标', ALIGN:'转向并确认稳定', HOVER:'转向后悬停', RECOVER:'恢复稳定', RETURN_HOME:'自动返航', DESCEND:'缓慢下降回收', COMPLETE:'返航回收完成', FAILED:'任务已停止'};
const same = (a, b) => JSON.stringify(a) === JSON.stringify(b);

function notify(text, error = false) {
  $('toast').textContent = text; $('toast').classList.toggle('error', error); $('toast').hidden = false;
  clearTimeout(toastTimer); toastTimer = setTimeout(() => $('toast').hidden = true, error ? 7000 : 4000);
}

async function api(path, data) {
  const options = data === undefined ? {} : {method:'POST', headers:{'Content-Type':'application/json', 'X-Panel-Token':token}, body:JSON.stringify(data)};
  const response = await fetch('/api/' + path, options);
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || '操作未完成');
  return result;
}

function readForm() {
  for (const input of document.querySelectorAll('input')) {
    if (!input.reportValidity()) throw new Error('请检查输入范围与必填项');
  }
  return structuredClone(draft);
}

function renderRows() {
  const body = $('stations'); body.replaceChildren();
  draft.stations.forEach((point, index) => {
    const row = document.createElement('tr');
    const name = document.createElement('td'); name.textContent = 'P' + (index + 1); row.append(name);
    for (const key of ['x', 'y', 'yaw_deg']) {
      const cell = document.createElement('td'), input = document.createElement('input');
      input.type = 'number'; input.value = point[key]; input.required = true;
      input.step = key === 'yaw_deg' ? '1' : '0.1'; input.min = key === 'yaw_deg' ? '-360' : '-18'; input.max = key === 'yaw_deg' ? '360' : '18';
      input.setAttribute('aria-label', `P${index + 1} ${key === 'yaw_deg' ? '目标航向' : key.toUpperCase() + ' 坐标'}`);
      input.addEventListener('input', () => { draft.stations[index][key] = input.valueAsNumber; changed(); });
      cell.append(input); row.append(cell);
    }
    const actions = document.createElement('td');
    for (const [symbol, label, offset] of [['↑','上移',-1], ['↓','下移',1], ['×','删除',0]]) {
      const button = document.createElement('button'); button.textContent = symbol; button.title = label;
      button.setAttribute('aria-label', label + ' P' + (index + 1)); button.className = 'row-action' + (offset ? '' : ' remove');
      button.disabled = offset ? (index + offset < 0 || index + offset >= draft.stations.length) : draft.stations.length === 1;
      button.addEventListener('click', () => {
        if (offset) [draft.stations[index], draft.stations[index + offset]] = [draft.stations[index + offset], draft.stations[index]];
        else draft.stations.splice(index, 1);
        renderRows(); changed();
      }); actions.append(button);
    }
    row.append(actions); body.append(row);
  });
  $('point-count').value = draft.stations.length;
  $('station-total').textContent = draft.stations.length;
  $('add-point').disabled = draft.stations.length >= 50;
}

function changed() {
  $('save-state').textContent = same(draft, lastSaved) ? '配置已保存在本机' : '有未保存的修改';
  drawMap(); renderState();
}

function drawMap() {
  if (!draft) return;
  const sx = x => 380 + Number(x) * 8.5, sy = y => 180 - Number(y) * 8.5;
  let svg = '<defs><marker id="arrow" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto"><path d="M0 0L10 5L0 10" fill="#8d74c7"/></marker></defs>';
  for (let x = -40; x <= 40; x += 5) {
    svg += `<line x1="${sx(x)}" y1="8" x2="${sx(x)}" y2="345" stroke="${x === 0 ? '#ced9e8' : '#e9eef5'}" stroke-width="${x === 0 ? 1.1 : .7}"/>`;
    if (x % 10 === 0) svg += `<text x="${sx(x)+4}" y="342" fill="#a4b2c5" font-size="9">${x}</text>`;
  }
  for (let y = -15; y <= 15; y += 5) {
    svg += `<line x1="12" y1="${sy(y)}" x2="748" y2="${sy(y)}" stroke="${y === 0 ? '#ced9e8' : '#e9eef5'}" stroke-width="${y === 0 ? 1.1 : .7}"/><text x="735" y="${sy(y)-4}" fill="#a4b2c5" font-size="9">${y}</text>`;
  }
  svg += `<rect x="${sx(-18)}" y="${sy(18)}" width="306" height="306" fill="none" stroke="#dce5f0" stroke-dasharray="4 5"/>`;
  const points = draft.stations.filter(p => [p.x, p.y, p.yaw_deg].every(Number.isFinite));
  const home = state.mission?.home || [-18,0,0];
  const origin = {x:home[0],y:home[1]};
  const route = [origin, ...points, origin];
  svg += `<polyline points="${route.map(p => `${sx(p.x)},${sy(p.y)}`).join(' ')}" fill="none" stroke="#5282e8" stroke-width="2" stroke-linejoin="round" stroke-dasharray="6 4" opacity=".85"/>`;
  svg += `<rect x="${sx(origin.x)-5}" y="${sy(origin.y)-5}" width="10" height="10" rx="2" fill="#289e8a"/><text x="${sx(origin.x)-30}" y="${sy(origin.y)+24}" fill="#289e8a" font-size="10">起点 / 返航</text>`;
  points.forEach((point, i) => {
    const x = sx(point.x), y = sy(point.y), yaw = point.yaw_deg * Math.PI / 180;
    const duplicate = points.slice(0, i).some(p => p.x === point.x && p.y === point.y);
    const shift = duplicate ? -18 : 0;
    svg += `<line x1="${x}" y1="${y}" x2="${x+Math.cos(yaw)*23}" y2="${y-Math.sin(yaw)*23}" stroke="#9b83ca" stroke-width="1.5" marker-end="url(#arrow)"/><circle cx="${x}" cy="${y}" r="11" fill="#e9f0ff" stroke="#c5d7fb"/><circle cx="${x}" cy="${y}" r="4.5" fill="#457de8"/><text x="${x+15}" y="${y-11+shift}" fill="#5274aa" font-size="11" font-weight="600">P${i+1}</text>`;
  });
  const actual = state.mission?.position;
  if (actual && actual.every(Number.isFinite)) {
    const x = sx(actual[0]), y = sy(actual[1]), angle = Number(state.mission.yaw_deg) * Math.PI / 180;
    svg += `<circle cx="${x}" cy="${y}" r="12" fill="#36aa9620"/><circle cx="${x}" cy="${y}" r="5" fill="#289e8a" stroke="white" stroke-width="2"/><line x1="${x}" y1="${y}" x2="${x+18*Math.cos(angle)}" y2="${y-18*Math.sin(angle)}" stroke="#289e8a" stroke-width="2"/>`;
  }
  $('route-map').innerHTML = svg;
  $('height-label').textContent = Number.isFinite(draft.height) ? draft.height.toFixed(1) + ' m' : '—';
  let distance = 0; for (let i=1;i<route.length;i++) distance += Math.hypot(route[i].x-route[i-1].x, route[i].y-route[i-1].y);
  $('distance-label').textContent = distance.toFixed(1) + ' m';
  $('route-meta').textContent = `${draft.stations.length} 个巡检点 · 含自动返航`;
  $('home-label').textContent = `(${origin.x.toFixed(2)}, ${origin.y.toFixed(2)})`;
}

function renderState() {
  if (!draft) return;
  const mission = state.mission;
  renderPlanning(state.planning || mission?.planning);
  const running = !!state.running;
  const active = mission && activeStates.includes(mission.state);
  $('launch').disabled = busy || (running && (!mission || active));
  $('launch').textContent = running ? '重新应用并启动仿真' : '应用配置并启动仿真';
  $('save').disabled = busy;
  $('start').disabled = busy || !running || !mission || !['WAIT_READY','COMPLETE','FAILED'].includes(mission.state);
  $('start').textContent = mission?.state === 'COMPLETE' || mission?.state === 'FAILED' ? '▷ 重新开始巡检' : '▷ 开始巡检';
  $('stop').disabled = busy || !running;
  $('mission-state').textContent = mission ? labels[mission.state] || mission.state : running ? '正在启动仿真…' : '未启动仿真';
  $('mission-state').className = 'state-badge' + (mission?.state === 'COMPLETE' ? ' complete' : mission?.state === 'FAILED' ? ' failed' : running ? ' active' : '');
  $('progress').textContent = mission ? `${mission.completed} / ${mission.total} 站` : '—';
  $('position').textContent = mission ? mission.position.map(x => x.toFixed(2)).join(' / ') + ' m' : '—';
  $('speed').textContent = mission ? mission.speed.toFixed(2) + ' m/s' : '—';
  $('yaw').textContent = mission ? mission.yaw_deg.toFixed(1) + '°' : '—';
  let note = '先启动仿真，地图与定位就绪后再开始巡检。RViz 将同时打开。';
  if (running && !mission) note = '正在加载地图与定位，请稍候；若长时间没有状态，请展开运行日志。';
  if (mission) note = mission.state === 'FAILED' ? '任务已停止派发目标：' + mission.reason : mission.state === 'COMPLETE' ? '巡检、返航及下降回收已完成，当前在起点上方低位保持。' : mission.state === 'RETURN_HOME' ? '所有巡检点已完成，正在返回本次任务实际起点；抵达并稳定后开始下降。' : mission.state === 'DESCEND' ? `正在起点上方逐步下降，最终目标高度 ${mission.recovery_height.toFixed(2)} 米；当前分段目标 ${mission.target[2].toFixed(2)} 米。` : `当前目标 P${mission.station} · 到达 → 转向稳定 → 悬停 ${state.active_config.hover_time} 秒 → 下一点 → 自动返航回收`;
  if (running && state.active_config && !same(draft, state.active_config)) note += ' 页面配置与运行配置不同，修改将在重新应用后生效。';
  if (!running && state.exit_code !== null && state.exit_code !== undefined && state.exit_code !== 0) note = '仿真已退出。请展开运行日志查看原因，然后重新启动。';
  $('live-note').textContent = note;
}

async function refresh() {
  try {
    state = await api('state'); token = state.token;
    if (!initialized) {
      draft = structuredClone(state.config); lastSaved = structuredClone(state.config);
      for (const key of ['max_vel','max_acc','height','hover_time']) $(key).value = draft[key];
      renderRows(); initialized = true; changed();
    }
    renderState(); drawMap();
    if ($('logs').open) $('log-content').textContent = (await api('logs')).text.replace(/\u001b\[[0-9;]*m/g,'');
  } catch (error) {
    $('mission-state').textContent = '面板连接已断开';
    $('live-note').textContent = '无法读取面板状态，请检查启动面板的终端。';
    for (const id of ['launch','start','stop','save']) $(id).disabled = true;
  }
}

async function action(kind) {
  if (busy) return;
  try {
    const data = ['config','launch'].includes(kind) ? readForm() : {};
    busy = true; renderState();
    const result = await api(kind, data);
    if (kind === 'config' || kind === 'launch') { lastSaved = structuredClone(result.config); draft = structuredClone(result.config); }
    notify({config:'配置已保存，下次应用时生效',launch:'仿真正在启动，准备好后点击“开始巡检”',start:'巡检开始请求已发送',stop:'面板管理的仿真已关闭'}[kind]);
    await refresh(); changed();
  } catch (error) { notify(error.message, true); }
  finally { busy = false; renderState(); }
}

for (const key of ['max_vel','max_acc','height','hover_time']) $(key).addEventListener('input', () => { if (draft) { draft[key] = $(key).valueAsNumber; changed(); } });
$('settings-form').addEventListener('submit', e => e.preventDefault());
$('point-count').addEventListener('change', () => {
  if (!$('point-count').reportValidity()) return;
  const count = $('point-count').valueAsNumber;
  while (draft.stations.length < count) draft.stations.push({x:0,y:0,yaw_deg:0});
  draft.stations.length = count; renderRows(); changed();
});
$('add-point').addEventListener('click', () => { if (draft.stations.length < 50) { draft.stations.push({x:0,y:0,yaw_deg:0}); renderRows(); changed(); } });
for (const [id, kind] of [['save','config'],['launch','launch'],['start','start'],['stop','stop']]) $(id).addEventListener('click', () => action(kind));
refresh(); setInterval(refresh, 1000);

let planningTableSignature;
function renderPlanning(metrics) {
  const records = metrics?.records || [];
  const ms = value => Number.isFinite(value) ? value.toFixed(3) + ' ms' : '—';
  $('plan-latest').textContent = ms(records.at(-1)?.duration_ms);
  $('plan-average').textContent = ms(metrics?.average_ms);
  $('plan-maximum').textContent = ms(metrics?.maximum_ms);
  $('plan-response').textContent = ms(metrics?.goal_response_ms);
  $('plan-count').textContent = metrics?.count ? `${metrics.archived ? '上次仿真记录 · ' : ''}已接收 ${metrics.count} 次 · 成功 ${metrics.count - metrics.failed} · 失败 ${metrics.failed} · 下表显示最近 100 次，CSV 包含本次仿真全部已接收记录` + (metrics.missed ? ` · 检测到 ${metrics.missed} 条缺失记录` : '') : '尚无规划记录，开始巡检后显示实测数据。';
  const signature = JSON.stringify(records);
  if (signature === planningTableSignature) return;
  planningTableSignature = signature;
  const fragment = document.createDocumentFragment();
  for (const record of [...records].reverse()) {
    const row = document.createElement('tr');
    const values = [`${record.seq} / G${record.goal_id}`, new Date(record.time * 1000).toLocaleTimeString('zh-CN', {timeZone: 'Asia/Shanghai', hour12: false}), {global:'全局规划', local_initial:'目标首条轨迹 / 重试', local_replan:'局部重规划'}[record.stage], record.success ? '成功' : '失败', record.duration_ms.toFixed(3), record.goal_response_ms === null ? '—' : record.goal_response_ms.toFixed(3)];
    for (const value of values) { const cell = document.createElement('td'); cell.textContent = value; row.append(cell); }
    fragment.append(row);
  }
  $('planning-records').replaceChildren(fragment);
}
