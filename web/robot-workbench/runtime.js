'use strict';
// This panel is independent of the renderer so visual refinements can merge separately.
(() => {
  let lease = null, loadedId = null, boundRevision = -1, busy = false, running = false;
  let polling = false, generation = null;
  let showRuntimePose = false;
  let actionError = '';
  const label = (text, error = false) => {
    $('runtime-status').textContent = text;
    $('runtime-status').classList.toggle('error', error);
  };
  const controls = () => {
    $('runtime-load').disabled = busy || running;
    $('runtime-run').disabled = busy || running || !lease;
    $('runtime-stop').disabled = !lease;
  };
  const apply = result => {
    running = result.state === 'RUNNING';
    if (lease && result.generation !== generation) {
      lease = null; loadedId = null;
      label('ROS 2 配置已由另一会话切换，请重新加载。', true);
    } else if (lease) {
      const labels = {UNLOADED:'未加载配置',READY:'已就绪',RUNNING:'试运行中',STOPPED:'已停止'};
      const reasons = {preview_client_heartbeat_lost:'连接中断，已保持当前位置',trajectory_complete:'试运行完成',operator_stop:'已保持当前位置',profile_loaded_into_ros2_joint_preview:'配置已加载',kinematic_trajectory_running:'使用已加载配置'};
      label(`ROS 2 ${labels[result.state] || result.state} · ${reasons[result.reason] || ''}`);
      if (showRuntimePose && boundRevision === revision && result.pose) {
        currentPose = result.pose; model = result.model; draw();
        $('progress').value = result.duration_s ? Math.min(1,result.elapsed_s/result.duration_s) : 0;
      }
      if (result.gravity_limit_exceeded?.length) {
        label('ROS 2 预览存在静态重力负载超限：' + result.gravity_limit_exceeded.join('、'),true);
      }
    } else {
      label(result.profile_id ? 'ROS 2 已连接 · 点击“保存并加载”建立本页预览会话。' : 'ROS 2 已连接 · 等待加载配置');
    }
    if (actionError) label(actionError,true);
    controls();
  };
  async function action(fn) {
    if (busy) return;
    busy = true; actionError = ''; controls();
    try { await fn(); } catch (e) { actionError = e.message; label(e.message,true); }
    finally { busy = false; controls(); }
  }
  $('runtime-load').onclick = () => action(async () => {
    stop();
    const rev = revision;
    const registered = await api('/v1/robot-profiles',payload());
    if (rev !== revision) throw Error('参数在保存期间改变，请重新加载。');
    const result = await api('/v1/runtime/load',{profile_id:registered.profile_id});
    lease = result.lease_token; loadedId = result.profile_id; generation = result.generation; boundRevision = rev;
    showRuntimePose = true;
    $('profile-id').value = loadedId; $('dirty').textContent = rev === revision ? '参数已保存' : '有修改 · 未保存';
    $('runtime-profile').textContent = 'ROS 2 配置：' + loadedId.slice(0,12) + '…';
    apply(result);
  });
  $('runtime-run').onclick = () => action(async () => {
    stop();
    const rev = revision;
    const registered = await api('/v1/robot-profiles',payload());
    if (rev !== revision || registered.profile_id !== loadedId) throw Error('当前参数与 ROS 2 已加载版本不同，请先“保存并加载”。');
    boundRevision = revision;
    showRuntimePose = true;
    const result = await api('/v1/runtime/run',{profile_id:loadedId,lease_token:lease,target:targets,duration_s:numeric($('duration').value)});
    apply(result);
  });
  $('runtime-stop').onclick = async () => {
    if (!lease) return;
    actionError = '';
    try { apply(await api('/v1/runtime/stop',{lease_token:lease})); }
    catch(e) { label(e.message,true); }
  };
  $('run').addEventListener('click', event => {
    if (running) {
      event.preventDefault(); event.stopImmediatePropagation();
      label('请先停止 ROS 2 预览，再启动本机运动学动画。',true);
    } else { showRuntimePose = false; }
  },true);
  async function poll() {
    if (polling || busy) return;
    polling = true;
    try {
      if (lease && running && revision !== boundRevision) {
        apply(await api('/v1/runtime/stop',{lease_token:lease}));
        label('参数或目标已修改，ROS 2 预览已停止。重新加载参数后可继续。');
      } else if (lease) {
        const result = await api('/v1/runtime/heartbeat',{lease_token:lease});
        apply(result);
      } else { apply(await api('/v1/runtime/status')); }
    } catch(e) {
      lease = null; loadedId = null; running = false; controls();
      label('ROS 2 未连接：'+e.message,true);
    } finally { polling = false; }
  }
  setInterval(poll,500); poll();
  addEventListener('pagehide', () => {
    if (lease) fetch('/v1/runtime/stop',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({lease_token:lease}),keepalive:true}).catch(()=>{});
  });
})();
