'use strict';
(() => {
  const cad=document.getElementById('cad-workspace'),parameters=document.querySelector('main');
  const bar=document.getElementById('workbench-model-tabs');
  if(document.body.dataset.product!=='quadruped')return;
  bar.hidden=false;
  let viewer,pending;
  async function show(mode){
    const active=mode==='cad';
    // Keep the existing editor and draft DOM mounted when switching views.
    // Cancel local preview animation before leaving its pane.
    if(active){
      window.dispatchEvent(new Event('workbench:leave-parameters'));
      const runtimeStop=document.getElementById('runtime-stop');
      if(!runtimeStop.disabled)runtimeStop.click();
    }
    cad.hidden=!active;parameters.hidden=active;
    document.getElementById('cad-tab').setAttribute('aria-pressed',String(active));
    document.getElementById('parameters-tab').setAttribute('aria-pressed',String(!active));
    document.getElementById('workbench-mode-label').textContent=active?'CAD 装配预览':'关节参数调试';
    document.body.classList.toggle('cad-active',active);
    if(!active){window.dispatchEvent(new Event('workbench:leave-cad'));window.dispatchEvent(new Event('resize'));return;}
    try{
      pending ||= import('/cad-viewer.js').then(module=>module.mountCAD(cad));
      viewer ||= await pending;viewer.resize();
    }catch(error){pending=null;document.getElementById('cad-status').textContent='模型加载失败：'+error.message;}
  }
  document.getElementById('cad-tab').onclick=()=>show('cad');
  document.getElementById('parameters-tab').onclick=()=>show('parameters');
  document.getElementById('cad-to-parameters').onclick=()=>show('parameters');
  show('cad');
})();
