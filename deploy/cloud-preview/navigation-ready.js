async function runNavigation(thenArm=false){
  const previewTicket=motionTicket,previewRevision=revision,assetsDeadline=performance.now()+15000;
  const preparationError=message=>{$('navigation-status').textContent=message;throw Error(message);};
  while(sceneEngine && sceneEngine.meshStats.loaded+sceneEngine.meshStats.failed<sceneEngine.meshStats.requested){
    status('正在加载三维模型，完成后启动导航…');
    $('navigation-status').textContent='正在加载三维模型，完成后启动导航…';
    if(performance.now()>assetsDeadline)preparationError('三维模型仍在加载，请加载完成后重新启动导航。');
    await new Promise(resolve=>setTimeout(resolve,100));
    if(previewTicket!==motionTicket||previewRevision!==revision||emergencyStop||safetyPending)preparationError('导航已取消，请重新启动。');
  }
  if(previewTicket!==motionTicket||previewRevision!==revision||emergencyStop||safetyPending)preparationError('导航已取消，请重新启动。');
  if(sceneEngine?.meshStats.failed)preparationError('三维模型资源加载失败，请重新加载模型后启动导航。');
