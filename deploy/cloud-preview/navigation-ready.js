async function runNavigation(thenArm=false){
  const previewTicket=motionTicket,assetsDeadline=performance.now()+15000;
  while(sceneEngine && sceneEngine.meshStats.loaded+sceneEngine.meshStats.failed<sceneEngine.meshStats.requested){
    status('正在加载三维模型，完成后启动导航…');
    if(performance.now()>assetsDeadline)throw Error('三维模型仍在加载，请加载完成后重新启动导航。');
    await new Promise(resolve=>setTimeout(resolve,100));
    if(previewTicket!==motionTicket||emergencyStop||safetyPending)throw Error('导航已取消，请重新启动。');
  }
