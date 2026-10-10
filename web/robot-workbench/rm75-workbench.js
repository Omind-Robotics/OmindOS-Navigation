'use strict';
// Reuse the existing workbench, renderer, stop latch, API and parameter store.
$('profile-mode').onchange = () => guarded(async () => {
  if (!profile) return;
  stop(); freezePose();
  if ($('profile-mode').value === 'kinematic_preview') profile.profile_mode = 'kinematic_preview';
  else delete profile.profile_mode;
  updateCooperativeDemo();
  edited({refresh:false});
  $('profile-mode-note').textContent = profile.profile_mode === 'kinematic_preview'
    ? '仅运动学：未知物理参数可留空；不计算电机目标、力矩或动力学。'
    : '完整参数配置：沿用原有严格校验，不自动补填物理参数。';
});
$('load-cad-model').onclick = () => guarded(async () => {
  const record = await api('/v1/workbench-model/' + $('cad-model').value);
  await importProfile(record.profile);
  $('planning-mapping').value = JSON.stringify(record.mapping, null, 2);
  planningStart = null;
  if (record.poses.preview) targets = {...record.poses.preview};
  mode = 'joint'; rebuildParts('waist_pitch');
  await refreshModel(true);
  $('profile-mode-note').textContent = '整机 CAD 运动学配置；腰部零位与传感器子型号仍须实机核对。';
  status('整机模型已载入编辑区；点击“写入程序”后才保存。普通点动不含碰撞验证。');
});
(async () => {
  try {
    const record = await api('/v1/workbench-models');
    const select = $('cad-model'); select.replaceChildren();
    for (const item of record.models) {
      const option = document.createElement('option'); option.value = item.id; option.textContent = item.label;
      select.append(option);
    }
    $('load-cad-model').disabled = !record.models.length;
    $('cad-bundle-note').textContent = record.models.length
      ? '本机已安装整机资源，加载不会覆盖已保存的配置。'
      : '当前安装未附带整机资源；仍可导入 URDF 做运动学预览。';
  } catch (error) {
    $('load-cad-model').disabled = true;
    $('cad-bundle-note').textContent = '整机资源不可用：' + error.message;
  }
})();
