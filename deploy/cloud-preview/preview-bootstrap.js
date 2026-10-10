'use strict';
(() => {
  const prefix = '/_cloud-preview/';
  const nativeFetch = window.fetch.bind(window);
  function scopedUrl(input) {
    const url = new URL(input, location.href);
    if (url.origin === location.origin && !url.pathname.startsWith(prefix)) {
      url.pathname = prefix.slice(0, -1) + url.pathname;
    }
    return url.href;
  }
  window.fetch = (input, options) => {
    if (input instanceof Request) {
      return nativeFetch(new Request(scopedUrl(input.url), input), options);
    }
    return nativeFetch(scopedUrl(input), options);
  };
})();

/* Keep the live model and existing controls together on compact screens. */
(() => {
  function mountLayout() {
    const main=document.querySelector('main'), view=document.querySelector('.viewport');
    const parameters=document.querySelector('.parameters'), actions=document.querySelector('.actions');
    const navigation=document.querySelector('.navigation-controls');
    if(!main||!view||!parameters||!actions||!navigation)return;
    const link=document.createElement('link');
    link.rel='stylesheet';link.href='/_cloud-preview/preview-layout.css';document.head.append(link);
    const media=matchMedia('(max-width: 1100px)');
    const tabs=document.createElement('nav');tabs.className='cloud-layout-tabs';
    tabs.setAttribute('aria-label','工作台操作切换');
    const navPanel=document.createElement('section');navPanel.className='cloud-control-panel cloud-navigation-panel';
    const viewPanel=document.createElement('section');viewPanel.className='cloud-control-panel cloud-view-panel';
    const liveControls=document.createElement('div');liveControls.className='cloud-live-controls';
    const panels={navigation:navPanel,parameters,motion:actions,view:viewPanel};
    const labels={navigation:'导航',parameters:'机器人参数',motion:'双臂操作',view:'视图'};
    let selected='navigation', mounted=false, moved=[];
    for(const [key,panel]of Object.entries(panels)){
      panel.id=panel.id||'cloud-panel-'+key;
      const b=document.createElement('button');b.type='button';b.textContent=labels[key];
      b.dataset.panel=key;b.setAttribute('aria-controls',panel.id);
      b.onclick=()=>select(key);tabs.append(b);
    }
    function select(key){
      selected=key;
      for(const [name,panel]of Object.entries(panels))panel.classList.toggle('cloud-panel-selected',name===key);
      for(const b of tabs.children)b.setAttribute('aria-pressed',String(b.dataset.panel===key));
    }
    function move(node,parent){
      if(!node)return;
      const marker=document.createComment('cloud-layout-position');node.before(marker);
      moved.push({node,marker});parent.append(node);
    }
    function sync(){
      const compact=media.matches&&!document.body.classList.contains('cad-active');
      if(compact===mounted)return;
      mounted=compact;document.body.classList.toggle('cloud-compact',compact);
      if(compact){
        main.append(tabs,navPanel,viewPanel);
        move(navigation,navPanel);
        for(const selector of ['.stage-toolbar','.view-controls','.hint','#warnings','.reference-photo','.scope','.scene-options'])move(view.querySelector(selector),viewPanel);
        view.append(liveControls);
        for(const id of ['fit','stage-stop','reset-stop'])move(document.getElementById(id),liveControls);
        const status=document.getElementById('emergency-state');move(status,liveControls);
        const detail=navigation.querySelector('details');
        if(detail){detail.dataset.cloudPreviousOpen=String(detail.open);detail.open=true;}
        const map=document.getElementById('navigation-map');
        const mapDetails=document.createElement('details');mapDetails.className='cloud-map-details';
        const mapSummary=document.createElement('summary');mapSummary.textContent='查看导航地图';
        mapDetails.append(mapSummary);navigation.append(mapDetails);move(map,mapDetails);
        select(selected);
      }else{
        for(const {node,marker}of moved.reverse()){marker.replaceWith(node);}
        moved=[];liveControls.remove();tabs.remove();navPanel.remove();viewPanel.remove();
        navigation.querySelector('.cloud-map-details')?.remove();
        const detail=navigation.querySelector('details[data-cloud-previous-open]');
        if(detail){detail.open=detail.dataset.cloudPreviousOpen==='true';delete detail.dataset.cloudPreviousOpen;}
        for(const panel of Object.values(panels))panel.classList.remove('cloud-panel-selected');
      }
      requestAnimationFrame(()=>window.dispatchEvent(new Event('resize')));
    }
    media.addEventListener('change',sync);
    new MutationObserver(sync).observe(document.body,{attributes:true,attributeFilter:['class']});
    sync();
  }
  if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',mountLayout,{once:true});
  else mountLayout();
})();
