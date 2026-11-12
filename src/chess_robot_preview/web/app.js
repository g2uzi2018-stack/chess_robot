'use strict';
const $=id=>document.getElementById(id);
const client=Date.now().toString(36)+'_'+Math.random().toString(36).slice(2);
let channels=[],selected=null,paused=false,epoch=0,controller=null,timer=null,blobURL=null,sequence='',fps=30;
let lastDisplay=0,displayIntervals=[];
const icon='<svg viewBox="0 0 32 32" aria-hidden="true"><rect x="4" y="5" width="24" height="22" rx="3"/><circle cx="12" cy="12" r="2"/><path d="m5 23 8-7 5 5 4-4 6 7"/></svg>';
function release(){fetch('/api/release?client='+client,{method:'POST',keepalive:true}).catch(()=>{});}
function clearFrame(){
  $('main-image').hidden=true;$('main-image').removeAttribute('src');$('stage-label').hidden=true;
  for(const img of document.querySelectorAll('.thumb img')){img.hidden=true;img.removeAttribute('src');}
  for(const p of document.querySelectorAll('.thumb-placeholder'))p.hidden=false;
  if(blobURL){URL.revokeObjectURL(blobURL);blobURL=null;}
}
function message(title,description){$('empty').hidden=false;$('empty-title').textContent=title;$('empty-description').textContent=description;}
function stop(){epoch++;clearTimeout(timer);if(controller)controller.abort();controller=null;sequence='';lastDisplay=0;displayIntervals=[];clearFrame();}
function select(id){
  stop();selected=channels.find(c=>c.id===id);paused=false;
  for(const b of document.querySelectorAll('.channel')){
    const active=b.dataset.id===id;b.setAttribute('aria-pressed',String(active));b.querySelector('.selected-label').textContent=active?'查看中':'';
  }
  $('view-title').textContent=selected.title;$('view-description').textContent=selected.description;
  $('pause').disabled=false;$('pause').textContent='暂停';$('fullscreen').disabled=false;
  message('正在连接图像','等待当前图像源');$('view-state').textContent='连接图像中';$('frame-info').textContent='—';
  pull(epoch);
}
async function pull(generation){
  if(generation!==epoch||!selected||paused||document.hidden)return;
  const started=performance.now();let retryDelay=0;controller=new AbortController();const timeout=setTimeout(()=>controller.abort(),2500);
  try{
    const res=await fetch('/api/frame/'+selected.id+'?client='+client+'&after='+sequence,{cache:'no-store',signal:controller.signal});
    if(generation!==epoch)return;
    if(res.status===200){
      const image=await res.blob();if(generation!==epoch)return;
      sequence=res.headers.get('X-Frame-Sequence')||'';
      const next=URL.createObjectURL(image),previous=blobURL;blobURL=next;
      const main=$('main-image'),thumb=document.querySelector('.channel[aria-pressed=true] .thumb img');
      main.onload=()=>{
        if(generation!==epoch)return;
        $('empty').hidden=true;main.hidden=false;$('stage-label').hidden=false;
        $('view-state').textContent='正在预览';
        const time=performance.now();if(lastDisplay)displayIntervals.push(time-lastDisplay);lastDisplay=time;
        displayIntervals=displayIntervals.slice(-15);
        const actual=displayIntervals.length?1000/(displayIntervals.reduce((a,b)=>a+b,0)/displayIntervals.length):0;
        $('frame-info').textContent=main.naturalWidth+' × '+main.naturalHeight+(actual?' · '+actual.toFixed(1)+' fps':'');
      };
      main.src=next;thumb.src=next;thumb.hidden=false;thumb.parentNode.querySelector('.thumb-placeholder').hidden=true;
      if(previous)URL.revokeObjectURL(previous);
    }else if(res.status!==204){
      clearFrame();message('无数据','当前图像源暂时没有可显示的画面');$('view-state').textContent='无数据';$('frame-info').textContent='无';
      const placeholder=document.querySelector('.channel[aria-pressed=true] .thumb-placeholder');
      if(placeholder)placeholder.innerHTML=icon+'无数据';
      retryDelay=250;
    }
  }catch(error){
    if(generation!==epoch)return;
    clearFrame();message('连接暂时中断','正在重试，旧画面已清除');$('view-state').textContent='重新连接中';
  }finally{
    clearTimeout(timeout);
    if(generation===epoch&&!paused&&!document.hidden)timer=setTimeout(()=>pull(generation),Math.max(1,retryDelay,1000/fps-(performance.now()-started)));
  }
}
async function loadChannels(){
  try{
    const res=await fetch('/api/channels',{cache:'no-store'});if(!res.ok)throw Error('connection');
    const data=await res.json();fps=data.fps;$('connection').textContent='预览节点在线';$('connection').classList.remove('offline');
    $('preview-settings').textContent='上限 '+fps+' fps · 按需加载';
    if(!channels.length){
      channels=data.channels;$('channel-count').textContent=channels.length+' 路';
      for(const [index,c]of channels.entries()){
        const button=document.createElement('button');button.className='channel';button.dataset.id=c.id;button.setAttribute('aria-pressed','false');
        button.innerHTML='<div class="thumb"><div class="thumb-placeholder">'+icon+'点击后加载</div><img hidden alt=""></div><div class="channel-title"><span class="channel-number"></span><span class="title"></span><span class="selected-label"></span></div>';
        button.querySelector('.title').textContent=c.title;button.querySelector('.channel-number').textContent=String(index+1).padStart(2,'0');
        button.addEventListener('click',()=>select(c.id));$('channels').append(button);
      }
    }
    for(const c of data.channels){
      const button=document.querySelector('.channel[data-id="'+c.id+'"]');
      if(button&&button.getAttribute('aria-pressed')!=='true'){
        const placeholder=button.querySelector('.thumb-placeholder');
        placeholder.innerHTML=icon+(c.has_publisher?'点击后加载':'无数据');
      }
    }
  }catch(error){$('connection').textContent='预览节点离线';$('connection').classList.add('offline');}
}
$('pause').addEventListener('click',()=>{
  if(!selected)return;paused=!paused;stop();release();$('pause').textContent=paused?'继续':'暂停';
  if(paused){message('预览已暂停','点击继续，重新加载当前图像');$('view-state').textContent='已暂停';$('frame-info').textContent='图像传输已暂停';}
  else{message('正在连接图像','等待当前图像源');pull(epoch);}
});
$('fullscreen').addEventListener('click',()=>{if(!document.fullscreenElement)$('stage').requestFullscreen().catch(()=>{});else document.exitFullscreen();});
document.addEventListener('visibilitychange',()=>{
  if(document.hidden){stop();release();}
  else if(selected&&!paused){message('正在连接图像','等待当前图像源');pull(epoch);}
});
window.addEventListener('pagehide',()=>{stop();release();});
loadChannels();setInterval(loadChannels,3000);
