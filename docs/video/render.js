// Renders why_the_arm.html frame by frame to MP4. Needs Playwright + ffmpeg.
// Usage: node docs/video/render.js docs/video/why_the_arm.mp4
const {chromium}=require('playwright');const {spawn}=require('child_process');
(async()=>{const out=process.argv[2];const b=await chromium.launch();const p=await b.newPage({viewport:{width:1280,height:720}});
await p.addInitScript(()=>{window.__RENDER__=true});
await p.goto('file://'+require('path').resolve(__dirname,'why_the_arm.html'));await p.evaluate(()=>document.fonts.ready);
const total=await p.evaluate(()=>TOTAL);const fps=30,n=Math.round(total*fps);
const ff=spawn('ffmpeg',['-y','-loglevel','error','-f','image2pipe','-framerate',String(fps),'-c:v','mjpeg','-i','-','-c:v','libx264','-pix_fmt','yuv420p','-crf','20','-preset','medium','-movflags','+faststart',out]);
ff.stderr.on('data',d=>process.stderr.write(d));
for(let i=0;i<n;i++){const data=await p.evaluate(t=>{draw(t);return document.getElementById('c').toDataURL('image/jpeg',0.93).split(',')[1]},i/fps);
 if(!ff.stdin.write(Buffer.from(data,'base64')))await new Promise(r=>ff.stdin.once('drain',r));}
ff.stdin.end();await new Promise(r=>ff.on('close',r));await b.close();console.log('frames',n);})();
