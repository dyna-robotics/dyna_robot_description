from pathlib import Path
from PIL import Image,ImageDraw
import numpy as np,json
r=Path(__file__).parent;out=r/'comparisons';out.mkdir(exist_ok=True);stats={}
for view in ['front','back','side','bent_back']:
 for mode in ['solid','transparent']:
  a=Image.open(r/f'renders/baseline/{view}_{mode}.png');b=Image.open(r/f'renders/candidate/{view}_{mode}.png');diff=np.abs(np.array(a).astype(float)-np.array(b).astype(float));stats[view+'_'+mode]={'mean_difference_255':float(diff.mean()),'pixels_over_10_percent':float((diff.max(2)>10).mean()*100)}
  canvas=Image.new('RGB',(1600,850),(32,35,40));canvas.paste(a.resize((800,800)),(0,50));canvas.paste(b.resize((800,800)),(800,50));d=ImageDraw.Draw(canvas);d.text((20,12),'Commit 1457ffd',fill='white',font_size=25);d.text((820,12),'Current working model',fill='white',font_size=25);canvas.save(out/f'{view}_{mode}.png')
for name,view,box in [('torso','back',(460,320,970,660)),('folding','back',(490,590,940,1060)),('agv','back',(420,920,1050,1310)),('shoulders','front',(450,330,980,700))]:
 a=Image.open(r/f'renders/baseline/{view}_transparent.png').crop(box);b=Image.open(r/f'renders/candidate/{view}_transparent.png').crop(box);w=800;h=round(a.height*w/a.width);canvas=Image.new('RGB',(1600,h+50),(32,35,40));canvas.paste(a.resize((w,h)),(0,50));canvas.paste(b.resize((w,h)),(800,50));d=ImageDraw.Draw(canvas);d.text((20,12),'Commit 1457ffd: '+name,fill='white',font_size=25);d.text((820,12),'Current: '+name,fill='white',font_size=25);canvas.save(out/f'{name}_detail.png')
(r/'render_comparison.json').write_text(json.dumps(stats,indent=2));print(json.dumps(stats,indent=2))
