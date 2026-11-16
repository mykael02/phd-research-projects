"""Render an offline GIF and still from saved trajectories; no UI automation."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from PIL import Image
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'visualisation';R=.045;L=.18

def body(ax,state,lane,color):
    x,_,theta,_=state
    def transform(points):
        a=np.array(points);a[:,[0,2]]=a[:,[0,2]]@np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]]);a+=np.array([x,lane,R]);return a
    points=transform([[a,b,c] for a in [-.0425,.0425] for b in [-.0725,.0725] for c in [L-.10,L+.10]])
    faces=[[points[i] for i in ids] for ids in [[0,1,3,2],[4,5,7,6],[0,4,5,1],[2,6,7,3],[0,2,6,4],[1,5,7,3]]]
    ax.add_collection3d(Poly3DCollection(faces,facecolor=color,edgecolor='#25445b',linewidth=.5,alpha=.95))
    for side in [-.065,.065]:
        line=transform([[0,side,0],[0,side,L]]);ax.plot(*line.T,color='#8395a2',lw=4)
    ax.plot([x,x],[lane-.13,lane+.13],[R,R],color='#73858f',lw=3)
    t=np.linspace(0,2*np.pi,40)
    for side in [-.115,.115]:
        rings=[]
        for dy in [-.0175,.0175]:rings.append(np.stack([x+R*np.cos(t),np.full_like(t,lane+side+dy),R+R*np.sin(t)],axis=1))
        faces=[rings[0],rings[1]]+[[rings[0][i],rings[0][i+1],rings[1][i+1],rings[1][i]] for i in range(len(t)-1)]
        ax.add_collection3d(Poly3DCollection(faces,facecolor='#293e4b',edgecolor='none'))
        angle=-x/R;ax.plot([x-.035*np.cos(angle),x+.035*np.cos(angle)],[lane+side-.018]*2,[R-.035*np.sin(angle),R+.035*np.sin(angle)],color='#b5c6cf',lw=2)
    com=transform([[0,0,L]])[0];ax.scatter(*com,color='#f2bd45',s=40,depthshade=False,edgecolors='white')
    return com

def main():
    data=json.loads((ROOT/'publication/data/analysis.json').read_text());examples=[next(e for e in data['examples'].values() if e['controller']==name and e['condition']==[5,5,.7]) for name in ('pid','lqr')]
    frames=[]
    for frame in range(101):
        time=frame/10;fig=plt.figure(figsize=(10,6),dpi=100,facecolor='#f3f2ed');ax=fig.add_axes([.04,.14,.92,.69],projection='3d');ax.set_facecolor('#f3f2ed')
        ax.set_xlim(-.17,.64);ax.set_ylim(-.55,.55);ax.set_zlim(0,.46);ax.set_box_aspect((.81,1.1,.46));ax.view_init(elev=20,azim=-58);ax.set_axis_off()
        for x in np.arange(-.2,.7,.1):ax.plot([x,x],[-.55,.55],[0,0],color='#d0d8dc',lw=.6)
        for y in np.arange(-.5,.6,.1):ax.plot([-.2,.7],[y,y],[0,0],color='#d0d8dc',lw=.6)
        for i,(trial,color,name,lane) in enumerate(zip(examples,['#bd531e','#087b88'],['PID-type','LQR'],[-.27,.27])):
            row=trial['rows'][min(round(time/.005),len(trial['rows'])-1)];com=body(ax,row['state'],lane,color);ax.plot([0,0],[lane-.13,lane+.13],[.001,.001],color='#758b9a',lw=2)
            if 2<=time<2.2:ax.quiver(*com,.18,0,0,color='#b77b00',arrow_length_ratio=.2,lw=2)
            fig.text(.08+i*.48,.10,f"{name}: tilt {np.rad2deg(row['state'][2]):+.2f}°  |  x {row['state'][0]:+.3f} m",color=color,size=12)
        fig.text(.07,.92,'BALANCING ROBOT / 3D REPLAY',size=19,weight='bold',color='#18364d');fig.text(.07,.865,'Recorded planar dynamics · 5° initial tilt · +5 N pulse · 0.70 N m torque limit',size=11,color='#536b7b');fig.text(.86,.92,f'{time:04.1f} s',size=16,color='#18364d');fig.text(.07,.045,'Michael Agbo  |  Illustrative geometry; no yaw or lateral dynamics  |  Grid: 0.1 m',size=10,color='#536b7b')
        fig.canvas.draw();im=Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:,:,:3].copy());frames.append(im)
        if frame==21:im.save(OUT/'robot-3d-preview.png')
        plt.close(fig)
    frames[0].save(OUT/'robot-3d-preview.gif',save_all=True,append_images=frames[1:],duration=100,loop=0,optimize=False)
    print('Rendered 101-frame, 10 Hz 3D preview and a still at t=2.1 s.')

if __name__=='__main__':main()
