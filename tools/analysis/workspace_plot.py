"""Publication-style static views of the offline workspace analysis."""
import argparse
import csv
import hashlib
import json
import pathlib
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.collections import PolyCollection
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch, Rectangle


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--project',type=pathlib.Path,required=True);parser.add_argument('--run',type=pathlib.Path,required=True);parser.add_argument('--out',type=pathlib.Path,required=True);a=parser.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    fonts=list(pathlib.Path('/usr/share/fonts').rglob('NotoSansCJK-Regular.ttc'))
    if fonts:font_manager.fontManager.addfont(str(fonts[0]));plt.rcParams['font.family']=font_manager.FontProperties(fname=str(fonts[0])).get_name()
    plt.rcParams.update({'axes.unicode_minus':False,'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    data=np.load(a.run/'workspace_data.npz');summary=json.loads((a.run/'summary.json').read_text());verification=json.loads((a.run/'verification.json').read_text())
    coordinates=[data[k] for k in ['x','y','z']];x,y,z=coordinates;step=summary['grid_step_m'];shape=data['reachable'].shape
    reachable=data['reachable'];healthy=data['healthy'];corridor=data['corridor'];sigma=data['best_sigma']
    raw=data['raw_metrics'];raw=raw[(raw[:,9]>.5)&(raw[:,10]>.5)]
    home=np.array(summary['home_tcp'])[:3,3]
    sys.path.insert(0,str(a.project/'tools/description'))
    from geometry_report import RobotGeometry
    robot=RobotGeometry(a.project/'src/chess_robot_description/urdf/nova5_with_ag95.urdf',[a.project/'src'])
    home_q=json.loads((a.project/'config/arm/home_reference.json').read_text())['joint_positions_rad']
    meshes,_=robot.posed({**{f'joint{i+1}':q for i,q in enumerate(home_q)},'left_outer_knuckle_joint':.47},'visual','base_link')
    COLORS={'raw':'#d9e0e5','risk':'#ecaa54','healthy':'#60bfb6','corridor':'#155c84'}
    cmap=ListedColormap(['#ffffff',COLORS['risk'],COLORS['healthy'],COLORS['corridor']])
    labels=['X','Y','Z']
    def arm(ax,projection):
        for label,mesh in meshes:
            skip=max(1,len(mesh.faces)//1500);triangles=mesh.triangles[::skip]
            ax.add_collection(PolyCollection(triangles[:,:,list(projection)],facecolors='#52616d',edgecolors='none',alpha=.64,rasterized=True,zorder=8))
        ax.scatter(home[projection[0]],home[projection[1]],s=110,marker='*',c='#d02f72',edgecolors='white',linewidths=.6,zorder=15)
    def style(ax,projection):
        ax.set_aspect('equal');ax.grid(alpha=.16,zorder=0);ax.set_xlabel(labels[projection[0]]+' / m');ax.set_ylabel(labels[projection[1]]+' / m')
        ax.set_xlim(-1.12,1.12)
        if projection[1]==2:
            ax.set_ylim(-.075,1.42);ax.axhspan(-.075,summary['ground_z_m'],facecolor='#bfc4c6',alpha=.65,zorder=2)
            ax.axhline(summary['ground_z_m'],color='#737c80',lw=1,zorder=3)
        else:ax.set_ylim(-1.12,1.12)
        ax.plot(0,0,'+',color='black',ms=8,zorder=16)
    fig,axes=plt.subplots(1,3,figsize=(15.5,6.5));fig.subplots_adjust(top=.82,bottom=.22,left=.055,right=.985,wspace=.25)
    for ax,projection,title in zip(axes,[(0,1),(0,2),(1,2)],['俯视 XY','正视 XZ','侧视 YZ']):
        other=({0,1,2}-set(projection)).pop();reach=reachable.any(axis=other);good=healthy.any(axis=other);complete=corridor.any(axis=other)
        colors=np.zeros(reach.shape,dtype=np.uint8);colors[reach]=1;colors[good]=2;colors[complete]=3
        ax.scatter(raw[:,projection[0]],raw[:,projection[1]],s=.5,c=COLORS['raw'],alpha=.5,rasterized=True,zorder=1)
        extent=[coordinates[projection[0]][0]-step/2,coordinates[projection[0]][-1]+step/2,coordinates[projection[1]][0]-step/2,coordinates[projection[1]][-1]+step/2]
        rgba=cmap(colors.T/3);rgba[colors.T==0,3]=0
        ax.imshow(rgba,origin='lower',extent=extent,interpolation='nearest',zorder=4)
        if projection==(0,1):ax.add_patch(Rectangle((-.8,-.8),1.6,1.6,fill=False,linestyle='--',linewidth=.8,edgecolor='#777',zorder=7))
        else:ax.add_patch(Rectangle((-.8,0),1.6,1.2,fill=False,linestyle='--',linewidth=.8,edgecolor='#777',zorder=7))
        arm(ax,projection);style(ax,projection);ax.set_title(title,pad=10,fontweight='bold')
    fig.suptitle('Nova5 + AG95：TCP 候选工作区三视图',fontsize=18,fontweight='bold',y=.97)
    fig.text(.5,.895,f'桌面 z = {summary["ground_z_m"]*1000:+.0f} mm · 活动部件对桌面余量 {summary["clearance_m"]*1000:.0f} mm · 非相邻包络自碰撞间隙 10 mm · 下潜参考速度 {summary["reference_descent_speed_m_s"]*1000:.0f} mm/s',ha='center',fontsize=10)
    legend=[Patch(color=COLORS['raw'],label='任意朝向：采样可达'),Patch(color=COLORS['risk'],label='向下抓取：奇异/限位/速度余量不足'),Patch(color=COLORS['healthy'],label='向下抓取：存在健康逆解'),Patch(color=COLORS['corridor'],label='同分支通过上方 250 mm 通道筛选')]
    fig.legend(handles=legend,loc='lower center',bbox_to_anchor=(.5,.102),ncol=2,frameon=False,fontsize=10)
    fig.text(.5,.06,'机械臂轮廓为 URDF 的 Home 姿态；粉色星为 Home TCP。虚线为当前软件坐标框，不是真实运动能力边界。',ha='center',fontsize=9,color='#43525d')
    fig.text(.5,.025,'25 mm 网格，固定 Home 夹爪朝向；三视图为深度方向的投影并集。白区是本次未覆盖区，不是严格不可达证明；不是实机扫掠或连续路径认证。',ha='center',fontsize=9,color='#43525d')
    fig.savefig(a.out/'workspace-three-views.png',dpi=190);fig.savefig(a.out/'workspace-three-views.svg');plt.close(fig)

    # Horizontal slices retain holes that projection unions can hide.
    heights=[.025,.1,.2,.275];table=[];fig,axes=plt.subplots(2,2,figsize=(10.6,10.5));fig.subplots_adjust(top=.88,bottom=.12,left=.08,right=.98,wspace=.2,hspace=.3)
    for ax,height in zip(axes.flat,heights):
        k=int(np.argmin(abs(z-height)));colors=np.zeros(shape[:2],np.uint8);colors[reachable[:,:,k]]=1;colors[healthy[:,:,k]]=2;colors[corridor[:,:,k]]=3
        ax.imshow(colors.T,origin='lower',extent=[x[0]-step/2,x[-1]+step/2,y[0]-step/2,y[-1]+step/2],cmap=cmap,vmin=0,vmax=3,interpolation='nearest')
        ax.scatter(home[0],home[1],marker='*',s=90,c='#d02f72',edgecolors='white',linewidth=.5,zorder=5)
        style(ax,(0,1));ax.set_title(f'TCP 距桌面 {(z[k]-summary["ground_z_m"])*1000:.0f} mm（基座 z={z[k]:.3f} m）')
        for name,mask in [('reachable',reachable[:,:,k]),('healthy',healthy[:,:,k]),('corridor',corridor[:,:,k])]:
            ii,jj=np.where(mask);radius=np.hypot(x[ii],y[jj]);table.append(dict(tcp_z_m=float(z[k]),height_above_table_mm=float((z[k]-summary['ground_z_m'])*1000),layer=name,points=len(radius),radial_min_m=float(radius.min()) if len(radius) else None,radial_max_m=float(radius.max()) if len(radius) else None,area_estimate_m2=float(len(radius)*step**2)))
    fig.suptitle('水平切片：观察桌面附近盲区',fontsize=17,fontweight='bold',y=.965)
    fig.text(.5,.92,'相同 TCP 高度、相同夹爪朝向；颜色同三视图。粉星为 Home 的 XY 投影，圆环或缺口不填平。',ha='center',fontsize=10)
    fig.legend(handles=legend[1:],loc='lower center',bbox_to_anchor=(.5,.035),ncol=1,frameon=False,fontsize=9)
    fig.savefig(a.out/'workspace-height-slices.png',dpi=185);fig.savefig(a.out/'workspace-height-slices.svg');plt.close(fig)
    with (a.out/'height-slices.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=list(table[0]));w.writeheader();w.writerows(table)

    # Separate posture-based singularity illustration; not a spatial exclusion map.
    if (a.run/'singularity_sweep_metrics.bin').exists():
        sweep=np.fromfile(a.run/'singularity_sweep_metrics.bin').reshape(-1,12);q=np.fromfile(a.run/'singularity_sweep_q.bin').reshape(-1,6);half=len(q)//2
        fig,axes=plt.subplots(1,2,figsize=(12,4.8));fig.subplots_adjust(top=.79,bottom=.2,wspace=.26)
        for ax,rows,joint,title in [(axes[0],slice(0,half),2,'肘部：J3 趋近 0°'),(axes[1],slice(half,None),4,'腕部：J5 趋近 0° / ±180°')]:
            angles=np.degrees(q[rows,joint]);values=np.maximum(sweep[rows,6],1e-13);ax.semilogy(angles,values,color='#155c84',lw=2)
            ax.axhline(summary['sigma_threshold'],color='#dc8b2e',ls='--',label='本次筛选阈值 0.05');ax.axvline(np.degrees(home_q[joint]),color='#d02f72',ls=':',label='保存的 Home')
            ax.set_xlabel(f'J{joint+1} / °');ax.set_ylabel('归一化雅可比最小奇异值');ax.set_title(title);ax.grid(alpha=.2,which='both');ax.legend(fontsize=8)
        fig.suptitle('奇异性取决于关节姿态，不能只由末端位置判断',fontsize=16,fontweight='bold',y=.97)
        fig.text(.5,.86,'曲线仅为离线 FK/Jacobian 分析；不是运动指令，也未对曲线上全部姿态做路径认证。',ha='center',fontsize=9)
        fig.text(.5,.055,f'Home：最小奇异值 {verification["home_metrics"][6]:.3f}，条件数 {verification["home_metrics"][7]:.2f}（长度归一化 L=0.85 m）。越接近 0 越差。',ha='center',fontsize=10)
        fig.savefig(a.out/'singularity-postures.png',dpi=185);fig.savefig(a.out/'singularity-postures.svg');plt.close(fig)
    assets={str(path.relative_to(a.project)):hashlib.sha256(path.read_bytes()).hexdigest() for path in robot.assets}
    assets['src/chess_robot_description/urdf/nova5_with_ag95.urdf']=hashlib.sha256(robot.path.read_bytes()).hexdigest()
    (a.out/'model-sha256.json').write_text(json.dumps(assets,indent=2)+'\n')
    (a.out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');(a.out/'verification.json').write_text(json.dumps(verification,indent=2)+'\n')
    print(json.dumps({'figures':list(str(p.name) for p in a.out.glob('*.png')),'height_slices':table},indent=2))


if __name__=='__main__':main()
