#!/usr/bin/env python3
"""Offline recorded-grid regression: no ROS nodes, map edits or motion commands.
Only load trusted local decoded.pkl files (Python pickle executes code).
"""
import argparse
import bisect
import json
import math
from pathlib import Path
import pickle
import subprocess
import numpy as np


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def previous(rows,t):
    i=bisect.bisect_right([p[0] for p in rows],t)-1
    if i<0:
        raise ValueError(f'No recorded sample before {t}')
    return rows[i]


def independent_check(path, grids):
    """Independent closed-form square/cell SAT, densely interpolated at <=1 mm.

    Unlike the C++ convex hull sweep, this checks individual interpolated poses;
    it is a useful cross-check, not a continuous-motion proof on its own.
    """
    from scipy.spatial import cKDTree
    prepared=[]
    for msg,tr in grids:
        values=np.asarray(msg.data).reshape(msg.info.height,msg.info.width)
        yy,xx=np.where((values==100)|(values<0))
        cells=np.column_stack([msg.info.origin.position.x+(xx+.5)*msg.info.resolution,
                               msg.info.origin.position.y+(yy+.5)*msg.info.resolution])
        prepared.append((cells,cKDTree(cells),msg.info.resolution/2,tr))
    samples=[]
    for i,p in enumerate(path):
        if not i:
            samples.append(p)
            continue
        a=path[i-1];delta=np.asarray(p)-a
        delta[2]=math.atan2(math.sin(delta[2]),math.cos(delta[2]))
        count=max(1,math.ceil(math.hypot(*delta[:2])/.001),math.ceil(abs(delta[2])/.002))
        samples.extend(np.asarray(a)+delta*j/count for j in range(1,count+1))
    collisions=0;minimum=math.inf
    for p in samples:
        for cells,tree,half,tr in prepared:
            tx,ty,h=tr;c,ss=math.cos(h),math.sin(h)
            center=np.array([tx+c*p[0]-ss*p[1],ty+ss*p[0]+c*p[1]])
            a=p[2]+h;c,ss=math.cos(a),math.sin(a);diagonal=abs(c)+abs(ss)
            ids=tree.query_ball_point(center,.27*math.sqrt(2)+half*math.sqrt(2)+.1)
            if not ids:
                continue
            delta=cells[ids]-center
            body=delta@np.array([[c,-ss],[ss,c]])
            gap=np.maximum(np.max(np.abs(body),axis=1)-(.27+half*diagonal),
                           np.max(np.abs(delta),axis=1)-(.27*diagonal+half))
            minimum=min(minimum,float(gap.min()))
            collisions+=int(np.any(gap<=0))
    return {'independent_sampled_collisions':collisions,'independent_samples':len(samples),
            'sampled_min_separating_gap_m':minimum if math.isfinite(minimum) else None}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag',type=Path)
    parser.add_argument('--executable',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--times',nargs='+',type=float,default=[60,63,65,67,70,72.8,73.63])
    parser.add_argument('--budget',type=float,default=1.)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    with (args.bag/'decoded.pkl').open('rb') as f:
        data=pickle.load(f)
    transforms=[]
    for t,msg in data['/tf']:
        for tr in msg.transforms:
            if tr.header.frame_id=='map' and tr.child_frame_id=='odom':
                transforms.append((t,(tr.transform.translation.x,tr.transform.translation.y,yaw(tr.transform.rotation))))
    summaries=[]
    for t in args.times:
        gt,glob=previous(data['/global_costmap/costmap'],t)
        lt,local=previous(data['/local_costmap/costmap'],t)
        tt,(tx,ty,h)=previous(transforms,t)
        ot,od=previous(data['/odom'],t);p=od.pose.pose
        start=[tx+math.cos(h)*p.position.x-math.sin(h)*p.position.y,
               ty+math.sin(h)*p.position.x+math.cos(h)*p.position.y,yaw(p.orientation)+h]
        pt,plan=previous(data['/plan'],t);goal=plan.poses[-1].pose
        target=[goal.position.x,goal.position.y,0.]
        # Current map->odom transform applied to the recorded odom grid.
        inverse=[-math.cos(h)*tx-math.sin(h)*ty,math.sin(h)*tx-math.cos(h)*ty,-h]
        parts=['2']
        for name,msg,transform,partial in [('global',glob,[0,0,0],0),('local',local,inverse,1)]:
            if abs(yaw(msg.info.origin.orientation))>1e-8:
                raise ValueError('Rotated grid unsupported')
            raw=np.asarray(msg.data,dtype=int)
            # OccupancyGrid projection loses exact soft costs; lethal/unknown
            # membership is exact. This replay proves geometry, not critic scores.
            raw=np.where(raw<0,255,np.where(raw==100,254,np.where(raw==99,253,
                np.where(raw==0,0,1+(raw-1)*251//97))))
            parts.append(' '.join(map(str,[name,msg.info.width,msg.info.height,msg.info.resolution,
                msg.info.origin.position.x,msg.info.origin.position.y,*transform,partial])))
            parts.append(' '.join(map(str,raw.tolist())))
        # Ver13's verified padded footprint is +/-0.27 m. Its published world
        # polygon cannot be inverted exactly using asynchronously recorded odom.
        # Verify its edge lengths rather than infer noisy body coordinates.
        ft,fp=previous(data['/local_costmap/published_footprint'],t)
        points=fp.polygon.points
        edges=[math.hypot(points[i].x-points[(i+1)%len(points)].x,
                          points[i].y-points[(i+1)%len(points)].y) for i in range(len(points))]
        if len(points)!=4 or any(abs(edge-.54)>.002 for edge in edges):
            raise ValueError('Replay expects the verified Ver13 padded 0.54 m square')
        vertices=[[.27,.27],[-.27,.27],[-.27,-.27],[.27,-.27]]
        parts.append(str(len(vertices)));parts.extend(' '.join(map(str,v)) for v in vertices)
        parts.append(' '.join(map(str,[*start,*target])))
        reference=[[q.pose.position.x,q.pose.position.y,yaw(q.pose.orientation)] for q in plan.poses]
        parts.append(str(len(reference)));parts.extend(' '.join(map(str,p)) for p in reference)
        prefix=args.output/f't{t:g}'
        # Preserve decimal times in filenames instead of Path.with_suffix truncation.
        fixture=Path(str(prefix)+'.fixture');fixture.write_text('\n'.join(parts)+'\n')
        run=subprocess.run([str(args.executable.resolve()),str(fixture),str(prefix),str(args.budget)],capture_output=True,text=True)
        if run.returncode not in (0,2):
            raise RuntimeError(run.stderr)
        report=json.loads(Path(str(prefix)+'.json').read_text())
        if report['validated']:
            path=np.loadtxt(Path(str(prefix)+'.csv'),delimiter=',',skiprows=1,ndmin=2)
            report.update(independent_check(path,[(glob,[0,0,0]),(local,inverse)]))
            if report['independent_sampled_collisions']:
                raise RuntimeError('Independent SAT found an unsafe output path')
        report.update(t=t,start=start,goal=target,global_age=t-gt,local_age=t-lt,plan_time=pt)
        summaries.append(report);print(json.dumps(report,ensure_ascii=False),flush=True)
    (args.output/'summary.json').write_text(json.dumps(summaries,indent=2)+'\n')


if __name__=='__main__':
    main()
