#!/usr/bin/env python3
"""One-step cached-path feasibility on recorded inputs; NOT closed-loop simulation.
Run with ROS system Python after decoding the trusted local bag into decoded.pkl.
"""
import argparse,bisect,json,math,pickle,subprocess
from collections import Counter
from pathlib import Path
import numpy as np


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def previous(rows,t):
    i=bisect.bisect_right([r[0] for r in rows],t)-1
    if i<0:
        raise ValueError('No recorded map/transform before query')
    return rows[i]


def poses(msg):
    return [[p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)] for p in msg.poses]


def directions(path):
    p=np.asarray(path);delta=np.diff(p[:,:2],axis=0)
    signs=np.where(delta[:,0]*np.cos(p[:-1,2])+delta[:,1]*np.sin(p[:-1,2])<0,-1,1)
    return [int(v) for i,v in enumerate(signs) if not i or v!=signs[i-1]]


def independent_check(path, grids, half_body):
    """Dense square/cell SAT cross-check, independent of C++ convex sweeps."""
    from scipy.spatial import cKDTree
    prepared=[]
    for msg,tr in grids:
        md=msg.metadata;costs=np.asarray(msg.data).reshape(md.size_y,md.size_x)
        yy,xx=np.where((costs==254)|(costs==255))
        cells=np.column_stack([md.origin.position.x+(xx+.5)*md.resolution,
                               md.origin.position.y+(yy+.5)*md.resolution])
        prepared.append((cells,cKDTree(cells),md.resolution/2,tr))
    collisions=0;samples=0
    for i,p in enumerate(path):
        a=p if not i else path[i-1];delta=np.asarray(p)-a
        delta[2]=math.atan2(math.sin(delta[2]),math.cos(delta[2]))
        count=max(1,math.ceil(math.hypot(*delta[:2])/.002),math.ceil(abs(delta[2])/.005))
        for j in range(1,count+1):
            q=a+delta*j/count;samples+=1
            for cells,tree,half,tr in prepared:
                tx,ty,h=tr;c,ss=math.cos(h),math.sin(h)
                center=np.array([tx+c*q[0]-ss*q[1],ty+ss*q[0]+c*q[1]])
                angle=q[2]+h;c,ss=math.cos(angle),math.sin(angle)
                ids=tree.query_ball_point(center,half_body*math.sqrt(2)+half*math.sqrt(2)+1e-6)
                if not ids:
                    continue
                dxy=cells[ids]-center;body=dxy@np.array([[c,-ss],[ss,c]])
                diagonal=abs(c)+abs(ss)
                hit=(np.max(abs(body),axis=1)<=half_body+half*diagonal)&(
                    np.max(abs(dxy),axis=1)<=half_body*diagonal+half)
                collisions+=int(np.any(hit))
    return samples,collisions


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag',type=Path);parser.add_argument('--executable',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    with (args.bag/'decoded.pkl').open('rb') as f:
        data=pickle.load(f)  # Trusted local files only.
    transforms=[]
    for t,msg in data['/tf']:
        for tr in msg.transforms:
            if tr.header.frame_id=='map' and tr.child_frame_id=='odom':
                transforms.append((t,[tr.transform.translation.x,tr.transform.translation.y,yaw(tr.transform.rotation)]))
    footprints=[]
    for topic in ['/global_costmap/published_footprint','/local_costmap/published_footprint']:
        sides=[]
        for _,msg in data[topic]:
            p=msg.polygon.points
            if len(p)!=4:
                raise ValueError('Replay supports recorded square footprints only')
            sides.extend(math.hypot(p[i].x-p[(i+1)%4].x,p[i].y-p[(i+1)%4].y) for i in range(4))
        if max(sides)-min(sides)>1e-4:
            raise ValueError('Changing/non-square footprint')
        footprints.append(sum(sides)/len(sides))
    if abs(footprints[0]-footprints[1])>1e-4:
        raise ValueError('Global/local footprint mismatch')
    half=round(footprints[0]/2,5)
    plans=[(t,msg) for t,msg in data['/PositionOnlyGrid/validated_path'] if msg.poses]
    reports=[]
    for i,((old_t,old),(t,new)) in enumerate(zip(plans[:-1],plans[1:])):
        cached,current=poses(old),poses(new);start=current[0];goal=current[-1]
        if math.hypot(goal[0]-cached[-1][0],goal[1]-cached[-1][1])>1e-6:
            continue
        gt,glob=previous(data['/global_costmap/costmap_raw'],t)
        lt,local=previous(data['/local_costmap/costmap_raw'],t)
        _,(tx,ty,h)=previous(transforms,t);c,s=math.cos(h),math.sin(h)
        inverse=[-c*tx-s*ty,s*tx-c*ty,-h]
        parts=['2']
        for name,msg,tr,partial in [('global',glob,[0,0,0],0),('local',local,inverse,1)]:
            md=msg.metadata
            if abs(yaw(md.origin.orientation))>1e-8:
                raise ValueError('Rotated grids unsupported')
            parts.append(' '.join(map(str,[name,md.size_x,md.size_y,md.resolution,
                md.origin.position.x,md.origin.position.y,*tr,partial])))
            parts.append(' '.join(map(str,msg.data)))
        parts.extend(['4',f'{half} {half}',f'{-half} {half}',f'{-half} {-half}',f'{half} {-half}',
                      ' '.join(map(str,[*start,*goal])),str(len(cached))])
        parts.extend(' '.join(map(str,p)) for p in cached)
        prefix=args.output/f'{i:03d}_t{t:.3f}';fixture=Path(str(prefix)+'.fixture')
        fixture.write_text('\n'.join(parts)+'\n')
        run=subprocess.run([str(args.executable.resolve()),str(fixture),str(prefix),'1','reuse'],
                           capture_output=True,text=True)
        if run.returncode not in (0,2):
            raise RuntimeError(run.stderr)
        report=json.loads(Path(str(prefix)+'.json').read_text())
        report.update(t=t,old_time=old_t,old_age=t-old_t,global_age=t-gt,local_age=t-lt,
                      old_directions=directions(cached),new_directions=directions(current))
        report['recorded_direction_strategy_changed']=report['old_directions']!=report['new_directions']
        if report['validated']:
            reused=np.loadtxt(Path(str(prefix)+'.csv'),delimiter=',',skiprows=1,ndmin=2)
            report['reuse_directions']=directions(reused)
            samples,collisions=independent_check(reused,[(glob,[0,0,0]),(local,inverse)],half)
            report['independent_samples']=samples
            report['independent_collisions']=collisions
            if collisions:
                raise RuntimeError('Independent dense SAT found an unsafe reuse path')
        reports.append(report)
        if 74<t<105:
            print(json.dumps({k:report[k] for k in ['t','old_directions','new_directions','reuse_reason','elapsed','nearest_index','join_index','connection_length']}),flush=True)
    (args.output/'summary.json').write_text(json.dumps(reports,indent=2)+'\n')
    print('REASONS',dict(Counter(r['reuse_reason'] for r in reports)))
    print('CHECK_SECONDS max',max(r['elapsed'] for r in reports))
    print('This evaluates adjacent recorded plans; robot motion is held fixed, not resimulated.')


if __name__=='__main__':
    main()
