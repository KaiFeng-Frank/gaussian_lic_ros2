#!/usr/bin/env python3
"""Deterministic loop-closure refinement for the discrete tracker trajectory.

Super-paper-level extension: detects the physical revisit at the end of the
CBD_Building_01 loop, derives the loop-closure constraint by registering the
end lidar submap against the start lidar submap (sensor-only, never uses the
reference trajectory), and redistributes the accumulated drift along the loop
via SE3 exp/log. Run with system python3.12 (ROS read stack).

Usage:
  /usr/bin/python3 scripts/loop_closure_refine.py \
     --bag <bag> --traj <M2.tum> --out <refined.tum> [--start-win 0,25] [--end-win 95,1e9]
"""
import argparse, sys
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot

EXTRINSIC_T = np.array([0.04165, 0.02326, -0.0284])  # lidar->imu translation (FAST-LIVO2), rotation identity

def load_tum(p):
    rows=[l.split() for l in open(p) if l.strip() and not l.startswith('#')]
    t=np.array([float(r[0]) for r in rows])
    xyz=np.array([[float(r[1]),float(r[2]),float(r[3])] for r in rows])
    quat=np.array([[float(r[4]),float(r[5]),float(r[6]),float(r[7])] for r in rows])  # x y z w
    return t,xyz,quat

def pose_mat(xyz,quat):
    T=np.eye(4); T[:3,:3]=Rot.from_quat(quat).as_matrix(); T[:3,3]=xyz; return T

def read_clouds_in_windows(bag, windows, max_pts=2000):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2 as pc2
    so=rosbag2_py.StorageOptions(uri=bag, storage_id="")
    r=rosbag2_py.SequentialReader(); r.open(so, rosbag2_py.ConverterOptions("",""))
    out=[]  # (stamp_s, Nx3 lidar points)
    t0=None
    while r.has_next():
        topic,data,_=r.read_next()
        if topic!="/livox/lidar": continue
        m=deserialize_message(data, PointCloud2)
        st=m.header.stamp.sec + m.header.stamp.nanosec*1e-9
        if t0 is None: t0=st
        rel=st-t0
        if not any(a<=rel<=b for a,b in windows): continue
        pts=pc2.read_points(m, field_names=("x","y","z"), skip_nans=True)
        arr=np.array([[p[0],p[1],p[2]] for p in pts], dtype=np.float64)
        if len(arr)==0: continue
        # range filter + random downsample
        d=np.linalg.norm(arr,axis=1); arr=arr[(d>1.0)&(d<60.0)]
        if len(arr)>max_pts:
            idx=np.linspace(0,len(arr)-1,max_pts).astype(int); arr=arr[idx]
        out.append((st, arr))
    return out, t0

def voxel_ds(pts, vox=0.3):
    keys=np.floor(pts/vox).astype(np.int64)
    _,uniq=np.unique(keys, axis=0, return_index=True)
    return pts[uniq]

def build_submap(clouds, cloud_stamps, traj_t, traj_xyz, traj_quat, center_stamp, half_window_s, vox=0.3):
    P=[]
    for st,arr in clouds:
        if abs(st-center_stamp)>half_window_s: continue
        j=np.argmin(np.abs(traj_t-st))
        if abs(traj_t[j]-st)>0.15: continue
        T=pose_mat(traj_xyz[j],traj_quat[j])
        pw=(T[:3,:3]@(arr+EXTRINSIC_T).T).T + T[:3,3]
        P.append(pw)
    if not P: return np.zeros((0,3))
    P=np.vstack(P)
    return voxel_ds(P, vox)

def icp_p2p(src, dst, iters=40, max_corr=1.0):
    """Register src onto dst. Returns 4x4 T (applied to src), inlier rmse, inlier count."""
    tree=cKDTree(dst)
    T=np.eye(4); cur=src.copy()
    last_rmse=1e9
    for _ in range(iters):
        d,idx=tree.query(cur, k=1)
        m=d<max_corr
        if m.sum()<50: break
        A=cur[m]; B=dst[idx[m]]
        ca=A.mean(0); cb=B.mean(0)
        H=(A-ca).T@(B-cb)
        U,_,Vt=np.linalg.svd(H); R=Vt.T@U.T
        if np.linalg.det(R)<0: Vt[-1]*=-1; R=Vt.T@U.T
        t=cb-R@ca
        dT=np.eye(4); dT[:3,:3]=R; dT[:3,3]=t
        T=dT@T; cur=(R@cur.T).T+t
        rmse=np.sqrt((d[m]**2).mean())
        if abs(last_rmse-rmse)<1e-5: break
        last_rmse=rmse
        max_corr=max(0.3, max_corr*0.9)
    d,idx=tree.query(cur,k=1); m=d<1.0
    return T, float(np.sqrt((d[m]**2).mean())) if m.sum() else 1e9, int(m.sum())

def se3_log(T):
    R=T[:3,:3]; t=T[:3,3]
    rv=Rot.from_matrix(R).as_rotvec(); th=np.linalg.norm(rv)
    if th<1e-8:
        return np.concatenate([t, rv])
    w=rv/th; W=np.array([[0,-w[2],w[1]],[w[2],0,-w[0]],[-w[1],w[0],0]])
    Vinv=np.eye(3)-0.5*th*W+(1-th*np.cos(th/2)/(2*np.sin(th/2)))*(W@W)
    return np.concatenate([Vinv@t, rv])

def se3_exp(xi):
    rho=xi[:3]; rv=xi[3:]; th=np.linalg.norm(rv)
    R=Rot.from_rotvec(rv).as_matrix()
    if th<1e-8:
        V=np.eye(3)
    else:
        w=rv/th; W=np.array([[0,-w[2],w[1]],[w[2],0,-w[0]],[-w[1],w[0],0]])
        V=np.eye(3)+((1-np.cos(th))/th**2)*(th*W)+((th-np.sin(th))/th**3)*(th*W)@(th*W)
        # simpler stable form:
        V=np.eye(3)+(1-np.cos(th))/th*W+(th-np.sin(th))/th*(W@W)
    T=np.eye(4); T[:3,:3]=R; T[:3,3]=V@rho; return T

def skew(v):
    return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])

def adjoint(T):
    R=T[:3,:3]; t=T[:3,3]
    Ad=np.zeros((6,6)); Ad[:3,:3]=R; Ad[:3,3:]=skew(t)@R; Ad[3:,3:]=R
    return Ad

def pgo(poses, edges, loop_weight=1.0, iters=8):
    """Sparse SE3 pose-graph GN. poses: list of 4x4 (init). edges: (i,j,Z,w).
    Anchors pose 0. Returns optimized poses."""
    from scipy.sparse import lil_matrix, csr_matrix
    from scipy.sparse.linalg import spsolve
    N=len(poses); T=[p.copy() for p in poses]
    for it in range(iters):
        rows=[]; cols=[]; vals=[]; rvec=[]
        ridx=0
        for (i,j,Z,w) in edges:
            r=se3_log(np.linalg.inv(Z)@(np.linalg.inv(T[i])@T[j]))
            Ji=-adjoint(np.linalg.inv(T[j])@T[i]); Jj=np.eye(6)
            for a_ in range(6):
                rvec.append(-w*r[a_])
                if i>0:
                    for b_ in range(6):
                        rows.append(ridx); cols.append(6*(i-1)+b_); vals.append(w*Ji[a_,b_])
                if j>0:
                    for b_ in range(6):
                        rows.append(ridx); cols.append(6*(j-1)+b_); vals.append(w*Jj[a_,b_])
                ridx+=1
        nv=6*(N-1)
        J=csr_matrix((vals,(rows,cols)), shape=(ridx,nv))
        H=(J.T@J).tolil();
        for k in range(nv): H[k,k]+=1e-6
        H=H.tocsr(); g=J.T@np.array(rvec)
        dx=spsolve(H,g)
        mx=np.max(np.abs(dx))
        for k in range(1,N):
            T[k]=T[k]@se3_exp(dx[6*(k-1):6*k])
        if mx<1e-5: break
    return T

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--bag", required=True); ap.add_argument("--traj", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--start-win", default="0,25"); ap.add_argument("--end-win", default="93,1e9")
    ap.add_argument("--vox", type=float, default=0.3)
    ap.add_argument("--mode", default="distribute", choices=["distribute","pgo"])
    ap.add_argument("--loop-weight", type=float, default=1.0)
    a=ap.parse_args()
    sw=tuple(float(x) for x in a.start_win.split(",")); ew=tuple(float(x) for x in a.end_win.split(","))
    traj_t,traj_xyz,traj_quat=load_tum(a.traj)
    print(f"trajectory: {len(traj_t)} poses, loopclose(end-start)={np.linalg.norm(traj_xyz[-1]-traj_xyz[0]):.3f}m")
    print("reading clouds in start+end windows...", flush=True)
    clouds,t0=read_clouds_in_windows(a.bag, [sw,ew])
    cs=np.array([c[0] for c in clouds])
    print(f"read {len(clouds)} clouds (start-win {sw}, end-win {ew})")
    # loop pair: end pose vs early poses
    end_idx=len(traj_t)-1
    early=int(0.25*len(traj_t))
    d=np.linalg.norm(traj_xyz[:early]-traj_xyz[end_idx],axis=1)
    start_idx=int(np.argmin(d))
    print(f"loop pair: end_idx={end_idx} (t+{traj_t[end_idx]-traj_t[0]:.1f}s) <-> start_idx={start_idx} (t+{traj_t[start_idx]-traj_t[0]:.1f}s), est-dist={d[start_idx]:.3f}m")
    start_map=build_submap(clouds,cs,traj_t,traj_xyz,traj_quat, traj_t[start_idx], (sw[1]-sw[0])/2+2.0, a.vox)
    end_map  =build_submap(clouds,cs,traj_t,traj_xyz,traj_quat, traj_t[end_idx], 12.0, a.vox)
    print(f"start_map={len(start_map)} pts, end_map={len(end_map)} pts")
    if len(start_map)<200 or len(end_map)<200:
        print("ABORT: insufficient submap overlap"); sys.exit(2)
    T_corr, rmse, ninl = icp_p2p(end_map, start_map)
    corr_t=np.linalg.norm(T_corr[:3,3]); corr_r=np.degrees(np.linalg.norm(Rot.from_matrix(T_corr[:3,:3]).as_rotvec()))
    print(f"ICP loop correction: ||t||={corr_t:.3f}m rot={corr_r:.2f}deg  inlier_rmse={rmse:.3f} inliers={ninl}")
    if rmse>0.8 or corr_t>3.0:
        print("ABORT: ICP did not converge confidently (rmse>0.8 or correction>3m) -> NOT applying"); sys.exit(3)
    out_xyz=traj_xyz.copy(); out_quat=traj_quat.copy()
    if a.mode=="distribute":
        # distribute correction over [start_idx, end_idx] by arc length (anchor <=start_idx fixed)
        seg=np.arange(start_idx, end_idx+1)
        dd=np.linalg.norm(np.diff(traj_xyz[seg],axis=0),axis=1)
        arc_seg=np.concatenate([[0],np.cumsum(dd)]); arc_seg/=max(arc_seg[-1],1e-9)
        xi=se3_log(T_corr)
        for k,i in enumerate(seg):
            Ti=se3_exp(arc_seg[k]*xi)  # 0 at start_idx -> full T_corr at end_idx
            Pnew=Ti@pose_mat(traj_xyz[i],traj_quat[i])
            out_xyz[i]=Pnew[:3,3]; out_quat[i]=Rot.from_matrix(Pnew[:3,:3]).as_quat()
    else:  # pgo
        poses=[pose_mat(traj_xyz[i],traj_quat[i]) for i in range(len(traj_t))]
        edges=[]
        for i in range(len(traj_t)-1):
            Z=np.linalg.inv(poses[i])@poses[i+1]; edges.append((i,i+1,Z,1.0))
        Zl=np.linalg.inv(poses[start_idx])@(T_corr@poses[end_idx])  # corrected start->end relative
        edges.append((start_idx,end_idx,Zl,a.loop_weight))
        print(f"PGO: {len(edges)} edges, loop_weight={a.loop_weight} ...", flush=True)
        Topt=pgo(poses, edges, a.loop_weight)
        for i in range(len(traj_t)):
            out_xyz[i]=Topt[i][:3,3]; out_quat[i]=Rot.from_matrix(Topt[i][:3,:3]).as_quat()
    with open(a.out,"w") as f:
        for i in range(len(traj_t)):
            q=out_quat[i]
            f.write(f"{traj_t[i]:.9f} {out_xyz[i,0]:.6f} {out_xyz[i,1]:.6f} {out_xyz[i,2]:.6f} {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n")
    print(f"refined loopclose(end-start)={np.linalg.norm(out_xyz[-1]-out_xyz[0]):.3f}m -> wrote {a.out}")

if __name__=="__main__":
    main()
