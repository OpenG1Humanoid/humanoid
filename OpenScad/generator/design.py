"""Single source of truth: features cut/added on the ORIGINAL OpenG1/Unitree arm meshes.
All coordinates in mm, LEFT-arm link frames (URDF). Right arm = mirror in Y."""
import numpy as np
SERVO={ # body L x W x H, shaft offset from near end, tab span (=L means no tabs), tab underside above bottom,
        # horn disc dia, horn face above case top, hub hex across-flats, hex length, screw size
 "DS3235":dict(L=40.0,W=20.0,H=38.5,off=10.0,tab=54.5,tabz=26.0,horn=20.5,htop=6.0,hex=22.0,hexlen=8.0,scr=3),
 "X10PRO":dict(L=30.0,W=10.0,H=33.5,off=7.0, tab=30.0,tabz=0.0, horn=14.5,htop=5.0,hex=16.0,hexlen=7.0,scr=3),
 "MG92B": dict(L=22.8,W=12.0,H=31.0,off=6.0, tab=32.3,tabz=15.5,horn=12.5,htop=4.0,hex=14.0,hexlen=5.0,scr=2)}
FIT=0.4
def frame(o,y,z):
    o,y,z=(np.array(v,float) for v in (o,y,z)); x=np.cross(y,z); M=np.eye(4); M[:3,0]=x; M[:3,1]=y; M[:3,2]=z; M[:3,3]=o; return M
def rotz(M,deg):  # rotate a joint frame about its own z
    a=np.radians(deg); R=np.eye(4); R[:2,:2]=[[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]]; return M@R
OPS=[]   # dict(link, kind, M, prim, tag)
def op(link,kind,M,prim,tag=""): OPS.append(dict(link=link,kind=kind,M=M,prim=prim,tag=tag))
def stack(S):
    disc0,disc1=0.6,3.6; ct=disc1+S["htop"]; return disc0,disc1,ct
def servo_host(link, M, S, tag):
    """pocket + plug (hatch of original material) + plain-bearing groove in the host link"""
    d0,d1,ct=stack(S); L,W,H,off=S["L"]+FIT,S["W"]+FIT,S["H"],S["off"]
    by0,by1=-off-FIT/2, S["L"]-off+FIT/2; cy=(S["L"]/2-off)
    ty0,ty1=cy-S["tab"]/2-FIT/2, cy+S["tab"]/2+FIT/2
    hexc=S["hex"]/np.cos(np.pi/6); hubr=max(S["horn"]/2, hexc/2)+1.2
    # plug prism (hatch) covers the servo footprint (+tabs) +4 mm, from the face to the servo case top
    m=4.0; hw=max(W/2+m, hubr+4.0)
    px0,px1,py0,py1=-hw,hw,min(by0,ty0,-hubr)-m,max(by1,ty1,hubr)+m
    op(link,"plug",M,("box",px0,px1,py0,py1,-6,ct),tag)
    op(link,"cut",M,("cyl",hubr,-0.01,ct+0.01),tag)                                   # hub + horn
    op(link,"cut",M,("box",-W/2,W/2,by0,by1,ct-0.01,ct+H+0.5),tag)                    # body
    if S["tab"]>S["L"]:
        tz=ct+H-S["tabz"]; op(link,"cut",M,("box",-W/2,W/2,ty0,ty1,tz-4.0,tz+1.0),tag)  # tab slot (tabs rest on plug side)
    op(link,"cut",M,("box",-2.6,2.6,by1-0.01,by1+3.0,-0.01,ct+H*0.5),tag)                # cable channel to the face
    # plain bearing groove for the child's ring
    rr=hexc/2+1.6; op(link,"cut",M,("ann",rr-0.3,rr+2.3,-0.01,2.0),tag)
    # plug screws (through the hatch into the host behind it)
    sd={3:(3.4,2.5,6.2),2:(2.4,1.7,4.2)}[S["scr"]]
    for sx in (-1,1):
        for sy in (py0+2.8,py1-2.8):
            x=sx*(hw-2.8)
            op(link,"cut",M,("cylat",x,sy,sd[0]/2,-6,ct),tag)          # clearance through hatch
            op(link,"cut",M,("cylat",x,sy,sd[2]/2,-6,2.6),tag)          # head counterbore at face
            op(link,"cut",M,("cylat",x,sy,sd[1]/2,ct-0.01,ct+8),tag)   # pilot behind hatch
    return dict(ct=ct,hubr=hubr,rr=rr,hexc=hexc)
def child_solid(link, M, S, angles, trim_r, tag, trim_h=60):
    """hex socket + ring + radial screws in a solid child; trims the child 0.5 mm off the joint plane"""
    hexc=S["hex"]/np.cos(np.pi/6); rr=hexc/2+1.6
    op(link,"cut",M,("cyl",trim_r,-0.5,trim_h),tag)
    op(link,"cut",M,("hex",S["hex"]+0.3,-(S["hexlen"]+0.3),0.6),tag)
    sd={3:(3.3,6.4),2:(2.3,4.4)}[S["scr"]]
    for a in angles:
        op(link,"cut",M,("rad",a,-S["hexlen"]/2-0.5,0,hexc/2+3.5,sd[0]),tag)
        op(link,"cut",M,("rad",a,-S["hexlen"]/2-0.5,hexc/2+3.0,70,sd[1]),tag)
    op(link,"ring",M,("ann",rr,rr+2.0,-0.6,1.8),tag)
def cover_drive(link, M, S, w, tag):
    """servo hub enters from OUTSIDE through the thin cover. w = cover inner-wall z (negative = child side).
    boss lives only inside the cover (clipped to the cover's hull); hub head sits under a cap of original skin"""
    hexc=S["hex"]/np.cos(np.pi/6); head=hexc+4
    op(link,"addhull",M,("cyl",hexc/2+3.5,w-0.5,-0.6),tag)            # boss: from the cover wall to 0.6 mm off the host face
    op(link,"cut",M,("hex",S["hex"]+0.25,w-3,-0.3),tag)               # hex hole through boss + skin
    op(link,"cut",M,("cyl",head/2+0.2,w-3,w+1.6),tag)                  # head counterbore (skin + 1.6 mm of boss)
    op(link,"cap",M,("cyl",head/2+0.15,w-3,w+0.1),tag)                 # cosmetic cap = the original skin disc
def cover_idler(link, M, w, tag):
    op(link,"addhull",M,("cyl",8,w-0.5,-0.6),tag)
    op(link,"cut",M,("cyl",2.65,w-3,-0.3),tag)                         # 5 mm pin runs in this hole (plain bearing)
    op(link,"cut",M,("cyl",4.7,w-3,w+1.2),tag)
    op(link,"cap",M,("cyl",4.6,w-3,w+0.1),tag)
def host_idler_hole(link,M,tag):
    op(link,"cut",M,("cyl",2.45,-0.01,7),tag)                          # pin glued/pressed into the host
# ---------------------------------------------------------------- joints
J={}
# shoulder roll: DS3235 in shoulder_pitch_link, drives roll link +X fork arm
M=rotz(frame((30.44,38,-13.831),(0,0,1),(-1,0,0)),0)
J["roll"]=servo_host("shoulder_pitch_link",M,SERVO["DS3235"],"roll")
Mc=frame((30.44,0,0),(0,0,1),(-1,0,0))
child_solid("shoulder_roll_link",Mc,SERVO["DS3235"],[30,150],42,"roll",trim_h=61.85)
# shoulder yaw: X10 PRO in shoulder_roll_link bottom, drives shoulder_yaw_link top
YAW_ANG=0
M=rotz(frame((0,6.24,-103.2),(0,1,0),(0,0,1)),YAW_ANG)
J["yaw"]=servo_host("shoulder_roll_link",M,SERVO["X10PRO"],"yaw")
child_solid("shoulder_yaw_link",rotz(frame((0,0,0),(0,1,0),(0,0,1)),YAW_ANG),SERVO["X10PRO"],[0,180],24.6,"yaw",trim_h=40)
# elbow: X10 PRO in shoulder_yaw_link, drives elbow_link +Y plate
ELB_ANG=0
M=rotz(frame((15.783,20.48,-80.518),(0,0,1),(0,-1,0)),ELB_ANG)
J["elbow"]=servo_host("shoulder_yaw_link",M,SERVO["X10PRO"],"elbow")
op("shoulder_yaw_link","cut",M,("cyl",34,-6,0.01),"elbow")   # G1 rim that stood 2 mm proud of the joint plane
child_solid("elbow_link",rotz(frame((0,20.48,0),(0,0,1),(0,-1,0)),ELB_ANG),SERVO["X10PRO"],[90,150],40,"elbow")
# wrist roll: MG92B in elbow_link forearm, drives the wrist_roll cover (thin) via an inner boss
WRR_ANG=0
M=rotz(frame((99.5,1.888,-10),(0,0,-1),(-1,0,0)),WRR_ANG)
op("elbow_link","cut",frame((100,1.888,-10),(0,0,-1),(-1,0,0)),("cyl",31,-3,0.5),"wroll")   # face 0.5 mm back
J["wroll"]=servo_host("elbow_link",M,SERVO["MG92B"],"wroll")
Mc=rotz(frame((-0.5,0,0),(0,0,-1),(-1,0,0)),WRR_ANG)   # in wrist_roll frame, plane 0.5 mm in front of the cover
S=SERVO["MG92B"]; hexc=S["hex"]/np.cos(np.pi/6)
op("wrist_roll_link","addhull",Mc,("cyl",hexc/2+3.5,-8.0,-0.5),"wroll")      # inner boss on the cover bottom
op("wrist_roll_link","cut",Mc,("hex",S["hex"]+0.3,-(S["hexlen"]+0.8),0.6),"wroll")
for a in (90,270):
    op("wrist_roll_link","cut",Mc,("rad",a,-S["hexlen"]/2-0.5,0,hexc/2+4,2.3),"wroll")
op("wrist_roll_link","ring",Mc,("ann",hexc/2+1.6,hexc/2+3.6,-0.6,1.3),"wroll")
# wrist pitch: MG92B in wrist_pitch_link (+Y face y=20.5), drives the wrist_roll cover (+Y wall at 28.5); idler on -Y
M=frame((0,20.5,0),(1,0,0),(0,-1,0))                       # into the pitch link = -Y ; servo body runs +X
J["wpitch"]=servo_host("wrist_pitch_link",M,SERVO["MG92B"],"wpitch")
Mw=frame((38,20.5,0),(1,0,0),(0,-1,0))                     # same joint frame in the wrist_roll frame (pitch origin at x=38)
cover_drive("wrist_roll_link",Mw,SERVO["MG92B"],-8.0,"wpitch")   # cover inner wall y=28.5 -> z=-8
Mi=frame((0,-22.5,0),(1,0,0),(0,1,0)); host_idler_hole("wrist_pitch_link",Mi,"wpitch_idler")
cover_idler("wrist_roll_link",frame((38,-22.5,0),(1,0,0),(0,1,0)),-6.5,"wpitch_idler")
# wrist yaw: MG92B in wrist_pitch_link (+Z face z=20.5 at x=46), drives the wrist_yaw cover (+Z wall 28.5); idler -Z
M=frame((46,0,20.5),(-1,0,0),(0,0,-1))                     # servo body runs towards -X (away from the front)
J["wyaw"]=servo_host("wrist_pitch_link",M,SERVO["MG92B"],"wyaw")
cover_drive("wrist_yaw_link",frame((0,0,20.5),(-1,0,0),(0,0,-1)),SERVO["MG92B"],-8.0,"wyaw")
host_idler_hole("wrist_pitch_link",frame((46,0,-22.5),(-1,0,0),(0,0,1)),"wyaw_idler")
cover_idler("wrist_yaw_link",frame((0,0,-22.5),(-1,0,0),(0,0,1)),-6.5,"wyaw_idler")
# torso interface on the shoulder pitch link (y=0 face): locating recess, 6 heat-set inserts PCD 40, cable hole
Mt=frame((0,0,0),(1,0,0),(0,1,0))
op("shoulder_pitch_link","cut",Mt,("cyl",25.3,-1,1.2),"torso")
op("shoulder_pitch_link","cut",Mt,("cyl",5.0,-1,15),"torso")
for i in range(6):
    a=np.radians(30+60*i); op("shoulder_pitch_link","cut",Mt,("cylat",20*np.cos(a),20*np.sin(a),2.0,-1,7),"torso")
LINKS=["shoulder_pitch_link","shoulder_roll_link","shoulder_yaw_link","elbow_link","wrist_roll_link","wrist_pitch_link","wrist_yaw_link"]
COVERS={"wrist_roll_link","wrist_yaw_link"}
HAND_TRIM_X=123.0   # wrist-roll-frame x where the rubber hand is cut; palm part is merged into the wrist_yaw cover

# ======================================================================= TORSO (shoulder pitch)
def rpy(r,p,y):
    cr,sr,cp,sp,cy,sy=np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
    Rx=np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]]);Ry=np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]]);Rz=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    M=np.eye(4); M[:3,:3]=Rz@Ry@Rx; return M
def trans(x,y,z): M=np.eye(4); M[:3,3]=[x,y,z]; return M
T_TORSO_L=trans(3.9563,100.22,247.78)@rpy(0.27931,5.4949e-05,-0.00019159)    # torso <- left shoulder_pitch_link
T_TORSO_R=trans(3.9563,-100.22,247.78)@rpy(-0.27931,5.4949e-05,0.00019159)   # torso <- right shoulder_pitch_link
PITCH_ANG=90     # servo long-axis angle about the pitch axis (set by gen/torso_check.py)
def pitch_frames(ang=PITCH_ANG):
    JL=rotz(frame((0,-0.6,0),(0,0,1),(0,-1,0)),ang)          # in LEFT pitch-link frame, z into the torso
    yl=JL[:3,1]; JR=frame((0,0.6,0),(yl[0],-yl[1],yl[2]),(0,1,0))   # mirror image in the RIGHT pitch-link frame
    return JL,JR
# remove the old torso-flange interface on the pitch link (replaced by a servo joint)
OPS[:]=[o for o in OPS if not (o["link"]=="shoulder_pitch_link" and o["tag"]=="torso")]
JL,JR=pitch_frames()
child_solid("shoulder_pitch_link",JL,SERVO["DS3235"],[0,180],30,"pitch",trim_h=0.4)
J["pitchL"]=servo_host("torso_link",T_TORSO_L@JL,SERVO["DS3235"],"pitchL")
J["pitchR"]=servo_host("torso_link",T_TORSO_R@JR,SERVO["DS3235"],"pitchR")
TORSO_BODIES=[f"torso_body{k:02d}.stl" for k in range(14)]   # bodies 14-19 (tiny inside-out logo bits) left out
