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
def servo_host(link, M, S, tag, bearing=None):
    """pocket + plug (hatch of original material) + plain-bearing groove in the host link.
    bearing: a 68xx ring bearing round the output instead of the groove (seat in the hatch)"""
    d0,d1,ct=stack(S); L,W,H,off=S["L"]+FIT,S["W"]+FIT,S["H"],S["off"]
    by0,by1=-off-FIT/2, S["L"]-off+FIT/2; cy=(S["L"]/2-off)
    ty0,ty1=cy-S["tab"]/2-FIT/2, cy+S["tab"]/2+FIT/2
    hexc=S["hex"]/np.cos(np.pi/6); hubr=max(S["horn"]/2, hexc/2)+1.2
    # plug prism (hatch) covers the servo footprint (+tabs) +4 mm, from the face to the servo case top
    m=4.0; hw=max(W/2+m, hubr+4.0, (bearing["od"]/2+3.0) if bearing else 0)
    px0,px1,py0,py1=-hw,hw,min(by0,ty0,-hubr,-(bearing["od"]/2+3.0) if bearing else 0)-m*(0 if bearing else 1),max(by1,ty1,hubr,(bearing["od"]/2+3.0) if bearing else 0)+m*(0 if bearing else 1)
    op(link,"plug",M,("box",px0,px1,py0,py1,-6,ct),tag)
    op(link,"cut",M,("cyl",hubr,-0.01,ct+0.01),tag)                                   # hub + horn
    op(link,"cut",M,("box",-W/2,W/2,by0,by1,ct-0.01,ct+H+0.5),tag)                    # body
    if S["tab"]>S["L"]:
        tz=ct+H-S["tabz"]; op(link,"cut",M,("box",-W/2,W/2,ty0,ty1,tz-4.0,tz+1.0),tag)  # tab slot (tabs rest on plug side)
    op(link,"cut",M,("box",-2.6,2.6,by1-0.01,by1+3.0,-0.01,ct+H*0.5),tag)                # cable channel to the face
    # plain bearing groove for the child's ring, or a ring-bearing seat
    rr=hexc/2+1.6
    if bearing:
        op(link,"cut",M,("cyl",bearing["od"]/2+0.05,-0.01,bearing["w"]),tag)                         # 68xx seat (press fit)
        op(link,"cut",M,("cyl",max(S["horn"]/2,hubr-0.8)+0.8,bearing["w"]-0.01,ct+0.01),tag)          # hub + horn behind it
    else:
        op(link,"cut",M,("ann",rr-0.3,rr+2.3,-0.01,2.0),tag)
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
# ---------------------------------------------------------------- servo cartridges (round actuator modules)
# Each servo joint (except the two small wrist ones) now takes a CARTRIDGE: a round printed housing
# with the servo inside, that slides into a round bore in the host link and is screwed in. A thin
# ring bearing in the cartridge nose carries the child's spigot, so every joint has two contacts:
# the servo output and the bearing. Any future motor only has to fit the same cartridge outline.
# Frame: the joint frame M of each joint (z into the host, joint plane z = 0, child on z < 0).
BEAR={"6807":dict(id=35.0,od=47.0,w=7.0),"6805":dict(id=25.0,od=37.0,w=7.0),"608":dict(id=8.0,od=22.0,w=7.0)}
SERVO_BEAR={"DS3235":"6807","X10PRO":"6805","MG92B":"6805"}
CART={}            # tag -> geometry for emit.py
SPIGOT_ROOT=4.0    # how far the child's spigot ring runs down into the child (mm below the joint plane) (printed cartridge, cap, bearings)
M25=dict(clr=2.9,head=5.2,head_h=2.2,ins=3.6,ins_l=5.5)     # M2.5 screws into heat-set inserts
M3=dict(clr=3.4,head=6.0,head_h=3.0,ins=4.0,ins_l=6.0)
def cart_geom(S, min_r=0.0):
    d0,d1,ct=stack(S); W=S["W"]+FIT; by0,by1=-S["off"]-FIT/2, S["L"]-S["off"]+FIT/2
    hexc=S["hex"]/np.cos(np.pi/6); hubr=max(S["horn"]/2,hexc/2)+1.2; hr=hubr-0.8
    B=BEAR[SERVO_BEAR[[k for k,v in SERVO.items() if v is S][0]]]
    need=max(np.hypot(W/2,by1),np.hypot(W/2,by0))           # servo body corner (mounting tabs removed)
    Rc=max(need+0.3+1.6, B["od"]/2+2.0, min_r)
    return dict(d0=d0,d1=d1,ct=ct,W=W,by0=by0,by1=by1,H=S["H"],hr=hr,r_in=max(S["horn"]/2,hr)+0.8,r_nose=B["od"]/2+3.0,
                need=need,Rc=Rc,B=B,zr=ct+S["H"]+0.2)
def cartridge_joint(link, M, S, tag, lug_angles=(90,210,330), flat=None, mode="front", host_len=None,
                    sleeve=None, min_r=0.0, cable_ang=90, cap_lug_angles=None):
    """host-side cuts for a cartridge. mode 'front': inserted from the joint face, held by 3 lugs
    (M2.5 into heat-set inserts in the host face). mode 'rear' (fork joints): inserted from the far
    face, a 608 far bearing in the rear cap, held by the cap's lugs on the far face."""
    g=cart_geom(S,min_r); Rc=g["Rc"]; Rb=Rc+0.2; B=g["B"]
    zf=0.0 if mode=="front" else 1.0            # cartridge front face (rear mode: 1 mm host lip holds the bearing in)
    g.update(tag=tag,link=link,M=M,mode=mode,zf=zf,flat=flat,lug_angles=lug_angles,cable_ang=cable_ang,
             host_len=host_len,cap_lug_angles=cap_lug_angles or ())
    shape=lambda r,z0,z1: ("dcyl",r,z0,z1,flat[0]+(r-Rc),flat[1]) if flat else ("cyl",r,z0,z1)
    if sleeve:                                   # host too thin here: a sleeve + face flange round the bore
        ro,rf,zf0,zf1,ztop=sleeve
        op(link,"add",M,("cyl",ro,-0.0,ztop),tag)
        op(link,"add",M,("cyl",rf,-0.0,zf0),tag)
        op(link,"add",M,("cone",rf,ro,zf0,zf1),tag)
    if mode=="front":
        zcap=g["zr"]+3.0
        op(link,"cut",M,shape(Rb,-0.01,zcap+0.3),tag)                                   # the bore
        for a in lug_angles:                                                            # lug pockets + inserts
            op(link,"cut",M,("boxr",a,Rc-2.0,Rc+5.6,-4.4,4.4,-0.01,4.0),tag)
            r=Rc+2.4; op(link,"cut",M,("cylat",r*np.cos(np.radians(a)),r*np.sin(np.radians(a)),M25["ins"]/2,3.9,4.0+M25["ins_l"]),tag)
        op(link,"cut",M,("boxr",cable_ang,Rb-1.0,90.0,-2.6,2.6,-0.01,3.0),tag)          # cable groove out over the face
    else:
        L=host_len; rn=g["r_nose"]
        op(link,"cut",M,("cyl",B["od"]/2-1.5,-0.01,zf+0.01),tag)                        # front lip hole
        op(link,"cut",M,("cyl",rn+0.2,zf,g["ct"]-0.3),tag)                              # narrow nose bore (bearing only)
        op(link,"cut",M,shape(Rb,g["ct"]-0.3,L+0.01),tag)                               # full bore, loaded from the far face
        fl2=("dcyl",Rb+1.5,L-3.0,L+0.01,flat[0]+1.7,flat[1]) if flat else ("cyl",Rb+1.5,L-3.0,L+0.01)
        op(link,"cut",M,fl2,tag)                                                        # cap flange counterbore (covers the rounded edge)
        for a in cap_lug_angles:
            op(link,"cut",M,("boxr",a,Rc-2.0,Rc+5.6,-4.4,4.4,L-4.0,L+0.01),tag)
            r=Rc+2.4; op(link,"cut",M,("cylat",r*np.cos(np.radians(a)),r*np.sin(np.radians(a)),M3["ins"]/2,L-4.0-M3["ins_l"],L-3.9),tag)
        op(link,"cut",M,("boxr",cable_ang,Rb-1.0,90.0,-2.6,2.6,L-3.0,L+0.01),tag)       # cable groove out over the far face
    CART[tag]=g
    return g
def child_cartridge(link, M, S, angles, trim_r, tag, trim_h=60, zf=0.0, trim=True, boss_depth=0.0):
    """child side: hex socket + radial screws (as before) + a spigot that runs in the cartridge bearing"""
    g=cart_geom(S); B=g["B"]; hexc=S["hex"]/np.cos(np.pi/6)
    if trim: op(link,"cut",M,("cyl",trim_r,-0.5,trim_h),tag)
    if boss_depth:   # solid boss under the spigot where the child is hollow or recessed (kept inside the link's outline)
        op(link,"addhull",M,("cyl",B["id"]/2+2.5,-boss_depth,-0.5),tag)
    op(link,"cut",M,("hex",S["hex"]+0.3,-(S["hexlen"]+0.3),0.6),tag)
    sd={3:(3.3,6.4),2:(2.3,4.4)}[S["scr"]]
    for a in angles:
        op(link,"cut",M,("rad",a,-S["hexlen"]/2-0.5,0,hexc/2+3.5,sd[0]),tag)
        op(link,"cut",M,("rad",a,-S["hexlen"]/2-0.5,hexc/2+3.0,70,sd[1]),tag)
    # spigot (bearing inner race) + shoulder; both run 3.5 mm down into the solid child so they print
    # as part of it (the child itself is trimmed 0.5 mm back from the joint plane)
    op(link,"ring",M,("ann",g["hr"]+0.3,B["id"]/2-0.05,-SPIGOT_ROOT,zf+B["w"]-0.3),tag)
    op(link,"ring",M,("ann",g["hr"]+0.3,B["id"]/2+1.2,-SPIGOT_ROOT,zf-0.05),tag)
def fork_far_pin(link, M, tag, z_face, arm_t):
    """far arm of a fork: hole for an 8 x 16 mm steel dowel, pressed in from outside, runs in the 608"""
    op(link,"cut",M,("cyl",3.95,z_face-0.01,z_face+arm_t+1.0),tag)
# ---------------------------------------------------------------- joints
J={}
# shoulder roll: DS3235 in shoulder_pitch_link, drives roll link +X fork arm
# rear-loaded cartridge (the fork's two arms block the joint face): bore right through the pitch link,
# 6807 in the cartridge nose, 608 in the rear cap on the far face, 8 mm dowel in the fork's far arm.
# Flat on the torso side clears the shoulder-pitch hex socket.
M=rotz(frame((30.44,38,-13.831),(0,0,1),(-1,0,0)),0)
ROLL_LEN=60.88                     # pitch-link thickness along the roll axis (x +30.44 .. -30.44)
J["roll"]=cartridge_joint("shoulder_pitch_link",M,SERVO["DS3235"],"roll",mode="rear",host_len=ROLL_LEN,
                          flat=(28.6,0.0),cap_lug_angles=(50.0,72.0),cable_ang=-90)
Mc=frame((30.44,0,0),(0,0,1),(-1,0,0))
child_cartridge("shoulder_roll_link",Mc,SERVO["DS3235"],[30,150],42,"roll",trim_h=61.85,zf=1.0)
fork_far_pin("shoulder_roll_link",Mc,"roll_far",ROLL_LEN+0.5,9.6)
# shoulder yaw: X10 PRO in shoulder_roll_link bottom, drives shoulder_yaw_link top
YAW_ANG=0
M=rotz(frame((0,6.24,-103.2),(0,1,0),(0,0,1)),YAW_ANG)
# the upper arm is a 44-48 mm tube here, the X10 Pro cartridge is 51.5 mm: 2 mm sleeve + face flange
_gy=cart_geom(SERVO["X10PRO"])
J["yaw"]=cartridge_joint("shoulder_roll_link",M,SERVO["X10PRO"],"yaw",lug_angles=(30,150,270),
                         sleeve=(_gy["Rc"]+2.2,_gy["Rc"]+7.8,9.0,15.0,_gy["zr"]+5.0),cable_ang=90)
child_cartridge("shoulder_yaw_link",rotz(frame((0,0,0),(0,1,0),(0,0,1)),YAW_ANG),SERVO["X10PRO"],[0,180],24.6,"yaw",trim_h=40)
# elbow: X10 PRO in shoulder_yaw_link, drives elbow_link +Y plate
ELB_ANG=0
M=rotz(frame((15.783,20.48,-80.518),(0,0,1),(0,-1,0)),ELB_ANG)
J["elbow"]=cartridge_joint("shoulder_yaw_link",M,SERVO["X10PRO"],"elbow",lug_angles=(165,210,255),cable_ang=195)
op("shoulder_yaw_link","cut",M,("cyl",34,-6,0.01),"elbow")   # G1 rim that stood 2 mm proud of the joint plane
child_cartridge("elbow_link",rotz(frame((0,20.48,0),(0,0,1),(0,-1,0)),ELB_ANG),SERVO["X10PRO"],[90,150],40,"elbow")
# wrist roll: MG92B in elbow_link forearm, drives the wrist_roll cover (thin) via an inner boss
WRR_ANG=0
M=rotz(frame((99.5,1.888,-10),(0,0,-1),(-1,0,0)),WRR_ANG)
op("elbow_link","cut",frame((100,1.888,-10),(0,0,-1),(-1,0,0)),("cyl",31,-3,0.5),"wroll")   # face 0.5 mm back
J["wroll"]=cartridge_joint("elbow_link",M,SERVO["MG92B"],"wroll",lug_angles=(30,150,270),cable_ang=90,min_r=22.5)   # room for the cable groove outside the 6805
Mc=rotz(frame((-0.5,0,0),(0,0,-1),(-1,0,0)),WRR_ANG)   # in wrist_roll frame, plane 0.5 mm in front of the cover
S=SERVO["MG92B"]; hexc=S["hex"]/np.cos(np.pi/6)
op("wrist_roll_link","addhull",Mc,("cyl",BEAR["6805"]["id"]/2+2.5,-8.0,-0.5),"wroll")   # inner boss on the cover bottom, wide enough to carry the spigot
op("wrist_roll_link","cut",Mc,("hex",S["hex"]+0.3,-(S["hexlen"]+0.8),0.6),"wroll")
for a in (90,270):
    op("wrist_roll_link","cut",Mc,("rad",a,-S["hexlen"]/2-0.5,0,hexc/2+4,2.3),"wroll")
_gw=cart_geom(S); _Bw=_gw["B"]
op("wrist_roll_link","ring",Mc,("ann",_gw["hr"]+0.3,_Bw["id"]/2-0.05,-SPIGOT_ROOT,_Bw["w"]-0.3),"wroll")    # spigot in the 6805
op("wrist_roll_link","ring",Mc,("ann",_gw["hr"]+0.3,_Bw["id"]/2+1.2,-SPIGOT_ROOT,-0.05),"wroll")
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
child_cartridge("shoulder_pitch_link",JL,SERVO["DS3235"],[0,180],30,"pitch",trim_h=0.4,boss_depth=9.0)
# Shoulder pitch stays a built-in servo: a 68 mm DS3235 cartridge cannot pass the torso's 62 mm shoulder
# socket, and from inside the torso the 2020 top rail and crossbars block it. It still gets the 6807
# ring bearing round its output (seat in the servo hatch), so this joint also has two contacts.
J["pitchL"]=servo_host("torso_link",T_TORSO_L@JL,SERVO["DS3235"],"pitchL",bearing=BEAR["6807"])
J["pitchR"]=servo_host("torso_link",T_TORSO_R@JR,SERVO["DS3235"],"pitchR",bearing=BEAR["6807"])
TORSO_BODIES=[f"torso_body{k:02d}.stl" for k in range(14)]   # bodies 14-19 (tiny inside-out logo bits) left out
