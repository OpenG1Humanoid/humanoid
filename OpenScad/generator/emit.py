import numpy as np, design as D, json, os
SHELLS=json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),"shells.json")))
F=lambda v: "%.4f"%v
def mm(M): return "multmatrix([%s])"%",".join("[%s]"%",".join(F(x) for x in row) for row in M)
def prim(p):
    k=p[0]
    if k=="cyl": z0,z1=sorted(p[2:4]); return f"translate([0,0,{F(z0)}]) cylinder(r={F(p[1])},h={F(z1-z0)});"
    if k=="ann": z0,z1=sorted(p[3:5]); return f"translate([0,0,{F(z0)}]) difference(){{cylinder(r={F(p[2])},h={F(z1-z0)}); translate([0,0,-1]) cylinder(r={F(p[1])},h={F(z1-z0+2)});}}"
    if k=="box": z0,z1=sorted(p[5:7]); return f"translate([{F(p[1])},{F(p[3])},{F(z0)}]) cube([{F(p[2]-p[1])},{F(p[4]-p[3])},{F(z1-z0)}]);"
    if k=="cylat": z0,z1=sorted(p[4:6]); return f"translate([{F(p[1])},{F(p[2])},{F(z0)}]) cylinder(r={F(p[3])},h={F(z1-z0)});"
    if k=="hex": z0,z1=sorted(p[2:4]); return f"translate([0,0,{F(z0)}]) cylinder(d={F(p[1]/np.cos(np.pi/6))},h={F(z1-z0)},$fn=6);"
    if k=="dcyl":   # cylinder cut by a flat at distance p[4] from the axis, towards angle p[5] (deg)
        z0,z1=sorted(p[2:4]); r=p[1]
        return (f"intersection() {{ translate([0,0,{F(z0)}]) cylinder(r={F(r)},h={F(z1-z0)}); "
                f"rotate([0,0,{F(p[5])}]) translate([{F(-r-1)},{F(-r-1)},{F(z0-1)}]) cube([{F(p[4]+r+1)},{F(2*r+2)},{F(z1-z0+2)}]); }}")
    if k=="boxr":   # box in a frame turned by p[1] degrees about z
        z0,z1=sorted(p[6:8]); return f"rotate([0,0,{F(p[1])}]) translate([{F(p[2])},{F(p[4])},{F(z0)}]) cube([{F(p[3]-p[2])},{F(p[5]-p[4])},{F(z1-z0)}]);"
    if k=="cone":   # radius p[1] at z = p[3] to radius p[2] at z = p[4]
        return f"translate([0,0,{F(p[3])}]) cylinder(r1={F(p[1])},r2={F(p[2])},h={F(p[4]-p[3])});"
    if k=="rad": return f"rotate([0,0,{F(p[1])}]) translate([{F(p[3])},0,{F(p[2])}]) rotate([0,90,0]) cylinder(d={F(p[5])},h={F(p[4]-p[3])});"
    raise ValueError(k)
def block(ops): return "\n".join(f"    {mm(o['M'])} {prim(o['prim'])}" for o in ops) or "    ;"
out=[]
out.append('''// =====================================================================
//  G1 / OpenG1 arm on hobby servos - GENERATED from gen/design.py
//  The bodies are YOUR original STL files (stl/), imported unchanged.
//  Everything below only CUTS into them (servo pockets, hatches, sockets,
//  screw holes) or adds material INSIDE the thin wrist covers (clipped to
//  each cover's own hull), so the outside shape is the original shape.
//  Units: mm. Left-arm URDF link frames; the right arm uses the right-hand
//  STLs, mirrored into the left frame, and the finished part is mirrored back.
// =====================================================================
side = "left";            // [left, right]
$side = side;             // modules read $side, so assembly.scad can place a left AND a right arm
part = "shoulder_pitch_link"; // see the PART LIST at the bottom
// Mesh folders, relative to the file you open. Every file you open sits one folder below
// OpenScad/ (like this library), so the same path works for all of them.
arm_stl_dir   = "../../STL/arms/";
torso_stl_dir = "../../STL/torso/";
$fn = 48;

module mesh(link) {
    if (link == "shoulder_yaw_link") {   // several solids in Unitree's file: imported as separate shells and unioned (split_shells.py)
        if ($side == "left") union() { %s }
        else mirror([0,1,0]) union() { %s }
    } else {
        f = (link == "shoulder_roll_link") ? str(arm_stl_dir, $side, "_shoulder_roll_link_fixed.stl")
                                            : str(arm_stl_dir, $side, "_", link, ".STL");
        if ($side == "left") scale(1000) import(f, convexity=10);
        else mirror([0,1,0]) scale(1000) import(f, convexity=10);
    }
}
module hand_trimmed() {   // rubber hand, wrist section removed, placed in the wrist_yaw frame
    f = str(arm_stl_dir, $side, "_hand_clean.stl");
    translate([-84,0,0]) intersection() {
        if ($side == "left") scale(1000) import(f, convexity=10); else mirror([0,1,0]) scale(1000) import(f, convexity=10);
        translate([%s,-200,-200]) cube(400);
    }
}
torso_mesh_source = "original";   // [original, voxel]  voxel = clean 1.5 mm remesh of the main torso shell
module torso_mesh() {   // your torso_link_rev_1_0.STL, split into its separate bodies (original triangles) and re-joined
    union() {
        if (torso_mesh_source == "voxel") import(str(torso_stl_dir, "torso_body00_voxel_mm.stl"), convexity=10);
        else scale(1000) import(str(torso_stl_dir, "torso_body00.stl"), convexity=10);
        for (k=[1:13]) scale(1000) import(str(torso_stl_dir, "torso_body", k<10 ? "0" : "", k, ".stl"), convexity=10);
    }
}
module body(link) {
    if (link == "wrist_yaw_link") union() { mesh(link); hand_trimmed(); }
    else if (link == "torso_link") torso_mesh();
    else mesh(link);
}
''' % (" ".join(f'scale(1000) import(str(arm_stl_dir, "{n}"), convexity=10);' for n in SHELLS["left_shoulder_yaw_link"]),
        " ".join(f'scale(1000) import(str(arm_stl_dir, "{n}"), convexity=10);' for n in SHELLS["right_shoulder_yaw_link"]),
        F(D.HAND_TRIM_X)))
parts=[]
for link in D.LINKS+["torso_link"]:
    ops=[o for o in D.OPS if o["link"]==link]
    g=lambda k:[o for o in ops if o["kind"]==k]
    adds=g("addhull"); adds2=g("add"); cuts=g("cut"); plugs=g("plug"); caps=g("cap"); rings=g("ring")
    n=link
    out.append(f"// ---------------------------------------------------------------- {n}")
    out.append(f"module {n}_bodyplus() {{\n  union() {{\n    body(\"{n}\");")
    if adds: out.append(f"    intersection() {{\n      union() {{\n{block(adds)}\n      }}\n      hull() mesh(\"{n}\");\n    }}")
    if adds2: out.append(block(adds2))
    out.append("  }\n}")
    out.append(f"module {n}_cuts() {{\n{block(cuts)}\n}}")
    out.append(f"module {n}_hatches() {{\n{block(plugs+caps)}\n}}")
    out.append(f"module {n}_main() {{\n  union() {{\n    difference() {{\n      {n}_bodyplus();\n      {n}_cuts();\n      {n}_hatches();\n    }}\n{block(rings)}\n  }}\n}}")
    parts.append(f"{n}")
    for o in plugs:
        out.append(f"module {n}_plug_{o['tag']}() {{\n  difference() {{\n    intersection() {{ {n}_bodyplus(); {mm(o['M'])} {prim(o['prim'])} }}\n    {n}_cuts();\n  }}\n}}")
        parts.append(f"{n}_plug_{o['tag']}")
    for o in caps:
        out.append(f"module {n}_cap_{o['tag']}() {{\n  intersection() {{ {n}_bodyplus(); {mm(o['M'])} {prim(o['prim'])} }}\n}}")
        parts.append(f"{n}_cap_{o['tag']}")
# ---------------- printed hardware (hubs, pins) - same for left and right
def hub_solid(name,S):
    d0,d1,ct=D.stack(S); hexc=S["hex"]/np.cos(np.pi/6); hubr=max(S["horn"]/2,hexc/2)+1.2; L=S["hexlen"]
    sc={3:(3.4,2.5,6.0),2:(2.4,1.7,4.2)}[S["scr"]]; pr=S["horn"]*0.34
    return f'''module hub_{name}() {{   // print hex-end down. Torque via hex, radial screws hold the child.
  difference() {{
    union() {{
      translate([0,0,{F(-L)}]) cylinder(d={F(S["hex"]/np.cos(np.pi/6))},h={F(L+d0+0.01)},$fn=6);
      translate([0,0,{F(d0)}]) cylinder(r={F(hubr-0.8)},h={F(d1-d0)});
    }}
    translate([0,0,{F(-L-1)}]) cylinder(d={F(sc[0]+0.4)},h={F(L+d1+2)});                 // horn centre screw (through)
    translate([0,0,{F(-L-1)}]) cylinder(d={F(sc[2]+0.4)},h={F(L+d0-2+1)});               // its head sits on a 2 mm floor
    for (a=[45,135,225,315]) rotate([0,0,a]) translate([{F(pr)},0,{F(d1-4)}]) cylinder(d={F(sc[1])},h=5);  // horn screws from the horn side
    translate([0,0,{F(d1-1)}]) cylinder(d={F(S["horn"]+0.3)},h=2);                      // 1 mm horn locating recess
    for (a=[30:60:330]) rotate([0,0,a]) translate([0,0,{F(-L/2-0.5)}]) rotate([0,90,0]) cylinder(d={F(sc[1])},h=30);
  }}
}}'''
def hub_outer(name,S,w=-8.0):
    d0,d1,ct=D.stack(S); hexc=S["hex"]/np.cos(np.pi/6); head=hexc+4
    return f'''module hub_{name}() {{   // goes in from OUTSIDE the cover, then the cap is glued over it
  difference() {{
    union() {{
      translate([0,0,{F(w+0.1)}]) cylinder(d={F(head-0.2)},h=1.5);
      translate([0,0,{F(w+1.6)}]) cylinder(d={F(S["hex"]/np.cos(np.pi/6))},h={F(d1-(w+1.6))},$fn=6);
    }}
    translate([0,0,{F(w-1)}]) cylinder(d=2.4,h=30);                                        // M2 horn screw
    translate([0,0,{F(w-1)}]) cylinder(d=4.2,h={F(1+1.2)});
    for (a=[0,180]) rotate([0,0,a]) translate([4,0,{F(d1-4)}]) cylinder(d=1.7,h=5);     // horn screws from the horn side
    translate([0,0,{F(d1-1)}]) cylinder(d={F(S["horn"]+0.3)},h=2);
  }}
}}'''
pin='''module idler_pin() {   // 5 mm pin: glue into the wrist_pitch link, runs in the cover boss
  translate([0,0,-6.4]) cylinder(d=9,h=1.1);
  translate([0,0,-5.3]) cylinder(d=4.95,h=12.3);
}'''

# ---------------- servo cartridges: round printed housings that slide into the link bores
def cart_scad(t, g):
    Rc, zf, zr, B = g["Rc"], g["zf"], g["zr"], g["B"]
    fl = g["flat"]
    def shp(r, z0, z1):
        if fl: return prim(("dcyl", r, z0, z1, fl[0] + (r - Rc), fl[1]))
        return prim(("cyl", r, z0, z1))
    ys = (g["by0"] - Rc) / 2; xs = min(0.42 * Rc, np.sqrt(max((Rc - 4.5) ** 2 - ys ** 2, 1)))
    if fl: xs = min(xs, fl[0] - 4.5)
    scr = [(xs, ys), (-xs, ys)]                                  # cap screws, in the thick wall opposite the servo
    ca = g["cable_ang"]; S25 = D.M25; S3 = D.M3
    lugs = "".join(f"rotate([0,0,{F(a)}]) translate([{F(Rc-1.8)},-4.2,{F(zf)}]) cube([7.2,8.4,3.8]); " for a in g["lug_angles"]) if g["mode"] == "front" else ""
    lugholes = "".join(
        f"rotate([0,0,{F(a)}]) translate([{F(Rc+2.4)},0,{F(zf-1)}]) {{ cylinder(d={F(S25['clr'])},h=6); cylinder(d={F(S25['head']+0.4)},h={F(1+S25['head_h'])}); }} "
        for a in g["lug_angles"]) if g["mode"] == "front" else ""
    body = f"""module cartridge_{t}() {{   // {g['link']} joint. Print BACK face down (bearing seat opens upwards, no overhang). Snip the servo's mounting tabs off.
  difference() {{
    union() {{ {shp(Rc, zf if g["mode"]=="front" else g["ct"]-0.3, zr)} {"" if g["mode"]=="front" else prim(("cyl", g["r_nose"], zf, g["ct"]-0.2))} {lugs}}}
    translate([0,0,{F(zf-1)}]) cylinder(r={F(B['od']/2+0.05)},h={F(B['w']+1)});                 // {B['od']:g} mm bearing seat (press fit)
    translate([0,0,{F(zf+B['w']-0.01)}]) cylinder(r={F(g['r_in'])},h={F(g['ct']-zf-B['w']+0.02)});   // hub + horn
    translate([{F(-g['W']/2)},{F(g['by0'])},{F(g['ct'])}]) cube([{F(g['W'])},{F(g['by1']-g['by0'])},{F(zr-g['ct']+1)}]);   // servo body
    rotate([0,0,{F(ca)}]) translate([0,-2.6,{F(zr-3)}]) cube([{F(Rc+1)},5.2,3.1]);                       // cable out of the back
    {"rotate([0,0,"+F(ca)+"]) translate(["+F(Rc-3)+",-2.6,"+F(zf-1)+"]) cube([4,5.2,"+F(zr-zf+2)+"]);   // cable groove along the side to the face" if g["mode"]=="front" else ""}
    {"".join(f"translate([{F(x)},{F(y)},{F(zr-S25['ins_l'])}]) cylinder(d={F(S25['ins'])},h={F(S25['ins_l']+0.01)}); " for x,y in scr)}  // cap inserts (M2.5)
    {lugholes}
  }}
}}"""
    if g["mode"] == "front":
        zc = zr + 3.0
        cap = f"""module cartridge_cap_{t}() {{   // screws onto the back of cartridge_{t} with 2 x M2.5 x 6 and clamps the servo
  difference() {{
    {shp(Rc, zr, zc)}
    {"".join(f"translate([{F(x)},{F(y)},{F(zr-1)}]) cylinder(d={F(S25['clr'])},h=6); translate([{F(x)},{F(y)},{F(zc-1.8)}]) cylinder(d={F(S25['head']+0.4)},h=3); " for x,y in scr)}
    rotate([0,0,{F(ca)}]) translate([{F(Rc-4)},-2.6,{F(zr-1)}]) cube([5,5.2,5]);
  }}
}}"""
    else:
        L = g["host_len"]
        lugsc = "".join(f"rotate([0,0,{F(a)}]) translate([{F(Rc-1.8)},-4.2,{F(L-3.9)}]) cube([7.2,8.4,3.9]); " for a in g["cap_lug_angles"])
        lugh = "".join(f"rotate([0,0,{F(a)}]) translate([{F(Rc+2.4)},0,{F(L-9)}]) {{ cylinder(d={F(S3['clr'])},h=10); translate([0,0,{F(9-1.9)}]) cylinder(d=6.2,h=3); }} " for a in g["cap_lug_angles"])
        cap = f"""module cartridge_cap_{t}() {{   // rear cap: carries the 608 for the fork's far arm; 2 x M3 button heads into the link face. Print the outer face down
  difference() {{
    union() {{ {shp(Rc, zr, L)} {prim(("dcyl", Rc+1.5, L-2.95, L, fl[0]+1.5, fl[1])) if fl else prim(("cyl", Rc+1.5, L-2.95, L))} {lugsc}}}
    translate([0,0,{F(L-7.3)}]) cylinder(r=11.05,h=8);           // 608 seat (8 x 22 x 7)
    translate([0,0,{F(L-9.5)}]) cylinder(r=5,h=2.3);            // dowel end relief
    {"".join(f"translate([{F(x)},{F(y)},{F(zr-1)}]) cylinder(d={F(S25['clr'])},h={F(L-zr+2)}); translate([{F(x)},{F(y)},{F(zr+5)}]) cylinder(d={F(S25['head']+0.4)},h={F(L-zr)}); " for x,y in scr)}
    rotate([0,0,{F(ca)}]) translate([{F(Rc-4)},-2.6,{F(zr-1)}]) cube([5,5.2,{F(L-zr+2)}]);
    {lugh}
  }}
}}"""
    vis = f"""module cartridge_bearings_{t}() {{   // bought parts, for the assembly view
  color("Silver") translate([0,0,{F(zf)}]) difference() {{ cylinder(r={F(B['od']/2)},h={F(B['w'])}); translate([0,0,-1]) cylinder(r={F(B['id']/2)},h={F(B['w']+2)}); }}
  {"color(\"Silver\") translate([0,0,"+F(g['host_len']-7.3)+"]) difference() { cylinder(r=11,h=7); translate([0,0,-1]) cylinder(r=4,h=9); } color(\"DimGray\") translate([0,0,"+F(g['host_len']-7.0)+"]) cylinder(d=8,h=16);" if g["mode"]=="rear" else ""}
}}"""
    return body + "\n" + cap + "\n" + vis
out.append("// ---------------------------------------------------------------- servo cartridges (generated from design.CART)")
for t, g in D.CART.items():
    out.append(cart_scad(t, g))
out.append('''module bearing_6807() {   // bought part, for the assembly view
  color("Silver") difference() { cylinder(r=23.5,h=7); translate([0,0,-1]) cylinder(r=17.5,h=9); }
}''')
CART_PARTS = [f"cartridge_{t}" for t in D.CART if not t.startswith("pitch")] + [f"cartridge_cap_{t}" for t in D.CART if not t.startswith("pitch")]
out.append("// ---------------------------------------------------------------- printed hardware")
out.append(hub_solid("DS3235",D.SERVO["DS3235"])); out.append(hub_solid("X10PRO",D.SERVO["X10PRO"]))
out.append(hub_solid("MG92B_wristroll",D.SERVO["MG92B"])); out.append(hub_outer("MG92B_cover",D.SERVO["MG92B"])); out.append(pin)
hw=["hub_DS3235","hub_X10PRO","hub_MG92B_wristroll","hub_MG92B_cover","idler_pin"]
out.append("// ---------------------------------------------------------------- PART LIST (set part = ...)")
parts += CART_PARTS
out.append("//   per side: "+", ".join(p for p in parts if not p.startswith("torso")))
out.append("//   torso (once): "+", ".join(p for p in parts if p.startswith("torso")))
out.append("//   hardware (no side): "+", ".join(hw))
disp=[]
for p in parts:
    mod=p+"_main" if (p in D.LINKS or p=="torso_link") else p
    if p.startswith("torso_link"): disp.append(f'if (part == "{p}") {mod}();   // one torso for both arms')
    else: disp.append(f'if (part == "{p}") {{ if (side == "left") {mod}(); else mirror([0,1,0]) {mod}(); }}')
for p in hw: disp.append(f'if (part == "{p}") {p}();')
out.append("\n".join(disp))
open("../lib/g1_servo_arm.scad","w").write("\n".join(out)+"\n")
# ------------------------------------------------------------------ assembly.scad
def M2s(M): return mm(M)
plugs={}; caps={}
for o in D.OPS:
    if o["kind"]=="plug": plugs.setdefault(o["link"],[]).append(o["tag"])
    if o["kind"]=="cap": caps.setdefault(o["link"],[]).append(o["tag"])
hostM={o["tag"]:(o["link"],o["M"]) for o in D.OPS if o["kind"]=="plug"}
idlerM={o["tag"]:(o["link"],o["M"]) for o in D.OPS if o["kind"]=="cut" and o["tag"].endswith("_idler") and o["link"]=="wrist_pitch_link"}
hubof={"roll":"hub_DS3235","yaw":"hub_X10PRO","elbow":"hub_X10PRO","wroll":"hub_MG92B_wristroll","wpitch":"hub_MG92B_cover","wyaw":"hub_MG92B_cover"}
def link_parts(l):
    s=[f"{l}_main();"]+[f"{l}_plug_{t}();" for t in plugs.get(l,[])]+[f"{l}_cap_{t}();" for t in caps.get(l,[])]
    return " ".join(s)
hubof.update({"pitchL":"hub_DS3235","pitchR":"hub_DS3235"})
def cartstr(l):
    s=[]
    for t,g in D.CART.items():
        if g["link"]==l and not t.startswith("pitch"):
            s.append(f"{M2s(g['M'])} {{ {hubof[t]}(); cartridge_{t}(); cartridge_cap_{t}(); cartridge_bearings_{t}(); }}")
    return " ".join(s)
def hwstr(l):
    s=[cartstr(l)]
    for t,(hl,M) in hostM.items():
        if hl==l and t in hubof: s.append(f"{M2s(M)} {hubof[t]}();")
    for t,(hl,M) in idlerM.items():
        if hl==l: s.append(f"{M2s(M)} idler_pin();")
    return " ".join(s)
cols=dict(shoulder_pitch_link="SteelBlue",shoulder_roll_link="DarkOrange",shoulder_yaw_link="ForestGreen",elbow_link="FireBrick",
          wrist_roll_link="MediumPurple",wrist_pitch_link="SaddleBrown",wrist_yaw_link="HotPink")
def L(l): return f'color("{cols[l]}") sided() {{ {link_parts(l)} }} if (show_hardware) color("Gold") sided() {{ {hwstr(l)} }}'
A=f"""// =====================================================================
//  assembly.scad - the whole robot: pelvis + legs, torso, both 7-DoF servo arms (generated)
//  F5 (preview) is quick; F6 (full render) of everything is slow - use the
//  Manifold backend (Preferences > Features > manifold) for F6.
//  Joint angles in degrees: arms [shoulder pitch, roll, yaw, elbow, wrist roll, pitch, yaw],
//  legs [hip pitch, hip roll, hip yaw, knee] (G1 URDF convention, both sides), waist yaw.
//  Frame: torso_link.
// =====================================================================
include <../lib/g1_servo_arm.scad>
include <../lib/torso_frame_lib.scad>   // framed torso (2020 aluminium + printed panels)
include <../lib/legs_lib.scad>          // pelvis + legs with the 6512 cycloidal actuators
part = "none";                      // stop g1_servo_arm.scad drawing a single part
qL = [0, 0, 0, 0, 0, 0, 0];         // left arm
qR = [0, 0, 0, 0, 0, 0, 0];         // right arm
qLegL = [0, 0, 0, 0, 0, 0];         // left leg: hip pitch, hip roll, hip yaw, knee, ankle pitch, ankle roll
qLegR = [0, 0, 0, 0, 0, 0];         // right leg
qWaist = 0;                         // waist yaw (deg)
show_legs     = true;
show_torso    = true;
torso_style   = "framed";           // "framed": 2020 frame + printable panels | "solid": one-piece printed torso
show_left     = true;
show_right    = true;
show_head     = true;               // printed dome head (STL/head)
show_hardware = true;               // servo cartridges, hubs, bearings and idler pins

module head_parts() {{
    translate([3.9635, 0, -44]) {{
        color("#5a5d66") import("../../STL/head/frame_mm.stl", convexity = 10);
        for (i = [0:4]) color("#6d7380") import(str("../../STL/head/panel_", i, "_mm.stl"), convexity = 10);
        color("#c9a227") {{ import("../../STL/head/brow_left_mm.stl", convexity = 10); import("../../STL/head/brow_right_mm.stl", convexity = 10); }}
    }}
}}

module sided() {{ if ($side == "left") children(); else mirror([0,1,0]) children(); }}

module arm(q) {{
    s = ($side == "left") ? 1 : -1;      // right arm: URDF offsets mirrored in Y
    rotate([0, q[0], 0]) {{
        {L("shoulder_pitch_link")}
        translate([0, s*38, -13.831]) rotate([s*-0.27925*180/PI, 0, 0]) rotate([q[1], 0, 0]) {{
            {L("shoulder_roll_link")}
            translate([0, s*6.24, -103.2]) rotate([0, 0, q[2]]) {{
                {L("shoulder_yaw_link")}
                translate([15.783, 0, -80.518]) rotate([0, q[3], 0]) {{
                    {L("elbow_link")}
                    translate([100, s*1.888, -10]) rotate([q[4], 0, 0]) {{
                        {L("wrist_roll_link")}
                        translate([38, 0, 0]) rotate([0, q[5], 0]) {{
                            {L("wrist_pitch_link")}
                            translate([46, 0, 0]) rotate([0, 0, q[6]]) {{
                                {L("wrist_yaw_link")}
                            }}
                        }}
                    }}
                }}
            }}
        }}
    }}
}}

if (show_torso) {{
    if (torso_style == "framed") torso_framed();
    else color("Gainsboro") {{ torso_link_main(); torso_link_plug_pitchL(); torso_link_plug_pitchR(); }}
    if (show_hardware) color("Gold") {{ {M2s(hostM["pitchL"][1])} {{ hub_DS3235(); bearing_6807(); }} {M2s(hostM["pitchR"][1])} {{ hub_DS3235(); bearing_6807(); }} }}
}}
if (show_left)  {M2s(D.T_TORSO_L)} {{ $side = "left";  arm(qL); }}
if (show_right) {M2s(D.T_TORSO_R)} {{ $side = "right"; arm(qR); }}
if (show_head) head_parts();       // dome head: frame, 5 panels, 2 brows (head_link frame = torso + [3.9635, 0, -44])
if (show_legs)  lower_body(qLegL, qLegR, qWaist);   // pelvis, legs, waist yaw link
"""
open("../assembly/assembly.scad","w").write(A)
open("parts.txt","w").write("\n".join(parts+hw))
# ---------------- one small .scad per printable part (what export_stls.py and "open, F6, export" use)
import os, glob
for d in ("../parts_left","../parts_right","../parts_torso","../hardware"):
    os.makedirs(d, exist_ok=True)
    for f in glob.glob(d+"/*.scad"): os.remove(f)
for p in parts:
    if p.startswith("torso_link"):
        open(f"../parts_torso/{p}.scad","w").write(f'// {p}: press F6 (render) then File > Export > STL\ninclude <../lib/g1_servo_arm.scad>\npart = "{p}";\n')
        continue
    for sd in ("left","right"):
        open(f"../parts_{sd}/{sd}_{p}.scad","w").write(f'// {sd} {p}: press F6 (render) then File > Export > STL\ninclude <../lib/g1_servo_arm.scad>\nside = "{sd}";\npart = "{p}";\n')
for p in hw:
    open(f"../hardware/{p}.scad","w").write(f'// {p}: F6 then export STL\ninclude <../lib/g1_servo_arm.scad>\npart = "{p}";\n')
print(len(parts),"link parts;",len(hw),"hardware; lines",sum(x.count("\n")+1 for x in out))
