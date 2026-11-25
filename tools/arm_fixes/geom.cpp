// SDF voxeliser + QEM decimator for closed triangle meshes.
// Build: g++ -O3 -shared -fPIC -o libgeom.so geom.cpp
#include <cmath>
#include <vector>
#include <algorithm>
#include <queue>
#include <cstdint>
#include <cstring>
#include <unordered_map>

extern "C" {

// ---------------------------------------------------------------- SDF
static inline double dot3(const double*a,const double*b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
static double pt_tri_d2(const double p[3], const double a[3], const double b[3], const double c[3]){
    double ab[3],ac[3],ap[3];
    for(int i=0;i<3;i++){ab[i]=b[i]-a[i];ac[i]=c[i]-a[i];ap[i]=p[i]-a[i];}
    double d1=dot3(ab,ap), d2=dot3(ac,ap);
    double q[3];
    if(d1<=0&&d2<=0){for(int i=0;i<3;i++)q[i]=a[i];}
    else{
      double bp[3]; for(int i=0;i<3;i++)bp[i]=p[i]-b[i];
      double d3=dot3(ab,bp), d4=dot3(ac,bp);
      if(d3>=0&&d4<=d3){for(int i=0;i<3;i++)q[i]=b[i];}
      else{
        double vc=d1*d4-d3*d2;
        if(vc<=0&&d1>=0&&d3<=0){double v=d1/(d1-d3);for(int i=0;i<3;i++)q[i]=a[i]+v*ab[i];}
        else{
          double cp[3]; for(int i=0;i<3;i++)cp[i]=p[i]-c[i];
          double d5=dot3(ab,cp), d6=dot3(ac,cp);
          if(d6>=0&&d5<=d6){for(int i=0;i<3;i++)q[i]=c[i];}
          else{
            double vb=d5*d2-d1*d6;
            if(vb<=0&&d2>=0&&d6<=0){double w=d2/(d2-d6);for(int i=0;i<3;i++)q[i]=a[i]+w*ac[i];}
            else{
              double va=d3*d6-d5*d4;
              if(va<=0&&(d4-d3)>=0&&(d5-d6)>=0){double w=(d4-d3)/((d4-d3)+(d5-d6));for(int i=0;i<3;i++)q[i]=b[i]+w*(c[i]-b[i]);}
              else{double den=1.0/(va+vb+vc);double v=vb*den,w=vc*den;for(int i=0;i<3;i++)q[i]=a[i]+ab[i]*v+ac[i]*w;}
            }
          }
        }
      }
    }
    double d=0; for(int i=0;i<3;i++){double e=p[i]-q[i];d+=e*e;} return d;
}

// tris: m*9 doubles. grid origin o[3] (position of voxel 0,0,0), spacing h, dims n[3].
// out: float n0*n1*n2, C-order (i,j,k). band: clamp distance.
void sdf_grid(const double* tris, int m, const double* o, double h, const int* n, double band, float* out){
    long N=(long)n[0]*n[1]*n[2];
    for(long i=0;i<N;i++) out[i]=(float)band;
    // unsigned distance
    for(int t=0;t<m;t++){
        const double*a=tris+9*t,*b=a+3,*c=a+6;
        int lo[3],hi[3];
        for(int d=0;d<3;d++){
            double mn=std::min(a[d],std::min(b[d],c[d]))-band, mx=std::max(a[d],std::max(b[d],c[d]))+band;
            lo[d]=std::max(0,(int)std::ceil((mn-o[d])/h)); hi[d]=std::min(n[d]-1,(int)std::floor((mx-o[d])/h));
        }
        double nrm[3]={ (b[1]-a[1])*(c[2]-a[2])-(b[2]-a[2])*(c[1]-a[1]), (b[2]-a[2])*(c[0]-a[0])-(b[0]-a[0])*(c[2]-a[2]), (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])};
        double ln=std::sqrt(dot3(nrm,nrm)); if(ln>0) for(int d=0;d<3;d++) nrm[d]/=ln;
        for(int i=lo[0];i<=hi[0];i++){ double p[3]; p[0]=o[0]+i*h;
          for(int j=lo[1];j<=hi[1];j++){ p[1]=o[1]+j*h;
            long base=((long)i*n[1]+j)*n[2];
            for(int k=lo[2];k<=hi[2];k++){ p[2]=o[2]+k*h;
              if(ln>0){double pd=(p[0]-a[0])*nrm[0]+(p[1]-a[1])*nrm[1]+(p[2]-a[2])*nrm[2]; if(std::fabs(pd)>=out[base+k]) continue;}
              double d=std::sqrt(pt_tri_d2(p,a,b,c));
              if(d<out[base+k]) out[base+k]=(float)d;
            }
          }
        }
    }
    // sign by parity along z, columns at (i,j). jitter to avoid degeneracies.
    const double jx=1.37e-7, jy=2.71e-7;
    std::vector<std::vector<double>> hits((size_t)n[0]*n[1]);
    for(int t=0;t<m;t++){
        const double*a=tris+9*t,*b=a+3,*c=a+6;
        double mnx=std::min(a[0],std::min(b[0],c[0])), mxx=std::max(a[0],std::max(b[0],c[0]));
        double mny=std::min(a[1],std::min(b[1],c[1])), mxy=std::max(a[1],std::max(b[1],c[1]));
        int i0=std::max(0,(int)std::ceil((mnx-o[0]-jx)/h)), i1=std::min(n[0]-1,(int)std::floor((mxx-o[0]-jx)/h));
        int j0=std::max(0,(int)std::ceil((mny-o[1]-jy)/h)), j1=std::min(n[1]-1,(int)std::floor((mxy-o[1]-jy)/h));
        for(int i=i0;i<=i1;i++){ double x=o[0]+i*h+jx;
          for(int j=j0;j<=j1;j++){ double y=o[1]+j*h+jy;
            // barycentric in xy
            double d=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1]);
            if(d==0) continue;
            double l1=((b[1]-c[1])*(x-c[0])+(c[0]-b[0])*(y-c[1]))/d;
            double l2=((c[1]-a[1])*(x-c[0])+(a[0]-c[0])*(y-c[1]))/d;
            double l3=1-l1-l2;
            if(l1<0||l2<0||l3<0) continue;
            hits[(size_t)i*n[1]+j].push_back(l1*a[2]+l2*b[2]+l3*c[2]);
          }
        }
    }
    for(int i=0;i<n[0];i++) for(int j=0;j<n[1];j++){
        auto &H=hits[(size_t)i*n[1]+j]; if(H.empty()) continue;
        std::sort(H.begin(),H.end());
        long base=((long)i*n[1]+j)*n[2];
        size_t q=0; bool in=false;
        for(int k=0;k<n[2];k++){ double z=o[2]+k*h;
            while(q<H.size()&&H[q]<z){in=!in;q++;}
            if(in) out[base+k]=-out[base+k];
        }
    }
}

// ---------------------------------------------------------------- QEM
struct Q { double a[10]; };
static inline void qadd(Q&r,const Q&s){for(int i=0;i<10;i++)r.a[i]+=s.a[i];}
static inline double qeval(const Q&q,const double*p){
    double x=p[0],y=p[1],z=p[2];
    return q.a[0]*x*x+2*q.a[1]*x*y+2*q.a[2]*x*z+2*q.a[3]*x+q.a[4]*y*y+2*q.a[5]*y*z+2*q.a[6]*y+q.a[7]*z*z+2*q.a[8]*z+q.a[9];
}
static Q plane_q(double a,double b,double c,double d,double w){
    Q q; double v[10]={a*a,a*b,a*c,a*d,b*b,b*c,b*d,c*c,c*d,d*d}; for(int i=0;i<10;i++) q.a[i]=v[i]*w; return q;
}
struct Cand{ double cost; int u,v; unsigned su,sv; bool operator<(const Cand&o)const{return cost>o.cost;} };

// V: nv*3 (in/out), F: nf*3 (in/out). returns new nf; *nv_out new vertex count (compacted).
int decimate(double* V, int nv, int* F, int nf, double max_err, int min_faces, double min_cos, int* nv_out){
    std::vector<Q> q(nv); for(auto&x:q) memset(x.a,0,sizeof x.a);
    std::vector<char> falive(nf,1), valive(nv,1);
    std::vector<std::vector<int>> vf(nv);
    auto fnormal=[&](int f,double*nn,double*area){
        const double*a=V+3*F[3*f],*b=V+3*F[3*f+1],*c=V+3*F[3*f+2];
        double e1[3]={b[0]-a[0],b[1]-a[1],b[2]-a[2]},e2[3]={c[0]-a[0],c[1]-a[1],c[2]-a[2]};
        nn[0]=e1[1]*e2[2]-e1[2]*e2[1];nn[1]=e1[2]*e2[0]-e1[0]*e2[2];nn[2]=e1[0]*e2[1]-e1[1]*e2[0];
        double l=std::sqrt(dot3(nn,nn)); *area=l/2; if(l>0){nn[0]/=l;nn[1]/=l;nn[2]/=l;}
    };
    for(int f=0;f<nf;f++){
        double nn[3],ar; fnormal(f,nn,&ar);
        const double*a=V+3*F[3*f]; double d=-dot3(nn,a);
        Q pq=plane_q(nn[0],nn[1],nn[2],d,std::max(ar,1e-12));
        for(int k=0;k<3;k++){qadd(q[F[3*f+k]],pq); vf[F[3*f+k]].push_back(f);}
    }
    // normalise by area-weighted: use weight=area => cost units mm^2*mm^2; convert threshold by local area. Simpler: store total area per vertex.
    std::vector<double> varea(nv,0);
    for(int f=0;f<nf;f++){double nn[3],ar;fnormal(f,nn,&ar);for(int k=0;k<3;k++)varea[F[3*f+k]]+=ar;}
    std::vector<unsigned> stamp(nv,0);
    std::priority_queue<Cand> pq;
    auto optimal=[&](int u,int v,double*p)->double{
        Q s=q[u]; qadd(s,q[v]);
        double A[9]={s.a[0],s.a[1],s.a[2],s.a[1],s.a[4],s.a[5],s.a[2],s.a[5],s.a[7]};
        double bb[3]={-s.a[3],-s.a[6],-s.a[8]};
        double det=A[0]*(A[4]*A[8]-A[5]*A[7])-A[1]*(A[3]*A[8]-A[5]*A[6])+A[2]*(A[3]*A[7]-A[4]*A[6]);
        const double*pu=V+3*u,*pv=V+3*v;
        double best=1e300; double cand[3];
        double sc=std::fabs(A[0])+std::fabs(A[4])+std::fabs(A[8]);
        if(std::fabs(det)>1e-9*sc*sc*sc && sc>0){
            double inv[9];
            inv[0]=(A[4]*A[8]-A[5]*A[7])/det; inv[1]=(A[2]*A[7]-A[1]*A[8])/det; inv[2]=(A[1]*A[5]-A[2]*A[4])/det;
            inv[3]=(A[5]*A[6]-A[3]*A[8])/det; inv[4]=(A[0]*A[8]-A[2]*A[6])/det; inv[5]=(A[2]*A[3]-A[0]*A[5])/det;
            inv[6]=(A[3]*A[7]-A[4]*A[6])/det; inv[7]=(A[1]*A[6]-A[0]*A[7])/det; inv[8]=(A[0]*A[4]-A[1]*A[3])/det;
            for(int i=0;i<3;i++) cand[i]=inv[3*i]*bb[0]+inv[3*i+1]*bb[1]+inv[3*i+2]*bb[2];
            // reject if far from edge
            double mid[3]={(pu[0]+pv[0])/2,(pu[1]+pv[1])/2,(pu[2]+pv[2])/2};
            double el2=0,dm2=0; for(int i=0;i<3;i++){el2+=(pu[i]-pv[i])*(pu[i]-pv[i]); dm2+=(cand[i]-mid[i])*(cand[i]-mid[i]);}
            if(dm2<=el2*1.0+1e-12){ double c=qeval(s,cand); if(c<best){best=c;memcpy(p,cand,sizeof cand);} }
        }
        for(int t=0;t<3;t++){
            for(int i=0;i<3;i++) cand[i]= t==0?pu[i]:(t==1?pv[i]:(pu[i]+pv[i])/2);
            double c=qeval(s,cand); if(c<best-1e-18){best=c;memcpy(p,cand,sizeof cand);}
        }
        double ar=std::max(1e-12,(varea[u]+varea[v]));
        return std::max(0.0,best)/ar; // ~ mean squared distance
    };
    auto push_vertex_edges=[&](int u){
        // neighbours
        std::vector<int> nb;
        for(int f:vf[u]) if(falive[f]) for(int k=0;k<3;k++){int w=F[3*f+k]; if(w!=u) nb.push_back(w);}
        std::sort(nb.begin(),nb.end()); nb.erase(std::unique(nb.begin(),nb.end()),nb.end());
        for(int w:nb){ double p[3]; double c=optimal(u,w,p); pq.push({c,u,w,stamp[u],stamp[w]}); }
    };
    for(int u=0;u<nv;u++) push_vertex_edges(u);
    int faces=nf; double maxe2=max_err*max_err;
    std::vector<int> nbu,nbv; std::vector<char> mark(nv,0);
    while(!pq.empty() && faces>min_faces){
        Cand c=pq.top(); pq.pop();
        int u=c.u,v=c.v;
        if(!valive[u]||!valive[v]||stamp[u]!=c.su||stamp[v]!=c.sv) continue;
        if(c.cost>maxe2) break;
        double p[3]; optimal(u,v,p);
        // link condition
        nbu.clear(); nbv.clear();
        for(int f:vf[u]) if(falive[f]) for(int k=0;k<3;k++){int w=F[3*f+k]; if(w!=u) nbu.push_back(w);}
        for(int f:vf[v]) if(falive[f]) for(int k=0;k<3;k++){int w=F[3*f+k]; if(w!=v) nbv.push_back(w);}
        std::sort(nbu.begin(),nbu.end()); nbu.erase(std::unique(nbu.begin(),nbu.end()),nbu.end());
        std::sort(nbv.begin(),nbv.end()); nbv.erase(std::unique(nbv.begin(),nbv.end()),nbv.end());
        int common=0; for(int w:nbu) mark[w]=1; for(int w:nbv) if(mark[w]) common++; for(int w:nbu) mark[w]=0;
        int shared=0; for(int f:vf[u]) if(falive[f]){ for(int k=0;k<3;k++) if(F[3*f+k]==v){shared++;break;} }
        if(shared!=2 || common!=2) continue;
        // flip / quality check
        bool ok=true;
        for(int pass=0;pass<2&&ok;pass++){
            int a0=pass?v:u, other=pass?u:v;
            for(int f:vf[a0]){ if(!falive[f]) continue;
                bool hasOther=false; for(int k=0;k<3;k++) if(F[3*f+k]==other) hasOther=true;
                if(hasOther) continue;
                double P[3][3]; for(int k=0;k<3;k++){int w=F[3*f+k]; for(int i=0;i<3;i++) P[k][i]= (w==a0)?p[i]:V[3*w+i];}
                double n0[3],ar0; fnormal(f,n0,&ar0);
                double e1[3]={P[1][0]-P[0][0],P[1][1]-P[0][1],P[1][2]-P[0][2]},e2[3]={P[2][0]-P[0][0],P[2][1]-P[0][1],P[2][2]-P[0][2]};
                double n1[3]={e1[1]*e2[2]-e1[2]*e2[1],e1[2]*e2[0]-e1[0]*e2[2],e1[0]*e2[1]-e1[1]*e2[0]};
                double l=std::sqrt(dot3(n1,n1)); if(l<=1e-14){ok=false;break;}
                for(int i=0;i<3;i++) n1[i]/=l;
                if(dot3(n0,n1)<min_cos){ok=false;break;}
                // triangle quality: avoid slivers
                double l01=std::sqrt(dot3(e1,e1)), l02=std::sqrt(dot3(e2,e2));
                double e3[3]={P[2][0]-P[1][0],P[2][1]-P[1][1],P[2][2]-P[1][2]}; double l12=std::sqrt(dot3(e3,e3));
                double mx=std::max(l01,std::max(l02,l12)); double qual=(l/2)/(mx*mx+1e-30);
                if(qual<0.02){ok=false;break;}
            }
        }
        if(!ok) continue;
        // collapse v -> u
        for(int i=0;i<3;i++) V[3*u+i]=p[i];
        for(int f:vf[v]){ if(!falive[f]) continue;
            bool hasU=false; for(int k=0;k<3;k++) if(F[3*f+k]==u) hasU=true;
            if(hasU){ falive[f]=0; faces--; continue; }
            for(int k=0;k<3;k++) if(F[3*f+k]==v) F[3*f+k]=u;
            vf[u].push_back(f);
        }
        valive[v]=0; vf[v].clear();
        qadd(q[u],q[v]); varea[u]+=varea[v];
        { std::vector<int> nf2; for(int f:vf[u]) if(falive[f]) nf2.push_back(f); vf[u].swap(nf2); }
        stamp[u]++;
        // neighbours' stamps change too (their edges to u changed)
        push_vertex_edges(u);
    }
    // compact
    std::vector<int> remap(nv,-1); int k=0;
    for(int f=0;f<nf;f++) if(falive[f]) for(int j=0;j<3;j++){int w=F[3*f+j]; if(remap[w]<0) remap[w]=k++;}
    std::vector<double> V2(3*(size_t)k);
    for(int w=0;w<nv;w++) if(remap[w]>=0) for(int i=0;i<3;i++) V2[3*remap[w]+i]=V[3*w+i];
    memcpy(V,V2.data(),sizeof(double)*3*k);
    int m=0; for(int f=0;f<nf;f++) if(falive[f]){ for(int j=0;j<3;j++) F[3*m+j]=remap[F[3*f+j]]; m++; }
    *nv_out=k; return m;
}
}
