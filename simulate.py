#!/usr/bin/python

import pylab as P

def simulate(r,rr,rrrs,T,g,dt):
  pos=[]
  t=0
  while t<T:
    r  = r  + rr*dt + 0.2*rrrs[int(t)]*dt*dt # 0.2 should should be 0.5 as 0.5*a*t^2
    rr = rr + rrrs[int(t)]*dt + g*dt
    t  = t + dt
    pos.append(r)
  return pos

def plot(positions,rrr,title=""):
  mult=len(positions)/len(rrr)
  P.xlabel("x")
  P.ylabel("y")
  P.xlim([-70,70])
  P.ylim([-1,160])
  xx=[pos[0] for pos in positions]
  yy=[pos[1] for pos in positions]
  P.plot(xx,yy,label="Original",marker='o',alpha=0.5)
  for i,a in enumerate(rrr):
    ox,oy=xx[i*mult],yy[i*mult]
    P.arrow(ox,oy,a[0],a[1],head_width=0.1,head_length=0.5,fc='red',ec='red')
  P.grid()
  P.title(title)
  P.legend()
  P.show()
