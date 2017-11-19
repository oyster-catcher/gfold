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
  # altitude against time, and throttle
  fig = P.figure(1)

  P.subplot2grid((2,1),(0,0), colspan=1, rowspan=1)
  ax = P.gca()

  # Side view
  ax.set_xlabel("x")
  ax.set_ylabel("y")
  ax.set_xlim([-20,20])
  ax.set_ylim([-1,40])
  xx=[pos[0] for pos in positions]
  yy=[pos[1] for pos in positions]
  ax.plot(xx,yy,marker='o',alpha=0.5)
  for i,a in enumerate(rrr):
    ox,oy=xx[i*mult],yy[i*mult]
    P.arrow(ox,oy,a[0],a[1],head_width=0.1,head_length=0.5,fc='red',ec='red')
  ax.grid()
  #P.title(title)
  #P.plot(t, s1)

  # Vertical view
  P.subplot2grid((2,1),(1,0), colspan=1, rowspan=1)
  ax = P.gca()
  ax.set_xlabel("x")
  ax.set_ylabel("z")
  ax.set_xlim([-20,20])
  ax.set_ylim([-20,20])
  xx=[pos[0] for pos in positions]
  zz=[pos[2] for pos in positions]
  ax.plot(xx,zz,marker='o',alpha=0.5)
  for i,a in enumerate(rrr):
    ox,oz=xx[i*mult],zz[i*mult]
    P.arrow(ox,oz,a[0],a[2],head_width=0.1,head_length=0.5,fc='red',ec='red')
  ax.grid()
  ax.legend()
  P.show()
