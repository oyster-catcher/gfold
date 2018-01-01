#!/usr/bin/python

import pylab as P
import math
import numpy as np
import utils

def correct_trajectory(traj):
  """Adjust trajectory to end at (0,0,0)"""

  offset = traj[-1][0]
  numpoints = len(traj)
  if numpoints==0:
    return traj

  print "offset:",offset
  traj2=[]
  for i,(r,rr,rrr) in enumerate(traj):
    traj2.append ( (r-(float(i)/(numpoints-1))*offset,rr,rrr) )

  return traj2

def compute_trajectory(r0,rr0,rrrs,T,g,dt=1):
  T=float(T)
  N=len(rrrs)
  pos=[]
  t=0
  r=r0
  rr=rr0
  traj=[]
  while t < T:
    #print "W:",utils.basis_weights(t,T,N),"t:",t,"T:",T,"N:",N
    rrr = utils.basis_weights(t,T,N).dot(rrrs)
    traj.append( (r,rr,rrr) )
    r = r + rr*dt + 0.5*rrr*dt*dt + 0.5*g*dt*dt
    rr = rr + rrr*dt + g*dt
    t = t + dt
  # remaining time to end
  dt=T-t
  r = r + rr*dt + 0.5*rrr*dt*dt + 0.5*g*dt*dt
  rr = rr + rrr*dt + g*dt
  traj.append( (r,rr,rrr) )
  return traj

def plot(traj,title="",fuel=[]):
  # altitude against time, and throttle
  fig = P.figure(1)

  P.subplot2grid((2,2),(0,0), colspan=1, rowspan=1)
  ax = P.gca()

  # Side view
  ax.set_xlabel("y")
  ax.set_ylabel("x")
  ax.set_xlim([-2000,2000])
  ax.set_ylim([-300,3000])
  yy=[tj[0][1] for tj in traj]
  xx=[tj[0][0] for tj in traj]
  ax.plot(yy,xx,marker='o',alpha=0.5)
  for tj in traj:
    oy,ox=tj[0][1],tj[0][0]
    P.arrow(oy,ox,tj[2][1],tj[2][0],head_width=0.1,head_length=0.5,fc='red',ec='red')
  ax.grid()
  #P.title(title)
  #P.plot(t, s1)

  # Vertical view
  P.subplot2grid((2,2),(1,0), colspan=1, rowspan=1)
  ax = P.gca()
  ax.set_xlabel("y")
  ax.set_ylabel("z")
  ax.set_xlim([-2000,2000])
  ax.set_ylim([-2000,2000])
  yy=[tj[0][1] for tj in traj]
  zz=[tj[0][2] for tj in traj]
  ax.plot(yy,zz,marker='o',alpha=0.5)
  for tj in traj:
    oy,oz=tj[0][1],tj[0][2]
    P.arrow(oy,oz,tj[2][1],tj[2][2],head_width=0.1,head_length=0.5,fc='red',ec='red')
  ax.grid()

  # Another side view
  P.subplot2grid((2,2),(0,1), colspan=1, rowspan=1)
  ax = P.gca()
  ax.set_xlabel("z")
  ax.set_ylabel("x")
  ax.set_xlim([-2000,2000])
  ax.set_ylim([-300,3000])
  zz=[tj[0][2] for tj in traj]
  xx=[tj[0][0] for tj in traj]
  ax.plot(zz,xx,marker='o',alpha=0.5)
  for tj in traj:
    oz,ox=tj[0][2],tj[0][0]
    P.arrow(oz,ox,tj[2][2],tj[2][0],head_width=0.1,head_length=0.5,fc='red',ec='red')
  ax.grid()
  ax.legend()

  # Fuel usage
  # fuel is array to tuples (time,fuel_needed)
  P.subplot2grid((2,2),(1,1), colspan=1, rowspan=1)
  ax = P.gca()
  ax.set_xlabel("duration /secs")
  ax.set_ylabel("fuel needed")
  ax.set_ylim([0,250])
  xx=[f[0] for f in fuel]
  yy=[f[1] for f in fuel]
  ax.plot(xx,yy,marker='o')
  ax.grid()
  P.show()
