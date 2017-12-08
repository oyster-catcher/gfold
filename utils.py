#!/usr/bin/python

import numpy as np
import math
def integral_cossq(x):
  return x/2 + 0.25*math.sin(2*x)

def closest_point_on_line(x1,x2,x0):
   """x1, x2 defines the line.
   x0 is the point. All vars are numpy arrays"""

   v = x2-x1
   t = -(x1-x0).dot(x2-x1) / sum([x*x for x in v])
   return x1 + (x2-x1)*t,t

# Shift basis weights to no thrust at start
# so with 3 thrust vectors in 10 seconds, instead
# of being at 0,5,10 they are at
# 3.33,6.66,10

def basis_weights(t,T,N):
  '''Returns vector of weights for each acceleration vector
     for time t in range 0...T, with N vectors'''
  w = np.zeros(N)
  dt = T/(N-1) # distance between basis points in time
  for j in range(0,N): # loop thro' each accel vector
    d = j - (t/dt)
    if d<-1 or d>1:
      b = 0
    else:
      b = math.cos(d*0.5*3.142)
    w[j] = b*b
  return w

def desired_from_trajectory(traj,x0,min_index=0):
  """Compute closest position, and hence velocity, and acceleration
     from nearest point on trajectory"""
  # assume 2 point trajectory to start
  if not traj:
    return None,None,None,0

  # First point
  cr   = traj[0][0]
  crr  = traj[0][1]
  crrr = traj[0][2]
  cdist = np.linalg.norm(cr-x0)
  ci   = 0
  ct   = 0

  for i in range(0,len(traj)-1):
    cx, t = closest_point_on_line(traj[i][0],traj[i+1][0],x0)
    d = np.linalg.norm(cx-x0)
    if (t > 0) and (t < 1) and (d < cdist):
      print "set ci=",i
      ci    = i
      ct    = t
      cdist = d

  # distance to final point on trajectory
  d = np.linalg.norm(traj[i]-x0)
  if d < cdist:
    print "end, ci=",len(traj)-2
    ci = len(traj)-2
    ct = 1

  if ci < min_index:
    ci = int(min_index)
    ct = min_index - ci

  try:
    cr   = traj[ci][0]*(1-ct) + traj[ci+1][0]*ct
    crr  = traj[ci][1]*(1-ct) + traj[ci+1][1]*ct
    crrr = traj[ci][2]*(1-ct) + traj[ci+1][2]*ct
  except IndexError:
    print "Out-of-range:",ci,len(traj)
    ci = min_index

  return cr,crr,crrr,ci+ct

def desired_from_trajectory2(traj,r0,rr0):
  """Find closest point in speed and position"""

  ci = 0
  cdist = None
  for i,(r,rr,rrr) in enumerate(traj):
    dr = np.linalg.norm(r0-r)
    drr = np.linalg.norm(rr0-rr)
    d = dr+drr*2
    if (cdist==None) or (d < cdist):
      ci = i
      cdist = d
  return traj[ci]

def golden_search(f, a, b, tol=1e-5, fvals=[]):
    '''
    golden section search
    to find the minimum of f on [a,b]
    f: a strictly unimodal function on [a,b]

    example:
    >>> f = lambda x: (x-2)**2
    >>> x = golden_search(f, 1, 5)
    >>> x
    2.000009644875678

    '''
    gr = (math.sqrt(5) + 1) / 2
    fc,fd = None,None

    c = b - (b - a) / gr
    d = a + (b - a) / gr 
    while abs(c - d) > tol:
        if fc==None:
          fc=f(c)
        if fd==None:
          fd=f(d)
        fvals.append( (c,fc) )
        fvals.append( (d,fd) )
        if fc < fd:
            b = d
            fd = fc
            fc = None
        else:
            a = c
            fd = fc
            fc = None

        # we recompute both c and d here to avoid loss of precision which may lead to incorrect results or infinite loop
        c = b - (b - a) / gr
        d = a + (b - a) / gr

    return (b + a) / 2

def orthogonal_vectors(R):
  """Return 2 orthogonal vectors to r"""

  R = R/np.linalg.norm(R)
  P = np.array([1.,0.,0.]) # arbitary point
  U = np.cross(P,R)
  V = np.cross(R,U)
  # is P close to co-linear with R?
  if np.linalg.norm(U)<0.1 or np.linalg.norm(V)<0.1:
    P = np.array([0.,1.,0.]) # arbitary point
    U = np.cross(P,R)
    V = np.cross(R,U)
  return U,V
 
def test_basis_weights(): 
  t=0
  while t<10:
    print basis_weights(t,10,3)
    t=t+0.5

if __name__=='__main__':
  print orthogonal_vectors(np.array([1.,1.,0.]))
  print orthogonal_vectors(np.array([1.,0.,0.]))
