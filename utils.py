#!/usr/bin/python

import numpy as np
import math

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

def desired_from_trajectory(traj,r0,rr0):
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

  if ci == len(traj)-1:
    return None,None,None

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
    print "golden_search(a=%.2f,b=%.2f)" % (a,b)
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
