#!/usr/bin/python

import sys
import math
import utils
from cvxopt import matrix, solvers
import numpy as np

min_Tx=0.1 # not used

#######################################################
# G-FOLD ALGORITHM
#######################################################
def solve(r0,rr0,att0,T,N,g,max_thrust=1.0,min_thrust=5.0,maxT_angle=90):

  # minimise   SUM(t0..tN)
  #
  # divide time T into N equal portions
  # ti is magnitude of xi,yi
  #
  # ||(xi,yi)|| < Ti
  #
  # solution x = (t1..tN,x1,y1 ... xN,yN,z1...zN)

  dt = T/float(N-1.0) # Since 3 accel in 10 secs, means they are at 0,5,10
  M = N*5

  # Minimise by this criteria
  c = np.array( np.zeros(4*N) )   # minimise SUM(t0...tN)
  c[0]     = 0.5*dt  # t(0)
  c[1:N-1] = dt      # t(1)...t(N-2)
  c[N-1]   = 0.5*dt  # t(N-1)

  # limit ||(Txi,Tyi,Tzi)|| < Ti

  ################################################################
  # Constraints on magnitude of accelerations related to Ti
  ################################################################
  Gq = []
  for i in range(0,N):
    m = np.zeros( (4,4*N) )
    m[0,i]       = -1.0 # Ti
    m[1,N+i*3]   = 1.0 # xi
    m[2,N+i*3+1] = 1.0 # yi
    m[3,N+i*3+2] = 1.0 # zi
    Gq.append(m)

  # constants
  hq = []
  for i in range(0,N):
    hq.append( np.zeros(4).transpose() )

  ################################################################
  # Constraint for Ti < max_thrust
  ################################################################
  for i in range(0,N):
    m = np.zeros( (2,4*N) )
    m[1,i] = 1.0  # Ti weight
    Gq.append(m)

  # constants
  for i in range(0,N):
    hq.append( np.array([max_thrust,0.]).transpose() )

  ################################################################
  # Constraint for Ti > min_thrust
  ################################################################
  for i in range(0,N):
    m = np.zeros( (2,4*N) )
    m[0,i] = -1.0  # Ti weight
    Gq.append(m)

  # constants
  for i in range(0,N):
    hq.append( np.array([0.,min_thrust]).transpose() )

  ################################################################
  # Constraint for Ty > min_Tx ( to keep thrust upwards)
  ################################################################
  # |min_Tx| <= Tx(i)
  #for i in range(0,N):
  #  m = np.zeros( (2,4*N) )
  #  # RHS, <= below
  #  m[0,N+i*3] = -1.0  # Ty weight
  #  Gq.append(m)

  # constants
  #for i in range(0,N):
  #  hq.append( np.[0.,min_Tx]).transpose() )

  ################################################################
  # Constraint for T within cone angle, maxT_angle, of vertial
  ################################################################
  # |Ty,Tz| < k.Tx
  # m = sin(a)/cos(a)
  a = math.radians(min(maxT_angle,89))
  k = math.sin(a)/math.cos(a)
  print "k:",k
  for i in range(0,N):
    m = np.zeros( (3,4*N) )
    # RHS, <= below
    m[0,N+i*3]   = -k    # Tx weight (gradient)
    # LHS, weights on Tx,Ty,Tz
    m[1,N+i*3+1] = 1.0  # Ty weight
    m[2,N+i*3+2] = 1.0  # Tz weight
    Gq.append(m)

  # constants
  # Note that cone is slightly raised by min_Tx so we have a
  # minimum upwards thrust
  for i in range(0,N):
    hq.append( np.array([min_Tx,0.,0.]).transpose() )

  ################################################################
  # EQUALITY CONSTRAINTS
  ################################################################
  #A = np.zeros( (6,4*N) )
  #b = np.zeros( 6 )
  A = np.zeros( (10,4*N) )
  b = np.zeros( 10 )

  ################################################################
  # Constraint that accelerations sum to 0 (after g applied)
  # and cancel out initial velocity
  ################################################################
  for i in range(0,N):
    # Each period of acceleration lasts dt seconds
    if i==0 or i==(N-1):
      w=0.5*dt
    else:
      w=dt
    A[0,N+i*3]   = w # accel. X
    A[1,N+i*3+1] = w # accel. Y
    A[2,N+i*3+2] = w # accel. Z

  # constants = sum to 0
  b[0] = - g[0]*T - rr0[0]
  b[1] = - g[1]*T - rr0[1]
  b[2] = - g[2]*T - rr0[2]

  ################################################################
  # Constraint that final position is (0,0)
  ################################################################
  for i in range(0,N):
    # Assume T=12
    # N=3, dt=4
    # Divide time into M steps and compute acceleration after this point
    dmt = T/float(M) # time step per M
    sw = 0
    for j in range(0,M):
      t  = dmt*j
      tr = T - t
      w  = utils.basis_weights(t,T,N)
      sw = sw + tr*w[i]*dmt
    A[3,N+i*3]   = sw
    A[4,N+i*3+1] = sw
    A[5,N+i*3+2] = sw

  # constants = sum to 0
  b[3] = -r0[0] -rr0[0]*T -0.5*g[0]*T*T
  b[4] = -r0[1] -rr0[1]*T -0.5*g[1]*T*T
  b[5] = -r0[2] -rr0[2]*T -0.5*g[2]*T*T

  ####### EXTRA CONSTRAINT TO MAKE FIRST VECTOR ATT0 #########
  # Ensure T1 is co-linear with att0
  # by making sure U,V vectors orthogonal to att0 give got products
  # of 0 ensure on the 2 intersecting planes
  # TODO: Ensure T1 is also in correct direction

  U,V = utils.orthogonal_vectors(att0)
  A[6,N]   = U[0]
  A[6,N+1] = U[1]
  A[6,N+2] = U[2]
  b[6]     = 0.0
  A[7,N]   = V[0]
  A[7,N+1] = V[1]
  A[7,N+2] = V[2]
  b[7]     = 0.0

  # Ensure T(N-1)y and T(N-1)z is 0 (thrust is up)
  A[8,N+i*3+1] = 1.0
  b[8] = 0
  A[9,N+i*3+2] = 1.0
  b[9] = 0

  # Ensure T1(X),T1(Y),T1(Z) is in direction att0
  m = np.zeros( (2,4*N) )
  m[0,N] = -np.sign(att0[0]) 
  Gq.append(m)
  hq.append( np.array([0.,0.]).transpose() )

  m = np.zeros( (2,4*N) )
  m[0,N+1] = -np.sign(att0[1]) 
  Gq.append(m)
  hq.append( np.array([0.,0.]).transpose() )

  m = np.zeros( (2,4*N) )
  m[0,N+2] = -np.sign(att0[2]) 
  Gq.append(m)
  hq.append( np.array([0.,0.]).transpose() )

  ################################################################
  # Convert matrices to CVXOPT
  ################################################################
  c = matrix(c.transpose())
  Gq= [ matrix(m) for m in Gq ]
  hq = [ matrix(m) for m in hq ]
  A = matrix(A)
  b = matrix(b)

  # SOLVER SETTINGS
  #solvers.options['maxiters'] = 5
  #solvers.options['show_progress'] = False
  #solvers.options['abstol'] = 0.1
  #solvers.options['feastol'] = 0.1

  # SOLVE!
  sol = solvers.socp(c, Gq=Gq, hq=hq, A=A, b=b)
  if sol['status']!='optimal':
    return float("inf"),None

  rrrs = np.zeros( (N,3) )
  for i in range(0,N):
    rrrs[i,0] = sol['x'][N+i*3]
    rrrs[i,1] = sol['x'][N+i*3+1]
    rrrs[i,2] = sol['x'][N+i*3+2]
  return sol['primal objective'],rrrs
