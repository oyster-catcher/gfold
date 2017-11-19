#!/usr/bin/python

from cvxopt import matrix, solvers
import numpy as np

#######################################################
# G-FOLD ALGORITHM
#######################################################
def solve(r0,rr0,N,g=np.array([0,-0.5]),max_thrust=1.0):
  # minimise   SUM(t0..tN)
  #
  # ti is magnitude of xi,yi
  #
  # ||(xi,yi)|| < Ti
  #
  # solution x = (t1..tN,x1,y1 ... xN,yN,z1...zN)

  # Try to make from scratch
  c = np.array( np.zeros(4*N) )   # minimise SUM(t0...tN)
  c[0:N]=1.0 # t0...tN

  # limit ||(xi,yi)|| < Ti

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
  # Constraint for Ti > 0
  ################################################################
  for i in range(0,N):
    m = np.zeros( (2,4*N) )
    m[0,i] = -1.0  # Ti
    Gq.append(m)

  # constants
  for i in range(0,N):
    hq.append( np.array([0.,0.]).transpose() )

  ################################################################
  # Constraint for Ti < max_thrust
  ################################################################
  for i in range(0,N):
    m = np.zeros( (2,4*N) )
    m[1,i] = 1.0  # Ti
    Gq.append(m)

  # constants
  for i in range(0,N):
    hq.append( np.array([max_thrust,0.]).transpose() )

  ################################################################
  # EQUALITY CONSTRAINTS
  ################################################################
  A=np.zeros( (6,4*N) )
  b=np.zeros( 6 )

  ################################################################
  # Constraint that accelerations sum to 0 (after g applied)
  # and cancel out initial velocity
  ################################################################
  for i in range(0,N):
    A[0,N+i*3]   = 1.0 # accel. X
    A[1,N+i*3+1] = 1.0 # accel. Y
    A[2,N+i*3+2] = 1.0 # accel. Z

  # constants = sum to 0
  b[0] = - g[0]*N - rr0[0]
  b[1] = - g[1]*N - rr0[1]
  b[2] = - g[2]*N - rr0[2]

  ################################################################
  # Constraint that final position is (0,0)
  ################################################################
  for i in range(0,N):
    t=max(0,N-i-1) # time after acceleration has ended
    A[3,N+i*3]   = max(0,t+0.5) # accel. X, 0.5 is 1 sec during accel 0.5*a*t^2
    A[4,N+i*3+1] = max(0,t+0.5)  # accel. Y
    A[5,N+i*3+2] = max(0,t+0.5)  # accel. Z

  # constants = sum to 0
  b[3] = -r0[0] -rr0[0]*N -0.5*g[0]*N*N
  b[4] = -r0[1] -rr0[1]*N -0.5*g[1]*N*N
  b[5] = -r0[2] -rr0[2]*N -0.5*g[2]*N*N

  ################################################################
  # Convert matrices to CVXOPT
  ################################################################
  c = matrix(c.transpose())
  Gq= [ matrix(m) for m in Gq ]
  hq = [ matrix(m) for m in hq ]
  A = matrix(A)
  b = matrix(b)

  # SOLVE!
  sol = solvers.socp(c, Gq=Gq, hq=hq, A=A, b=b)
  print "status:",sol['status']
  if sol['status']!='optimal':
    return 0,None

  # Run simulation
  rrr = []
  for i in range(0,N):
    rrr.append( np.array([sol['x'][N+i*3],sol['x'][N+i*3+1],sol['x'][N+i*3+2]]) )
  return sol['primal objective'],rrr
