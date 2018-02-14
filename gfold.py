#!/usr/bin/python2.7

import sys
import math
import utils
from cvxopt import matrix, solvers
import numpy as np

min_Tx=0.1 # not used

#######################################################
# G-FOLD ALGORITHM
#######################################################
def solve(r0,rr0,att0,T,N,g,max_thrust=1.0,min_thrust=0.1,maxT_angle=90,safe_angle=10,dt=0.1,maxLand_angle=10,min_height=20):

  T = float(T)
  # minimise   SUM(t0..tN)
  #
  # divide time T into N equal portions
  # ti is magnitude of xi,yi
  #
  # ||(xi,yi)|| < Ti
  #
  # solution x = (t1..tN,x1,y1 ... xN,yN,z1...zN)

  # Minimise by this criteria
  c = np.array( np.zeros(4*N) )   # minimise SUM(t0...tN)
  c[0]     = T/(N-1)  # t(0)
  c[1:N-1] = T/N      # t(1)...t(N-2)
  c[N-1]   = T/(N-1)  # t(N-1)

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
    # RHS
    m[0,i] = -1.0  # Ti weight
    Gq.append(m)

  # constants
  for i in range(0,N):
    hq.append( np.array([0.,min_thrust]).transpose() )

  ################################################################
  # Constraint for T within cone angle, maxT_angle, of vertial
  ################################################################
  # |Ty,Tz| < k.Tx
  # m = sin(a)/cos(a)
  # Note T(0) is ignored since later this is tied to the initial attitude
  # of the craft
  for i in range(1,N):
    m = np.zeros( (3,4*N) )
    if i < N-1:
      a = math.radians(min(maxT_angle,89))
    else:
      a = math.radians(min(maxLand_angle,89))
    k = math.sin(a)/math.cos(a)
    # RHS, <= below
    m[0,N+i*3]   = -k    # Tx weight (gradient)
    # LHS, weights on Tx,Ty,Tz
    m[1,N+i*3+1] = 1.0  # Ty weight
    m[2,N+i*3+2] = 1.0  # Tz weight
    Gq.append(m)

  # constants
  # Note that cone is slightly raised by min_Tx so we have a
  # minimum upwards thrust
  for i in range(1,N):
    hq.append( np.array([min_Tx,0.,0.]).transpose() )

  ################################################################
  # EQUALITY CONSTRAINTS
  ################################################################
  A = np.zeros( (8,4*N) )
  b = np.zeros( 8 )

  ################################################################
  # Constraint that accelerations sum mean vel. vector=(0,0,0) (after g applied)
  # canceling out initial velocity
  ################################################################
  for i in range(0,N):
    # Assume T=12
    # N=3, dt=4
    # Divide time into M steps and compute acceleration after this point
    sw = 0
    t = 0.0
    while(t < T):
      w  = utils.basis_weights(t,T,N)
      sw = sw + w[i]*dt
      t  = t + dt
    A[0,N+i*3]   = sw
    A[1,N+i*3+1] = sw
    A[2,N+i*3+2] = sw

  # constants = sum to 0
  b[0] = - (g[0]*T + rr0[0])
  b[1] = - (g[1]*T + rr0[1])
  b[2] = - (g[2]*T + rr0[2])

  ################################################################
  # Constraint that final position is (0,0)
  ################################################################
  for i in range(0,N):
    # Assume T=12
    # N=3, dt=4
    # Divide time into M steps and compute acceleration after this point
    sw = 0
    t = 0
    while( t<T ):
      tr = T - t - dt
      w  = utils.basis_weights(t,T,N)
      sw = sw + tr*w[i]*dt + 0.5*w[i]*dt*dt
      t = t + dt
    A[3,N+i*3]   = sw
    A[4,N+i*3+1] = sw
    A[5,N+i*3+2] = sw

  # constants = sum to 0
  b[3] = -(r0[0] + rr0[0]*T + 0.5*g[0]*T*T)
  b[4] = -(r0[1] + rr0[1]*T + 0.5*g[1]*T*T)
  b[5] = -(r0[2] + rr0[2]*T + 0.5*g[2]*T*T)

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
  #A[8,N+i*3+1] = 1.0
  #b[8] = 0
  #A[9,N+i*3+2] = 1.0
  #b[9] = 0

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
  # Add constraint to keep trajectory above surface
  ################################################################
  # m[1..].X + hq[1] > m[0].X + hq[0]
  # use k=-k to invert inequality

  posm = []

  # proportion of total time we want position of
  prop = 0.1
  while prop <= 0.9:
    mid_t = T*prop
    # Ensure T1(X),T1(Y),T1(Z) is in direction att0
    m = np.zeros( (3,4*N) )
    for i in range(0,N): # each thrust vector
      # Assume T=12
      # N=3, dt=4
      wa = 0
      t = 0
      while( t<mid_t ):
        tr = mid_t - t - dt
        w  = utils.basis_weights(t,T,N)
        wa = wa + tr*w[i]*dt + 0.5*w[i]*dt*dt
        t = t + dt
      # RHS
      m[0,N+i*3]   = -wa # weight in x(i) acceleration
      # LHS weights on magnitude
      sw = math.tan(math.radians(safe_angle))
      sw = 0
      m[1,N+i*3+1] = wa*sw
      m[2,N+i*3+2] = wa*sw

    Gq.append(m)

    x_final = r0[0] + rr0[0]*mid_t + 0.5*g[0]*mid_t*mid_t
    y_final = r0[1] + rr0[1]*mid_t + 0.5*g[1]*mid_t*mid_t
    z_final = r0[2] + rr0[2]*mid_t + 0.5*g[2]*mid_t*mid_t
    # x_final is on RHS as constant
    # |y_final*W,z_final*W| <= -( x_final + sum(accelerations)*weights )

    hq.append( np.array([x_final,min_height,0.]).transpose() )

    prop = prop + 0.1

    posm.append( (mid_t,-m[0,:],x_final) )

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
  solvers.options['abstol'] = 0.5
  #solvers.options['feastol'] = 0.1

  # SOLVE!
  sol = solvers.socp(c, Gq=Gq, hq=hq, A=A, b=b)
  if sol['status']!='optimal':
    return float("inf"),None

  accels = np.zeros( (N,3) )
  for i in range(0,N):
    accels[i,0] = sol['x'][N+i*3]
    accels[i,1] = sol['x'][N+i*3+1]
    accels[i,2] = sol['x'][N+i*3+2]

  return sol['primal objective'],accels

if __name__=='__main__':
  # Test
  r0 = np.array([20.,50.,0.])
  v0 = np.array([0.,1.,0.])
  g = np.array([-9.8,0.,0.])
  att = np.array([1.,0.,0.])
  for t in range(0,50,1):
    T = float(t)
    # def solve(r0,rr0,att0,T,N,g,max_thrust=1.0,min_thrust=0.1,maxT_angle=90,safe_angle=10,dt=0.1,maxLand_angle=10,min_height=20):
    fuel, accels = solve(r,v,att,T,N,g,max_thrust=20.,min_thrust=0.5,maxT_angle=15,safe_angle=10,dt=0.1,maxLand_angle=5,min_height=0)
    print T,fuel
