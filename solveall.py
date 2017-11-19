#!/usr/bin/python

import numpy as np
import simulate
import gfold

f=open("fuel.dat","w")
dat=[]

#######################################################
# MAIN PARAMETERS
#######################################################
N = 20
# X=East, Y=Up, Z=North
r0  = np.array([-15.,40.,0.])
rr0 = np.array([6.,-4.0,4.0])
rf  = np.array([0 ,0, 0])
g   = np.array([0 ,-0.6, 0])
max_thrust = 1.2

# Returns array of accelerations
min_fuel = 9999
min_N    = None
for N in range(5,20,1):
  fuel, rrr = gfold.solve(r0,rr0,N,g,max_thrust=max_thrust)
#  if rrr:
#    positions = simulate.simulate(r0,rr0,rrr,N,g,dt=1)
#    simulate.plot(positions,rrr,title="T=%d, fuel=%.1f" % (N,fuel))
  if rrr and fuel < min_fuel:
    min_fuel = fuel
    min_N    = N
  print >>f, N,fuel
  dat.append( fuel )
f.close()

# Simulate best duration
fuel, rrr = gfold.solve(r0,rr0,min_N,g,max_thrust=max_thrust)
positions = simulate.simulate(r0,rr0,rrr,min_N,g,dt=1)

# Plot best duration
for i,f in enumerate(dat):
  print "%d %.2f" % (5+i,f)
simulate.plot(positions,rrr,title="Optimal T=%d, fuel=%.1f" % (min_N,min_fuel))
