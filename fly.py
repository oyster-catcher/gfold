#!/usr/bin/python

import sys
import krpc
import time
import numpy as np
import utils
import gfold
import simulate
import math

twr_reduction = 0.91
launch_dir = (1.0,0.1,0.1)
traj_dt=0.2

conn = krpc.connect(name='tracking')
vessel = conn.space_center.active_vessel

# Set state
vessel.control.rcs = True
vessel.control.sas = False
# Point up with no throttle
vessel.auto_pilot.target_direction = (1.0,0.0,0.0)
vessel.control.throttle = 0

pos_ksc = np.array([159780.56344896,-1018.08096585,-578410.64800618])
vUp     = pos_ksc/np.linalg.norm(pos_ksc)
vNorth  = np.array([0,1,0])
vEast   = np.cross(vUp,vNorth)
#
i=0
t=0
dt=1

vessel.auto_pilot.target_pitch_and_heading(90,90)
vessel.auto_pilot.engage()
vessel.auto_pilot.reference_frame = vessel.surface_reference_frame

# set faster PID controllers for auto pilot
Kp=5.0
Kd=0.5
stopt=0.2
vessel.auto_pilot.auto_tune = False
vessel.auto_pilot.stopping_time = (stopt,stopt,stopt)
vessel.auto_pilot.deceleration_time = (stopt,stopt,stopt)
vessel.auto_pilot.overshoot = (0.02,0.02,0.02)
vessel.auto_pilot.pitch_pid_gains = (Kp,0,Kd)
vessel.auto_pilot.yaw_pid_gains = (Kp,0,Kd)
#vessel.auto_pilot.roll_pid_gains = (Kp,0,Kd)

# reference frame
ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
  position=vessel.orbit.body.reference_frame,
  rotation=vessel.surface_reference_frame)

# Wait for physics intialisation
time.sleep(0.2)

# Launch on a predetermined flight path
vessel.auto_pilot.target_direction = launch_dir
print "LIFT OFF!"
vessel.control.throttle = 1.0
vessel.control.activate_next_stage()
time.sleep(4)
print "STOP ENGINES!"
vessel.control.throttle = 0.0
# wait until vertical velocity close at zero
rr0 = np.array([0.0,100.0,0.])
# Steer to upright
vessel.auto_pilot.target_direction = (1.0,0.0,0.0)
while rr0[1] > 10.0:
  velocity = vessel.flight(ref_frame).velocity
  velocity = np.array(velocity)
  rr0 = np.array([velocity[2],velocity[0],velocity[1]])
  time.sleep(0.1)

# compute trajectory and follow it
runmode = 1

# Get position and velocity of vessel, compute trajectory and plot
pos = np.array(vessel.position(vessel.orbit.body.reference_frame)) - pos_ksc
r0  = np.array([pos.dot(vEast),pos.dot(vUp),pos.dot(vNorth)])
velocity = vessel.flight(ref_frame).velocity
velocity = np.array(velocity)
rr0 = np.array([velocity[2],velocity[0],velocity[1]])
twr = vessel.available_thrust/vessel.mass
twr = twr*twr_reduction # reduce so we get a higher throttle value
if twr < 0.01:
  twr = 1 # to stop divide by zero
g=np.array([0,-vessel.orbit.body.surface_gravity,0])

# Number of discrete acceleration vectors
N = 5
last_T = None
min_fuel = None
min_rrrs = []

# CALL GOLDEN LINE SEARCH
def f(T):
  global min_rrrs,min_fuel
  # Add compensation to time to solve so add to starting position by 0.2 secs
  fuel, rrr = gfold.solve(r0,rr0,T=T,N=N,g=g,max_thrust=twr,min_thrust=5.0)
  print "SOLVING FOR T=",T, " -> ",fuel
  if (rrr!=None) and (fuel<min_fuel or min_fuel==None):
    min_rrrs = rrr
    min_fuel = fuel
  last_T = T
  return fuel

t1=time.time()
fvals=[]
t1 = time.time()
min_T = utils.golden_search(f,1,60,tol=0.2,fvals=fvals)
t2=time.time()
print "Time to solve: %.3f secs" % (t2-t1)
solve_t = conn.space_center.ut - (t2-t1) # was a slight time in past now

print "Solving for T=",min_T

# Need to do this since golden search could return min_T in infeasible region
rrrs = min_rrrs
#min_fuel,rrrs = gfold.solve(r0,rr0,T=min_T,N=N,g=g,max_thrust=twr)
traj = simulate.compute_trajectory(r0,rr0,rrrs,T=min_T,g=g,dt=traj_dt)

# Draw this trajectory
r1 = None
for i,(r,rr,rrr) in enumerate(traj):
  if i%3==0:
    r2 = pos_ksc + vUp*r[1] + vNorth*r[2] + vEast*r[0]
    if r1!=None:
      l = conn.drawing.add_line( r1,r2,vessel.orbit.body.reference_frame )
      l.thickness = 0.1
      # Force vector
      vF = vUp*rrr[1] + vNorth*rrr[2] + vEast*rrr[0]
      l = conn.drawing.add_line( r2,r2 + 2.0*vF,vessel.orbit.body.reference_frame )
      l.color = (1.0,0.0,0.0)
      l.thickness = 0.2
    r1 = r2

f=file('traj.dat','w')
print >>f,"time rx ry rz vx vy vz ax ay az"
t=0
for r,rr,rrr in traj:
  print >>f, t, "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f" % (r[0],r[1],r[2],rr[0],rr[1],rr[2],rrr[0],rrr[1],rrr[2])
  t=t+traj_dt
f.close()

t_start = time.time()
prev_t = 0

fdat=file("fly4.dat","w")
print >>fdat, "time rx ry rz rrx rry rrz wrrx wrry wrrz attx atty attz"
pos = np.array(vessel.position(vessel.orbit.body.reference_frame)) - pos_ksc
prev_r = np.array([pos.dot(vEast),pos.dot(vUp),pos.dot(vNorth)])
velocity = np.array([0.,0.,0.])
while True:
  t = time.time() - t_start
  dt = t - prev_t
  pos = np.array(vessel.position(vessel.orbit.body.reference_frame)) - pos_ksc
  r = np.array([pos.dot(vEast),pos.dot(vUp),pos.dot(vNorth)])

  # Calculate vessel velocity as a vector
  velocity = vessel.flight(ref_frame).velocity
  velocity = np.array(velocity)
  rr = np.array([velocity[2],velocity[0],velocity[1]])
  # use accelerations from trajectory to set direction and throttle
  twr = twr_reduction * vessel.available_thrust/vessel.mass

  # Follow trajectory
  if runmode == 1:
    # Set Force direction based on time, t
    #ti = (conn.space_center.ut - solve_t)/traj_dt
    #tj = traj[ min(int(ti+0.5),len(traj)-1) ]
    #dr, drr, F = tj # desired pos,vel,accel

    # Set Force/direction based on closest trajectory point
    dr, drr, F = utils.desired_from_trajectory(traj,r)

    throttle = np.linalg.norm(F)/twr

    # Difference to desired position
    g1 = 0.3
    g2 = 0.2
    F = F + (dr-r)*g1 + (drr-rr)*g2

  # final landing
  if runmode == 2:
    h = vessel.flight(ref_frame).bedrock_altitude
    throttle_gain = 0.16
    a_up = twr - vessel.orbit.body.surface_gravity
    v_wanted = min(-0.1,-a_up*0.25*math.sqrt(max(0.1,h-3)))
    err      = (v_wanted - rr[1])
    F        = -rr
    throttle = err * throttle_gain
    if h < 0.5:
      vessel.control.throttle = 0
      sys.exit(0)

  # set direction and throttle
  vessel.auto_pilot.target_direction = (F[1],F[2],F[0])
  vessel.control.throttle = throttle

  if vessel.flight(ref_frame).bedrock_altitude < 25 and rr[1]<0:
    runmode = 2 # final landing
 
  # previous state
  prev_r = r
  prev_t = t

  # write telemetry
  if drr!=None:
    print >>fdat, t,r[0],r[1],r[2],rr[0],rr[1],rr[2],drr[0],drr[1],drr[2],F[0],F[1],F[2]
  fdat.flush()

fdat.close()
