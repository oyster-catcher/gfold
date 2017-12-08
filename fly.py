#!/usr/bin/python

import sys
import krpc
import time
import numpy as np
import utils
import gfold
import simulate
import math
import gui

twr_reduction = 0.85 # leave headroom in thrust required
launch_dir = (1.0,0.05,0.08)
traj_dt=0.2
lift_off = False
predict_time = 0.3 # start solution this much in future
maxT_angle = 45 # maximum angle of thrust from vertical
min_throttle = 0.05

conn = krpc.connect(name='tracking')
vessel = conn.space_center.active_vessel
compute,compute_clicked = gui.setup(conn)

# Set state
vessel.control.rcs = True
vessel.control.sas = False
vessel.control.throttle = 0

# Solving
min_fuel = None
min_rrrs = []

def compute_trajectory(r0,rr0,att0,current_t):
  global min_fuel,min_rrrs,r1,rr1,predict_time

  min_fuel = None
  min_rrrs = []
  r1  = r0 + rr0*predict_time + 0.5*g*predict_time*predict_time
  rr1 = rr0 + g*predict_time

  # CALL GOLDEN LINE SEARCH
  def f(T):
    global min_rrrs,min_fuel,r1,rr1
    # Add compensation to time to solve so add to starting position by 0.2 secs
    num_thrust = int(T/t_per_N)
    num_thrust = max(3,min(num_thrust,max_N))
    print "r0:",r0
    print "r1:",r1
    fuel, rrr = gfold.solve(r1,rr1,att0,T=T,N=num_thrust,g=g,max_thrust=twr,min_thrust=twr*min_throttle,maxT_angle=maxT_angle)
    print "Solving for T=%.2f N=%d fuel=%.1f" % (T,num_thrust,fuel)
    if (rrr!=None) and (fuel<min_fuel or min_fuel==None):
      min_rrrs = rrr
      min_fuel = fuel
    last_T = T
    return fuel

  fvals=[]
  t1 = conn.space_center.ut
  min_T = utils.golden_search(f,1,120,tol=0.2,fvals=fvals)
  t2 = conn.space_center.ut
  print "Time to solve (Space Center time): %.3f secs" % (t2-t1)
  print "Solving for T=",min_T

  # Need to do this since golden search could return min_T in infeasible region
  if min_rrrs==[]:
    print "FAILURE TO SOLVE!!!!"
  rrrs = min_rrrs
  if len(rrrs)>0:
    return simulate.compute_trajectory(r1,rr1,rrrs,T=min_T,g=g,dt=traj_dt),current_t+predict_time
  else:
    return [],0

def draw_trajectory(traj):
  ########## Draw this trajectory ##########
  r1 = None
  for i,(r,rr,rrr) in enumerate(traj):
    if i%3==0:
      r2 = vel2pos.dot(r) + pos_ksc
      if r1!=None:
        l = conn.drawing.add_line( r1,r2,pos_frame )
        l.thickness = 0.1
        # Force vector
        vF = vel2pos.dot(rrr)
        l = conn.drawing.add_line( r2,r2 + 2.0*vF,pos_frame )
        l.color = (1.0,0.0,0.0)
        l.thickness = 0.2
      r1 = r2

def save_trajectory(traj,fname,solve_t):
  f=file(fname,'w')
  print >>f,"time x y z vx vy vz ax ay az"
  t=0
  for r,rr,rrr in traj:
    print >>f, t+solve_t, "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f" % (r[0],r[1],r[2],rr[0],rr[1],rr[2],rrr[0],rrr[1],rrr[2])
    t=t+traj_dt
  f.close()

############################# SET UP REFERENCE FRAMES #############################
# reference frame
pos_frame = vessel.orbit.body.reference_frame
vel_frame = conn.space_center.ReferenceFrame.create_hybrid(
  position=vessel.orbit.body.reference_frame,
  rotation=vessel.surface_reference_frame)

# Co-Ords in surface (not body)
#   X = up
#   Y = North
#   Z = East
ksc_lat,ksc_lng = -0.0972071692165,-74.5576808614
ksc_height = 5
#ksc_lat,ksc_lng = -0.0972071692165,-74.5576808614 - 0.0595 # H landing pad
#ksc_height = 150

pos_ksc = np.array(vessel.orbit.body.surface_position( ksc_lat,ksc_lng,pos_frame ))
#pos_ksc = np.array(vessel.position(pos_frame))  # starting point - a little off bedrock heigh

# rotation matrix to convert position into the rotation of surface_reference_frame
pos_kscY = vessel.orbit.body.surface_position( ksc_lat+1,ksc_lng,pos_frame )
pos_kscZ = vessel.orbit.body.surface_position( ksc_lat,ksc_lng+1,pos_frame )
vX = pos_ksc # up vector is continuation away from origin
vY = pos_kscY - pos_ksc
vZ = pos_kscZ - pos_ksc
# normalise
vX = vX/np.linalg.norm(vX)
vY = vY/np.linalg.norm(vY)
vZ = vZ/np.linalg.norm(vZ)
pos2vel = np.array([vX,vY,vZ])
vel2pos = np.linalg.inv(pos2vel)

# adjust landing pos as its slightly raised on platform
#pos_ksc = pos_ksc + vX*7
pos_ksc = pos_ksc + vX*ksc_height

################################ SET UP AUTOPILOT #############################
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
vessel.auto_pilot.roll_pid_gains = (Kp,0,Kd)

################################ LIFT OFF #############################
# Wait for physics intialisation
time.sleep(0.2)

if lift_off:
  # Launch on a predetermined flight path
  vessel.auto_pilot.target_direction = launch_dir
  print "LIFT OFF!"
  vessel.control.throttle = 1.0
  vessel.control.activate_next_stage()
  time.sleep(4.0)
  print "STOP ENGINES!"
  vessel.control.throttle = 0.0
  # Steer to upright
  vessel.auto_pilot.target_direction = (1.0,0.0,0.0)
  rr0 = np.array([100.0,0.0,0.0])
  while rr0[0] > 10.0: # vertical speed
    rr0 = np.array(vessel.flight(vel_frame).velocity)
    print rr0
    time.sleep(0.1)

# compute trajectory and follow it
runmode = 1

######################### COMPUTE TRAJECTORY ##########################
# Get position and velocity of vessel, compute trajectory and plot
r0 = np.array(vessel.position(pos_frame)) - pos_ksc
r0 = pos2vel.dot(r0)
rr0 = np.array(vessel.flight(vel_frame).velocity)
twr = vessel.available_thrust/vessel.mass
twr = twr*twr_reduction # reduce so we get a higher throttle value
if twr < 0.01:
  twr = 1 # to stop divide by zero
g=np.array([-vessel.orbit.body.surface_gravity,0,0])

# Number of discrete acceleration vectors
t_per_N = 5 # Number of thrust vector per T seconds
max_N   = 8 # maximum number of thrust vectors
last_T = None


#### COMPUTE TRAJECTORY ####
start_t = conn.space_center.ut
traj,solve_t = compute_trajectory(r0,rr0,vessel.flight(vel_frame).direction,0)
draw_trajectory(traj)
save_trajectory(traj,"traj.dat",solve_t)
# Draw line to (0,0,0)
l=conn.drawing.add_line(vessel.position(pos_frame),pos_ksc,pos_frame)
l.color = (1.0,0.0,0.0)

t_start = time.time()
prev_t = 0

fdat=file("fly.dat","w")
# x,y,z(position) vx,vy,vz(velocity) ax,ay,az(actual accel) tx,ty,tz(autopilot steer/thrust) throttle
print >>fdat, "time x y z vx vy vz ax ay az tx ty tz throttle"
pos = np.array(vessel.position(pos_frame)) - pos_ksc
pos = pos2vel.dot(pos)
prev_r = pos
velocity = np.array([0.,0.,0.])
lshould=None
min_index=0
start_t = conn.space_center.ut
drr = np.array([1.,0.,0.])
##################################### MAIN LOOP ########################################
while True:
  
  t = conn.space_center.ut - start_t
  dt = t - prev_t
  r = np.array(vessel.position(pos_frame)) - pos_ksc
  r = pos2vel.dot(r)

  # Calculate vessel velocity as a vector
  velocity = vessel.flight(vel_frame).velocity
  rr = np.array(velocity)
  # use accelerations from trajectory to set direction and throttle
  twr = twr_reduction * vessel.available_thrust/vessel.mass

  # Compute new trajectory?
  if compute_clicked():
    compute.clicked = False
    print "Recomputing trajectory..."
    vessel.control.throttle = 0
    traj,solve_t = compute_trajectory(r,rr,vessel.flight(vel_frame).direction,t)
    conn.drawing.clear()
    lshould = None
    draw_trajectory(traj)
    print "done"
    runmode = 1

  # if no trajectory switch to final landing mode all the way
  if len(traj)==0 and runmode == 1:
    print "No solution - switch to final landing mode"
    runmode = 2

  # Follow trajectory
  if runmode == 1:
    # Set Force direction based on time, t
    #ti = (t - solve_t)/traj_dt
    #tj = traj[ min(int(ti+0.5),len(traj)-1) ]
    #dr, drr, F = tj # desired pos,vel,accel

    # Set Force/direction based on closest trajectory point
    #dr, drr, F, index = utils.desired_from_trajectory(traj,r,min_index=min_index)
    #if (min_index==None) or (index>min_index):
    #  min_index = index
    # uses combination of position and velocity distance
    dr, drr, F = utils.desired_from_trajectory2(traj,r,rr)

    # Draw a line to where we should be
    if lshould!=None:
      lshould.remove()
    lshould = conn.drawing.add_line( vel2pos.dot(r)+pos_ksc,vel2pos.dot(dr)+pos_ksc,pos_frame )

    # Difference to desired position
    # wav 0.2,0.4 for larger rockets
    g1 = 0.2
    g2 = 0.4
    F = F + (dr-r)*g1 + (drr-rr)*g2
    throttle = np.linalg.norm(F)/twr

  # height above local surface height
  h = vessel.flight(vel_frame).surface_altitude

  # final landing
  if runmode == 2:
    throttle_gain = 0.2
    a_up = twr - vessel.orbit.body.surface_gravity
    v_wanted = min(-2.5,-a_up*0.25*math.sqrt(max(0.01,h-5)))
    err      = (v_wanted - rr[0])
    F        = -rr
    throttle = err * throttle_gain + (vessel.orbit.body.surface_gravity/twr) # last component to hover
    # tweak attitude to be a little more vertical
    F[0] = F[0] + 5.0

    # Have we landed?
    #if h < 0 and np.linalg.norm(rr[0]) < 0.01:
    #  vessel.control.throttle = 0
    #  sys.exit(0)

  # handle case where attitude and desired throttle directions are very different
  att = vessel.flight(vel_frame).direction
  ddot = np.dot(F/np.linalg.norm(F),att/np.linalg.norm(att))
  if ddot < 0:
    print "dot(F,att)=%.1f -> no throttle" % ddot
    throttle = 0

  # set direction and throttle
  vessel.auto_pilot.target_direction = (F[0]+5.0,F[1],F[2])
  vessel.control.throttle = throttle

  # 20m over landing spot and falling
  if (h < 25) and (rr[0] < 0) and (runmode == 1):
    print "FINAL LANDING (at %.1fm)" % vessel.flight(pos_frame).bedrock_altitude
    runmode = 2 # final landing
 
  # previous state
  prev_r = r
  prev_t = t

  # write telemetry
  if drr!=None:
    print >>fdat, t,r[0],r[1],r[2],rr[0],rr[1],rr[2],att[0]*throttle*twr,att[1]*throttle*twr,att[2]*throttle*twr,F[0],F[1],F[2],throttle
  fdat.flush()

fdat.close()
