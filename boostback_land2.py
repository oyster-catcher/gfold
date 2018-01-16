#!/usr/bin/python
#
# More complex version that deals with slowing down different craft
# If a small craft is going to fast it needs parachutes

import sys
import krpc
import time
import numpy as np
import utils
import gfold
import simulate
import glidepath3 as glidepath # rotating planet ref frame
import math
import gui

global pos_frame,pos2vel,vel2pos,pos_ksc,last_T,traj_dt

launch_dir = (1.0,0.0,0.0)
traj_dt = 0.25
predict_time = 1.5 # start solution this much in future
maxT_angle = 45 # for smaller rocket
maxT_angle = 8 # for larger rocket
maxLand_angle = 2
min_throttle = 0.4
max_throttle = 1.0
landing_factor = 0.10
throttle_gain = 0.1
cog_height = 15
boostback_gain = 0.0001 # throttle/m
boostback_maxerr = 100 # get prediction within this many metres
Cd = 1.1 # large lander
Cd = 1.5 # F9R
slow_speed = 40
slow_height = 800
gfold_T_err = 0.25

conn = krpc.connect(name='tracking')
vessel = conn.space_center.active_vessel
buttons,clicked = gui.setup(conn)

# Set up reference frames
pos_frame = vessel.orbit.body.reference_frame
vel_frame = conn.space_center.ReferenceFrame.create_hybrid(
  position=vessel.orbit.body.reference_frame,
  rotation=vessel.surface_reference_frame)
rotspeed = vessel.orbit.body.rotational_speed
print "rotspeed:",rotspeed

# Set up targets
tgts={}
tgts["Rocket Pad"] = (-0.0972071692165,-74.5576808614,10)
tgts["H Pad"]      = (-0.0972071692165+0.0004,-74.5576808614 - 0.0595 - 0.0001,112)
tgts["Old Runway"] = (-1.51901098107,-71.9005276576,150)
tgts["Drone Ship"] = (-0.1,-67.8,20.0)
tgts["Drone Ship"] = (-0.1,-68.,20.0)
diff=0.0051 # was 46
tgts["Tower"] = (-0.0972071692165+diff,-74.5576808614+diff,44)
tgt_name="Rocket Pad"
#tgt_name="Old Runway"
tgt_name="Drone Ship"
buttons['target'].content = tgt_name

def get_transform(body,lat,lng,ref_frame):
  pos   = np.array(body.surface_position( lat,lng,ref_frame ))
  pos_Y = np.array(body.surface_position( lat+1,lng,ref_frame ))
  pos_Z = np.array(body.surface_position( lat,lng+1,ref_frame ))
  vX = pos
  vY = pos_Y - pos
  vZ = pos_Z - pos
  # normalise
  vX = vX/np.linalg.norm(vX)
  vY = vY/np.linalg.norm(vY)
  vZ = vZ/np.linalg.norm(vZ)
  return pos,np.array([vX,vY,vZ])

def set_target(name):
  global pos2vel,vel2pos,vessel,pos_frame,vel_frame,vX

  lat,lng,height = tgts[name]
  pos_tgt = np.array(vessel.orbit.body.surface_position( lat,lng,pos_frame ))

  # adjust landing pos as its slightly raised on platform
  vX = pos_tgt/np.linalg.norm(pos_tgt)
  pos_tgt = pos_tgt + vX*height
  return pos_tgt

# rotation matrix to convert position into the rotation of surface_reference_frame
pos_ksc = set_target(tgt_name)
ksc_lat,ksc_lng,ksc_height = tgts[tgt_name]
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

# Solving
min_fuel = None
min_rrrs = []
fuels = {}

########## Compute trajectory for landing ##########
def compute_trajectory(r0,rr0,att0,current_t,gfold_max_T=120):
  global min_fuel,min_rrrs,r1,rr1,predict_time,twr,fuels

  min_fuel = None
  min_rrrs = []
  r1  = r0 + rr0*predict_time + 0.5*g*predict_time*predict_time
  rr1 = rr0 + g*predict_time

  # CALL GOLDEN LINE SEARCH
  def f(T):
    global min_rrrs,min_fuel,r1,rr1,min_T,last_T
    print "Evaluating f(T=%.3f)" % T
    # Add compensation to time to solve so add to starting position by 0.2 secs
    num_thrust = int(T/t_per_N)
    num_thrust = max(3,min(num_thrust,max_N))
    fuel, rrr, thrusts, xpos = gfold.solve(r1,rr1,att0,T=T,N=num_thrust,g=g,max_thrust=twr*max_throttle,min_thrust=9.8,maxT_angle=maxT_angle,maxLand_angle=maxLand_angle)
    #fuel, rrr, thrusts, xpos = gfold.solve(r1,rr1,att0,T=T,N=num_thrust,g=g,max_thrust=twr*max_throttle,min_thrust=twr*min_throttle,maxT_angle=maxT_angle,maxLand_angle=maxLand_angle)
    fuels[T] = fuel
    print "Solving for T=%.2f N=%d fuel=%.1f" % (T,num_thrust,fuel)
    if (rrr!=None) and (fuel<min_fuel or min_fuel==None):
      min_rrrs = rrr
      min_fuel = fuel
    last_T = T
    return fuel

  fvals=[]
  t1 = conn.space_center.ut
  fuels = {}
  min_T = utils.golden_search(f,1,gfold_max_T,tol=gfold_T_err,fvals=fvals)
  t2 = conn.space_center.ut
  print "Solving for T=",min_T,"(solving took %.2f simulation secs)" % (t2-t1)

  # Need to do this since golden search could return min_T in infeasible region
  if min_rrrs==[]:
    print "FAILURE TO SOLVE!!!! - May your souls Rest In Peace!"
  if len(min_rrrs)>0:
    traj,solve_t = simulate.compute_trajectory(r1,rr1,min_rrrs,T=min_T,g=g,dt=traj_dt),current_t+predict_time
    traj = simulate.correct_trajectory(traj)
    return traj,solve_t
  else:
    return [],0

########## Draw this trajectory ##########
def draw_trajectory(traj,vel2pos,pos_frame,pos_ksc):
  r1 = None
  for i,(r,rr,rrr) in enumerate(traj):
    if i%3==0:
      r2 = vel2pos.dot(r) + pos_ksc
      if r1!=None:
        l = conn.drawing.add_line( r1,r2,pos_frame )
        l.thickness = 0.3
        # Force vector
        vF = vel2pos.dot(rrr)
        l = conn.drawing.add_line( r2,r2 + 2.0*vF,pos_frame )
        l.color = (1.0,0.0,0.0)
        l.thickness = 0.3
      r1 = r2

def draw_lines(positions,vel2pos,pos_frame,pos_origin,thickness=1):
  lines = []
  num = len(positions)
  for i in range(0,num):
    r1 = vel2pos.dot(positions[i])         + pos_origin
    r2 = vel2pos.dot(positions[(i+1)%num]) + pos_origin
    l = conn.drawing.add_line( r1,r2,pos_frame)
    l.color = (1.0,0.0,0.0)
    l.thickness = thickness
    lines.append(l)
  return lines

########## Save trajectory to file ##########
def save_trajectory(traj,fname,solve_t=0,dt=1):
  f=file(fname,'w')
  print >>f,"time x y z vx vy vz ax ay az"
  t=0
  for r,rr,rrr in traj:
    print >>f, t+solve_t, "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f" % (r[0],r[1],r[2],rr[0],rr[1],rr[2],rrr[0],rrr[1],rrr[2])
    t=t+dt
  f.close()

def set_runmode(a_runmode):
  global runmode,latt,lpredict

  if a_runmode==0:
    vessel.auto_pilot.disengage()
    vessel.control.rcs = False
    vessel.control.sas = True
  else:
    vessel.auto_pilot.engage()
    vessel.control.rcs = True
    vessel.control.sas = False
  conn.drawing.clear()
  latt,lpredict = None,[]
  runmode = a_runmode
  print "Switching to runmode",a_runmode

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

################################ SET UP AUTOPILOT #############################
vessel.auto_pilot.target_pitch_and_heading(90,90)
vessel.auto_pilot.reference_frame = vessel.surface_reference_frame

# set faster PID controllers for auto pilot
# Kp=2.5, Kd=0.5 for smaller craft
#Kp=22 # was 5.0
#Kd=4 # was 0.5
#stopt=0.5
#vessel.auto_pilot.auto_tune = False
#vessel.auto_pilot.stopping_time = (stopt,stopt,stopt)
#vessel.auto_pilot.deceleration_time = (stopt,stopt,stopt)
#vessel.auto_pilot.overshoot = (0.02,0.02,0.02)
#vessel.auto_pilot.pitch_pid_gains = (Kp,0,Kd)
#vessel.auto_pilot.yaw_pid_gains = (Kp,0,Kd)
#vessel.auto_pilot.roll_pid_gains = (Kp,0,Kd)

################################ SET INITIAL RUNMODE #############################
runmode = 0 # free flying
#runmode = 1 # follow computed trajectory
#runmode = 2 # final landing (or trajectory solution failed)
runmode = 3 # boostback
set_runmode(runmode)

g=np.array([-vessel.orbit.body.surface_gravity,0,0])

# Formulae for number of thrust vectors
t_per_N = 3 # Number of thrust vector per T seconds
max_N   = 8 # maximum number of thrust vectors
max_N   = 5 # for final descent only
last_T  = None

############### Log telemetry to file #################
fdat=file("fly.dat","w")
fpred=file("predictions.dat","w")
# x,y,z(position) vx,vy,vz(velocity) ax,ay,az(actual accel) tx,ty,tz(autopilot steer/thrust) throttle
print >>fdat, "time runmode x y z vx vy vz ax ay az tx ty tz throttle"
print >>fpred, "time runmode dy dz"
pos = np.array(vessel.position(pos_frame)) - pos_ksc
pos = pos2vel.dot(pos)
latt,lpredict = None,[]
start_t = conn.space_center.ut
traj = []
r = np.array([0.,0.,0.])
rr = np.array([0.,0.,0.])
last_t = start_t
##################################### MAIN LOOP ########################################
while True:
  ds = conn.space_center.target_vessel
  if ds:
    #print "Target drone ship lat:%.3f lng:%.3f" % (ds.flight().latitude,ds.flight().longitude)
    tgts["Drone Ship"] = (ds.flight().latitude,ds.flight().longitude,ds.flight().surface_altitude)
  twr = vessel.available_thrust/vessel.mass
  if twr < 0.001:
    twr = 1 # for vehicle that has no thrust left, don't get divide by zero

  t = conn.space_center.ut - start_t
  last_r = r
  r = np.array(vessel.position(vessel.orbit.body.reference_frame)) - pos_ksc
  r = pos2vel.dot(r)
  att = vessel.flight(vel_frame).direction
  # height above local surface height
  h = vessel.flight(vel_frame).surface_altitude

  # calculate smoothed velocity as difference in r
  velocity = vessel.flight(pos_frame).velocity
  rr = pos2vel.dot(np.array(velocity))
  # use accelerations from trajectory to set direction and throttle
  throttle = vessel.control.throttle
  F = np.zeros(3)

  # Compute new trajectory?
  if clicked['compute']():
    buttons['compute'].clicked = False
    vessel.control.throttle = 0
    pos_ksc = set_target(tgt_name)
    traj,solve_t = compute_trajectory(r,rr,vessel.flight(vel_frame).direction,t)
    save_trajectory(traj,"traj.dat",solve_t,dt=traj_dt)
    set_runmode(1)
    draw_trajectory(traj,vel2pos,pos_frame,pos_ksc)

  # Change target
  if clicked['change']():
    buttons['change'].clicked = False
    names=tgts.keys()
    i=names.index(tgt_name)
    tgt_name = names[(i+1)%len(tgts)]
    pos_ksc = set_target(tgt_name)
    print "Setting target to:",tgt_name
    buttons['target'].content = tgt_name

  if clicked['autoOn']():
    buttons['autoOn'].clicked = False
    if len(traj)>0:
      set_runmode(1)
    else:
      set_runmode(2)

  if clicked['autoOff']():
    buttons['autoOff'].clicked = False
    set_runmode(0)

  if clicked['emergency']():
    buttons['emergency'].clicked = False
    set_runmode(2)

  # if no trajectory switch to final landing mode all the way
  if len(traj)==0 and runmode == 1:
    runmode = 2
    print "No solution - Switching to runmode",runmode,"(safe landing)"

  # Follow trajectory
  if runmode == 1:
    # Set Force/direction based on closest trajectory point in speed/position
    dr, drr, F = utils.desired_from_trajectory(traj,r,rr)

    if dr==None: # off-end of trajectory
      runmode = 2
      continue

    # Difference to desired position and velocity gains
    g1 = 0.2
    g2 = 0.4
    F = F + (dr-r)*g1 + (drr-rr)*g2
    throttle = np.linalg.norm(F)/twr
    # correction to steer more vertical for large rockets
    F = np.array([F[0]+2,F[1],F[2]])

    # close to ground and falling, switch to non-trajectory soft landing
    if (h-cog_height < 20) and (rr[0] < 10) and (runmode == 1):
      runmode = 2 # final landing
      print "Switching to runmode",runmode,"(final landing) at",h,"metres"

  # final landing
  if runmode == 2:
    vessel.control.brakes = True
    a_up = twr - vessel.orbit.body.surface_gravity
    v_wanted = min(-5.0,-a_up*landing_factor*math.sqrt(max(0.01,h-cog_height-5)))
    err      = (v_wanted - rr[0])
    F        = -rr
    throttle = err * throttle_gain + (vessel.orbit.body.surface_gravity/twr) # last component to hover
    # tweak attitude to be more vertical to avoid oscillations
    F[0] = F[0] + 15.0

  # Compute predicted position and draw it
  if runmode == 3 or runmode == 4:
    # co-ords are rotating planet co-ord system
    pr = np.array(vessel.position(vessel.orbit.body.non_rotating_reference_frame))
    prr = np.array(vessel.flight(vessel.orbit.body.non_rotating_reference_frame).velocity)
    # for large lander - should really change mass and frontal area too
    if r[0] > 15000:
      pos_predict,impact_time,glide = glidepath.simulate(pr,prr,dt=5.0,rotspeed=rotspeed,Cd=Cd,twr=2,slow_height=slow_height,slow_speed=slow_speed)
    else:
      pos_predict,impact_time,glide = glidepath.simulate(pr,prr,dt=0.5,rotspeed=rotspeed,Cd=Cd,twr=2,slow_height=slow_height,slow_speed=slow_speed)
    # compute glide slope angle
    #dist_fall   = -vessel.flight(vel_frame).vertical_speed
    #dist_ground = vessel.flight(vel_frame).horizontal_speed
    #slope = math.degrees(math.tan( dist_fall/dist_ground ))
    #print "Glide slope: %.1f degrees (ground=%1.f,fall=%.1f)" % (slope,dist_ground,dist_fall)

    if pos_predict==None:
      print "Glidepath doesn't intersect surface - firing retrograde"
      F = -rr
      throttle = 1
    else:
      lat,lng,h = tgts[tgt_name]
      pos_tgt,rot = get_transform(vessel.orbit.body,lat,lng,vessel.orbit.body.non_rotating_reference_frame)

      # rotate target position by impact time
      x,z = utils.rotate(pos_tgt[0],pos_tgt[2],rotspeed*impact_time)
      pos_tgt = np.array([x,pos_tgt[1],z])
      pos_predict = pos_predict - np.array(pos_tgt)

      # If low draw prediction of impact
      if runmode==4:
        size = max(80,r[0]*0.05)
        thickness = size*0.02
        for l1 in lpredict:
          for l2 in l1:
            l2.remove()
        #pos_predict = np.array([0.,1000.,0.])
        # rotate into axes frame with X vertical, Y North, Z East
        pos_predict = rot.dot(pos_predict)
        lpredict = [ draw_lines([np.array([1.,pos_predict[1]-size,pos_predict[2]]),
                                 np.array([1.,pos_predict[1]+size,pos_predict[2]])],
                              vel2pos,pos_frame,pos_ksc,thickness=thickness),
                    draw_lines([np.array([1.,pos_predict[1],pos_predict[2]-size]),
                                np.array([1.,pos_predict[1],pos_predict[2]+size])],
                              vel2pos,pos_frame,pos_ksc,thickness=thickness) ]

      print "Prediction from target: %.fm N, %.2fm East. Impact time: %.1f" % (pos_predict[1],pos_predict[2],impact_time)
      print >>fpred, t, runmode, pos_predict[1],pos_predict[2]

  # Boostback
  if runmode == 3 and pos_predict!=None:
    F = -pos_predict
    #F[0] = np.linalg.norm(F) * 0.3 # to angle this proportion upwards
    throttle = np.linalg.norm(F) * boostback_gain # 10km = 1% throttle

    ddot = np.dot(F/np.linalg.norm(F),att/np.linalg.norm(att))
    if ddot < 0.95:
      throttle = 0.01 # enough to steer

    # need to stage?
    if vessel.available_thrust < 1: # out of fuel?
      vessel.control.activate_next_stage()

    if (np.linalg.norm(pos_predict) < boostback_maxerr) or (vessel.flight(pos_frame).surface_altitude < 25000): # close or under 10,000m
      throttle = 0
      vessel.control.throttle = throttle
      set_runmode(4)
      lat,lng,h = tgts[tgt_name]
      # pos_tgt2 is the position of the target now, transform_to_local will rotate it to gte
      # the targets future position
      pos_tgt2,rot = get_transform(vessel.orbit.body,lat,lng,vessel.orbit.body.non_rotating_reference_frame)
      glide2 = glidepath.transform_to_local(glide,lat,lng,body=vessel.orbit.body,ref_frame=vessel.orbit.body.non_rotating_reference_frame,rotspeed=rotspeed,dt=3)
      print "Saving glidepath trajectory to: glidepath.dat"
      save_trajectory(glide2,"glidepath.dat",dt=3)

  # Coast
  if runmode == 4:
    vessel.control.brakes = True
    throttle = 0
    F  = -rr + 0.3*pos_predict
    if r[0] < slow_height: # slow down then compute trajectory
      print "Slowing down before computing trajectory"
      # METHOD TO SLOW DOWN
      if rr[0] < -slow_speed and len(traj)==0: # not slowed already - only do once
        F = -rr
        throttle = 1
      else:
        if len(traj)==0:
          set_target(tgt_name)
          F = -rr
          throttle = 0
          vessel.control.throttle = throttle
          traj,solve_t = compute_trajectory(r,rr,vessel.flight(vel_frame).direction,t,gfold_max_T=impact_time*3)
          save_trajectory(traj,"traj.dat",solve_t,dt=traj_dt)
          set_runmode(1)
          draw_trajectory(traj,vel2pos,pos_frame,pos_ksc)

  if runmode==1: # use autopilot
    # handle case where attitude and desired throttle directions are very different
    # by cutting thrust and waiting until steered within 80 degrees
    ddot = np.dot(F/np.linalg.norm(F),att/np.linalg.norm(att))
    if (runmode == 1) and (ddot < math.cos(math.radians(80))):
      print "pointing > 80 degrees off desired attitude -> no throttle"
      throttle = 0.01 # enough to steer

  if latt!=None:
    latt.remove()
    latt = None

  latt = conn.drawing.add_line((0,0,0),F*20,vessel.surface_reference_frame)
  latt.color = (0,1,0)

  # set direction and throttle
  if runmode!=0:
    vessel.auto_pilot.target_direction = (F[0],F[1],F[2])
    vessel.control.throttle = throttle

  # write telemetry
  print >>fdat, t,runmode,r[0],r[1],r[2],rr[0],rr[1],rr[2],att[0]*throttle*twr,att[1]*throttle*twr,att[2]*throttle*twr,F[0],F[1],F[2],throttle
  fdat.flush()

  last_r = r
  last_t = t

fdat.close()
