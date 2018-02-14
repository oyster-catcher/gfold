#!/usr/bin/python2.7
"""Neatened up script of boostback_land2.py. Does
   - boostback to a target (current target or selected waypoint)
   - after boostback coast through atmosphere steering aerodynamically
   - switches to final descent when required to cancel out the vessels velocity to zero"""

import sys
import argparse
import time
import numpy as np
import krpc
import trajectory
import utils
from runprograms import FinalLanding,Boostback,Coasting

# close range, Cd=0.8, A=38

parser = argparse.ArgumentParser(description='Compute a trajectory and follow it for a soft-landing')
parser.add_argument('--target_lat',type=float,help="Latitude of target",default=None)
parser.add_argument('--target_lng',type=float,help="Longitude of target",default=None)
parser.add_argument('--target_alt',type=float,help="Altitude of target",default=None)
parser.add_argument('--land_factor',type=float,help="Higher to a faster/later landing",default=0.5)
parser.add_argument('--Cd',type=float,help="Coefficient of drag",default=0.7)
parser.add_argument('--frontal_area',type=float,help="Frontal area of vessel",default=27)
parser.add_argument('--aim_alt',type=float,help="Aim altitude",default=0)
parser.add_argument('--cog_height',type=float,help="Distance from bottom of vessel to CoG",default=25)
parser.add_argument('--throttle_Kp',type=float,help="Throttle = velocity_error * throttle_Kp",default=0.25)
parser.add_argument('--boostback_deadzone',type=float,help="Maximum error in metres for any boostback thrust corrections",default=200)
parser.add_argument('--aim_height',type=float,help="Aim this many metres above the target",default=2000)
parser.add_argument('--aim_below',type=float,help="Aim this many metres below the current altitude",default=3000)
parser.add_argument('--logfile',type=str,help="Log telemetry to this file",default=None)
parser.add_argument('--runmode',type=int,help="Initial run mode (0=boostback,1=coasting,2=final descent)",default=0)
args = parser.parse_args()

conn = krpc.connect(name='boostback')
vessel = conn.space_center.active_vessel

# Add waypoint for rocket launch pad
conn.space_center.waypoint_manager.add_waypoint(-0.0972071692165,-74.5576808614,vessel.orbit.body,"Rocket Pad")

runmodes='boostback coasting final_descent'.split(" ")

if args.target_lat:
  tgt_lat, tgt_lng, tgt_alt = args.target_lat, args.target_lng, args.target_alt
else:
  if conn.space_center.target_vessel:
    target = conn.space_center.target_vessel
    tgt_lat,tgt_lng,tgt_alt = target.flight().latitude,target.flight().longitude,target.flight().surface_altitude
    print "Aiming to target:",target.name,"lat:",tgt_lat,"lng:",tgt_lng,"alt:",tgt_alt
  else:
    print >>sys.stderr,"Error! No target selected - target is directly below vessel"
    tgt_lat,tgt_lng = vessel.flight().latitude,vessel.flight().longitude
    tgt_alt = 400
    #sys.exit(1)

if args.logfile:
  flog = file(args.logfile,"w")
  print >>flog, "time runmode x y z vx vy vz throttle"
else:
  flog = None

def set_runmode(a_runmode):
  global runmode,latt
  print "Switching to runmode: %d (%s)" % (runmode,runmodes[a_runmode])
  latt = None
  conn.drawing.clear()
  runmode = a_runmode

################################ SET UP AUTOPILOT #############################
vessel.auto_pilot.target_pitch_and_heading(90,90)
vessel.auto_pilot.reference_frame = vessel.orbit.body.reference_frame
vessel.auto_pilot.engage()
vessel.control.rcs = True
vessel.control.sas = False

########################### INITIALIZE RUN PROGRAMS ###########################
final_landing = FinalLanding(surface_g=vessel.orbit.body.surface_gravity,
                             cog_height=args.cog_height,
                             land_factor=args.land_factor,
                             throttle_Kp=args.throttle_Kp,
                             conn=conn)
rotspeed = vessel.orbit.body.rotational_speed
boostback = Boostback(surface_g=vessel.orbit.body.surface_gravity,
                      tgt_lat=tgt_lat,
                      tgt_lng=tgt_lng,
                      tgt_alt=tgt_alt,
                      body_radius=vessel.orbit.body.equatorial_radius,
                      body_rotspeed=rotspeed,
                      Cd=args.Cd,A=args.frontal_area,mass=vessel.mass,
                      deadzone=args.boostback_deadzone,conn=conn,
                      aim_height=args.aim_height)
coasting = Coasting(surface_g=vessel.orbit.body.surface_gravity,
                    tgt_lat=tgt_lat,
                    tgt_lng=tgt_lng,
                    tgt_alt=tgt_alt,
                    body_radius=vessel.orbit.body.equatorial_radius,
                    body_rotspeed=rotspeed,
                    Cd=args.Cd,A=args.frontal_area,mass=vessel.mass,
                    conn=conn,
                    aim_height=args.aim_height,
                    aim_below=args.aim_below)

############################# SET INITIAL RUNMODE #############################
runmode = args.runmode
set_runmode(runmode)

# returns position in given reference frame
# and rotation matrix that converts from vessel.orbit.body.reference_frame to
# X(Up), Y(North), Z(East) in local target co-ordinates
pos_tgt,rotm = utils.target_position_and_rotation(vessel,tgt_lat,tgt_lng,tgt_alt)
t0 = conn.space_center.ut
t_draw = 0
latt = None
##################################### MAIN LOOP ########################################
while True:
  # Main instrument/guidance data
  t   = conn.space_center.ut - t0
  r   = np.array(vessel.position(vessel.orbit.body.reference_frame))
  v   = np.array(vessel.flight(vessel.orbit.body.reference_frame).velocity)
  att = np.array(vessel.flight(vessel.orbit.body.reference_frame).direction)
  h   = vessel.flight(vessel.surface_velocity_reference_frame).surface_altitude
  if vessel.available_thrust > 0:
    twr = vessel.available_thrust/vessel.mass
  else:
    twr = 1 # for vehicle that has no thrust left, don't get divide by zero

  # Steering/thrust parameters we wish to control
  throttle = 0
  F = att

  if runmode == 0:
    boostback.mass = vessel.mass
    throttle,F = boostback.compute_thrust(vessel,t)

    if t > t_draw + 0.1:
      boostback.draw(vessel,conn,t)
      t_draw = t

    # Must be after draw() since set_runmode() clears all drawing
    if boostback.in_deadzone or (h < 45000):
      boostback.traj.save("traj.dat",vessel,tgt_lat,tgt_lng,tgt_alt)
      set_runmode(1) # coasting

  if runmode == 1:
    throttle,F = coasting.compute_thrust(vessel,t)

    if t > t_draw + 0.5:
      coasting.draw(vessel,conn,t)
      t_draw = t

    if h < 750:
      set_runmode(2)

  if runmode == 2:
    throttle,F = final_landing.compute_thrust(vessel)

  # Set thrust and attitude
  if latt!= None:
    latt.remove()
  F = F*20
  latt = conn.drawing.add_line( (r[0],r[1],r[2]), (F[0],F[1],F[2]), vessel.orbit.body.reference_frame)
  latt.color = (0.,1.,0.)
  vessel.auto_pilot.target_direction = (F[0],F[1],F[2])
  vessel.control.throttle = throttle

  # Logging telemetry
  if flog:
    a = throttle*twr*att
    print >>flog, t, runmode, r[0], r[1], r[2], v[0], v[1], v[2], a[0], a[1], a[2], throttle

  time.sleep(0.02)

if flog:
  flog.close()
