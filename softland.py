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
from runprograms import FinalLanding,Boostback,Coasting,SoftLanding

# close range, Cd=0.8, A=38
# RocketPad = -0.0972024707346 -74.5576964649 2.28073599411
parser = argparse.ArgumentParser(description='Compute a trajectory and follow it for a soft-landing')
parser.add_argument('--target_lat',type=float,help="Latitude of target",default=-0.0972071692165)
parser.add_argument('--target_lng',type=float,help="Longitude of target",default=-74.5576808614)
parser.add_argument('--target_alt',type=float,help="Altitude of target",default=76)
parser.add_argument('--land_factor',type=float,help="Higher to a faster/later landing",default=0.25)
parser.add_argument('--cog_height',type=float,help="Distance from bottom of vessel to CoG",default=0)
parser.add_argument('--throttle_Kp',type=float,help="Throttle = velocity_error * throttle_Kp",default=0.25)
parser.add_argument('--logfile',type=str,help="Log telemetry to this file",default=None)
parser.add_argument('--softlandlogfile',type=str,help="Log soft landing trajectory to this file",default=None)
parser.add_argument('--runmode',type=int,help="Initial run mode (0=follow trajectory,1=final descent)",default=0)
args = parser.parse_args()

conn = krpc.connect(name='boostback')
vessel = conn.space_center.active_vessel

runmodes='follow final_descent'.split(" ")

# Current position
print vessel.flight().latitude,vessel.flight().longitude,vessel.flight().surface_altitude

target = conn.space_center.target_vessel
if not target:
  tgt_lat, tgt_lng, tgt_alt = args.target_lat, args.target_lng, args.target_alt
else:
  tgt_lat,tgt_lng,tgt_alt = target.flight().latitude,target.flight().longitude,target.flight().surface_altitude
  print "Aiming to target:",target.name,"lat:",tgt_lat,"lng:",tgt_lng,"alt:",tgt_alt

if args.logfile:
  flog = file(args.logfile,"w")
  print >>flog, "time\trunmode\tx\ty\tz\tvx\tvy\tvz\tthrottle"
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
vessel.control.throttle = 0

############################# SET INITIAL RUNMODE #############################
runmode = args.runmode
set_runmode(runmode)

########################### INITIALIZE RUN PROGRAMS ###########################
soft_landing = SoftLanding(conn,vessel,
                           tgt_lat,tgt_lng,tgt_alt,
                           logfile=args.softlandlogfile)
soft_landing.draw(vessel)

final_landing = FinalLanding(surface_g=vessel.orbit.body.surface_gravity,
                             cog_height=args.cog_height,
                             land_factor=args.land_factor,
                             throttle_Kp=args.throttle_Kp,
                             conn=conn)


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
    throttle,F = soft_landing.compute_thrust(vessel)
    if (F==None) or (h<3): # off trajectory (near surface or failure to solve)
      set_runmode(1)

  if runmode == 1:
    throttle,F = final_landing.compute_thrust(vessel)

  # Set thrust and attitude
  if latt!= None:
    latt.remove()
  F = F*20
  latt = conn.drawing.add_line( (r[0],r[1],r[2]), (F[0],F[1],F[2]), vessel.orbit.body.reference_frame)
  latt.color = (0.,1.,0.)
  vessel.auto_pilot.target_direction = (F[0],F[1],F[2])
  vessel.control.throttle = utils.clamp(throttle,0,1)

  # Logging telemetry
  if flog:
    a = throttle*twr*att
    # Convert to target co-ord frame
    r2 = rotm.dot(r-pos_tgt)
    v2 = rotm.dot(v)
    a2 = rotm.dot(a)
    print >>flog, "%.2f\t%d\t%.1f\t%.1f\t%.1f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f" % (t, runmode, r2[0], r2[1], r2[2], v2[0], v2[1], v2[2], a2[0], a2[1], a2[2], throttle)

  time.sleep(0.02)

if flog:
  flog.close()
