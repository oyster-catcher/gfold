#!/usr/bin/python2.7
"""Programs for specific run modes"""

import math
import numpy as np
from math import sqrt
from trajectory import Trajectory
import utils

################################ FINAL LANDING #################################

class FinalLanding:
  def __init__(self,conn,surface_g=9.8,cog_height=0,min_speed=5,throttle_Kp=0.2,land_factor=0.4):
    self.surface_g = surface_g
    self.cog_height = cog_height
    self.min_speed = min_speed
    self.land_factor = land_factor
    self.throttle_Kp = throttle_Kp
    self.in_deadzone = False
    self.conn = conn

  def compute_thrust(self,vessel):
    h = vessel.flight().surface_altitude
    r = np.array(vessel.position(vessel.orbit.body.reference_frame))
    v = np.array(vessel.flight(vessel.orbit.body.reference_frame).velocity)
    vel_frame = self.conn.space_center.ReferenceFrame.create_hybrid(
                  position=vessel.orbit.body.reference_frame,
                  rotation=vessel.surface_reference_frame)
    sv = np.array(vessel.flight(vel_frame).velocity) # relative to surface
    twr = vessel.max_thrust / vessel.mass
    v_wanted = max(self.min_speed,self.surface_g * self.land_factor * sqrt(max(0.01,h-self.cog_height)))
    err = (-sv[0]) - v_wanted
    throttle = err * self.throttle_Kp + self.surface_g/twr # last component to hover
    F  = -v
    vUp = r/np.linalg.norm(r)
    return throttle,F + 0.5*vUp

  def draw(self,vessel):
    pass

################################ SOFT LANDING #################################

class SoftLanding:
  def __init__(self,conn,vessel,tgt_lat,tgt_lng,tgt_height,pos_weight=0.2,vel_weight=0.4,t=0,logfile=None):
    self.conn = conn
    surface_g = vessel.orbit.body.surface_gravity
    self.pos_weight = pos_weight
    self.vel_weight = vel_weight
    self.traj = Trajectory(surface_g=surface_g,body_radius=600000,mass=vessel.mass)
    # compute initial trajectory
    self.traj.compute_soft_landing(self.conn.space_center,vessel,tgt_lat,tgt_lng,tgt_height,dt=0.5,t0=t,logfile=logfile)
    self.tgt_lat = tgt_lat
    self.tgt_lng = tgt_lng
    self.tgt_alt = tgt_height

  def compute_thrust(self,vessel):
    """Follow the computed trajectory - if there is one"""

    # Set Force/direction based on closest trajectory point in speed/position
    r = np.array(vessel.position(vessel.orbit.body.reference_frame))
    v = np.array(vessel.flight(vessel.orbit.body.reference_frame).velocity)
    g = np.array([-vessel.orbit.body.surface_gravity,0.,0.]) # in local target ref frame
    twr = vessel.max_thrust / vessel.mass

    # Convert to local co-ords for target
    r_Tgt,rotm = utils.target_position_and_rotation(vessel,self.tgt_lat,self.tgt_lng,self.tgt_alt)

    dr, dv, F = self.traj.closest_to_trajectory(r,v,self.pos_weight,self.vel_weight)

    if dr==None:
      return 0,None # not on trajectory

    # Difference to desired position and velocity gains
    F = F + (dr-r)*self.pos_weight + (dv-v)*self.vel_weight
    # convert F to local co-ords
    irotm = np.transpose(rotm)
    F = rotm.dot(F)
    throttle = np.linalg.norm(F)/twr
    F = np.array([F[0],F[1],F[2]])

    if F[0] < 0.1:
      throttle = 0.05
      F = np.array([0.1,F[1],F[2]])

    F = irotm.dot(F)

    # Shut-off throttle if pointing away from desired direction
    att = np.array(vessel.flight(vessel.orbit.body.reference_frame).direction)
    ddot = np.dot(F/np.linalg.norm(F),att/np.linalg.norm(att))
    if (ddot < math.cos(math.radians(80))):
      throttle = 0.01 # enough to steer

    return throttle,F

  def draw(self,vessel):
    self.traj.draw_all(vessel,self.conn)

################################ BOOSTBACK #################################

class Boostback:
  def __init__(self,tgt_lat,tgt_lng,tgt_alt,
               conn,
               surface_g=9.8,body_radius=600000,body_rotspeed=0,
               Cd=1.0,A=1,mass=1000,throttle_Kp=0.0001,deadzone=200,
               aim_height=0):
    self.tgt_lat = tgt_lat
    self.tgt_lng = tgt_lng
    self.tgt_alt = tgt_alt
    self.body_rotspeed = body_rotspeed
    self.throttle_Kp = 0.0001
    self.deadzone = deadzone
    self.conn = conn
    self.traj = Trajectory(surface_g=surface_g,body_radius=body_radius,
                           body_rotspeed=body_rotspeed,
                           Cd=Cd,A=A,mass=mass)
    self.aim_height = aim_height

  def clear():
    self.traj.clear()

  def compute_thrust(self,vessel,t):
    v = np.array(vessel.flight(vessel.orbit.body.reference_frame).velocity)
    if vessel.flight().surface_altitude > 20000:
      dt = 5
    else:
      dt = 1
    if self.traj.compute_coasting(self.conn.space_center,vessel,t=t,dt=dt,tgt_alt=self.tgt_alt+self.aim_height):
      # Find impact position on surface
      t0,r0,v0,a0 = self.traj.path[0]
      tT,rT,vF,aT = self.traj.path[-1]

      # Compute target position in the future - all in rotating reference frame now
      rTgt,rotm = utils.target_position_and_rotation(vessel,self.tgt_lat,self.tgt_lng,self.tgt_alt+self.aim_height,vessel.orbit.body.reference_frame)

      r = np.linalg.norm(rT)
      dE = rT - rTgt
      # convert to local surface reference frame

      # Convert from non_rotating co-ords into surface/target relative co-ords 
      F = -dE
      print "Boostback prediction dist:",np.linalg.norm(dE)
      throttle = np.linalg.norm(F) * self.throttle_Kp

      # compute thrust
      att = np.array(vessel.flight(vessel.orbit.body.reference_frame).direction)
      ddot = F.dot(att)/np.linalg.norm(F)
      print "ddot:",ddot
      if ddot < 0.9:
        throttle = 0.01 # enough to steer

      if np.linalg.norm(F) < self.deadzone:
        if not self.in_deadzone:
          print "Prediction within deadzone of %dm" % self.deadzone
          self.in_deadzone = True
        throttle = 0
        # Steer retrograde
        F = -v
      else:
        self.in_deadzone = False

      return throttle,F

  def in_deadzone(self):
    return self.in_deadzone

  def draw(self,vessel,conn,t):
    """Draw full trajectory if computed"""
    self.traj.draw_final(vessel,conn,t)

################################ COASTING #################################

class Coasting:
  def __init__(self,tgt_lat,tgt_lng,tgt_alt,
               conn,
               surface_g=9.8,body_radius=600000,body_rotspeed=0,
               Cd=1.0,A=1,mass=1000,throttle_Kp=0.0001,deadzone=200,
               aim_height=0,aim_below=10000):
    self.tgt_lat = tgt_lat
    self.tgt_lng = tgt_lng
    self.tgt_alt = tgt_alt
    self.body_rotspeed = body_rotspeed
    self.throttle_Kp = 0.0001
    self.deadzone = deadzone
    self.conn = conn
    self.traj = Trajectory(surface_g=surface_g,body_radius=body_radius,
                           body_rotspeed=body_rotspeed,
                           Cd=Cd,A=A,mass=mass)
    self.aim_height = aim_height
    self.aim_below  = aim_below

  def compute_thrust(self,vessel,t):
    h = vessel.flight().surface_altitude
    r = np.array(vessel.position(vessel.orbit.body.reference_frame))
    v = np.array(vessel.flight(vessel.orbit.body.reference_frame).velocity)
    twr = vessel.max_thrust/vessel.mass

    # Get target altitude between tgt_alt and tgt_alt+aim_height aimed current_alt-aim_below
    aim_height = utils.clamp(vessel.flight().surface_altitude-self.aim_below,self.tgt_alt,self.tgt_alt+self.aim_height)
    # Compute target position in the future - all in rotating reference frame now
    rTgt,rotm = utils.target_position_and_rotation(vessel,self.tgt_lat,self.tgt_lng,aim_height,vessel.orbit.body.reference_frame)

    if h > 20000:
      dt = 5
    else:
      dt = 1
    # Compute trajectory in rotating co-ords
    if self.traj.compute_coasting(self.conn.space_center,vessel,t=t,dt=dt,tgt_alt=aim_height):
      # Find impact position on surface
      t0,r0,v0,a0 = self.traj.path[0]
      tT,rT,vF,aT = self.traj.path[-1]

      dE = rT - rTgt
      print "Coasting predict err:",dE,"aim alt:",aim_height

      vst = dE/np.linalg.norm(dE) # prediction direction
      mag = min(1,np.linalg.norm(dE)*0.006) # 200m = maximum steering
      F = -v/np.linalg.norm(v) + 0.3*vst*mag

      # Use thrust to stop horizontal motion?
      # convert into horizontal co-ords
      dF = r - rTgt
      hdF = rotm.dot(dF)
      hdF[0] = 0
      hv  = rotm.dot(v)
      hv[0] = 0
      #print "Horizontal distance:",hdF[1],"N",hdF[2],"E (m)"
      #print "Horizontal speed:",np.linalg.norm(hv)
      dhd = np.linalg.norm(hdF+hv)-np.linalg.norm(hdF)
      #print "Change in hor. distance:",dhd
      # Find point of closest approach (ignoring altitude)
      rClose,u = utils.closest_point_on_line(hdF,hv,np.zeros(3))
      if u < 0: # behind current position, so current position is closest
        rClose = hdF
      #print "Closest approach dist:",np.linalg.norm(rClose),"at:",u
      d = np.linalg.norm(rClose) # dist to closest point
      dz = rClose[2] - hdF[2] # dist in z direction
      print "Dist to closest point (East):",dz

      throttle = 0
      if np.linalg.norm(hv) > 10 and np.linalg.norm(hdF) < 20000 and h>10000:
        factor = 0.0015
        Kp = 0.5
        v_wanted = twr * factor * dz
        err = hv[2] - v_wanted
        print "wanted:",v_wanted,"actual:",hv[2]
        throttle = err * Kp # last component to hover
        print

      return throttle,F

    return 0,-v

  def draw(self,vessel,conn,t):
    self.traj.draw_final(vessel,conn,t)
