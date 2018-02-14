#!/usr/bin/python2.7

import numpy as np
from numpy.linalg import norm
from math import pow,cos,sin,exp,sqrt
import sys
import utils
import math

def kerbin_to_earth(h):
  return h/0.8

# h is Earth geopotential altitude
# from http://www.braeunig.us/space/atmmodel.htm#table4 "U.S. Standard Atmosphere, 0 to 86km"
def atmos_pressure(h):
  h = kerbin_to_earth(h)
  h = h/1000.0
  # BIG FUDGE FACTOR AS KSP ATMOSPHERE APPEARS TO BE VERY THICK AT THIS LOW ALTITUDE
  if h < 11:
    return 25000
  # Troposphere
  if h < 11:
    return 101325 * pow(288.15 / (288.15 - 6.5*h),34.1632/-6.5) * 1.05
  # Stratosphere
  if h < 20:
    return 22632.06 * exp(-34.1632*(h-11)/216.65)
  if h < 32:
    return 5474.889 * pow(216.65 / (216.65 + (h-20)),34.1632)
  if h < 47:
    return 868.0187 * pow(228.65/(228.65+2.8*(h-32)),34.1632/2.8)
  # Mesosphere
  if h < 51:
    return 110.9063 * exp(-34.162*(h-47)/270.65)
  if h < 71:
    return 66.93887 * pow(270.65/(270.65-2.8*(h-51)),34.1632/-2.8)
  if h < 84.852:
    return 3.956420 * pow(214.65/(214.65-2*(h-71)),34.1632/-2)
  return 0

# h is Earth equivalent
def atmos_temperature(h):
  h = kerbin_to_earth(h)
  table=[ (0,11000,288.15,-0.0065),
          (11000,20000,216.65,0),
          (20000,32000,216.65,0.001),
          (32000,47000,228.65,0.0028),
          (47000,51000,270.65,0),
          (51000,71000,270.65,-0.0028),
          (71000,232940,214.65,-0.002) ]
  for h_start,h_end,t_start,lapse_rate in table:
    if h < h_end:
      return t_start + (h-h_start)*lapse_rate
  return 180

def atmos_density(h):
  return atmos_pressure(h) / (287.053 * atmos_temperature(h))

def air_vector(r,rotation_speed_in_radians):
  x,y,z = r[0],r[1],r[2]
  a = rotation_speed_in_radians
  v_air = np.array( [x*cos(-a) + z*sin(-a),y,-x*sin(-a) + z*cos(-a)] ) - r
  return v_air

def simulate(r0,v0,dt=1.0,Cd=0.6,A=1,mass=1000,
             t=0,body_rotspeed=0,body_radius=600000,
             max_t=600,surface_g=9.8,pos_origin=None):
  """simulate the glide path down to the surface with aerodynamic drag assuming
     the vessel has no lift drag component, and drag is irrespective of orientation
     or the orientation is always same, typically pointing in retrograde direction"""

  if not pos_origin:
    pos_origin = np.array([0.,0.,0.])
  data = []
  t0 = t
  r = r0
  v = v0
  D = 0
  a = np.array([0.,0.,0.])
  cur_radius = np.linalg.norm(r)
  last_radius = cur_radius
  while cur_radius > body_radius and (t-t0) < max_t:

    # velocity of local atmosphere in non-rotating reference frame
    v_air = air_vector(r,body_rotspeed)
    v_relair = v - v_air
    airspeed = np.linalg.norm(v_relair)
    local_g = surface_g*(body_radius/cur_radius)*(body_radius/cur_radius)
    g = -(r/cur_radius) * local_g
    D = Cd * atmos_density(cur_radius-body_radius) * airspeed * airspeed * A * 0.5
    a = -(v_relair/airspeed)*D/mass + g
    data.append( (t,r-pos_origin,v,a) )

    r = r + v * dt
    v = v + a * dt
    last_radius = cur_radius
    cur_radius = np.linalg.norm(r)
    t = t + dt

    if t - t0 > max_t:
      print >>sys.stderr, "t > %.1f. Taking too long initial_radius=%.1f final_radius=%.1f" % (max_t,np.linalg.norm(r0),np.linalg.norm(r)-radius)
      return []

  # windback if intersected surface - this is an approximation (since t_adj is an approximation)
  if (last_radius > cur_radius+0.1):
    t_adj = -dt * (body_radius - cur_radius) / (last_radius - cur_radius)
    r = r + v * t_adj
  else:
    t_adj = 0
  data.append( (t+t_adj,r-pos_origin,v,a) )

  return data # trajectory of (r,v,a)

def save(trajectory,filename):
  f = open(filename,'w')
  print >>f,"time x y z vx vy vz ax ay az"
  for t,r,v,a in trajectory:
    print >>f, t, r[0], r[1], r[2], v[0], v[1], v[2], a[0], a[1], a[2]
  f.close()

if __name__ == "__main__":
  r = np.array([700000.,0.,0.])
  v = np.array([0.,100.,0.])
  traj = simulate(r,v,dt=1)
  save(traj,"traj.dat")

