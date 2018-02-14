#!/usr/bin/python2.7

import numpy as np
import sys
import aero
import gfold
import utils

class Trajectory:
  def __init__(self,surface_g=9.8,Cd=1.0,A=1,mass=1000,body_rotspeed=0,body_radius=600000):
    self.surface_g = surface_g
    self.Cd = Cd
    self.A = A
    self.mass = mass
    self.body_radius = body_radius
    self.body_rotspeed = body_rotspeed
    self.throttle_Kp = 0.0001
    self.path = []
    self.lines = []

  def convert_nonrotating_to_rotating_reference_frame(self):
    path2 = []
    t0 = None
    for t,r,v,a in self.path:
      if t0==None:
        t0 = t # first time
      x,y,z = r[0],r[1],r[2]
      x,z = utils.rotate(x,z,(t-t0)*self.body_rotspeed)
      r2 = np.array([x,y,z])
      path2.append( (t,r2,v,a) )
    return path2

  def convert_local_to_rotating_reference_frame(self,r_Tgt,rotm):
    irotm = rotm.transpose()
    path2 = []
    for t,r,v,a in self.path:
      r2 = irotm.dot(r) + r_Tgt
      v2 = irotm.dot(v)
      a2 = irotm.dot(a)
      path2.append( (t,r2,v2,a2) )
    return path2

  def compute_coasting(self,space_center,vessel,t,dt=1,tgt_alt=0):
    """Compute trajectory with aerodynamic drag and no thrust corrections"""
    vel_frame = space_center.ReferenceFrame.create_hybrid(
       position=vessel.orbit.body.reference_frame,
       rotation=vessel.surface_reference_frame)

    # rotating ref frame
    r = np.array(vessel.position(vessel.orbit.body.reference_frame))
    v = np.array(vessel.flight(vessel.orbit.body.reference_frame).velocity)
    self.path = aero.simulate(r,v,Cd=self.Cd,A=self.A,mass=self.mass,surface_g=self.surface_g,dt=dt,t=t,body_rotspeed=0,body_radius = self.body_radius+tgt_alt)
    return len(self.path)>0

    # non-rotating ref frame
    r = np.array(vessel.position(vessel.orbit.body.reference_frame))
    v = np.array(vessel.flight(vessel.orbit.body.reference_frame).velocity)
    # No increase velocity due to planets rotation
    x,y,z = r[0],r[1],r[2]
    x,z = utils.rotate(x,z,self.body_rotspeed)
    r2 = np.array([x,y,z])
    gv = v + (r2-r)
    print "v:",np.linalg.norm(v),"->",np.linalg.norm(gv)
    self.path = aero.simulate(r,gv,Cd=self.Cd,A=self.A,mass=self.mass,surface_g=self.surface_g,dt=dt,t=t,body_rotspeed=self.body_rotspeed,body_radius = self.body_radius)
    self.convert_to_rotating_reference_frame()

    return len(self.path)>0

  def compute_soft_landing(self,space_center,vessel,tgt_lat,tgt_lng,tgt_alt,dt=1,compute_t=0.5,t0=0,logfile=None):
    """Compute trajectory for soft-landing phase"""

    r = np.array(vessel.position(vessel.orbit.body.reference_frame))
    v = np.array(vessel.flight(vessel.orbit.body.reference_frame).velocity)

    rTgt,rotm = utils.target_position_and_rotation(vessel,tgt_lat,tgt_lng,tgt_alt)

    # vars
    twr = vessel.max_thrust/vessel.mass
    min_throttle = 0.01
    max_throttle = 0.85
    maxT_angle = 90
    maxLand_angle = 5
    t_per_N = 5
    max_N = 8
    r2 = rotm.dot(r-rTgt)
    v2 = rotm.dot(v)
    att = rotm.dot(np.array(vessel.flight(vessel.orbit.body.reference_frame).direction))
    g = np.array([-vessel.orbit.body.surface_gravity,0.,0.])
    results = {}

    # progress time forward to compute_t to start trajectory in future, allowing for real time to compute it
    r2 = r2 + v2*compute_t + 0.5*g*compute_t*compute_t
    v2 = v2 + g*compute_t

    # Function solver called from golden_search()
    def f(T):
      num_thrust = int(T/t_per_N)
      num_thrust = max(3,min(num_thrust,max_N))
      print "Evaluating f(T=%.3f)" % T,"num_thrust=",num_thrust
      fuel, accels = gfold.solve(r2,v2,att,T=T,N=num_thrust,g=g,max_thrust=twr*max_throttle,min_thrust=twr*min_throttle,maxT_angle=maxT_angle,maxLand_angle=maxLand_angle,dt=T/100.0,min_height=10)
      if accels!=None:
        results[fuel] = (T,accels)
        print "Saving fuel=",fuel,"T=",T,"Accels=",len(accels)
      print "Solving for T=%.2f N=%d fuel=%.1f" % (T,num_thrust,fuel)
      return fuel

    # Convert co-ords to target surface reference frame
    min_T = utils.golden_search(f,1,120,tol=0.1)
    # Find minimum T from results (as above might return an infeasible region)

    if results:
      fuels = sorted(list(results.keys()))
      for f in fuels:
        print "Fuel:",f,"T:",results[f][0]
      min_fuel = fuels[0]
      min_T = results[min_fuel][0]
      accels = results[min_fuel][1]

      self.path = utils.compute_trajectory(r2,v2,accels,min_T,g,dt=dt,t0=t0+compute_t)

      # make sure trajectory ends at (0,0,0) - bit of a cheat
      self.path = utils.correct_trajectory(self.path)

      print "logfile",logfile
      if logfile:
        self.save(logfile)

      # Convert co-ords back to rotating reference frame - for drawing
      self.path = self.convert_local_to_rotating_reference_frame(rTgt,rotm)
      return True

    self.path = []
    return False

  def prediction_at(altitude=0):
    return 0

  def closest_to_trajectory(self,r,v,pos_weight,vel_weight):
    return utils.closest_to_trajectory(self.path,r,v,pos_weight,vel_weight)

  def clear():
    # Called of drawing clear called
    self.l = None

  def draw_final(self,vessel,conn,t):
    for l in self.lines:
      l.remove()
    self.lines = []
    if len(self.path) > 0:
      t_final,r_final,v_final,a_final = self.path[-1]
     
      vUp = r_final/np.linalg.norm(r_final)
      vX,vY = utils.orthogonal_vectors(vUp)
      size = 50
      l1 = conn.drawing.add_line(r_final-vX*size,r_final+vX*size,vessel.orbit.body.reference_frame)
      l1.color = (1.,0.,0.)
      l1.thickness = 5
      l2 = conn.drawing.add_line(r_final-vY*size,r_final+vY*size,vessel.orbit.body.reference_frame)
      l2.thickness = 5
      l2.color = (1.,0.,0.)
      self.lines.append(l1)
      self.lines.append(l2)

  def draw_all(self,vessel,conn):
    for l in self.lines:
      l.remove()
    self.lines = []
    r2=None
    for (t1,r1,v1,a1) in self.path:
      if r2 is not None:
        l = conn.drawing.add_line(r1,r2,vessel.orbit.body.reference_frame)
        l.thickness = 0.5
        l.color = (1,1,1)
        self.lines.append(l)
        l = conn.drawing.add_line(r1,r1+a1,vessel.orbit.body.reference_frame)
        l.thickness = 0.5
        l.color = (1,0,0)
        self.lines.append(l)
      r2 = r1

  def save(self,filename):
    f=open(filename,"w")
    print >>f, "time\tx\ty\tz\tvx\tvy\tvz\tax\tay\taz"
    for t,r,v,a in self.path:
      print >>f, "%.2f\t%.1f\t%.1f\t%.1f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f" % (t,r[0],r[1],r[2],v[0],v[1],v[2],a[0],a[1],a[2])
    f.close()

if __name__ == "__main__":
  sys.exit(0)
