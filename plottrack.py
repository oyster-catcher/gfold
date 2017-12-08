#!/usr/bin/python

import sys
import pylab as P

def plot_line(data,fx,fy,color='black'):
  xx=[float(d[fx]) for d in data]
  yy=[float(d[fy]) for d in data]
  P.plot(xx,yy,color=color)

def plot_vectors(data,fx,fy,fvx,fvy,color='black',every=1):
  xx=[float(d[fx]) for d in data]
  yy=[float(d[fy]) for d in data]
  vx=[float(d[fvx]) for d in data]
  vy=[float(d[fvy]) for d in data]
  i=0
  for x1,y1,vx1,vy1 in zip(xx,yy,vx,vy):
    if (i%every)==0:
      P.arrow(x1,y1,vx1,vy1,head_width=0.1,head_length=0.5,fc=color,ec=color)
    i=i+1

def plot(data,traj,title="",size=100):
  # altitude against time, and throttle
  fig = P.figure(1)

  P.subplot2grid((2,2),(0,0), colspan=1, rowspan=1)
  ax = P.gca()

  # X = up
  # Y = North
  # Z = East

  # Side view: ZX (East/up)
  ax.set_xlabel("z")
  ax.set_ylabel("x")
  ax.set_xlim([-size,size])
  ax.set_ylim([-200,size*2])
  # craft
  plot_line(data,'z','x',color='black')
  plot_vectors(data,fx='z',fy='x',fvx='tz',fvy='tx',color='black',every=10) # position
  plot_vectors(data,fx='z',fy='x',fvx='az',fvy='ax',color='grey',every=10) # attitude
  # trajectory
  plot_line(traj,'z','x',color='blue')
  plot_vectors(traj,fx='z',fy='x',fvx='az',fvy='ax',color='blue',every=4) # desired thrust
  ax.grid()

  # Top view: YZ
  P.subplot2grid((2,2),(1,0), colspan=1, rowspan=1)
  ax = P.gca()
  ax.set_xlabel("y")
  ax.set_ylabel("z")
  ax.set_xlim([-size,size])
  ax.set_ylim([-size,size])
  # craft
  plot_line(data,'y','z',color='black')
  plot_vectors(data,'y','z','ty','tz','black')
  # trajectory
  plot_line(traj,'y','z',color='blue')
  plot_vectors(traj,'y','z','ay','az','blue')
  ax.grid()

  # Side view: YX
  P.subplot2grid((2,2),(0,1), colspan=1, rowspan=1)
  ax = P.gca()
  ax.set_xlabel("y")
  ax.set_ylabel("x")
  ax.set_xlim([-size,size])
  ax.set_ylim([-200,size*2])
  # craft
  plot_line(data,'y','x',color='black')
  plot_vectors(data,'y','x','ty','tx','black')
  # trajectory
  plot_line(traj,'y','x',color='blue')
  plot_vectors(traj,'y','x','ay','ax','blue')
  ax.grid()

  P.show()

def read_data(fname):
  fields=None
  dat=[]
  for line in file(fname):
    line=line.strip("\n\r")
    if not fields:
      fields = line.split(" ")
    else:
      data = [float(x) for x in line.split(" ")]
      dat.append( dict(zip(fields,data)) )
  return dat

size=2000
fncraft=sys.argv[1]
try:
  fntraj=sys.argv[2]
  dat_traj=read_data(fntraj)
except:
  dat_traj=[]

dat_craft=read_data(fncraft)
plot(dat_craft,dat_traj,size=size)
