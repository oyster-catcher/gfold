#!/usr/bin/python

import sys
import pylab as P
import numpy as np

def plot_line(ax,data,fx,fy,color='black'):
  xx=[float(d[fx]) for d in data]
  yy=[float(d[fy]) for d in data]
  ax.plot(xx,yy,color=color)

def plot(data,traj,title="",size=400,duration=30):
  # altitude against time, and throttle
  fig = P.figure(1)

  P.subplot2grid((2,4),(0,0), colspan=1, rowspan=1)
  ax = P.gca()
  ax.set_xlabel("t")
  ax.set_ylabel("x")
  ax.set_xlim([0,duration])
  ax.set_ylim([-size,size])
  plot_line(ax,data,'time','x',color='black')
  plot_line(ax,traj,'time','x',color='blue')
  ax.grid()
  P.subplot2grid((2,4),(0,1), colspan=1, rowspan=1)
  ax = P.gca()
  ax.set_xlabel("t")
  ax.set_ylabel("vx")
  ax.set_xlim([0,duration])
  ax.set_ylim([-size,size])
  plot_line(ax,data,'time','vx',color='black')
  plot_line(ax,traj,'time','vx',color='blue')
  ax.grid()

  P.subplot2grid((2,4),(1,0), colspan=1, rowspan=1)
  ax = P.gca()
  ax.set_xlabel("t")
  ax.set_ylabel("y")
  ax.set_xlim([0,duration])
  ax.set_ylim([-size,size])
  plot_line(ax,data,'time','y',color='black')
  plot_line(ax,traj,'time','y',color='blue')
  ax.grid()
  P.subplot2grid((2,4),(1,1), colspan=1, rowspan=1)
  ax = P.gca()
  ax.set_xlabel("t")
  ax.set_ylabel("vy")
  ax.set_xlim([0,duration])
  ax.set_ylim([-size,size])
  plot_line(ax,data,'time','vy',color='black')
  plot_line(ax,traj,'time','vy',color='blue')
  ax.grid()

  P.subplot2grid((2,4),(0,2), colspan=1, rowspan=1)
  ax = P.gca()
  ax.set_xlabel("t")
  ax.set_ylabel("z")
  ax.set_xlim([0,duration])
  ax.set_ylim([-size,size])
  plot_line(ax,data,'time','z',color='black')
  plot_line(ax,traj,'time','z',color='blue')
  ax.grid()
  P.subplot2grid((2,4),(0,3), colspan=1, rowspan=1)
  ax = P.gca()
  ax.set_xlabel("t")
  ax.set_ylabel("vz")
  ax.set_xlim([0,duration])
  ax.set_ylim([-size,size])
  plot_line(ax,data,'time','vz',color='black')
  plot_line(ax,traj,'time','vz',color='blue')
  ax.grid()

  # Throttle
  P.subplot2grid((2,4),(1,2), colspan=2, rowspan=1)
  ax = P.gca()
  ax.set_xlabel("t")
  ax.set_ylabel("mag(accel)")
  ax.set_xlim([0,duration])
  ax.set_ylim([0,20])
  # craft
  #t=[float(d['time']) for d in data]
  #throttle=[float(d['throttle']) for d in data]
  #ax.plot(t,throttle,color='black')

  # plot desired magnitude of acceleration
  tt=[]
  throttle=[]
  for d in traj:
    T=np.array([d['ax'],d['ay'],d['az']])
    tt.append(d['time'])
    throttle.append( np.linalg.norm(T) )
  ax.plot(tt,throttle,color='blue')
  # plot desired magnitude of acceleration
  tt=[]
  throttle=[]
  for d in data:
    T=np.array([d['ax'],d['ay'],d['az']])
    tt.append(d['time'])
    throttle.append( np.linalg.norm(T) )
  ax.plot(tt,throttle,color='black')
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

size=1200
fncraft=sys.argv[1]
try:
  fntraj=sys.argv[2]
  dat_traj=read_data(fntraj)
except:
  dat_traj=[]

dat_craft=read_data(fncraft)
xmin = min([d['x'] for d in dat_craft])
xmax = max([d['x'] for d in dat_craft])
ymin = min([d['y'] for d in dat_craft])
ymax = max([d['y'] for d in dat_craft])
zmin = min([d['z'] for d in dat_craft])
zmax = max([d['z'] for d in dat_craft])
tmax = max([d['time'] for d in dat_craft])
if dat_traj:
  tmax2 = max([d['time'] for d in dat_traj])
  tmax = max(tmax,tmax2)
size = max([-xmin,xmax,-ymin,ymax,-zmin,zmax])
size = size*1.2
print tmax
plot(dat_craft,dat_traj,size=size,duration=tmax)
