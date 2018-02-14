#!/usr/bin/python

import sys
import pylab as P
import numpy as np

def plot_line(ax,data,fx,fy,color='black'):
  xx=[float(d[fx]) for d in data]
  yy=[float(d[fy]) for d in data]
  ax.plot(xx,yy,color=color)

def plot(datas,title="",size=400,duration=30,names=[]):
  # altitude against time, and throttle
  fig = P.figure(1)

  colors=['black','blue','red']

  P.subplot2grid((2,3),(0,0), colspan=1, rowspan=1)
  ax = P.gca()
  ax.set_xlabel("t")
  ax.set_ylabel("x")
  ax.set_xlim([0,duration])
  ax.set_ylim([-size,size])
  for i,data in enumerate(datas):
    plot_line(ax,data,'time','x',color=colors[i])
  ax.grid()
  ax.legend(names)
  P.subplot2grid((2,3),(0,1), colspan=1, rowspan=1)
  ax = P.gca()
  ax.set_xlabel("t")
  ax.set_ylabel("vx")
  ax.set_xlim([0,duration])
  ax.set_ylim([-100,100])
  for i,data in enumerate(datas):
    plot_line(ax,data,'time','vx',color=colors[i])
  ax.grid()
  ax.legend(names)

  P.subplot2grid((2,3),(1,0), colspan=1, rowspan=1)
  ax = P.gca()
  ax.set_xlabel("t")
  ax.set_ylabel("y")
  ax.set_xlim([0,duration])
  ax.set_ylim([-size,size])
  for i,data in enumerate(datas):
    plot_line(ax,data,'time','y',color=colors[i])
  ax.grid()
  P.subplot2grid((2,3),(1,1), colspan=1, rowspan=1)
  ax = P.gca()
  ax.set_xlabel("t")
  ax.set_ylabel("vy")
  ax.set_xlim([0,duration])
  ax.set_ylim([-100,100])
  for i,data in enumerate(datas):
    plot_line(ax,data,'time','vy',color=colors[i])
  ax.grid()

  P.subplot2grid((2,3),(0,2), colspan=1, rowspan=1)
  ax = P.gca()
  ax.set_xlabel("t")
  ax.set_ylabel("z")
  ax.set_xlim([0,duration])
  ax.set_ylim([-size,size])
  for i,data in enumerate(datas):
    plot_line(ax,data,'time','z',color=colors[i])
  ax.grid()
  P.subplot2grid((2,3),(1,2), colspan=1, rowspan=1)
  ax = P.gca()
  ax.set_xlabel("t")
  ax.set_ylabel("vz")
  ax.set_xlim([0,duration])
  ax.set_ylim([-100,100])
  for i,data in enumerate(datas):
    plot_line(ax,data,'time','vz',color=colors[i])
  ax.grid()

  # Throttle
  #P.subplot2grid((2,3),(1,2), colspan=2, rowspan=1)
  #ax = P.gca()
  #ax.set_xlabel("t")
  #ax.set_ylabel("mag(accel)")
  #ax.set_xlim([0,duration])
  #ax.set_ylim([0,20])
  # craft
  #t=[float(d['time']) for d in data]
  #throttle=[float(d['throttle']) for d in data]
  #ax.plot(t,throttle,color='black')

  P.show()

def read_data(fname):
  fields=None
  dat=[]
  for line in file(fname):
    line=line.strip("\n\r")
    if not fields:
      fields = line.split("\t")
    else:
      data = [float(x) for x in line.split("\t")]
      dat.append( dict(zip(fields,data)) )
  return dat

size=1200
fndatas=sys.argv[1:]

datas=[]
for fndata in fndatas:
  datas.append(read_data(fndata))

dat_craft = datas[0]
xmin = min([d['x'] for d in dat_craft])
xmax = max([d['x'] for d in dat_craft])
ymin = min([d['y'] for d in dat_craft])
ymax = max([d['y'] for d in dat_craft])
zmin = min([d['z'] for d in dat_craft])
zmax = max([d['z'] for d in dat_craft])
tmax = max([d['time'] for d in dat_craft])
size = max([-xmin,xmax,-ymin,ymax,-zmin,zmax])
size = size*1.2
plot(datas,size=size,duration=tmax,names=fndatas)
