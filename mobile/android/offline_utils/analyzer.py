#!/usr/bin/python

from xml.dom import minidom
import sys,os
import numpy as np
import matplotlib.pyplot as plt
import glob
import datetime

if len(sys.argv) != 2:
    print "Usage: ./analyzer.py sequence_folder"
    sys.exit(1)

foldername=sys.argv[1]

if not os.path.isdir(foldername):
    print "Not a valid filename"
    sys.exit(1)

fnames = glob.glob(foldername + "/*.xml")
if len(fnames) > 1:
    print "Two many xml-files in folder!"
    sys.exit(1)
fname = fnames[0]


xmldoc = minidom.parse(fname)

#arr_frames = np.empty((0,17))
flist = []

sequence = xmldoc.getElementsByTagName("sequence")
for s in sequence:
    frames = s.getElementsByTagName("Frame")
    for f in frames:
        ts_cam = int(f.attributes["ts_cam"].value)
        lat = float(f.attributes["lat"].value)
        lon = float(f.attributes["lon"].value)
        acc = float(f.attributes["acc"].value)
        speed = float(f.attributes["speed"].value)
        img_w = float(f.attributes["img_w"].value)
        img_h = float(f.attributes["img_h"].value)
        wb_val = float(f.attributes["wb_value"].value)
        iso_value = float(f.attributes["iso_value"].value)
        exp_t = float(f.attributes["exp_time"].value)
        foc_dist = float(f.attributes["foc_dist"].value)
        avelx = float(f.attributes["avelx"].value)
        avely = float(f.attributes["avely"].value)
        avelz = float(f.attributes["avelz"].value)
        accx = float(f.attributes["accx"].value)
        accy = float(f.attributes["accy"].value)
        accz = float(f.attributes["accz"].value)

        flist.append([ts_cam, lat, lon, acc, speed, img_w, img_h, wb_val, iso_value, exp_t, foc_dist, avelx, avely, avelz, accx, accy, accz])
        print ".",

arr = np.array(flist)
print arr.shape

rows = 3
cols = 3

fig = plt.figure(0)
date = datetime.datetime.fromtimestamp(arr[0,0] / 1000000.)
datestring = date.strftime("%Y-%m-%d %H:%M:%S")
plt.suptitle("%s\r\n%dx%d, iso: %d, exp: %d ms, avg. fps: %f" % (datestring, np.mean(arr[:,5]), np.mean(arr[:,6]), np.mean(arr[:,8]), np.mean(arr[:,9]) / 1000000, 1000./np.mean(np.diff(arr[:,0]))))


fig.add_subplot(rows,cols,1)
plt.title("coords")
plt.plot(arr[:,1], arr[:,2], '.')

fig.add_subplot(rows,cols,2)
plt.title("delta(ts_cam)")
plt.plot(np.diff(arr[:,0])/1000000., '.')

fig.add_subplot(rows,cols,3)
plt.title("speed")
plt.plot(arr[:,4], '-')

fig.add_subplot(rows,cols,4)
plt.title("avelx")
plt.plot(arr[:,11])

fig.add_subplot(rows,cols,5)
plt.title("avely")
plt.plot(arr[:,12])

fig.add_subplot(rows,cols,6)
plt.title("avelz")
plt.plot(arr[:,13])

fig.add_subplot(rows,cols,7)
plt.title("accx")
plt.plot(arr[:,14])

fig.add_subplot(rows,cols,8)
plt.title("accy")
plt.plot(arr[:,15])

fig.add_subplot(rows,cols,9)
plt.title("accz")
plt.plot(arr[:,16])



plt.show()
