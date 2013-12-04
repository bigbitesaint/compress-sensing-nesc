import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gs
import math

import mytool as mt

def _import(filename, shift=0, starttime=0, stoptime=1000000):
	data = []
	ts = []
	openfile = open(filename, "r")

	for line in openfile:
		tmp = line.rstrip().rsplit(",")
		start = float(tmp[0]) / 1000.0
		stop = float(tmp[1]) / 1000.0
		length = len(tmp) - 2

		if len(ts) > 0 and start < ts[-1]:
			continue

		for i in range(length):
			time = start + 1.0 * i * (stop - start) / (length - 1) + shift
			if starttime <= time <= stoptime:
				ts.append(time)
				data.append(int(tmp[2+i]))

	openfile.close()

	return ts, data

p = 1e-2
q = 1e-1
def process(data):
	return data
	# return mt.average(data, 16)

def l2norm(x):
	tmp = 0
	for i in range(len(x)):
		tmp += (x[i]/100.0)**2
	return 100.0*math.sqrt(tmp)

###########################################################################

width = 5.0
quantum = 1000

def estimateShift(ID, start, stop):
	ts1, data1 = _import(CAL[1]["file"], 0, start - width, stop + width)
	ts2, data2 = _import(CAL[ID]["file"], 0, start - width, stop + width)

	for i in range(len(ts1)-1):
		if ts1[i] < start <= ts1[i+1]:
			idx1 = i
		if ts1[i] < stop <= ts1[i+1]:
			idx2 = i

	Shift = []
	Correlation = []

	for i in range(-quantum, quantum, 1):
		shift = i * width / quantum
		ts2_ = [ts2[j] + shift for j in range(len(ts2))]
		data2_ = mt.interpolate(data2, ts2_, ts1)
		x = data1[idx1:idx2]
		y = data2_[idx1:idx2]
		correlation = np.correlate(x, y)
		Shift.append(shift)
		Correlation.append(correlation)

	tmp = max(Correlation)
	idx = Correlation.index(tmp)
	shift = Shift[idx]

	plt.plot(Shift, Correlation, "-x", label="%d %.2f"%(ID, shift))
	print ID, idx, shift

	return shift

###########################################################################

# CAL = {1:{"file":"gt.csv", "shift":0.0},
# 	   2:{"file":"downsampling.csv", "shift":-0.66}, 
# 	   3:{"file":"cs_result.csv", "shift":-0.485}, 
# 	   4:{"file":"baseline_128_result.csv", "shift":-0.835}, 
# 	   5:{"file":"baseline_16_result.csv", "shift":-1.04},
# 	   6:{"file":"lossless_result.csv", "shift":-1.845}}


CAL = {1:{"file":"gt.csv", "shift":0.0},
	   3:{"file":"cs_result.csv", "shift":-0.385}}
	   # 4:{"file":"baseline_128_result.csv", "shift":-0.215}}

###########################################################################

START = 6400
STOP = 7800

def foo2():
	ts1, data1 = _import(CAL[1]["file"], CAL[1]["shift"], START, STOP)
	a = l2norm(data1)

	for ID in [3]:
		ts2, data2 = _import(CAL[ID]["file"], CAL[ID]["shift"], START, STOP)
		data2_in = mt.interpolate(data2, ts2, ts1)
		diff = [data2_in[i] - data1[i] for i in range(len(ts1))]
		b = l2norm(diff)
		print "%d\t%.3f"%(ID, b / a)

###########################################################################

def foo3():
	for ID in [1, 3]:
		ts, data = _import(CAL[ID]["file"], CAL[ID]["shift"])
		plt.plot(ts, data, "-x", label="%d"%ID)

	plt.ylim(0, 500)
	plt.xlabel("Time (s)")
	plt.ylabel("Power (W)")
	plt.legend()
	plt.grid(True)
	plt.show()

###########################################################################

def foo4():
	for ID in [3]:
		estimateShift(ID, 4530, 4535)

	plt.legend()
	plt.grid(True)
	plt.show()

foo2()
