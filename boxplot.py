## numpy is used for creating fake data
import numpy as np 
import matplotlib as mpl 
import os
import sys
import pandas as pd
import statistics as st

## agg backend is used to create plot as a .png file
#mpl.use('agg')

import matplotlib.pyplot as plt

###############################################################

save = True
LIM_Y = 0.15

input_dir = "/home/pablo/ws/log/errors"
print("Reading from", input_dir) 

## -- type of traj
if len(sys.argv) == 2:
	traj_types = [sys.argv[1]]
else:
	traj_types = ["l", "c"]

###############################################################

for traj_type in traj_types:

	print("traj_type:", traj_type)

	means_ex_no_preds = []
	means_ey_no_preds = []

	means_ex_preds = []
	means_ey_preds = []

	for i in range(1, 6):

		######### no pred
		errors_no_pred = pd.read_csv(input_dir + "/errors_NO_pred_{}{}.csv".format(traj_type, i))

		ex_no_pred = errors_no_pred['ex_real'].tolist()
		means_ex_no_preds.append(st.mean(ex_no_pred))

		ey_no_pred = errors_no_pred['ey_real'].tolist()
		means_ey_no_preds.append(st.mean(ey_no_pred))


		########## pred
		errors_pred = pd.read_csv(input_dir + "/errors_pred_{}{}.csv".format(traj_type, i))

		ex_pred = errors_pred['ex_real'].tolist()
		means_ex_preds.append(st.mean(ex_pred))

		ey_pred = errors_pred['ey_real'].tolist()
		means_ey_preds.append(st.mean(ey_pred))


	## combine these different collections into a list    
	ex = [means_ex_no_preds, means_ex_preds]
	ey = [means_ey_no_preds, means_ey_preds]

	## ex
	fig, ax = plt.subplots()
	bp = ax.boxplot(ex, labels=("w/o pred","w/ pred"))
	ax.set(xlabel='trajectory type', ylabel='error in x (m)')
	plt.ylim((-LIM_Y, LIM_Y))   # set the ylim to bottom, top
	if traj_type == "c":
		plt.title("circular trajectory")
	elif traj_type == "l":
		plt.title("linear trajectory")
	ax.grid()
	ax.legend()
	if save:
		fig.savefig("/home/pablo/ws/log/ex_boxplot_{}.pdf".format(traj_type), format='pdf')

	## ey
	fig, ax = plt.subplots()
	bp = ax.boxplot(ey, labels=("w/o pred","w/ pred"))
	ax.set(xlabel='trajectory type', ylabel='error in y (m)')
	plt.ylim((-LIM_Y, LIM_Y))   # set the ylim to bottom, top
	if traj_type == "c":
		plt.title("circular trajectory")
	elif traj_type == "l":
		plt.title("linear trajectory")
	ax.grid()
	ax.legend()
	if save:
		fig.savefig("/home/pablo/ws/log/ey_boxplot_{}.pdf".format(traj_type), format='pdf')


plt.show()