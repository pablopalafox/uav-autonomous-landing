import matplotlib
import matplotlib.pyplot as plt 
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import os
import matplotlib.pyplot as plt
import sys
import itertools


mpl.rcParams['legend.fontsize'] = 12
DPI = 5000

input_dir = "/home/pablo/ws/log/trajectories"
print("Reading from", input_dir) 



##########################################################
################### w/ PREDICTION ########################
##########################################################
## linear speed
if len(sys.argv) == 2:
	linear_speed = sys.argv[1]
else:
	print("Exiting...")
	exit()
trajectories_pred = pd.read_csv(input_dir + "/trajectories_pred_lifelong_{}.csv".format(linear_speed))


# get only some lines from the data
zoom_in = True
if zoom_in:
	initial_row = 2300 # corresponds to roughly a timestamp of 50 s
	row_200_sec = 10000 # corresponds to roughly a timestamp of 100 s
	trajectories_pred = trajectories_pred.iloc[initial_row:row_200_sec]


## -- save dir
save_dir = "/home/pablo/ws/log/trajectories/lifelong_{}".format(linear_speed)
if not os.path.exists(save_dir):
	os.makedirs(save_dir)

# extract data from DataFrame
status = trajectories_pred['status'].tolist()
ardrone_x = trajectories_pred['aX'].tolist()
ardrone_y = trajectories_pred['aY'].tolist()
ardrone_z = trajectories_pred['aZ'].tolist()
summit_x = trajectories_pred['sX'].tolist()
summit_y = trajectories_pred['sY'].tolist()
summit_z = trajectories_pred['sZ'].tolist()

# 3D
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(ardrone_x, ardrone_y, ardrone_z, 'r', label='UAV')
ax.plot(summit_x, summit_y, summit_z, 'g', label='UGV')
ax.set(xlabel='x (m)', ylabel='y (m)', zlabel='z (m)')
bottom, top = plt.ylim()  # return the current ylim
ax.legend()
ax.view_init(elev=11, azim=125)
plt.show()

fig.savefig(os.path.join(save_dir, "traj3D_pred.pdf"), format='pdf', dpi=DPI)

# 2D
fig, ax = plt.subplots()
ax.plot(ardrone_x, ardrone_y, 'r', label='UAV')
ax.plot(summit_x, summit_y, 'g', label='UGV')
ax.set(xlabel='x (m)', ylabel='y (m)')
ax.legend()
ax.grid()
fig.savefig(os.path.join(save_dir, "traj2D_pred.pdf"), format='pdf', dpi=DPI)

