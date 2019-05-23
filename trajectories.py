import matplotlib
import matplotlib.pyplot as plt 
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import os
import matplotlib.pyplot as plt
import sys

mpl.rcParams['legend.fontsize'] = 12

input_dir = "/home/pablo/ws/log/trajectories"
print("Reading from", input_dir) 

## -- type of trajectory and experiment number
if len(sys.argv) == 3:
	traj_type = sys.argv[1]
	test_num = sys.argv[2]
else:
	traj_type = "c" # c for circular ---- l for linear
	test_num = "1"

print("traj_type:", traj_type)
print("test_num:", test_num)


## -- save dir
save_dir = "/home/pablo/ws/log/trajectories/{}{}".format(traj_type, test_num)
if not os.path.exists(save_dir):
	os.makedirs(save_dir)

##########################################################
################### w/ PREDICTION ########################
##########################################################
trajectories_pred = pd.read_csv(input_dir + "/trajectories_pred_{}{}.csv".format(traj_type, test_num))

# extract data from DataFrame
ardrone_x = trajectories_pred['aX'].tolist()
ardrone_y = trajectories_pred['aY'].tolist()
ardrone_y[:] = [x - 2.0 for x in ardrone_y]
ardrone_z = trajectories_pred['aZ'].tolist()
summit_x = trajectories_pred['sX'].tolist()
summit_y = trajectories_pred['sY'].tolist()
summit_y[:] = [x - 2.0 for x in summit_y]
summit_z = trajectories_pred['sZ'].tolist()

# 3D
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(ardrone_x, ardrone_y, ardrone_z, 'r', label='UAV')
ax.plot(summit_x, summit_y, summit_z, 'g', label='UGV')
ax.set(xlabel='x (m)', ylabel='y (m)', zlabel='z (m)')
bottom, top = plt.ylim()  # return the current ylim
plt.ylim((bottom-1, top+1))   # set the ylim to bottom, top
ax.legend()
fig.savefig(os.path.join(save_dir, "traj3D_pred.pdf"), format='pdf')
fig.savefig(os.path.join(save_dir, "traj3D_pred.png"), format='png')

# 2D
fig, ax = plt.subplots()
ax.plot(ardrone_x, ardrone_y, 'r', label='UAV')
ax.plot(summit_x, summit_y, 'g', label='UGV')
ax.set(xlabel='x (m)', ylabel='y (m)')
bottom, top = plt.ylim()  # return the current ylim
plt.ylim((bottom-1, top+1))   # set the ylim to bottom, top
ax.legend()
ax.grid()
fig.savefig(os.path.join(save_dir, "traj2D_pred.pdf"), format='pdf')
fig.savefig(os.path.join(save_dir, "traj2D_pred.png"), format='png')

# plt.show()
# exit()

##########################################################
##################### w/o PREDICTION #####################
##########################################################
trajectories_no_pred = pd.read_csv(input_dir + "/trajectories_NO_pred_{}{}.csv".format(traj_type, test_num))

# extract data from DataFrame
ardrone_x = trajectories_no_pred['aX'].tolist()
ardrone_y = trajectories_no_pred['aY'].tolist()
ardrone_y[:] = [x - 2.0 for x in ardrone_y]
ardrone_z = trajectories_no_pred['aZ'].tolist()
summit_x = trajectories_no_pred['sX'].tolist()
summit_y = trajectories_no_pred['sY'].tolist()
summit_y[:] = [x - 2.0 for x in summit_y]
summit_z = trajectories_no_pred['sZ'].tolist()

# 3D
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(ardrone_x, ardrone_y, ardrone_z, 'r', label='UAV')
ax.plot(summit_x, summit_y, summit_z, 'g', label='UGV')
ax.set(xlabel='x (m)', ylabel='y (m)', zlabel='z (m)')
bottom, top = plt.ylim()  # return the current ylim
plt.ylim((bottom-1, top+1))   # set the ylim to bottom, top
ax.legend()
fig.savefig(os.path.join(save_dir, "traj3D_NO_pred.pdf"), format='pdf')
fig.savefig(os.path.join(save_dir, "traj3D_NO_pred.png"), format='png')

# 2D
fig, ax = plt.subplots()
ax.plot(ardrone_x, ardrone_y, 'r', label='UAV')
ax.plot(summit_x, summit_y, 'g', label='UGV')
ax.set(xlabel='x (m)', ylabel='y (m)')
bottom, top = plt.ylim()  # return the current ylim
plt.ylim((bottom-1, top+1))   # set the ylim to bottom, top
ax.legend()
ax.grid()
fig.savefig(os.path.join(save_dir, "traj2D_NO_pred.pdf"), format='pdf')
fig.savefig(os.path.join(save_dir, "traj2D_NO_pred.png"), format='png')


plt.show()
