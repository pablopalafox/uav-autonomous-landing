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

save = False

input_dir = "/home/pablo/ws/log/errors"
print("Reading from", input_dir) 

## -- type of traj
if len(sys.argv) == 2:
	traj_type = sys.argv[1]
else:
	traj_type = "circle"
print("traj_type:", traj_type)

###############################################################

means_ex_no_preds = []
means_ey_no_preds = []
means_ez_no_preds = []

means_ex_preds = []
means_ey_preds = []
means_ez_preds = []

for i in range(1, 6):

	######### no pred
	errors_no_pred = pd.read_csv(input_dir + "/errors_NO_pred_{}_{}.csv".format(traj_type, i))

	ex_no_pred = errors_no_pred['ex'].tolist()
	means_ex_no_preds.append(st.mean(ex_no_pred))

	ey_no_pred = errors_no_pred['ey'].tolist()
	means_ey_no_preds.append(st.mean(ey_no_pred))

	ez_no_pred = errors_no_pred['ez'].tolist()
	means_ez_no_preds.append(st.mean(ez_no_pred))

	########## pred
	errors_pred = pd.read_csv(input_dir + "/errors_pred_{}_{}.csv".format(traj_type, i))

	ex_pred = errors_pred['ex'].tolist()
	means_ex_preds.append(st.mean(ex_pred))

	ey_pred = errors_pred['ey'].tolist()
	means_ey_preds.append(st.mean(ey_pred))

	ez_pred = errors_pred['ez'].tolist()
	means_ez_preds.append(st.mean(ez_pred))



## combine these different collections into a list    
ex = [means_ex_no_preds, means_ex_preds]
ey = [means_ey_no_preds, means_ey_preds]
ez = [means_ez_no_preds, means_ez_preds]

# ex
fig, ax = plt.subplots()
bp = ax.boxplot(ex)
ax.set(xlabel='trajectory type', ylabel='error in x (m)')
ax.grid()
ax.legend()
if save:
	fig.savefig(os.path.join(save_dir, "ex_boxplot_{}.pdf".format(traj_type)), format='pdf')
	fig.savefig(os.path.join(save_dir, "ex_boxplot_{}.png".format(traj_type)), format='png')

# ey
fig, ax = plt.subplots()
bp = ax.boxplot(ey)
ax.set(xlabel='trajectory type', ylabel='error in y (m)')
ax.grid()
ax.legend()
if save:
	fig.savefig(os.path.join(save_dir, "ey_boxplot_{}.pdf".format(traj_type)), format='pdf')
	fig.savefig(os.path.join(save_dir, "ey_boxplot_{}.png".format(traj_type)), format='png')

# ez
fig, ax = plt.subplots()
bp = ax.boxplot(ez)
ax.set(xlabel='trajectory type', ylabel='error in z (m)')
ax.grid()
ax.legend()
if save:
	fig.savefig(os.path.join(save_dir, "ez_boxplot_{}.pdf".format(traj_type)), format='pdf')
	fig.savefig(os.path.join(save_dir, "ez_boxplot_{}.png".format(traj_type)), format='png')

# Create a figure instance
# fig = plt.figure(1, figsize=(9, 6))

# # Create an axes instance
# ax = fig.add_subplot(111)

# # Create the boxplot
# bp = ax.boxplot(data_to_plot)

# Save the figure
# fig.savefig('fig1.png', bbox_inches='tight')

plt.show()