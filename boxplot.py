## numpy is used for creating fake data
import numpy as np
import os
import sys
import pandas as pd
import statistics as st

import matplotlib.pyplot as plt
import seaborn as sns
sns.set(style="whitegrid")

###############################################################

save = True
LIM_Y_MAX = 0.15
LIM_Y_MIN = 0.05
DPI = 5000

input_dir = "/home/pablo/ws/log/errors"
print("Reading from", input_dir) 

save_dir = "/home/pablo/ws/log"
save = True

ylabel_x = -0.11	
ylabel_y = 0.5
###############################################################

ex = []
ex_trajs = []
ex_modes = []

ey = []
ey_trajs = []
ey_modes = []


for i in range(1, 6):

	######### no pred
	errors_no_pred_linear = pd.read_csv(input_dir + "/errors_NO_pred_l{}.csv".format( i))
	errors_no_pred_circular = pd.read_csv(input_dir + "/errors_NO_pred_c{}.csv".format( i))

	# linear
	ex_no_pred_linear = errors_no_pred_linear['ex_real'].tolist()
	ex.append(st.mean(ex_no_pred_linear))
	ex_trajs.append('linear')
	ex_modes.append('w/o pred')

	ey_no_pred_linear = errors_no_pred_linear['ey_real'].tolist()
	ey.append(st.mean(ey_no_pred_linear))
	ey_trajs.append('linear')
	ey_modes.append('w/o pred')

	# circular
	ex_no_pred_circular = errors_no_pred_circular['ex_real'].tolist()
	ex.append(st.mean(ex_no_pred_circular))
	ex_trajs.append('circular')
	ex_modes.append('w/o pred')

	ey_no_pred_circular = errors_no_pred_circular['ey_real'].tolist()
	ey.append(st.mean(ey_no_pred_circular))
	ey_trajs.append('circular')
	ey_modes.append('w/o pred')


	######### pred
	errors_pred_linear = pd.read_csv(input_dir + "/errors_pred_l{}.csv".format( i))
	errors_pred_circular = pd.read_csv(input_dir + "/errors_pred_c{}.csv".format( i))

	# linear
	ex_pred_linear = errors_pred_linear['ex_real'].tolist()
	ex.append(st.mean(ex_pred_linear))
	ex_trajs.append('linear')
	ex_modes.append('w/ pred')

	ey_pred_linear = errors_pred_linear['ey_real'].tolist()
	ey.append(st.mean(ey_pred_linear))
	ey_trajs.append('linear')
	ey_modes.append('w/ pred')

	# circular
	ex_pred_circular = errors_pred_circular['ex_real'].tolist()
	ex.append(st.mean(ex_pred_circular))
	ex_trajs.append('circular')
	ex_modes.append('w/ pred')

	ey_pred_circular = errors_pred_circular['ey_real'].tolist()
	ey.append(st.mean(ey_pred_circular))
	ey_trajs.append('circular')
	ey_modes.append('w/ pred')

###################################################################3

fig = plt.figure()
ex_dic = {'traj': ex_trajs, 
	  	  'mode': ex_modes,
	  	  'ex': ex}
df_ex = pd.DataFrame(ex_dic, columns= ['traj', 'mode', 'ex'])
ax = sns.boxplot(y='ex', x='traj', 
     	    data=df_ex, 
         	palette="colorblind",
         	hue='mode',
         	width=0.5)
ax.set(xlabel='trajectory', ylabel='error in x axis (m)')
ax.yaxis.set_label_coords(ylabel_x, ylabel_y)
for i,artist in enumerate(ax.artists):
    # Set the linecolor on the artist to the facecolor, and set the facecolor to None
    col = artist.get_facecolor()
    artist.set_edgecolor(col)

if save:
	fig.savefig(os.path.join(save_dir, "boxplot_ex.pdf"), format='pdf', dpi=DPI)

###################################################################

fig = plt.figure()
ey_dic = {'traj': ey_trajs, 
	  	  'mode': ey_modes,
	  	  'ey': ey}
df_ey = pd.DataFrame(ey_dic, columns= ['traj', 'mode', 'ey'])
ax = sns.boxplot(y='ey', x='traj', 
     	    data=df_ey, 
         	palette="colorblind",
         	hue='mode',
         	width=0.5)
ax.set(xlabel='trajectory', ylabel='error in y axis (m)')
ax.yaxis.set_label_coords(ylabel_x, ylabel_y)
for i,artist in enumerate(ax.artists):
    # Set the linecolor on the artist to the facecolor, and set the facecolor to None
    col = artist.get_facecolor()
    print(col)
    artist.set_edgecolor(col)

if save:
	fig.savefig(os.path.join(save_dir, "boxplot_ey.pdf"), format='pdf', dpi=DPI)

plt.show()