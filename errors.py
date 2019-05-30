import matplotlib
import matplotlib.pyplot as plt 
import pandas as pd
import os
import sys
import itertools

DPI = 5000

###########################################################################################

## input folder
# if len(sys.argv) == 2:
# 	input_dir = sys.argv[1]
# else:
# 	input_dir = "/home/pablo/ws/log/errors"

input_dir = "/home/pablo/ws/log/errors"
print("Reading from", input_dir) 


## -- type of trajectory and experiment number
if len(sys.argv) == 3:
	traj_types = [sys.argv[1]]
	test_nums = [sys.argv[2]]
else:
	traj_types = ["l", "c"] # c for circular ---- l for linear
	test_nums = [str(x) for x in range(1, 6)]

###########################################################################################

for traj_type, test_num in itertools.product(traj_types, test_nums):

	print("traj_type:", traj_type, "- test_num: ", test_num)
	
	## save dir
	save_dir = "/home/pablo/ws/log/errors/{}{}".format(traj_type, test_num)
	if not os.path.exists(save_dir):
		os.makedirs(save_dir)

	## read files
	errors_pred = pd.read_csv(input_dir + "/errors_pred_{}{}.csv".format(traj_type, test_num))
	errors_no_pred = pd.read_csv(input_dir + "/errors_NO_pred_{}{}.csv".format(traj_type, test_num))

	# extract data from DataFrame
	t_pred = errors_pred['t'].tolist()
	ex_pred_real = errors_pred['ex_real'].tolist()
	ey_pred_real = errors_pred['ey_real'].tolist()
	ez_pred_real = errors_pred['ez_real'].tolist()
	ex_pred_target = errors_pred['ex_target'].tolist()
	ey_pred_target = errors_pred['ey_target'].tolist()
	ez_pred_target = errors_pred['ez_target'].tolist()

	t_no_pred = errors_no_pred['t'].tolist()
	ex_no_pred_real = errors_no_pred['ex_real'].tolist()
	ey_no_pred_real = errors_no_pred['ey_real'].tolist()
	ez_no_pred_real = errors_no_pred['ez_real'].tolist()
	ex_no_pred_target = errors_no_pred['ex_target'].tolist()
	ey_no_pred_target = errors_no_pred['ey_target'].tolist()
	ez_no_pred_target = errors_no_pred['ez_target'].tolist()

	## error x
	fig, ax = plt.subplots()
	ax.plot(t_pred, ex_pred_real, 'g', label='predictive')
	ax.plot(t_no_pred, ex_no_pred_real, 'r', label='non-predictive')
	ax.set(xlabel='time (s)', ylabel='error in x (m)')
	bottom, top = plt.ylim()  # return the current ylim
	plt.ylim((-1, 1))   # set the ylim to bottom, top
	ax.grid()
	ax.legend()
	fig.savefig(os.path.join(save_dir, "ex.pdf"), format='pdf', dpi=DPI)
	#fig.savefig(os.path.join(save_dir, "ex.png"), format='png')
	plt.close()

	## error y
	fig, ax = plt.subplots()
	ax.plot(t_pred, ey_pred_real, 'g', label='predictive')
	ax.plot(t_no_pred, ey_no_pred_real, 'r', label='non-predictive')
	ax.set(xlabel='time (s)', ylabel='error in y (m)')
	plt.ylim((-0.5, 0.5))   # set the ylim to bottom, top
	ax.grid()
	ax.legend()
	fig.savefig(os.path.join(save_dir, "ey.pdf"), format='pdf', dpi=DPI)
	#fig.savefig(os.path.join(save_dir, "ey.png"), format='png')
	plt.close()

	# error z
	fig, ax = plt.subplots()
	ax.plot(t_pred, ez_pred_real, 'g', label='predictive')
	ax.plot(t_no_pred, ez_no_pred_real, 'r', label='non-predictive')
	ax.set(xlabel='time (s)', ylabel='error in z real (m)')
	ax.grid()
	ax.legend()
	fig.savefig(os.path.join(save_dir, "ez.pdf"), format='pdf', dpi=DPI)
	#fig.savefig(os.path.join(save_dir, "ez.png"), format='png')
	plt.close()



