import matplotlib
import matplotlib.pyplot as plt 
import pandas as pd
import os
import sys
import itertools
import statistics as st
DPI = 5000

###########################################################################################

input_dir = "/home/pablo/ws/log/errors"
print("Reading from", input_dir) 

###########################################################################################
	
## linear speed
if len(sys.argv) == 2:
	linear_speed = sys.argv[1]
else:
	print("Exiting...")
	exit()
## read files
errors_pred = pd.read_csv(input_dir + "/errors_pred_lifelong_{}.csv".format(linear_speed))

## save dir
save_dir = "/home/pablo/ws/log/errors/lifelong_{}".format(linear_speed)
if not os.path.exists(save_dir):
	os.makedirs(save_dir)


# get only some lines from the data
zoom_in = True
if zoom_in:
	row_200_sec = errors_pred[errors_pred['t'].gt(200)].index[0]
	errors_pred = errors_pred.iloc[0:row_200_sec]


# extract data from DataFrame
t_pred = errors_pred['t'].tolist()
ex_pred_real = errors_pred['ex_real'].tolist()
ey_pred_real = errors_pred['ey_real'].tolist()
ez_pred_real = errors_pred['ez_real'].tolist()
ex_pred_target = errors_pred['ex_target'].tolist()
ey_pred_target = errors_pred['ey_target'].tolist()
ez_pred_target = errors_pred['ez_target'].tolist()

## error x
fig, ax = plt.subplots()
ax.plot(t_pred, ex_pred_real, 'g')
ax.set(xlabel='time (s)', ylabel='error in x (m)')
bottom, top = plt.ylim()  # return the current ylim
#plt.ylim((-1, 1))   # set the ylim to bottom, top
ax.grid()
fig.savefig(os.path.join(save_dir, "ex.pdf"), format='pdf', dpi=DPI)
plt.close()

## error y
fig, ax = plt.subplots()
ax.plot(t_pred, ey_pred_real, 'g')
ax.set(xlabel='time (s)', ylabel='error in y (m)')
#plt.ylim((-0.5, 0.5))   # set the ylim to bottom, top
ax.grid()
fig.savefig(os.path.join(save_dir, "ey.pdf"), format='pdf', dpi=DPI)
plt.close()

# error z
fig, ax = plt.subplots()
ax.plot(t_pred, ez_pred_real, 'g')
ax.set(xlabel='time (s)', ylabel='error in z (m)')
ax.grid()
fig.savefig(os.path.join(save_dir, "ez.pdf"), format='pdf', dpi=DPI)
plt.close()

abs_ex = [abs(ex) for ex in ex_pred_real]
abs_ey = [abs(ey) for ey in ey_pred_real]

print("Total test time", t_pred[-1])
print()
print("Min error x", min(abs_ex))
print("Max error x", max(abs_ex))
print("Mean error x", st.mean(abs_ex))
print()
print("Min error y", min(abs_ey))
print("Max error y", max(abs_ey))
print("Mean error y", st.mean(abs_ey))
