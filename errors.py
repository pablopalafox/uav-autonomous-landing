import matplotlib
import matplotlib.pyplot as plt 
import pandas as pd
import os
import sys

if len(sys.argv) == 2:
	save_dir = sys.argv[1]
else:
	save_dir = "/home/pablo/ws/log/errors"

print("Reading from", save_dir) 


errors_pred = pd.read_csv(save_dir + "/errors_pred.csv")
errors_no_pred = pd.read_csv(save_dir + "/errors_NO_pred.csv")

# extract data from DataFrame
t_pred = errors_pred['t'].tolist()
ex_pred = errors_pred['ex'].tolist()
ey_pred = errors_pred['ey'].tolist()
ez_pred = errors_pred['ez'].tolist()

t_no_pred = errors_no_pred['t'].tolist()
ex_no_pred = errors_no_pred['ex'].tolist()
ey_no_pred = errors_no_pred['ey'].tolist()
ez_no_pred = errors_no_pred['ez'].tolist()

print(len(ez_pred))
print(len(ez_no_pred))

# align z
diff = ez_no_pred[0] - ez_pred[0]
ez_pred = [z + diff for z in ez_pred] 


# error x
fig, ax = plt.subplots()
ax.plot(t_pred, ex_pred, 'g', label='predictive')
ax.plot(t_no_pred, ex_no_pred, 'r', label='non-predictive')
ax.set(xlabel='time (s)', ylabel='error in x (m)')
ax.grid()
ax.legend()
fig.savefig(os.path.join(save_dir, "ex.pdf"), format='pdf')
fig.savefig(os.path.join(save_dir, "ex.png"), format='png')

# error y
fig, ax = plt.subplots()
ax.plot(t_pred, ey_pred, 'g', label='predictive')
ax.plot(t_no_pred, ey_no_pred, 'r', label='non-predictive')
ax.set(xlabel='time (s)', ylabel='error in y (m)')
ax.grid()
ax.legend()
fig.savefig(os.path.join(save_dir, "ey.pdf"), format='pdf')
fig.savefig(os.path.join(save_dir, "ey.png"), format='png')

# error z
fig, ax = plt.subplots()
ax.plot(t_pred, ez_pred, 'g', label='predictive')
ax.plot(t_no_pred, ez_no_pred, 'r', label='non-predictive')
ax.set(xlabel='time (s)', ylabel='error in z (m)')
ax.grid()
ax.legend()
fig.savefig(os.path.join(save_dir, "ez.pdf"), format='pdf')
fig.savefig(os.path.join(save_dir, "ez.png"), format='png')

plt.show()

