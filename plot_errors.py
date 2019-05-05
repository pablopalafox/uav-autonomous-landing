import matplotlib
import matplotlib.pyplot as plt 
import pandas as pd
import os

errors = pd.read_csv("/home/pablo/ws/log/errors/errors_test.csv")
#errors = pd.read_csv("/home/pablo/.ros/errors.csv")

# create log dir if it does not exist yet
save_dir = "/home/pablo/ws/log/errors"
if not os.path.exists(save_dir):
	os.makedirs(save_dir)


# extract data from DataFrame
t = errors['t'].tolist()
ex = errors['ex'].tolist()
ey = errors['ey'].tolist()
ez = errors['ez'].tolist()

# error x
fig, ax = plt.subplots()
ax.plot(t, ex)
ax.set(xlabel='time (s)', ylabel='error x (m)')
ax.grid()
fig.savefig(os.path.join(save_dir, "ex.pdf"), format='pdf')

# error y
fig, ax = plt.subplots()
ax.plot(t, ey)
ax.set(xlabel='time (s)', ylabel='error y (m)')
ax.grid()
fig.savefig(os.path.join(save_dir, "ey.pdf"), format='pdf')

# error z
fig, ax = plt.subplots()
ax.plot(t, ez)
ax.set(xlabel='time (s)', ylabel='error z (m)')
ax.grid()
fig.savefig(os.path.join(save_dir, "ez.pdf"), format='pdf')

plt.show()

