import pickle as pkl
import matplotlib.pyplot as plt

# path = 'logs_up_down_vertical_controller.pkl'
# path = 'logs_up_down_horizontal_controller.pkl'
# path = 'logs_head_move_still_controller.pkl'
path = 'logs_up_down_sideways_controller.pkl'
with open(path, 'rb') as f:
    logs = pkl.load(f)

# logs is a list of 3 points (np.arrays of shape 3) Plot the trajectory of 3D points and show
# the trajectory of the 3 points in 3D space.
x_coords = []
y_coords = []
z_coords = []
for log in logs:
    x_coords.append(log[0])
    y_coords.append(log[1])
    z_coords.append(log[2])
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_coords, y_coords, z_coords)
# label the axes
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
# axis equals
ax.axis('equal')
plt.show()

# goal, see if the controller axes or the global axes are consistent

"""
Up down vertical controller is along the y-axis
Up down horizontal controller is along the y-axis.

So the global axes are consistent.

Maybe orientation and translation are not in the same frame?
"""