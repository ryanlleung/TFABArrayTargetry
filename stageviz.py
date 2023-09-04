
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, TextBox, Button

from scipy.spatial.transform import Rotation
from kinematics import get_geometry, get_transformation, get_transformation_TP, get_target_plane, get_stage_positions


# Plot the workspace
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.view_init(elev=30,azim=-70)

# Set the limits of the plot
xlim = np.array([-500, 500])
ylim = np.array([0, 800])
zlim = np.array([-100, 100])

ax.set_xlim(xlim)
ax.set_ylim(ylim)
ax.set_zlim(zlim)
lims = np.array([xlim, ylim, zlim])
min_value = np.min(lims)
max_value = np.max(lims)
norm_lims = (lims - min_value) / (max_value - min_value)
ax.set_box_aspect(np.diff(norm_lims, axis=1).flatten())

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')


# Add sliders
ax_TPx = plt.axes([0.25, 0.05, 0.65, 0.03])
ax_TPy = plt.axes([0.25, 0.1, 0.65, 0.03])

slider_x = Slider(ax_TPx, 'TPx', -95, 95, valinit=0, valstep=1)
slider_y = Slider(ax_TPy, 'TPy', -145, 145, valinit=145, valstep=1)

# Add textbox
ax_stages = plt.axes([0.25, 0.83, 0.65, 0.05])
ax_stageTB = TextBox(ax_stages, 'Stage Positions:', initial='--')

ax_setpoint = plt.axes([0.25, .9, 0.65, 0.05])
ax_setpointTB = TextBox(ax_setpoint, 'Setpoint:', initial='-20, 681.29, 0, 0, 0')


# Initialise the dots
dot0 = ax.scatter([0], [0], [0], color='g', marker='X')
dot1 = ax.scatter([0], [0], [0], color='b', marker='s')
dot2 = ax.scatter([0], [0], [0], color='b', marker='s')
dot3 = ax.scatter([0], [0], [0], color='b', marker='x')
dot4 = ax.scatter([0], [0], [0], color='b', marker='s')
dot5 = ax.scatter([0], [0], [0], color='b', marker='o')
dot6 = ax.scatter([0], [0], [0], color='b', marker='.')
dotTP = ax.scatter([0], [0], [0], color='r', marker='.')

# Initialise the surface
X = np.arange(0, 1, 0.1)
Y = np.arange(0, 1, 0.1)
X, Y = np.meshgrid(X, Y)
Z = np.zeros_like(X)
surf = ax.plot_surface(X, Y, Z, visible=False)


# Update the plot when the sliders are changed
def update(setpoint='-20, 681.29, 0, 0, 0', TPx=0, TPy=0):

    # Get values from sliders
    TPx = slider_x.val
    TPy = slider_y.val

    # Get values from textboxes
    setpoint = np.array([float(x) for x in ax_setpointTB.text.split(',')])

    # Get the transformation matrix
    stagePos = get_stage_positions(setpoint, [TPx,TPy,0,0,0])

    # Get the transformation matrices
    T01, T02, T03, T04, T05, T06 = get_transformation(stagePos)
    T0TP = get_transformation_TP(stagePos, [TPx,TPy,0,0,0])

    # Set the position of dots
    dot1._offsets3d = ([T01[0][3]],[T01[1][3]],[T01[2][3]])
    dot2._offsets3d = ([T02[0][3]],[T02[1][3]],[T02[2][3]])
    dot3._offsets3d = ([T03[0][3]],[T03[1][3]],[T03[2][3]])
    dot4._offsets3d = ([T04[0][3]],[T04[1][3]],[T04[2][3]])
    dot5._offsets3d = ([T05[0][3]],[T05[1][3]],[T05[2][3]])
    dot6._offsets3d = ([T06[0][3]],[T06[1][3]],[T06[2][3]])
    dotTP._offsets3d = ([T0TP[0][3]],[T0TP[1][3]],[T0TP[2][3]])

    # Get Euler angles from T0TP
    R0TP = Rotation.from_matrix(T0TP[0:3,0:3])
    Rx, Ry = R0TP.as_euler('xyz', degrees=True)[:2]

    # Set the position of the plane
    TS = get_target_plane(stagePos)

    # Remove the previous plane and plot the new one
    global surf
    surf.remove()
    surf = ax.plot_surface(TS[:,:,0], TS[:,:,1], TS[:,:,2], color='r', alpha=0.2)

    desc = f"X: {stagePos[0]:.2f}, Y: {stagePos[1]:.2f}, Z: {stagePos[2]:.2f}, Rx: {stagePos[3]:.2f}, Ry: {stagePos[4]:.2f}"
    ax_stageTB.set_val(desc)

    fig.canvas.draw_idle()

    return

# Function to reset the plot view and values
def reset(event):
    ax.view_init(elev=30,azim=-70)
    slider_x.reset()
    slider_y.reset()
    ax_setpointTB.set_val('-20, 681.29, 0, 0, 0')
    return
reset_button_ax = fig.add_axes([0.8, 0.15, 0.1, 0.05])
reset_button = Button(reset_button_ax, 'Reset')
reset_button.on_clicked(reset)


update() # Initialise the plot
slider_x.on_changed(update)
slider_y.on_changed(update)
ax_setpointTB.on_submit(update)

plt.show()
