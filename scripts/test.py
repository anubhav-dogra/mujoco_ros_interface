import mujoco
import mujoco.viewer as viewer
import os


curr_path = os.path.dirname(os.path.abspath(__file__))
# get one step back from the current path
curr_path = os.path.abspath(os.path.join(curr_path, os.pardir))

# Load the 2R robot model
model = mujoco.MjModel.from_xml_path(curr_path + '/test/iiwa14_new.xml')
data = mujoco.MjData(model)

mujoco.mj_setKeyframe(model, data, 0)


print('positions', data.qpos)
print('velocities', data.qvel)

# Open a viewer window
viewer.launch_passive(model, data)

# data.ctrl[:] = [0.5, 0.5]
# Step the simulation
mujoco.mj_step(model, data)

input("Press Enter to continue...")
