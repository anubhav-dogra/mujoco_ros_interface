### MujocoInterface
every message length is taken from _model->nu rather than nq, coz number of joints are alot more if soft body is added!

#### plugin is required to load if using plugins in xml.


### hw_interface_node
`mujoco_hw_interface_node` renamed to iiwa in CmakeLists.txt, and 
the controller is also spawned with ns=/"iiwa". Controller yaml ns is also iiwa.
Now controller manager services are loaded based on the node handle. If the node handle is `ros::Nodehandle nh("~")`, the rosservice will be `/node_name/controller_manager/...`, if the 
`ros::Nodehandle nh()`, the rosservice will be `/controller_manager/...`


### ROS Controller important
hardware interfaces are registered in the `mujoco_hw_interface_test`. When the controller is loaded, the commands are populated automatically through the interface!!
i.e. there is no need to subscribe from the controller topic !! 