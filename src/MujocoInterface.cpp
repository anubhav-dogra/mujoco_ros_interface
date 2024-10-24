
#include <mujoco_ros_interface/MujocoInterface.h>
#include <ros/ros.h>

MujocoInterface* MujocoInterface::instance = nullptr;


////////////////////////////////////////////////////////////////////////////////////////////////////
//                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
MujocoInterface::MujocoInterface(const std::string &xmlLocation,
                                 const std::string &jointStateTopicName,
                                 const std::string &endEffectorStatePoseTopicName,
                                 const std::string &jointControlTopicName,
                                 const std::string &endEffectorName,
                                 const std::string &pluginDirectory,
                                 ControlMode controlMode,
                                 int simulationFrequency,
                                 int visualizationFrequency)
                                 : _controlMode(controlMode),
                                   _simFrequency(simulationFrequency)
                                //    stop_vis_thread_(false)
{
    // Initialize ROS1 node handle
    _nh = ros::NodeHandle("~");  
    instance = this;

    // load all plugins before loading model!!   
    // mj_loadPluginLibrary("/home/terabotics/mujoco_ws/src/mujoco/mujoco-3.2.3/bin/mujoco_plugin/libelasticity.so");
    mj_loadAllPluginLibraries(pluginDirectory.c_str(), nullptr);

    // Load the robot model
    char error[1000] = "Could not load binary model";
    _model = mj_loadXML(xmlLocation.c_str(), nullptr, error, 1000);

    if (!_model)
    {
        ROS_ERROR("Error loading model: %s", error);
        ros::shutdown();
    }

    _jointState = mj_makeData(_model);  // Initialize joint state

    //set intial state with the keyframe mechanism from xml
    mju_copy(_jointState->qpos, _model->key_qpos, _model->nq);
    mj_step(_model, _jointState);

    // Resize arrays based on the number of joints/controls (nq/ nu)in the model
    _jointStateMessage.name.resize(_model->nu);
    _jointStateMessage.position.resize(_model->nu);
    _jointStateMessage.velocity.resize(_model->nu);
    _jointStateMessage.effort.resize(_model->nu);
    _error.resize(_model->nu);
    _errorDerivative.resize(_model->nu);
    _errorIntegral.resize(_model->nu);
    
    _referencePosition.resize(_model->nu, 0.0);

    _referenceTorque.resize(_model->nu, 0.0);
    _referenceTorqueSim.resize(_model->nu, 0.0);
    // end_effector body id
    _endeffector_bodyId = mj_name2id(_model, mjOBJ_BODY, endEffectorName.c_str()); // later to be replace with site! 

    // Record joint names
    // for (int i = 0; i < _model->nq; i++)
    for (int i = 0; i < _model->nu; i++)
    {
        _jointStateMessage.name[i] = mj_id2name(_model, mjOBJ_JOINT, i);
    }

    // Create joint state publisher and joint command subscriber
    _jointStatePublisher = _nh.advertise<sensor_msgs::JointState>(jointStateTopicName, 1);

    _endeffectorPosePublisher = _nh.advertise<geometry_msgs::PoseStamped>(endEffectorStatePoseTopicName, 1);
    
    _jointCommandSubscriber = _nh.subscribe(jointControlTopicName, 1, &MujocoInterface::joint_command_callback, this);

    _f_t_sensorPublisher = _nh.advertise<geometry_msgs::WrenchStamped>("mujoco_f_t_sensor", 1);
    // Initialize Graphics Library FrameWork (GLFW)
    if (!glfwInit())
    {
        ROS_ERROR("Failed to initialize Graphics Library FrameWork (GLFW).");
        ros::shutdown();
        return;
    }

    // Create a GLFW window
    _window = glfwCreateWindow(1200, 900, "MuJoCo Visualization", nullptr, nullptr);
    if (!_window)
    {
        ROS_ERROR("Failed to create GLFW window");
        glfwTerminate();
        ros::shutdown();
        return;
    }

    // Make the OpenGL context current
    glfwMakeContextCurrent(_window);
    glfwSwapInterval(1);  // Set swap interval for vsync

    // Initialize MuJoCo rendering context
    mjv_defaultCamera(&_camera);
    mjv_defaultOption(&_renderingOptions);
    mjv_defaultPerturb(&_perturbation);
    mjr_defaultContext(&_context);
    mjv_makeScene(_model, &_scene, 2000);

    // Create MuJoCo rendering context
    // glfwMakeContextCurrent(_window);
    mjr_makeContext(_model, &_context, mjFONTSCALE_100);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(_window, MujocoInterface::keyboardCallback);
    glfwSetCursorPosCallback(_window, MujocoInterface::mouse_moveCallback);
    glfwSetMouseButtonCallback(_window, MujocoInterface::mouse_buttonCallback);
    glfwSetScrollCallback(_window, MujocoInterface::scrollCallback);

    // visualize contact force
    // _renderingOptions.flags[mjVIS_CONTACTFORCE] = 1;
    // _model->vis.map.force = 0.05;

    // Install control callback for gravity compensation
    // mjcb_control = MujocoInterface::controlCallback;

    // Create timers
    _simTimer = _nh.createTimer(ros::Duration(1.0 / simulationFrequency), &MujocoInterface::update_simulation, this);
    _visTimer = _nh.createTimer(ros::Duration(1.0 / visualizationFrequency), &MujocoInterface::update_visualization, this);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           Destructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
MujocoInterface::~MujocoInterface()
{
    mj_deleteData(_jointState);
    mj_deleteModel(_model);
    mjv_freeScene(&_scene);
    mjr_freeContext(&_context);
    glfwDestroyWindow(_window);
    glfwTerminate();
    // Signal the visualization thread to stop
    // stop_vis_thread_ = true;
    // if (vis_thread_.joinable())
    // {
    //     vis_thread_.join();
    // }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                               Set the gains for feedback control                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool MujocoInterface::set_feedback_gains(const double &proportional,
                                         const double &integral,
                                         const double &derivative)
{
    if(proportional < 0 or integral < 0 or derivative < 0)
    {
        ROS_WARN("Gains cannot be negative.");
        return false;
    }
    else
    {
        _proportionalGain = proportional;
        _derivativeGain   = derivative;
        _integralGain     = integral;
        return true;
    }
}          

////////////////////////////////////////////////////////////////////////////////////////////////////
//                          Sets camera viewing position & angle                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MujocoInterface::set_camera_properties(const std::array<double, 3> &focalPoint,
                                            const double &distance,
                                            const double &azimuth,
                                            const double &elevation,
                                            const bool &orthographic)
{
    _camera.lookat[0]    = focalPoint[0];
    _camera.lookat[1]    = focalPoint[1];
    _camera.lookat[2]    = focalPoint[2];
    _camera.distance     = distance; 
    _camera.azimuth      = azimuth;    
    _camera.elevation    = elevation;
    _camera.orthographic = orthographic;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
                        // get info for harware interface
////////////////////////////////////////////////////////////////////////////////////////////////////

int MujocoInterface::getNumberOfJoints() const
{
    return _model->njnt;
}

std::vector<std::string> MujocoInterface::getJointNames() const
{
    std::vector<std::string> jointNames;
    for (int i = 0; i < _model->njnt; i++) {
        jointNames.push_back(mj_id2name(_model, mjOBJ_JOINT, i));
    }
    return jointNames;
}
std::vector<double> MujocoInterface::getJointPositions()
{
    // mj_step(_model, _jointState);
    std::vector<double> jointPositions;
    for (int i = 0; i < _model->nu; i++) {
        jointPositions.push_back(_jointState->qpos[i]);  // Use qpos for joint positions
    }
    return jointPositions;
}
std::vector<double> MujocoInterface::getJointVelocities()
{
    // mj_step(_model, _jointState);
    std::vector<double> jointVelocities;
    for (int i = 0; i < _model->nv; i++) {
        jointVelocities.push_back(_jointState->qvel[i]);  // Use qvel for joint velocities
    }
    return jointVelocities;
}
std::vector<double> MujocoInterface::getJointEfforts()
{
    // mj_step(_model, _jointState);
    std::vector<double> jointEfforts;
    for (int i = 0; i < _model->nu; i++) {
        jointEfforts.push_back(_jointState->actuator_force[i]);  // Use qfrc_actuators for torques
    }
    return jointEfforts;
}

// DOUBT::: Remove or modify the callback for joint commands if it's redundant
// with the ros_control framework, which will take over control.
void MujocoInterface::setJointCommands(const std::vector<double>& efforts)
{
    // mj_step(_model, _jointState);
    if (efforts.size() != _model->nu) {
        ROS_ERROR("Size of efforts vector does not match the number of actuated joints.");
        return;
    }
    // std::cout << "command_recieved" << efforts[1] <<std::endl;
    for (int i = 0; i < _model->nu; i++) {
        _referenceTorque[i] = efforts[i];
        
        // _jointState->ctrl[i] = efforts[i];  // Apply efforts (torques) to the actuators
    }
}

//                                    Update the simulation                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MujocoInterface::update_simulation(const ros::TimerEvent&)
{
    // std::lock_guard<std::mutex> lock(mtx_);  // Ensure thread safety
    if(!_model || !_jointState)
    {
        ROS_ERROR("MuJoCo model or data is not initialized.");
        return;
    }
    
    // Compute control input based on mode
    switch(_controlMode)
    {
        case POSITION:
        case VELOCITY:
        {
            for(int i = 0; i < _model->nu; i++)
            {     
                double error = _referencePosition[i] - _jointState->qpos[i];                         // Position error

                _errorIntegral[i] += error / (double)_simFrequency;                                 // Cumulative error

                double errorDerivative = (error - _error[i]) * _simFrequency;                       // Change in error over time

                _jointState->ctrl[i] = _proportionalGain * error
                                     + _integralGain * _errorIntegral[i]
                                     + _derivativeGain * errorDerivative;                           // Apply PID control

                _error[i] = error;                                                                  // Update error for next iteration           
            }  
            break;
        }
        case TORQUE:
        {
            // // No need to do anything as control has been set in command callback function.
            // break;

            // mjtNum gravity_torques[_model->nv];

            // // Compute gravity torques (velocities and accelerations are set to zero)
            // mjtNum zero[_model->nv] = {0};
            // mj_rne(_model, _jointState, 0, gravity_torques);

            // qfrc_bias = gravity_torques+coriolis component;
            if (_referenceTorque.empty())
            {
                
                for (int i = 0; i < _model->nv; i++) _jointState->ctrl[i] = _jointState->qfrc_bias[i];  // gravity torques. +  coriolis is qdot is there
            }
            else
            {   
                for (int i = 0; i < _model->nu; i++) _jointState->ctrl[i] = _jointState->qfrc_bias[i]+_referenceTorque[i];  // Assuming _jointState->ctrl[i] contains additional torques
            }
            // std::cout << _jointState->ctrl[0] << std::endl;
            break;
            

        }
        default:
        {
            for(int i = 0; i < _model->nu; i++) _jointState->ctrl[i] = 0.0;                         // Don't move?
            ROS_WARN_THROTTLE(0.1, "Unknown control mode.");
            break;
        }
    }
    // std::cout << "Before mj_step: " << _jointState->qpos[0] << std::endl;
    mj_step(_model, _jointState);  // Take a step in the simulation
    // std::cout << "After mj_step: " << _jointState->qpos[0] << std::endl;
    // Add joint state data to ROS1 message, then publish
    _jointStateMessage.header.stamp = ros::Time::now();
    _jointStateMessage.header.frame_id = "world";
    for(int i = 0; i < _model->nu; i++)
    {
        _jointStateMessage.position[i] = _jointState->qpos[i];
        _jointStateMessage.velocity[i] = _jointState->qvel[i];
        _jointStateMessage.effort[i]   = _jointState->actuator_force[i];
    }
    
    _jointStatePublisher.publish(_jointStateMessage);  // As it says

  
    mjtNum* body_position = _jointState->xpos + 3*_endeffector_bodyId;
    mjtNum* body_orientation = _jointState->xquat + 4*_endeffector_bodyId;

    _endeffectorPoseMessage.header.stamp = ros::Time::now();
    _endeffectorPoseMessage.header.frame_id = "world";
    _endeffectorPoseMessage.pose.position.x = body_position[0];
    _endeffectorPoseMessage.pose.position.y = body_position[1];
    _endeffectorPoseMessage.pose.position.z = body_position[2];
    _endeffectorPoseMessage.pose.orientation.w = body_orientation[0];
    _endeffectorPoseMessage.pose.orientation.x = body_orientation[1];
    _endeffectorPoseMessage.pose.orientation.y = body_orientation[2];
    _endeffectorPoseMessage.pose.orientation.z = body_orientation[3];

    _endeffectorPosePublisher.publish(_endeffectorPoseMessage);

    
    _f_t_sensorMessage.header.stamp = ros::Time::now();
    _f_t_sensorMessage.header.frame_id = "sensor_link";
    _f_t_sensorMessage.wrench.force.x = _jointState->sensordata[0];
    _f_t_sensorMessage.wrench.force.y = _jointState->sensordata[1];
    _f_t_sensorMessage.wrench.force.z = _jointState->sensordata[2]; // direction points from child to parent !!
    _f_t_sensorMessage.wrench.torque.x = _jointState->sensordata[3];
    _f_t_sensorMessage.wrench.torque.y = _jointState->sensordata[4];
    _f_t_sensorMessage.wrench.torque.z = _jointState->sensordata[5];
    _f_t_sensorPublisher.publish(_f_t_sensorMessage);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                    Update the 3D simulation                                    //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MujocoInterface::update_visualization(const ros::TimerEvent&)
{
    {
        glfwMakeContextCurrent(_window);  // Ensure OpenGL context is current
        
        mjv_updateScene(_model, _jointState, &_renderingOptions, NULL, &_camera, mjCAT_ALL, &_scene);  // Update 3D rendering

        // Get framebuffer size
        int width, height;
        glfwGetFramebufferSize(_window, &width, &height);
        mjrRect viewport = {0, 0, width, height};

        mjr_render(viewport, &_scene, &_context);  // Render scene
        
        // Swap buffers and process events
        glfwSwapBuffers(_window);
        glfwPollEvents();
    }

}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                    Handle joint commands                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MujocoInterface::joint_command_callback(const std_msgs::Float64MultiArray::ConstPtr msg)
{ 
    if (msg->data.size() != _model->nu)  // Expecting one more element for mode
    {
        ROS_WARN_THROTTLE(5, "Received joint command with incorrect size.");
        return;
    }
    else
    {
        switch(_controlMode)
        {
            case POSITION:
            {
                for(int i = 0; i < _model->nu; i++) _referencePosition[i] = msg->data[i];  // Assign new reference position
                break;
            }
            case VELOCITY:
            {
                for(int i = 0; i < _model->nu; i++) _referencePosition[i] += msg->data[i] / (double)_simFrequency;  // Integrate velocity to get position
                break;
            }
            case TORQUE:
            {
                // for(int i = 0; i < _model->nu; i++) _jointState->ctrl[i] = msg->data[i];  // Assign control inputs directly
                // break;

                for(int i = 0; i < _model->nu; i++) _referenceTorque[i] = msg->data[i];  // Assign control inputs directly
                break;

            }
            default:
            {
                ROS_WARN_THROTTLE(5, "Unknown control mode.");
                break;
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////
/////////////////// GLFW callback functions /////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
    
void MujocoInterface::scrollCallback(GLFWwindow* _window, double xoffset, double yoffset)
{
    if (instance){
        instance->scroll(_window, xoffset, yoffset);
    }
}
void MujocoInterface::keyboardCallback(GLFWwindow* _window, int key, int scancode, int action, int mods)
{
    // backspace: reset simulation
    if (instance){
        instance->keyboard(_window, key, scancode, action, mods);
    }
}
void MujocoInterface::mouse_buttonCallback(GLFWwindow* _window, int button, int action, int mods)
{
    if (instance){
        instance->mouse_button(_window, button, action, mods);
    }
}
void MujocoInterface::mouse_moveCallback(GLFWwindow* _window, double xpos, double ypos)
{
    if (instance){
        instance->mouse_move(_window, xpos, ypos);
    }
}
void MujocoInterface::scroll(GLFWwindow* _window, double xoffset, double yoffset)
{
    mjv_moveCamera(_model, mjMOUSE_ZOOM, 0, -0.05*yoffset, &_scene, &_camera);
}
void MujocoInterface::keyboard(GLFWwindow* _window, int key, int scancode, int action, int mods)
{
    if (action==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
    mj_resetData(_model, _jointState);
    mj_forward(_model, _jointState);
  }
}
void MujocoInterface::mouse_button(GLFWwindow* _window, int button, int action, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right = (glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(_window, &lastx, &lasty);
}
void MujocoInterface::mouse_move(GLFWwindow* _window, double xpos, double ypos)
{
    // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(_window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(_window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(_window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(_model, action, dx/height, dy/height, &_scene, &_camera);
}

// void MujocoInterface::controlCallback(const mjModel *_model, mjData *_jointState)
// {
//     mjtNum gravity_torques[_model->nv];

//     // Compute gravity torques (velocities and accelerations are set to zero)
//     std::cout << _jointState->ctrl[1] << std::endl;
//     mjtNum zero[_model->nv] = {0};
//     mj_rne(_model, _jointState, 0, gravity_torques);

//     for (int i = 0; i < _model->nv; i++) _jointState->ctrl[i] = gravity_torques[i]; // Assuming _jointState->ctrl[i] contains additional torques   

// }

