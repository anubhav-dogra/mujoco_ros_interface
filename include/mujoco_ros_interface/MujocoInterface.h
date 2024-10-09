/**
 * @file   MuJoCoInterface.h
 * @author Jon Woolfrey
 * @data   August 2024
 * @brief  A class for connecting a MuJoCo simulation with ROS2.
 */
 
#ifndef MUJOCOINTERFACE_H
#define MUJOCOINTERFACE_H

#include <GLFW/glfw3.h>                                                                             // Graphics Library Framework; for visualisation
#include <iostream>                                                                                 // std::cerr, std::cout
#include <mujoco/mujoco.h>                                                                          // Dynamic simulation library
#include <ros/ros.h>                                                                        // ROS2 C++ libraries.
#include <sensor_msgs/JointState.h>                                                          // For publishing / subscribing to joint states.
#include <std_msgs/Float64MultiArray.h>                                                         // For subscribing to joint commands>
#include <geometry_msgs/PoseStamped.h> 
#include <geometry_msgs/WrenchStamped.h> 
#include <thread>
#include <mutex>                                              

 enum ControlMode {POSITION, VELOCITY, TORQUE, UNKNOWN};                                            // This needs a global scope
        
/**
 * This class launches both a MuJoCo simulation, and ROS1 node for communication.
 */
class MujocoInterface {
    public:
            
        /**
         * Contructor.
         * @param filePath Location of an .xml file specifying a robot model and/or scene.
         */
        MujocoInterface(const std::string &xmlLocation,
                        const std::string &jointStateTopicName,
                        const std::string &endEffectorStatePoseTopicName,
                        const std::string &jointControlTopicName,
                        const std::string &endEffectorName,
                        const std::string &pluginDirectory,
                        ControlMode controlMode = TORQUE,
                        int simulationFrequency = 1000,
                        int visualizationFrequency = 20);
        
        /**
        * Deconstructor.
        */
        ~MujocoInterface();
        
        /**
         * Set the gains for feedback control.
         * @param proportional The gain in position error.
         * @param integral The gain on the accumulated position error.
         * @param derivative The gain in the change in position error.
         * @return False if there is a problem with the input arguments.
         */
        bool
        set_feedback_gains(const double &proportional,
                           const double &integral,
                           const double &derivative);
                           
        
        /**
         * Sets the viewing properties in the window.
         * @param focalPoint Defines the x, y, z coordinate for the focus of the camera.
         * @param distance The distance from said focal point to the camera.
         * @param azimuth The angle of rotation around the focal point.
         * @param elevation The angle of the line of sight relative to the ground plane.
         * @param orthographic Type of projection. True for orthographics, false for perspective.
         */
        void
        set_camera_properties(const std::array<double,3> &focalPoint,
                              const double &distance = 2.5,
                              const double &azimuth = 135.0,
                              const double &elevation = -30.0,
                              const bool   &orthographic = false);


        int getNumberOfJoints() const;
        std::vector<std::string> getJointNames() const;
        std::vector<double> getJointPositions();
        std::vector<double> getJointVelocities();
        std::vector<double> getJointEfforts();
        void setJointCommands(const std::vector<double>& j_commands);
        // void getJointCommands(std::vector<double>& j_commands) const;

        /**
         * @brief GLFW callback function for scroll events. Calls MujocoInterface::scroll() for the singleton instance.
         *
         * @param[in] _window GLFW window
         * @param[in] xoffset horizontal scroll offset
         * @param[in] yoffset vertical scroll offset
         */
        static void scrollCallback(GLFWwindow* _window, double xoffset, double yoffset);


        static void keyboardCallback(GLFWwindow* _window, int key, int scancode, int action, int mods);
        static void mouse_buttonCallback(GLFWwindow* _window, int button, int action, int mods);
        static void mouse_moveCallback(GLFWwindow* _window, double xpos, double ypos);


    private:

        ros::NodeHandle _nh;
        ControlMode _controlMode;

        mjModel *_model;                                                                            ///< Underlying model of the robot.
        mjData  *_jointState;                                                                   ///< Joint state data (position, velocity, acceleration)

        mjvCamera  _camera;                                                                         ///< Camera for viewing
        mjvOption  _renderingOptions;                                                               ///< As it says
        mjvPerturb _perturbation;                                                                   ///< Allows manual interaction
        mjvScene   _scene;                                                                          ///< The environment that the robot is rendered in

        mjrContext _context;                                                                        ///< No idea what this does.

        GLFWwindow *_window;                                                                        ///< This displays the robot and environment.

        ros::Publisher _jointStatePublisher;            ///< As it says on the label
        ros::Publisher _endeffectorPosePublisher;       ///< As it says on the label
        ros::Publisher _f_t_sensorPublisher;
        ros::Subscriber _jointCommandSubscriber;  ///< Subscriber for joint commands
        
        ros::Timer _simTimer, _visTimer;                                          ///< Regulates the ROS2 node

        sensor_msgs::JointState _jointStateMessage;                                            ///< For publishing joint state data over ROS2
        geometry_msgs::PoseStamped _endeffectorPoseMessage;                                     ///< For publishing end effector pose data over ROS2
        geometry_msgs::WrenchStamped _f_t_sensorMessage;
        
        int _simFrequency = 1000;                                                                   ///< Speed at which the frequency runs
        int _endeffector_bodyId;
        
        double _proportionalGain = 1.0;                                                             ///< Feedback on tracking error in position, velocity control mode
        double _derivativeGain   = 0.0;                                                             ///< Feedback on change in tracking error in position, velocity control mode
        double _integralGain     = 0.0;                                                             ///< Feedback on accumulated error in position, velocity control mode
        
        std::vector<double> _referencePosition;    
        std::vector<double> _referenceTorque;                                                      ///< For position, velocity control mode
        std::vector<double> _referenceTorqueSim; // for mujoco hw sim interface.
        std::vector<double> _error;                                                                 ///< Difference between reference and actual joint position
        std::vector<double> _errorDerivative;                                                       ///< Change in error
        std::vector<double> _errorIntegral;                                                         ///< Cumulative error
        
        bool _cameraOrthographic = false;                                                           ///< Sets the type of projection
        double _cameraAzimuth    = 135.0;                                                            ///< Rotation around camera focal point
        double _cameraDistance   = 2.5;                                                             ///< Distance from camera focal point
        double _cameraElevation  = -30;                                                             ///< Dictates height of camera
        std::vector<double> _cameraFocalPoint = {0.0, 0.0, 0.5};                                    ///< Where the camera is directed at

        std::thread vis_thread_;
        std::mutex mtx_;
        bool stop_vis_thread_;

        bool button_left = false;
        bool button_middle = false;
        bool button_right =  false;
        double lastx = 0;
        double lasty = 0;
        /**
         * Updates the robot state, publishes joint state information.
         */
        void
        update_simulation(const ros::TimerEvent&);
        
        /**
         * Updates the visualisation.
         */
        void
        update_visualization(const ros::TimerEvent&);
        
        /**
         * Callback function to handle incoming joint commands.
         * @param msg The message containing joint commands.
         */
        void 
        joint_command_callback(const std_msgs::Float64MultiArray::ConstPtr msg);

        /**
         * Callback function to compute gravity torques and updates the simulation.
         * @param _model The model of the robot.
         * @param _jointState The data of the robot.
         */
        static void controlCallback(const mjModel* _model, mjData *_jointState);


        static MujocoInterface* instance; // Static instance pointer

        /**
         * Scroll callback function for GLFW.
         * @param[in] _window The window from which the callback was triggered.
         * @param[in] xoffset The x-position of the cursor.
         * @param[in] yoffset The y-position of the cursor.
         */
        void scroll(GLFWwindow* _window, double xoffset, double yoffset);


        void keyboard(GLFWwindow* _window, int key, int scancode, int action, int mods);
        void mouse_button(GLFWwindow* _window, int button, int action, int mods);
        void mouse_move(GLFWwindow* _window, double xpos, double ypos);

        
};

#endif