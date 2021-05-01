;; Auto-generated. Do not edit!


(when (boundp 'towr_ros::TowrCommand)
  (if (not (find-package "TOWR_ROS"))
    (make-package "TOWR_ROS"))
  (shadow 'TowrCommand (find-package "TOWR_ROS")))
(unless (find-package "TOWR_ROS::TOWRCOMMAND")
  (make-package "TOWR_ROS::TOWRCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude TowrCommand.msg.html
(if (not (find-package "XPP_MSGS"))
  (ros::roseus-add-msgs "xpp_msgs"))


(defclass towr_ros::TowrCommand
  :super ros::object
  :slots (_goal_lin _goal_ang _total_duration _replay_trajectory _play_initialization _plot_trajectory _replay_speed _optimize _robot _terrain _gait _optimize_phase_durations ))

(defmethod towr_ros::TowrCommand
  (:init
   (&key
    ((:goal_lin __goal_lin) (instance xpp_msgs::StateLin3d :init))
    ((:goal_ang __goal_ang) (instance xpp_msgs::StateLin3d :init))
    ((:total_duration __total_duration) 0.0)
    ((:replay_trajectory __replay_trajectory) nil)
    ((:play_initialization __play_initialization) nil)
    ((:plot_trajectory __plot_trajectory) nil)
    ((:replay_speed __replay_speed) 0.0)
    ((:optimize __optimize) nil)
    ((:robot __robot) 0)
    ((:terrain __terrain) 0)
    ((:gait __gait) 0)
    ((:optimize_phase_durations __optimize_phase_durations) nil)
    )
   (send-super :init)
   (setq _goal_lin __goal_lin)
   (setq _goal_ang __goal_ang)
   (setq _total_duration (float __total_duration))
   (setq _replay_trajectory __replay_trajectory)
   (setq _play_initialization __play_initialization)
   (setq _plot_trajectory __plot_trajectory)
   (setq _replay_speed (float __replay_speed))
   (setq _optimize __optimize)
   (setq _robot (round __robot))
   (setq _terrain (round __terrain))
   (setq _gait (round __gait))
   (setq _optimize_phase_durations __optimize_phase_durations)
   self)
  (:goal_lin
   (&rest __goal_lin)
   (if (keywordp (car __goal_lin))
       (send* _goal_lin __goal_lin)
     (progn
       (if __goal_lin (setq _goal_lin (car __goal_lin)))
       _goal_lin)))
  (:goal_ang
   (&rest __goal_ang)
   (if (keywordp (car __goal_ang))
       (send* _goal_ang __goal_ang)
     (progn
       (if __goal_ang (setq _goal_ang (car __goal_ang)))
       _goal_ang)))
  (:total_duration
   (&optional __total_duration)
   (if __total_duration (setq _total_duration __total_duration)) _total_duration)
  (:replay_trajectory
   (&optional __replay_trajectory)
   (if __replay_trajectory (setq _replay_trajectory __replay_trajectory)) _replay_trajectory)
  (:play_initialization
   (&optional __play_initialization)
   (if __play_initialization (setq _play_initialization __play_initialization)) _play_initialization)
  (:plot_trajectory
   (&optional __plot_trajectory)
   (if __plot_trajectory (setq _plot_trajectory __plot_trajectory)) _plot_trajectory)
  (:replay_speed
   (&optional __replay_speed)
   (if __replay_speed (setq _replay_speed __replay_speed)) _replay_speed)
  (:optimize
   (&optional __optimize)
   (if __optimize (setq _optimize __optimize)) _optimize)
  (:robot
   (&optional __robot)
   (if __robot (setq _robot __robot)) _robot)
  (:terrain
   (&optional __terrain)
   (if __terrain (setq _terrain __terrain)) _terrain)
  (:gait
   (&optional __gait)
   (if __gait (setq _gait __gait)) _gait)
  (:optimize_phase_durations
   (&optional __optimize_phase_durations)
   (if __optimize_phase_durations (setq _optimize_phase_durations __optimize_phase_durations)) _optimize_phase_durations)
  (:serialization-length
   ()
   (+
    ;; xpp_msgs/StateLin3d _goal_lin
    (send _goal_lin :serialization-length)
    ;; xpp_msgs/StateLin3d _goal_ang
    (send _goal_ang :serialization-length)
    ;; float64 _total_duration
    8
    ;; bool _replay_trajectory
    1
    ;; bool _play_initialization
    1
    ;; bool _plot_trajectory
    1
    ;; float64 _replay_speed
    8
    ;; bool _optimize
    1
    ;; int32 _robot
    4
    ;; int32 _terrain
    4
    ;; int32 _gait
    4
    ;; bool _optimize_phase_durations
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; xpp_msgs/StateLin3d _goal_lin
       (send _goal_lin :serialize s)
     ;; xpp_msgs/StateLin3d _goal_ang
       (send _goal_ang :serialize s)
     ;; float64 _total_duration
       (sys::poke _total_duration (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _replay_trajectory
       (if _replay_trajectory (write-byte -1 s) (write-byte 0 s))
     ;; bool _play_initialization
       (if _play_initialization (write-byte -1 s) (write-byte 0 s))
     ;; bool _plot_trajectory
       (if _plot_trajectory (write-byte -1 s) (write-byte 0 s))
     ;; float64 _replay_speed
       (sys::poke _replay_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _optimize
       (if _optimize (write-byte -1 s) (write-byte 0 s))
     ;; int32 _robot
       (write-long _robot s)
     ;; int32 _terrain
       (write-long _terrain s)
     ;; int32 _gait
       (write-long _gait s)
     ;; bool _optimize_phase_durations
       (if _optimize_phase_durations (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; xpp_msgs/StateLin3d _goal_lin
     (send _goal_lin :deserialize buf ptr-) (incf ptr- (send _goal_lin :serialization-length))
   ;; xpp_msgs/StateLin3d _goal_ang
     (send _goal_ang :deserialize buf ptr-) (incf ptr- (send _goal_ang :serialization-length))
   ;; float64 _total_duration
     (setq _total_duration (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _replay_trajectory
     (setq _replay_trajectory (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _play_initialization
     (setq _play_initialization (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _plot_trajectory
     (setq _plot_trajectory (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _replay_speed
     (setq _replay_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _optimize
     (setq _optimize (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _robot
     (setq _robot (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _terrain
     (setq _terrain (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _gait
     (setq _gait (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _optimize_phase_durations
     (setq _optimize_phase_durations (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get towr_ros::TowrCommand :md5sum-) "9687aa81cfa0759f837eb5ec3b2f880e")
(setf (get towr_ros::TowrCommand :datatype-) "towr_ros/TowrCommand")
(setf (get towr_ros::TowrCommand :definition-)
      "# The command for the robot specified by the user

xpp_msgs/StateLin3d    goal_lin                  # the linear state to reach (x,y,z and derivatives)
xpp_msgs/StateLin3d    goal_ang                  # the angular state to reach (roll, pitch, yaw and derivatives)
float64                total_duration            # the total time to reach the goal state
bool                   replay_trajectory         # Replay the already optimized trajectory in RVIZ
bool                   play_initialization       # Play motion generated by unoptimized initial variables
bool                   plot_trajectory           # Plot the optimized trajectory
float64                replay_speed              # speed at which to playback the motion.
bool                   optimize                  # run TOWR optimization
int32                  robot                     # Monoped, Biped, Quadruped, Anymal
int32                  terrain                   # some information about the used terrain (e.g stairs, gap, slope)
int32                  gait                      # Type of Motion (Walk, Trott, Bound, Pace)
bool                   optimize_phase_durations  # If true, the gait is optimized over as well

================================================================================
MSG: xpp_msgs/StateLin3d
# This contains the 3D representation of a linear state, including:
# position, velocity, acceleration

geometry_msgs/Point pos
geometry_msgs/Vector3 vel
geometry_msgs/Vector3 acc
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :towr_ros/TowrCommand "9687aa81cfa0759f837eb5ec3b2f880e")


