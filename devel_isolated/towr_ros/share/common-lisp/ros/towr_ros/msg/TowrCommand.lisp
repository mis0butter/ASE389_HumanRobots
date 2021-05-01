; Auto-generated. Do not edit!


(cl:in-package towr_ros-msg)


;//! \htmlinclude TowrCommand.msg.html

(cl:defclass <TowrCommand> (roslisp-msg-protocol:ros-message)
  ((goal_lin
    :reader goal_lin
    :initarg :goal_lin
    :type xpp_msgs-msg:StateLin3d
    :initform (cl:make-instance 'xpp_msgs-msg:StateLin3d))
   (goal_ang
    :reader goal_ang
    :initarg :goal_ang
    :type xpp_msgs-msg:StateLin3d
    :initform (cl:make-instance 'xpp_msgs-msg:StateLin3d))
   (total_duration
    :reader total_duration
    :initarg :total_duration
    :type cl:float
    :initform 0.0)
   (replay_trajectory
    :reader replay_trajectory
    :initarg :replay_trajectory
    :type cl:boolean
    :initform cl:nil)
   (play_initialization
    :reader play_initialization
    :initarg :play_initialization
    :type cl:boolean
    :initform cl:nil)
   (plot_trajectory
    :reader plot_trajectory
    :initarg :plot_trajectory
    :type cl:boolean
    :initform cl:nil)
   (replay_speed
    :reader replay_speed
    :initarg :replay_speed
    :type cl:float
    :initform 0.0)
   (optimize
    :reader optimize
    :initarg :optimize
    :type cl:boolean
    :initform cl:nil)
   (robot
    :reader robot
    :initarg :robot
    :type cl:integer
    :initform 0)
   (terrain
    :reader terrain
    :initarg :terrain
    :type cl:integer
    :initform 0)
   (gait
    :reader gait
    :initarg :gait
    :type cl:integer
    :initform 0)
   (optimize_phase_durations
    :reader optimize_phase_durations
    :initarg :optimize_phase_durations
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass TowrCommand (<TowrCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TowrCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TowrCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name towr_ros-msg:<TowrCommand> is deprecated: use towr_ros-msg:TowrCommand instead.")))

(cl:ensure-generic-function 'goal_lin-val :lambda-list '(m))
(cl:defmethod goal_lin-val ((m <TowrCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader towr_ros-msg:goal_lin-val is deprecated.  Use towr_ros-msg:goal_lin instead.")
  (goal_lin m))

(cl:ensure-generic-function 'goal_ang-val :lambda-list '(m))
(cl:defmethod goal_ang-val ((m <TowrCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader towr_ros-msg:goal_ang-val is deprecated.  Use towr_ros-msg:goal_ang instead.")
  (goal_ang m))

(cl:ensure-generic-function 'total_duration-val :lambda-list '(m))
(cl:defmethod total_duration-val ((m <TowrCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader towr_ros-msg:total_duration-val is deprecated.  Use towr_ros-msg:total_duration instead.")
  (total_duration m))

(cl:ensure-generic-function 'replay_trajectory-val :lambda-list '(m))
(cl:defmethod replay_trajectory-val ((m <TowrCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader towr_ros-msg:replay_trajectory-val is deprecated.  Use towr_ros-msg:replay_trajectory instead.")
  (replay_trajectory m))

(cl:ensure-generic-function 'play_initialization-val :lambda-list '(m))
(cl:defmethod play_initialization-val ((m <TowrCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader towr_ros-msg:play_initialization-val is deprecated.  Use towr_ros-msg:play_initialization instead.")
  (play_initialization m))

(cl:ensure-generic-function 'plot_trajectory-val :lambda-list '(m))
(cl:defmethod plot_trajectory-val ((m <TowrCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader towr_ros-msg:plot_trajectory-val is deprecated.  Use towr_ros-msg:plot_trajectory instead.")
  (plot_trajectory m))

(cl:ensure-generic-function 'replay_speed-val :lambda-list '(m))
(cl:defmethod replay_speed-val ((m <TowrCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader towr_ros-msg:replay_speed-val is deprecated.  Use towr_ros-msg:replay_speed instead.")
  (replay_speed m))

(cl:ensure-generic-function 'optimize-val :lambda-list '(m))
(cl:defmethod optimize-val ((m <TowrCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader towr_ros-msg:optimize-val is deprecated.  Use towr_ros-msg:optimize instead.")
  (optimize m))

(cl:ensure-generic-function 'robot-val :lambda-list '(m))
(cl:defmethod robot-val ((m <TowrCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader towr_ros-msg:robot-val is deprecated.  Use towr_ros-msg:robot instead.")
  (robot m))

(cl:ensure-generic-function 'terrain-val :lambda-list '(m))
(cl:defmethod terrain-val ((m <TowrCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader towr_ros-msg:terrain-val is deprecated.  Use towr_ros-msg:terrain instead.")
  (terrain m))

(cl:ensure-generic-function 'gait-val :lambda-list '(m))
(cl:defmethod gait-val ((m <TowrCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader towr_ros-msg:gait-val is deprecated.  Use towr_ros-msg:gait instead.")
  (gait m))

(cl:ensure-generic-function 'optimize_phase_durations-val :lambda-list '(m))
(cl:defmethod optimize_phase_durations-val ((m <TowrCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader towr_ros-msg:optimize_phase_durations-val is deprecated.  Use towr_ros-msg:optimize_phase_durations instead.")
  (optimize_phase_durations m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TowrCommand>) ostream)
  "Serializes a message object of type '<TowrCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal_lin) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal_ang) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'total_duration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'replay_trajectory) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'play_initialization) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'plot_trajectory) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'replay_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'optimize) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'robot)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'terrain)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'gait)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'optimize_phase_durations) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TowrCommand>) istream)
  "Deserializes a message object of type '<TowrCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal_lin) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal_ang) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'total_duration) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'replay_trajectory) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'play_initialization) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'plot_trajectory) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'replay_speed) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'optimize) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'terrain) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gait) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'optimize_phase_durations) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TowrCommand>)))
  "Returns string type for a message object of type '<TowrCommand>"
  "towr_ros/TowrCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TowrCommand)))
  "Returns string type for a message object of type 'TowrCommand"
  "towr_ros/TowrCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TowrCommand>)))
  "Returns md5sum for a message object of type '<TowrCommand>"
  "9687aa81cfa0759f837eb5ec3b2f880e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TowrCommand)))
  "Returns md5sum for a message object of type 'TowrCommand"
  "9687aa81cfa0759f837eb5ec3b2f880e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TowrCommand>)))
  "Returns full string definition for message of type '<TowrCommand>"
  (cl:format cl:nil "# The command for the robot specified by the user~%~%xpp_msgs/StateLin3d    goal_lin                  # the linear state to reach (x,y,z and derivatives)~%xpp_msgs/StateLin3d    goal_ang                  # the angular state to reach (roll, pitch, yaw and derivatives)~%float64                total_duration            # the total time to reach the goal state~%bool                   replay_trajectory         # Replay the already optimized trajectory in RVIZ~%bool                   play_initialization       # Play motion generated by unoptimized initial variables~%bool                   plot_trajectory           # Plot the optimized trajectory~%float64                replay_speed              # speed at which to playback the motion.~%bool                   optimize                  # run TOWR optimization~%int32                  robot                     # Monoped, Biped, Quadruped, Anymal~%int32                  terrain                   # some information about the used terrain (e.g stairs, gap, slope)~%int32                  gait                      # Type of Motion (Walk, Trott, Bound, Pace)~%bool                   optimize_phase_durations  # If true, the gait is optimized over as well~%~%================================================================================~%MSG: xpp_msgs/StateLin3d~%# This contains the 3D representation of a linear state, including:~%# position, velocity, acceleration~%~%geometry_msgs/Point pos~%geometry_msgs/Vector3 vel~%geometry_msgs/Vector3 acc~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TowrCommand)))
  "Returns full string definition for message of type 'TowrCommand"
  (cl:format cl:nil "# The command for the robot specified by the user~%~%xpp_msgs/StateLin3d    goal_lin                  # the linear state to reach (x,y,z and derivatives)~%xpp_msgs/StateLin3d    goal_ang                  # the angular state to reach (roll, pitch, yaw and derivatives)~%float64                total_duration            # the total time to reach the goal state~%bool                   replay_trajectory         # Replay the already optimized trajectory in RVIZ~%bool                   play_initialization       # Play motion generated by unoptimized initial variables~%bool                   plot_trajectory           # Plot the optimized trajectory~%float64                replay_speed              # speed at which to playback the motion.~%bool                   optimize                  # run TOWR optimization~%int32                  robot                     # Monoped, Biped, Quadruped, Anymal~%int32                  terrain                   # some information about the used terrain (e.g stairs, gap, slope)~%int32                  gait                      # Type of Motion (Walk, Trott, Bound, Pace)~%bool                   optimize_phase_durations  # If true, the gait is optimized over as well~%~%================================================================================~%MSG: xpp_msgs/StateLin3d~%# This contains the 3D representation of a linear state, including:~%# position, velocity, acceleration~%~%geometry_msgs/Point pos~%geometry_msgs/Vector3 vel~%geometry_msgs/Vector3 acc~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TowrCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal_lin))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal_ang))
     8
     1
     1
     1
     8
     1
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TowrCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'TowrCommand
    (cl:cons ':goal_lin (goal_lin msg))
    (cl:cons ':goal_ang (goal_ang msg))
    (cl:cons ':total_duration (total_duration msg))
    (cl:cons ':replay_trajectory (replay_trajectory msg))
    (cl:cons ':play_initialization (play_initialization msg))
    (cl:cons ':plot_trajectory (plot_trajectory msg))
    (cl:cons ':replay_speed (replay_speed msg))
    (cl:cons ':optimize (optimize msg))
    (cl:cons ':robot (robot msg))
    (cl:cons ':terrain (terrain msg))
    (cl:cons ':gait (gait msg))
    (cl:cons ':optimize_phase_durations (optimize_phase_durations msg))
))
