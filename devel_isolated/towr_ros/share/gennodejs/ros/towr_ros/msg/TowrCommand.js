// Auto-generated. Do not edit!

// (in-package towr_ros.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let xpp_msgs = _finder('xpp_msgs');

//-----------------------------------------------------------

class TowrCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.goal_lin = null;
      this.goal_ang = null;
      this.total_duration = null;
      this.replay_trajectory = null;
      this.play_initialization = null;
      this.plot_trajectory = null;
      this.replay_speed = null;
      this.optimize = null;
      this.robot = null;
      this.terrain = null;
      this.gait = null;
      this.optimize_phase_durations = null;
    }
    else {
      if (initObj.hasOwnProperty('goal_lin')) {
        this.goal_lin = initObj.goal_lin
      }
      else {
        this.goal_lin = new xpp_msgs.msg.StateLin3d();
      }
      if (initObj.hasOwnProperty('goal_ang')) {
        this.goal_ang = initObj.goal_ang
      }
      else {
        this.goal_ang = new xpp_msgs.msg.StateLin3d();
      }
      if (initObj.hasOwnProperty('total_duration')) {
        this.total_duration = initObj.total_duration
      }
      else {
        this.total_duration = 0.0;
      }
      if (initObj.hasOwnProperty('replay_trajectory')) {
        this.replay_trajectory = initObj.replay_trajectory
      }
      else {
        this.replay_trajectory = false;
      }
      if (initObj.hasOwnProperty('play_initialization')) {
        this.play_initialization = initObj.play_initialization
      }
      else {
        this.play_initialization = false;
      }
      if (initObj.hasOwnProperty('plot_trajectory')) {
        this.plot_trajectory = initObj.plot_trajectory
      }
      else {
        this.plot_trajectory = false;
      }
      if (initObj.hasOwnProperty('replay_speed')) {
        this.replay_speed = initObj.replay_speed
      }
      else {
        this.replay_speed = 0.0;
      }
      if (initObj.hasOwnProperty('optimize')) {
        this.optimize = initObj.optimize
      }
      else {
        this.optimize = false;
      }
      if (initObj.hasOwnProperty('robot')) {
        this.robot = initObj.robot
      }
      else {
        this.robot = 0;
      }
      if (initObj.hasOwnProperty('terrain')) {
        this.terrain = initObj.terrain
      }
      else {
        this.terrain = 0;
      }
      if (initObj.hasOwnProperty('gait')) {
        this.gait = initObj.gait
      }
      else {
        this.gait = 0;
      }
      if (initObj.hasOwnProperty('optimize_phase_durations')) {
        this.optimize_phase_durations = initObj.optimize_phase_durations
      }
      else {
        this.optimize_phase_durations = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TowrCommand
    // Serialize message field [goal_lin]
    bufferOffset = xpp_msgs.msg.StateLin3d.serialize(obj.goal_lin, buffer, bufferOffset);
    // Serialize message field [goal_ang]
    bufferOffset = xpp_msgs.msg.StateLin3d.serialize(obj.goal_ang, buffer, bufferOffset);
    // Serialize message field [total_duration]
    bufferOffset = _serializer.float64(obj.total_duration, buffer, bufferOffset);
    // Serialize message field [replay_trajectory]
    bufferOffset = _serializer.bool(obj.replay_trajectory, buffer, bufferOffset);
    // Serialize message field [play_initialization]
    bufferOffset = _serializer.bool(obj.play_initialization, buffer, bufferOffset);
    // Serialize message field [plot_trajectory]
    bufferOffset = _serializer.bool(obj.plot_trajectory, buffer, bufferOffset);
    // Serialize message field [replay_speed]
    bufferOffset = _serializer.float64(obj.replay_speed, buffer, bufferOffset);
    // Serialize message field [optimize]
    bufferOffset = _serializer.bool(obj.optimize, buffer, bufferOffset);
    // Serialize message field [robot]
    bufferOffset = _serializer.int32(obj.robot, buffer, bufferOffset);
    // Serialize message field [terrain]
    bufferOffset = _serializer.int32(obj.terrain, buffer, bufferOffset);
    // Serialize message field [gait]
    bufferOffset = _serializer.int32(obj.gait, buffer, bufferOffset);
    // Serialize message field [optimize_phase_durations]
    bufferOffset = _serializer.bool(obj.optimize_phase_durations, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TowrCommand
    let len;
    let data = new TowrCommand(null);
    // Deserialize message field [goal_lin]
    data.goal_lin = xpp_msgs.msg.StateLin3d.deserialize(buffer, bufferOffset);
    // Deserialize message field [goal_ang]
    data.goal_ang = xpp_msgs.msg.StateLin3d.deserialize(buffer, bufferOffset);
    // Deserialize message field [total_duration]
    data.total_duration = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [replay_trajectory]
    data.replay_trajectory = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [play_initialization]
    data.play_initialization = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [plot_trajectory]
    data.plot_trajectory = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [replay_speed]
    data.replay_speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [optimize]
    data.optimize = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [robot]
    data.robot = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [terrain]
    data.terrain = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [gait]
    data.gait = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [optimize_phase_durations]
    data.optimize_phase_durations = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 177;
  }

  static datatype() {
    // Returns string type for a message object
    return 'towr_ros/TowrCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9687aa81cfa0759f837eb5ec3b2f880e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # The command for the robot specified by the user
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TowrCommand(null);
    if (msg.goal_lin !== undefined) {
      resolved.goal_lin = xpp_msgs.msg.StateLin3d.Resolve(msg.goal_lin)
    }
    else {
      resolved.goal_lin = new xpp_msgs.msg.StateLin3d()
    }

    if (msg.goal_ang !== undefined) {
      resolved.goal_ang = xpp_msgs.msg.StateLin3d.Resolve(msg.goal_ang)
    }
    else {
      resolved.goal_ang = new xpp_msgs.msg.StateLin3d()
    }

    if (msg.total_duration !== undefined) {
      resolved.total_duration = msg.total_duration;
    }
    else {
      resolved.total_duration = 0.0
    }

    if (msg.replay_trajectory !== undefined) {
      resolved.replay_trajectory = msg.replay_trajectory;
    }
    else {
      resolved.replay_trajectory = false
    }

    if (msg.play_initialization !== undefined) {
      resolved.play_initialization = msg.play_initialization;
    }
    else {
      resolved.play_initialization = false
    }

    if (msg.plot_trajectory !== undefined) {
      resolved.plot_trajectory = msg.plot_trajectory;
    }
    else {
      resolved.plot_trajectory = false
    }

    if (msg.replay_speed !== undefined) {
      resolved.replay_speed = msg.replay_speed;
    }
    else {
      resolved.replay_speed = 0.0
    }

    if (msg.optimize !== undefined) {
      resolved.optimize = msg.optimize;
    }
    else {
      resolved.optimize = false
    }

    if (msg.robot !== undefined) {
      resolved.robot = msg.robot;
    }
    else {
      resolved.robot = 0
    }

    if (msg.terrain !== undefined) {
      resolved.terrain = msg.terrain;
    }
    else {
      resolved.terrain = 0
    }

    if (msg.gait !== undefined) {
      resolved.gait = msg.gait;
    }
    else {
      resolved.gait = 0
    }

    if (msg.optimize_phase_durations !== undefined) {
      resolved.optimize_phase_durations = msg.optimize_phase_durations;
    }
    else {
      resolved.optimize_phase_durations = false
    }

    return resolved;
    }
};

module.exports = TowrCommand;
