// Auto-generated. Do not edit!

// (in-package plan_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class HmiControl {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stamp = null;
      this.e_stop = null;
      this.speed = null;
      this.ang_velo = null;
      this.distance = null;
      this.angle = null;
      this.control_flag = null;
      this.action_flag = null;
      this.mition_num = null;
      this.mition_point_x = null;
      this.mition_point_y = null;
      this.mition_point_speed = null;
      this.mition_point_a = null;
      this.origin_x = null;
      this.origin_y = null;
      this.origin_z = null;
      this.origin_yaw = null;
      this.mition_finish = null;
    }
    else {
      if (initObj.hasOwnProperty('stamp')) {
        this.stamp = initObj.stamp
      }
      else {
        this.stamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('e_stop')) {
        this.e_stop = initObj.e_stop
      }
      else {
        this.e_stop = 0;
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0.0;
      }
      if (initObj.hasOwnProperty('ang_velo')) {
        this.ang_velo = initObj.ang_velo
      }
      else {
        this.ang_velo = 0.0;
      }
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = 0.0;
      }
      if (initObj.hasOwnProperty('angle')) {
        this.angle = initObj.angle
      }
      else {
        this.angle = 0.0;
      }
      if (initObj.hasOwnProperty('control_flag')) {
        this.control_flag = initObj.control_flag
      }
      else {
        this.control_flag = 0;
      }
      if (initObj.hasOwnProperty('action_flag')) {
        this.action_flag = initObj.action_flag
      }
      else {
        this.action_flag = 0;
      }
      if (initObj.hasOwnProperty('mition_num')) {
        this.mition_num = initObj.mition_num
      }
      else {
        this.mition_num = 0;
      }
      if (initObj.hasOwnProperty('mition_point_x')) {
        this.mition_point_x = initObj.mition_point_x
      }
      else {
        this.mition_point_x = [];
      }
      if (initObj.hasOwnProperty('mition_point_y')) {
        this.mition_point_y = initObj.mition_point_y
      }
      else {
        this.mition_point_y = [];
      }
      if (initObj.hasOwnProperty('mition_point_speed')) {
        this.mition_point_speed = initObj.mition_point_speed
      }
      else {
        this.mition_point_speed = [];
      }
      if (initObj.hasOwnProperty('mition_point_a')) {
        this.mition_point_a = initObj.mition_point_a
      }
      else {
        this.mition_point_a = [];
      }
      if (initObj.hasOwnProperty('origin_x')) {
        this.origin_x = initObj.origin_x
      }
      else {
        this.origin_x = 0.0;
      }
      if (initObj.hasOwnProperty('origin_y')) {
        this.origin_y = initObj.origin_y
      }
      else {
        this.origin_y = 0.0;
      }
      if (initObj.hasOwnProperty('origin_z')) {
        this.origin_z = initObj.origin_z
      }
      else {
        this.origin_z = 0.0;
      }
      if (initObj.hasOwnProperty('origin_yaw')) {
        this.origin_yaw = initObj.origin_yaw
      }
      else {
        this.origin_yaw = 0.0;
      }
      if (initObj.hasOwnProperty('mition_finish')) {
        this.mition_finish = initObj.mition_finish
      }
      else {
        this.mition_finish = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HmiControl
    // Serialize message field [stamp]
    bufferOffset = _serializer.time(obj.stamp, buffer, bufferOffset);
    // Serialize message field [e_stop]
    bufferOffset = _serializer.int32(obj.e_stop, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.float32(obj.speed, buffer, bufferOffset);
    // Serialize message field [ang_velo]
    bufferOffset = _serializer.float32(obj.ang_velo, buffer, bufferOffset);
    // Serialize message field [distance]
    bufferOffset = _serializer.float32(obj.distance, buffer, bufferOffset);
    // Serialize message field [angle]
    bufferOffset = _serializer.float32(obj.angle, buffer, bufferOffset);
    // Serialize message field [control_flag]
    bufferOffset = _serializer.int32(obj.control_flag, buffer, bufferOffset);
    // Serialize message field [action_flag]
    bufferOffset = _serializer.int32(obj.action_flag, buffer, bufferOffset);
    // Serialize message field [mition_num]
    bufferOffset = _serializer.int32(obj.mition_num, buffer, bufferOffset);
    // Serialize message field [mition_point_x]
    bufferOffset = _arraySerializer.float32(obj.mition_point_x, buffer, bufferOffset, null);
    // Serialize message field [mition_point_y]
    bufferOffset = _arraySerializer.float32(obj.mition_point_y, buffer, bufferOffset, null);
    // Serialize message field [mition_point_speed]
    bufferOffset = _arraySerializer.float32(obj.mition_point_speed, buffer, bufferOffset, null);
    // Serialize message field [mition_point_a]
    bufferOffset = _arraySerializer.int32(obj.mition_point_a, buffer, bufferOffset, null);
    // Serialize message field [origin_x]
    bufferOffset = _serializer.float32(obj.origin_x, buffer, bufferOffset);
    // Serialize message field [origin_y]
    bufferOffset = _serializer.float32(obj.origin_y, buffer, bufferOffset);
    // Serialize message field [origin_z]
    bufferOffset = _serializer.float32(obj.origin_z, buffer, bufferOffset);
    // Serialize message field [origin_yaw]
    bufferOffset = _serializer.float32(obj.origin_yaw, buffer, bufferOffset);
    // Serialize message field [mition_finish]
    bufferOffset = _serializer.bool(obj.mition_finish, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HmiControl
    let len;
    let data = new HmiControl(null);
    // Deserialize message field [stamp]
    data.stamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [e_stop]
    data.e_stop = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ang_velo]
    data.ang_velo = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [distance]
    data.distance = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angle]
    data.angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [control_flag]
    data.control_flag = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [action_flag]
    data.action_flag = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [mition_num]
    data.mition_num = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [mition_point_x]
    data.mition_point_x = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [mition_point_y]
    data.mition_point_y = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [mition_point_speed]
    data.mition_point_speed = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [mition_point_a]
    data.mition_point_a = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [origin_x]
    data.origin_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [origin_y]
    data.origin_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [origin_z]
    data.origin_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [origin_yaw]
    data.origin_yaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [mition_finish]
    data.mition_finish = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.mition_point_x.length;
    length += 4 * object.mition_point_y.length;
    length += 4 * object.mition_point_speed.length;
    length += 4 * object.mition_point_a.length;
    return length + 73;
  }

  static datatype() {
    // Returns string type for a message object
    return 'plan_msgs/HmiControl';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dd2b29065bf0fb426c0ae55225b9903d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time    stamp             # timestamp
    int32   e_stop            # emergency stop flag
    float32 speed             # speed in m/s
    float32 ang_velo          # turn angle velocity deg/s
    float32 distance          # run distance in m
    float32 angle             # turn angle in degree(L:+/R:-)
    int32   control_flag      # is start the remote control 0 means free ，1 means pause ,2 means stop ,3 means remote
    int32   action_flag       # 0 means none ,1 means apply mitionPoint ,2 means navigation,3 reset init location
    int32   mition_num        # mitionPoint number
    float32[] mition_point_x  # mitionPoint x
    float32[] mition_point_y  # mitionPoint y
    float32[] mition_point_speed  # mitionPoint speed
    int32[]   mition_point_a  # mitionPoint attribute
    float32 origin_x
    float32 origin_y
    float32 origin_z
    float32 origin_yaw
    bool mition_finish
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HmiControl(null);
    if (msg.stamp !== undefined) {
      resolved.stamp = msg.stamp;
    }
    else {
      resolved.stamp = {secs: 0, nsecs: 0}
    }

    if (msg.e_stop !== undefined) {
      resolved.e_stop = msg.e_stop;
    }
    else {
      resolved.e_stop = 0
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0.0
    }

    if (msg.ang_velo !== undefined) {
      resolved.ang_velo = msg.ang_velo;
    }
    else {
      resolved.ang_velo = 0.0
    }

    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = 0.0
    }

    if (msg.angle !== undefined) {
      resolved.angle = msg.angle;
    }
    else {
      resolved.angle = 0.0
    }

    if (msg.control_flag !== undefined) {
      resolved.control_flag = msg.control_flag;
    }
    else {
      resolved.control_flag = 0
    }

    if (msg.action_flag !== undefined) {
      resolved.action_flag = msg.action_flag;
    }
    else {
      resolved.action_flag = 0
    }

    if (msg.mition_num !== undefined) {
      resolved.mition_num = msg.mition_num;
    }
    else {
      resolved.mition_num = 0
    }

    if (msg.mition_point_x !== undefined) {
      resolved.mition_point_x = msg.mition_point_x;
    }
    else {
      resolved.mition_point_x = []
    }

    if (msg.mition_point_y !== undefined) {
      resolved.mition_point_y = msg.mition_point_y;
    }
    else {
      resolved.mition_point_y = []
    }

    if (msg.mition_point_speed !== undefined) {
      resolved.mition_point_speed = msg.mition_point_speed;
    }
    else {
      resolved.mition_point_speed = []
    }

    if (msg.mition_point_a !== undefined) {
      resolved.mition_point_a = msg.mition_point_a;
    }
    else {
      resolved.mition_point_a = []
    }

    if (msg.origin_x !== undefined) {
      resolved.origin_x = msg.origin_x;
    }
    else {
      resolved.origin_x = 0.0
    }

    if (msg.origin_y !== undefined) {
      resolved.origin_y = msg.origin_y;
    }
    else {
      resolved.origin_y = 0.0
    }

    if (msg.origin_z !== undefined) {
      resolved.origin_z = msg.origin_z;
    }
    else {
      resolved.origin_z = 0.0
    }

    if (msg.origin_yaw !== undefined) {
      resolved.origin_yaw = msg.origin_yaw;
    }
    else {
      resolved.origin_yaw = 0.0
    }

    if (msg.mition_finish !== undefined) {
      resolved.mition_finish = msg.mition_finish;
    }
    else {
      resolved.mition_finish = false
    }

    return resolved;
    }
};

module.exports = HmiControl;
