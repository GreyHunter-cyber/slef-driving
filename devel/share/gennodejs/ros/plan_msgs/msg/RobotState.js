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

class RobotState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mition_arrived = null;
      this.mition_arrive_num = null;
      this.Speed = null;
      this.Azimuth = null;
      this.Stop = null;
      this.loc_fix = null;
    }
    else {
      if (initObj.hasOwnProperty('mition_arrived')) {
        this.mition_arrived = initObj.mition_arrived
      }
      else {
        this.mition_arrived = false;
      }
      if (initObj.hasOwnProperty('mition_arrive_num')) {
        this.mition_arrive_num = initObj.mition_arrive_num
      }
      else {
        this.mition_arrive_num = 0;
      }
      if (initObj.hasOwnProperty('Speed')) {
        this.Speed = initObj.Speed
      }
      else {
        this.Speed = 0.0;
      }
      if (initObj.hasOwnProperty('Azimuth')) {
        this.Azimuth = initObj.Azimuth
      }
      else {
        this.Azimuth = 0.0;
      }
      if (initObj.hasOwnProperty('Stop')) {
        this.Stop = initObj.Stop
      }
      else {
        this.Stop = false;
      }
      if (initObj.hasOwnProperty('loc_fix')) {
        this.loc_fix = initObj.loc_fix
      }
      else {
        this.loc_fix = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotState
    // Serialize message field [mition_arrived]
    bufferOffset = _serializer.bool(obj.mition_arrived, buffer, bufferOffset);
    // Serialize message field [mition_arrive_num]
    bufferOffset = _serializer.int32(obj.mition_arrive_num, buffer, bufferOffset);
    // Serialize message field [Speed]
    bufferOffset = _serializer.float32(obj.Speed, buffer, bufferOffset);
    // Serialize message field [Azimuth]
    bufferOffset = _serializer.float32(obj.Azimuth, buffer, bufferOffset);
    // Serialize message field [Stop]
    bufferOffset = _serializer.bool(obj.Stop, buffer, bufferOffset);
    // Serialize message field [loc_fix]
    bufferOffset = _serializer.int32(obj.loc_fix, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotState
    let len;
    let data = new RobotState(null);
    // Deserialize message field [mition_arrived]
    data.mition_arrived = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [mition_arrive_num]
    data.mition_arrive_num = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [Speed]
    data.Speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Azimuth]
    data.Azimuth = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Stop]
    data.Stop = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [loc_fix]
    data.loc_fix = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 18;
  }

  static datatype() {
    // Returns string type for a message object
    return 'plan_msgs/RobotState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ab96e98e2f0ba56ad199d5f5d6baa6be';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #topic: /RobotState
    bool mition_arrived # arrived
    int32 mition_arrive_num # arrived whitch misstion point
    float32 Speed
    float32 Azimuth
    bool  Stop     # pause or stop flag enable
    int32 loc_fix  # Positioning quality 0: miss; 1: fixed; 2: float;
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotState(null);
    if (msg.mition_arrived !== undefined) {
      resolved.mition_arrived = msg.mition_arrived;
    }
    else {
      resolved.mition_arrived = false
    }

    if (msg.mition_arrive_num !== undefined) {
      resolved.mition_arrive_num = msg.mition_arrive_num;
    }
    else {
      resolved.mition_arrive_num = 0
    }

    if (msg.Speed !== undefined) {
      resolved.Speed = msg.Speed;
    }
    else {
      resolved.Speed = 0.0
    }

    if (msg.Azimuth !== undefined) {
      resolved.Azimuth = msg.Azimuth;
    }
    else {
      resolved.Azimuth = 0.0
    }

    if (msg.Stop !== undefined) {
      resolved.Stop = msg.Stop;
    }
    else {
      resolved.Stop = false
    }

    if (msg.loc_fix !== undefined) {
      resolved.loc_fix = msg.loc_fix;
    }
    else {
      resolved.loc_fix = 0
    }

    return resolved;
    }
};

module.exports = RobotState;
