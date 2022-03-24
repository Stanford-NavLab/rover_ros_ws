// Auto-generated. Do not edit!

// (in-package planner.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let State = require('./State.js');
let Control = require('./Control.js');

//-----------------------------------------------------------

class NominalTrajectory {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.states = null;
      this.controls = null;
    }
    else {
      if (initObj.hasOwnProperty('states')) {
        this.states = initObj.states
      }
      else {
        this.states = [];
      }
      if (initObj.hasOwnProperty('controls')) {
        this.controls = initObj.controls
      }
      else {
        this.controls = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NominalTrajectory
    // Serialize message field [states]
    // Serialize the length for message field [states]
    bufferOffset = _serializer.uint32(obj.states.length, buffer, bufferOffset);
    obj.states.forEach((val) => {
      bufferOffset = State.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [controls]
    // Serialize the length for message field [controls]
    bufferOffset = _serializer.uint32(obj.controls.length, buffer, bufferOffset);
    obj.controls.forEach((val) => {
      bufferOffset = Control.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NominalTrajectory
    let len;
    let data = new NominalTrajectory(null);
    // Deserialize message field [states]
    // Deserialize array length for message field [states]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.states = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.states[i] = State.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [controls]
    // Deserialize array length for message field [controls]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.controls = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.controls[i] = Control.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 32 * object.states.length;
    length += 16 * object.controls.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'planner/NominalTrajectory';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8aa3d7b09e7dbafc476534ce12baadb2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    State[] states
    Control[] controls
    ================================================================================
    MSG: planner/State
    float64 x
    float64 y
    float64 theta
    float64 v
    ================================================================================
    MSG: planner/Control
    float64 omega
    float64 a
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NominalTrajectory(null);
    if (msg.states !== undefined) {
      resolved.states = new Array(msg.states.length);
      for (let i = 0; i < resolved.states.length; ++i) {
        resolved.states[i] = State.Resolve(msg.states[i]);
      }
    }
    else {
      resolved.states = []
    }

    if (msg.controls !== undefined) {
      resolved.controls = new Array(msg.controls.length);
      for (let i = 0; i < resolved.controls.length; ++i) {
        resolved.controls[i] = Control.Resolve(msg.controls[i]);
      }
    }
    else {
      resolved.controls = []
    }

    return resolved;
    }
};

module.exports = NominalTrajectory;
