# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from plan_msgs/HmiControl.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy

class HmiControl(genpy.Message):
  _md5sum = "dd2b29065bf0fb426c0ae55225b9903d"
  _type = "plan_msgs/HmiControl"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """time    stamp             # timestamp
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
bool mition_finish"""
  __slots__ = ['stamp','e_stop','speed','ang_velo','distance','angle','control_flag','action_flag','mition_num','mition_point_x','mition_point_y','mition_point_speed','mition_point_a','origin_x','origin_y','origin_z','origin_yaw','mition_finish']
  _slot_types = ['time','int32','float32','float32','float32','float32','int32','int32','int32','float32[]','float32[]','float32[]','int32[]','float32','float32','float32','float32','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       stamp,e_stop,speed,ang_velo,distance,angle,control_flag,action_flag,mition_num,mition_point_x,mition_point_y,mition_point_speed,mition_point_a,origin_x,origin_y,origin_z,origin_yaw,mition_finish

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(HmiControl, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.stamp is None:
        self.stamp = genpy.Time()
      if self.e_stop is None:
        self.e_stop = 0
      if self.speed is None:
        self.speed = 0.
      if self.ang_velo is None:
        self.ang_velo = 0.
      if self.distance is None:
        self.distance = 0.
      if self.angle is None:
        self.angle = 0.
      if self.control_flag is None:
        self.control_flag = 0
      if self.action_flag is None:
        self.action_flag = 0
      if self.mition_num is None:
        self.mition_num = 0
      if self.mition_point_x is None:
        self.mition_point_x = []
      if self.mition_point_y is None:
        self.mition_point_y = []
      if self.mition_point_speed is None:
        self.mition_point_speed = []
      if self.mition_point_a is None:
        self.mition_point_a = []
      if self.origin_x is None:
        self.origin_x = 0.
      if self.origin_y is None:
        self.origin_y = 0.
      if self.origin_z is None:
        self.origin_z = 0.
      if self.origin_yaw is None:
        self.origin_yaw = 0.
      if self.mition_finish is None:
        self.mition_finish = False
    else:
      self.stamp = genpy.Time()
      self.e_stop = 0
      self.speed = 0.
      self.ang_velo = 0.
      self.distance = 0.
      self.angle = 0.
      self.control_flag = 0
      self.action_flag = 0
      self.mition_num = 0
      self.mition_point_x = []
      self.mition_point_y = []
      self.mition_point_speed = []
      self.mition_point_a = []
      self.origin_x = 0.
      self.origin_y = 0.
      self.origin_z = 0.
      self.origin_yaw = 0.
      self.mition_finish = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_2Ii4f3i().pack(_x.stamp.secs, _x.stamp.nsecs, _x.e_stop, _x.speed, _x.ang_velo, _x.distance, _x.angle, _x.control_flag, _x.action_flag, _x.mition_num))
      length = len(self.mition_point_x)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.mition_point_x))
      length = len(self.mition_point_y)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.mition_point_y))
      length = len(self.mition_point_speed)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.mition_point_speed))
      length = len(self.mition_point_a)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.mition_point_a))
      _x = self
      buff.write(_get_struct_4fB().pack(_x.origin_x, _x.origin_y, _x.origin_z, _x.origin_yaw, _x.mition_finish))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.stamp is None:
        self.stamp = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 40
      (_x.stamp.secs, _x.stamp.nsecs, _x.e_stop, _x.speed, _x.ang_velo, _x.distance, _x.angle, _x.control_flag, _x.action_flag, _x.mition_num,) = _get_struct_2Ii4f3i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.mition_point_x = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.mition_point_y = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.mition_point_speed = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.mition_point_a = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 17
      (_x.origin_x, _x.origin_y, _x.origin_z, _x.origin_yaw, _x.mition_finish,) = _get_struct_4fB().unpack(str[start:end])
      self.mition_finish = bool(self.mition_finish)
      self.stamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_2Ii4f3i().pack(_x.stamp.secs, _x.stamp.nsecs, _x.e_stop, _x.speed, _x.ang_velo, _x.distance, _x.angle, _x.control_flag, _x.action_flag, _x.mition_num))
      length = len(self.mition_point_x)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.mition_point_x.tostring())
      length = len(self.mition_point_y)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.mition_point_y.tostring())
      length = len(self.mition_point_speed)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.mition_point_speed.tostring())
      length = len(self.mition_point_a)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.mition_point_a.tostring())
      _x = self
      buff.write(_get_struct_4fB().pack(_x.origin_x, _x.origin_y, _x.origin_z, _x.origin_yaw, _x.mition_finish))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.stamp is None:
        self.stamp = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 40
      (_x.stamp.secs, _x.stamp.nsecs, _x.e_stop, _x.speed, _x.ang_velo, _x.distance, _x.angle, _x.control_flag, _x.action_flag, _x.mition_num,) = _get_struct_2Ii4f3i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.mition_point_x = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.mition_point_y = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.mition_point_speed = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.mition_point_a = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      _x = self
      start = end
      end += 17
      (_x.origin_x, _x.origin_y, _x.origin_z, _x.origin_yaw, _x.mition_finish,) = _get_struct_4fB().unpack(str[start:end])
      self.mition_finish = bool(self.mition_finish)
      self.stamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2Ii4f3i = None
def _get_struct_2Ii4f3i():
    global _struct_2Ii4f3i
    if _struct_2Ii4f3i is None:
        _struct_2Ii4f3i = struct.Struct("<2Ii4f3i")
    return _struct_2Ii4f3i
_struct_4fB = None
def _get_struct_4fB():
    global _struct_4fB
    if _struct_4fB is None:
        _struct_4fB = struct.Struct("<4fB")
    return _struct_4fB