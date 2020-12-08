# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from quadrotor_msgs/PPROutputData.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class PPROutputData(genpy.Message):
  _md5sum = "732c0e3ca36f241464f8c445e78a0d0a"
  _type = "quadrotor_msgs/PPROutputData"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header
uint16 quad_time
float64 des_thrust
float64 des_roll
float64 des_pitch
float64 des_yaw
float64 est_roll
float64 est_pitch
float64 est_yaw
float64 est_angvel_x
float64 est_angvel_y
float64 est_angvel_z
float64 est_acc_x
float64 est_acc_y
float64 est_acc_z
uint16[4] pwm

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id
"""
  __slots__ = ['header','quad_time','des_thrust','des_roll','des_pitch','des_yaw','est_roll','est_pitch','est_yaw','est_angvel_x','est_angvel_y','est_angvel_z','est_acc_x','est_acc_y','est_acc_z','pwm']
  _slot_types = ['std_msgs/Header','uint16','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','uint16[4]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,quad_time,des_thrust,des_roll,des_pitch,des_yaw,est_roll,est_pitch,est_yaw,est_angvel_x,est_angvel_y,est_angvel_z,est_acc_x,est_acc_y,est_acc_z,pwm

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PPROutputData, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.quad_time is None:
        self.quad_time = 0
      if self.des_thrust is None:
        self.des_thrust = 0.
      if self.des_roll is None:
        self.des_roll = 0.
      if self.des_pitch is None:
        self.des_pitch = 0.
      if self.des_yaw is None:
        self.des_yaw = 0.
      if self.est_roll is None:
        self.est_roll = 0.
      if self.est_pitch is None:
        self.est_pitch = 0.
      if self.est_yaw is None:
        self.est_yaw = 0.
      if self.est_angvel_x is None:
        self.est_angvel_x = 0.
      if self.est_angvel_y is None:
        self.est_angvel_y = 0.
      if self.est_angvel_z is None:
        self.est_angvel_z = 0.
      if self.est_acc_x is None:
        self.est_acc_x = 0.
      if self.est_acc_y is None:
        self.est_acc_y = 0.
      if self.est_acc_z is None:
        self.est_acc_z = 0.
      if self.pwm is None:
        self.pwm = [0] * 4
    else:
      self.header = std_msgs.msg.Header()
      self.quad_time = 0
      self.des_thrust = 0.
      self.des_roll = 0.
      self.des_pitch = 0.
      self.des_yaw = 0.
      self.est_roll = 0.
      self.est_pitch = 0.
      self.est_yaw = 0.
      self.est_angvel_x = 0.
      self.est_angvel_y = 0.
      self.est_angvel_z = 0.
      self.est_acc_x = 0.
      self.est_acc_y = 0.
      self.est_acc_z = 0.
      self.pwm = [0] * 4

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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_H13d().pack(_x.quad_time, _x.des_thrust, _x.des_roll, _x.des_pitch, _x.des_yaw, _x.est_roll, _x.est_pitch, _x.est_yaw, _x.est_angvel_x, _x.est_angvel_y, _x.est_angvel_z, _x.est_acc_x, _x.est_acc_y, _x.est_acc_z))
      buff.write(_get_struct_4H().pack(*self.pwm))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 106
      (_x.quad_time, _x.des_thrust, _x.des_roll, _x.des_pitch, _x.des_yaw, _x.est_roll, _x.est_pitch, _x.est_yaw, _x.est_angvel_x, _x.est_angvel_y, _x.est_angvel_z, _x.est_acc_x, _x.est_acc_y, _x.est_acc_z,) = _get_struct_H13d().unpack(str[start:end])
      start = end
      end += 8
      self.pwm = _get_struct_4H().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_H13d().pack(_x.quad_time, _x.des_thrust, _x.des_roll, _x.des_pitch, _x.des_yaw, _x.est_roll, _x.est_pitch, _x.est_yaw, _x.est_angvel_x, _x.est_angvel_y, _x.est_angvel_z, _x.est_acc_x, _x.est_acc_y, _x.est_acc_z))
      buff.write(self.pwm.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 106
      (_x.quad_time, _x.des_thrust, _x.des_roll, _x.des_pitch, _x.des_yaw, _x.est_roll, _x.est_pitch, _x.est_yaw, _x.est_angvel_x, _x.est_angvel_y, _x.est_angvel_z, _x.est_acc_x, _x.est_acc_y, _x.est_acc_z,) = _get_struct_H13d().unpack(str[start:end])
      start = end
      end += 8
      self.pwm = numpy.frombuffer(str[start:end], dtype=numpy.uint16, count=4)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_H13d = None
def _get_struct_H13d():
    global _struct_H13d
    if _struct_H13d is None:
        _struct_H13d = struct.Struct("<H13d")
    return _struct_H13d
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_4H = None
def _get_struct_4H():
    global _struct_4H
    if _struct_4H is None:
        _struct_4H = struct.Struct("<4H")
    return _struct_4H
