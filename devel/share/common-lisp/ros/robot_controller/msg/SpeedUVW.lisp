; Auto-generated. Do not edit!


(cl:in-package robot_controller-msg)


;//! \htmlinclude SpeedUVW.msg.html

(cl:defclass <SpeedUVW> (roslisp-msg-protocol:ros-message)
  ((pose1
    :reader pose1
    :initarg :pose1
    :type cl:boolean
    :initform cl:nil)
   (pose2
    :reader pose2
    :initarg :pose2
    :type cl:boolean
    :initform cl:nil)
   (lVoila
    :reader lVoila
    :initarg :lVoila
    :type cl:boolean
    :initform cl:nil)
   (rVoila
    :reader rVoila
    :initarg :rVoila
    :type cl:boolean
    :initform cl:nil)
   (flat
    :reader flat
    :initarg :flat
    :type cl:boolean
    :initform cl:nil)
   (leftArm
    :reader leftArm
    :initarg :leftArm
    :type cl:boolean
    :initform cl:nil)
   (rightArm
    :reader rightArm
    :initarg :rightArm
    :type cl:boolean
    :initform cl:nil)
   (header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pos
    :reader pos
    :initarg :pos
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (vel
    :reader vel
    :initarg :vel
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass SpeedUVW (<SpeedUVW>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpeedUVW>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpeedUVW)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_controller-msg:<SpeedUVW> is deprecated: use robot_controller-msg:SpeedUVW instead.")))

(cl:ensure-generic-function 'pose1-val :lambda-list '(m))
(cl:defmethod pose1-val ((m <SpeedUVW>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_controller-msg:pose1-val is deprecated.  Use robot_controller-msg:pose1 instead.")
  (pose1 m))

(cl:ensure-generic-function 'pose2-val :lambda-list '(m))
(cl:defmethod pose2-val ((m <SpeedUVW>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_controller-msg:pose2-val is deprecated.  Use robot_controller-msg:pose2 instead.")
  (pose2 m))

(cl:ensure-generic-function 'lVoila-val :lambda-list '(m))
(cl:defmethod lVoila-val ((m <SpeedUVW>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_controller-msg:lVoila-val is deprecated.  Use robot_controller-msg:lVoila instead.")
  (lVoila m))

(cl:ensure-generic-function 'rVoila-val :lambda-list '(m))
(cl:defmethod rVoila-val ((m <SpeedUVW>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_controller-msg:rVoila-val is deprecated.  Use robot_controller-msg:rVoila instead.")
  (rVoila m))

(cl:ensure-generic-function 'flat-val :lambda-list '(m))
(cl:defmethod flat-val ((m <SpeedUVW>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_controller-msg:flat-val is deprecated.  Use robot_controller-msg:flat instead.")
  (flat m))

(cl:ensure-generic-function 'leftArm-val :lambda-list '(m))
(cl:defmethod leftArm-val ((m <SpeedUVW>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_controller-msg:leftArm-val is deprecated.  Use robot_controller-msg:leftArm instead.")
  (leftArm m))

(cl:ensure-generic-function 'rightArm-val :lambda-list '(m))
(cl:defmethod rightArm-val ((m <SpeedUVW>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_controller-msg:rightArm-val is deprecated.  Use robot_controller-msg:rightArm instead.")
  (rightArm m))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SpeedUVW>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_controller-msg:header-val is deprecated.  Use robot_controller-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <SpeedUVW>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_controller-msg:pos-val is deprecated.  Use robot_controller-msg:pos instead.")
  (pos m))

(cl:ensure-generic-function 'vel-val :lambda-list '(m))
(cl:defmethod vel-val ((m <SpeedUVW>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_controller-msg:vel-val is deprecated.  Use robot_controller-msg:vel instead.")
  (vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpeedUVW>) ostream)
  "Serializes a message object of type '<SpeedUVW>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'pose1) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'pose2) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'lVoila) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'rVoila) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flat) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'leftArm) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'rightArm) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pos) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'vel) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpeedUVW>) istream)
  "Deserializes a message object of type '<SpeedUVW>"
    (cl:setf (cl:slot-value msg 'pose1) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'pose2) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'lVoila) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'rVoila) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'flat) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'leftArm) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'rightArm) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pos) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'vel) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpeedUVW>)))
  "Returns string type for a message object of type '<SpeedUVW>"
  "robot_controller/SpeedUVW")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpeedUVW)))
  "Returns string type for a message object of type 'SpeedUVW"
  "robot_controller/SpeedUVW")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpeedUVW>)))
  "Returns md5sum for a message object of type '<SpeedUVW>"
  "bf1de8ae7418bff109196d08604e9fda")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpeedUVW)))
  "Returns md5sum for a message object of type 'SpeedUVW"
  "bf1de8ae7418bff109196d08604e9fda")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpeedUVW>)))
  "Returns full string definition for message of type '<SpeedUVW>"
  (cl:format cl:nil "bool pose1~%bool pose2~%bool lVoila~%bool rVoila~%bool flat~%bool leftArm~%bool rightArm~%~%#for following project~%Header header~%geometry_msgs/Vector3 pos~%geometry_msgs/Vector3 vel~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpeedUVW)))
  "Returns full string definition for message of type 'SpeedUVW"
  (cl:format cl:nil "bool pose1~%bool pose2~%bool lVoila~%bool rVoila~%bool flat~%bool leftArm~%bool rightArm~%~%#for following project~%Header header~%geometry_msgs/Vector3 pos~%geometry_msgs/Vector3 vel~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpeedUVW>))
  (cl:+ 0
     1
     1
     1
     1
     1
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pos))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'vel))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpeedUVW>))
  "Converts a ROS message object to a list"
  (cl:list 'SpeedUVW
    (cl:cons ':pose1 (pose1 msg))
    (cl:cons ':pose2 (pose2 msg))
    (cl:cons ':lVoila (lVoila msg))
    (cl:cons ':rVoila (rVoila msg))
    (cl:cons ':flat (flat msg))
    (cl:cons ':leftArm (leftArm msg))
    (cl:cons ':rightArm (rightArm msg))
    (cl:cons ':header (header msg))
    (cl:cons ':pos (pos msg))
    (cl:cons ':vel (vel msg))
))
