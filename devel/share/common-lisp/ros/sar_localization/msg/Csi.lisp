; Auto-generated. Do not edit!


(cl:in-package sar_localization-msg)


;//! \htmlinclude Csi.msg.html

(cl:defclass <Csi> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (csi1_real
    :reader csi1_real
    :initarg :csi1_real
    :type cl:float
    :initform 0.0)
   (csi1_image
    :reader csi1_image
    :initarg :csi1_image
    :type cl:float
    :initform 0.0)
   (csi2_real
    :reader csi2_real
    :initarg :csi2_real
    :type cl:float
    :initform 0.0)
   (csi2_image
    :reader csi2_image
    :initarg :csi2_image
    :type cl:float
    :initform 0.0))
)

(cl:defclass Csi (<Csi>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Csi>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Csi)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sar_localization-msg:<Csi> is deprecated: use sar_localization-msg:Csi instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Csi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sar_localization-msg:header-val is deprecated.  Use sar_localization-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'csi1_real-val :lambda-list '(m))
(cl:defmethod csi1_real-val ((m <Csi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sar_localization-msg:csi1_real-val is deprecated.  Use sar_localization-msg:csi1_real instead.")
  (csi1_real m))

(cl:ensure-generic-function 'csi1_image-val :lambda-list '(m))
(cl:defmethod csi1_image-val ((m <Csi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sar_localization-msg:csi1_image-val is deprecated.  Use sar_localization-msg:csi1_image instead.")
  (csi1_image m))

(cl:ensure-generic-function 'csi2_real-val :lambda-list '(m))
(cl:defmethod csi2_real-val ((m <Csi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sar_localization-msg:csi2_real-val is deprecated.  Use sar_localization-msg:csi2_real instead.")
  (csi2_real m))

(cl:ensure-generic-function 'csi2_image-val :lambda-list '(m))
(cl:defmethod csi2_image-val ((m <Csi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sar_localization-msg:csi2_image-val is deprecated.  Use sar_localization-msg:csi2_image instead.")
  (csi2_image m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Csi>) ostream)
  "Serializes a message object of type '<Csi>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'csi1_real))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'csi1_image))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'csi2_real))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'csi2_image))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Csi>) istream)
  "Deserializes a message object of type '<Csi>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'csi1_real) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'csi1_image) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'csi2_real) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'csi2_image) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Csi>)))
  "Returns string type for a message object of type '<Csi>"
  "sar_localization/Csi")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Csi)))
  "Returns string type for a message object of type 'Csi"
  "sar_localization/Csi")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Csi>)))
  "Returns md5sum for a message object of type '<Csi>"
  "49a032c6fcc10451fb69ed7a76a6c778")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Csi)))
  "Returns md5sum for a message object of type 'Csi"
  "49a032c6fcc10451fb69ed7a76a6c778")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Csi>)))
  "Returns full string definition for message of type '<Csi>"
  (cl:format cl:nil "Header header~%float64 csi1_real~%float64 csi1_image~%float64 csi2_real~%float64 csi2_image~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Csi)))
  "Returns full string definition for message of type 'Csi"
  (cl:format cl:nil "Header header~%float64 csi1_real~%float64 csi1_image~%float64 csi2_real~%float64 csi2_image~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Csi>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Csi>))
  "Converts a ROS message object to a list"
  (cl:list 'Csi
    (cl:cons ':header (header msg))
    (cl:cons ':csi1_real (csi1_real msg))
    (cl:cons ':csi1_image (csi1_image msg))
    (cl:cons ':csi2_real (csi2_real msg))
    (cl:cons ':csi2_image (csi2_image msg))
))
