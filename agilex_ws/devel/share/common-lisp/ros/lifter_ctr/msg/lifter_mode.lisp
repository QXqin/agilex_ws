; Auto-generated. Do not edit!


(cl:in-package lifter_ctr-msg)


;//! \htmlinclude lifter_mode.msg.html

(cl:defclass <lifter_mode> (roslisp-msg-protocol:ros-message)
  ((updown_mode
    :reader updown_mode
    :initarg :updown_mode
    :type cl:string
    :initform ""))
)

(cl:defclass lifter_mode (<lifter_mode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <lifter_mode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'lifter_mode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lifter_ctr-msg:<lifter_mode> is deprecated: use lifter_ctr-msg:lifter_mode instead.")))

(cl:ensure-generic-function 'updown_mode-val :lambda-list '(m))
(cl:defmethod updown_mode-val ((m <lifter_mode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lifter_ctr-msg:updown_mode-val is deprecated.  Use lifter_ctr-msg:updown_mode instead.")
  (updown_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <lifter_mode>) ostream)
  "Serializes a message object of type '<lifter_mode>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'updown_mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'updown_mode))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <lifter_mode>) istream)
  "Deserializes a message object of type '<lifter_mode>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'updown_mode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'updown_mode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<lifter_mode>)))
  "Returns string type for a message object of type '<lifter_mode>"
  "lifter_ctr/lifter_mode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lifter_mode)))
  "Returns string type for a message object of type 'lifter_mode"
  "lifter_ctr/lifter_mode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<lifter_mode>)))
  "Returns md5sum for a message object of type '<lifter_mode>"
  "f7990a6d0129a755d88d46f3f5051aed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'lifter_mode)))
  "Returns md5sum for a message object of type 'lifter_mode"
  "f7990a6d0129a755d88d46f3f5051aed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<lifter_mode>)))
  "Returns full string definition for message of type '<lifter_mode>"
  (cl:format cl:nil "string updown_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'lifter_mode)))
  "Returns full string definition for message of type 'lifter_mode"
  (cl:format cl:nil "string updown_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <lifter_mode>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'updown_mode))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <lifter_mode>))
  "Converts a ROS message object to a list"
  (cl:list 'lifter_mode
    (cl:cons ':updown_mode (updown_mode msg))
))
