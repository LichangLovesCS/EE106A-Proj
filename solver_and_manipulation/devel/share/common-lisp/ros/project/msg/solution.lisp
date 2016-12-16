; Auto-generated. Do not edit!


(cl:in-package project-msg)


;//! \htmlinclude solution.msg.html

(cl:defclass <solution> (roslisp-msg-protocol:ros-message)
  ((number
    :reader number
    :initarg :number
    :type cl:fixnum
    :initform 0)
   (face
    :reader face
    :initarg :face
    :type cl:string
    :initform "")
   (radian
    :reader radian
    :initarg :radian
    :type cl:float
    :initform 0.0))
)

(cl:defclass solution (<solution>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <solution>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'solution)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name project-msg:<solution> is deprecated: use project-msg:solution instead.")))

(cl:ensure-generic-function 'number-val :lambda-list '(m))
(cl:defmethod number-val ((m <solution>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader project-msg:number-val is deprecated.  Use project-msg:number instead.")
  (number m))

(cl:ensure-generic-function 'face-val :lambda-list '(m))
(cl:defmethod face-val ((m <solution>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader project-msg:face-val is deprecated.  Use project-msg:face instead.")
  (face m))

(cl:ensure-generic-function 'radian-val :lambda-list '(m))
(cl:defmethod radian-val ((m <solution>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader project-msg:radian-val is deprecated.  Use project-msg:radian instead.")
  (radian m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <solution>) ostream)
  "Serializes a message object of type '<solution>"
  (cl:let* ((signed (cl:slot-value msg 'number)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'face))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'face))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'radian))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <solution>) istream)
  "Deserializes a message object of type '<solution>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'number) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'face) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'face) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'radian) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<solution>)))
  "Returns string type for a message object of type '<solution>"
  "project/solution")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'solution)))
  "Returns string type for a message object of type 'solution"
  "project/solution")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<solution>)))
  "Returns md5sum for a message object of type '<solution>"
  "0eb1e6722a11b1e46249ddcba193853f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'solution)))
  "Returns md5sum for a message object of type 'solution"
  "0eb1e6722a11b1e46249ddcba193853f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<solution>)))
  "Returns full string definition for message of type '<solution>"
  (cl:format cl:nil "int16 number~%string face~%float32 radian~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'solution)))
  "Returns full string definition for message of type 'solution"
  (cl:format cl:nil "int16 number~%string face~%float32 radian~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <solution>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'face))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <solution>))
  "Converts a ROS message object to a list"
  (cl:list 'solution
    (cl:cons ':number (number msg))
    (cl:cons ':face (face msg))
    (cl:cons ':radian (radian msg))
))
