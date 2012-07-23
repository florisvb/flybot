; Auto-generated. Do not edit!


(cl:in-package camera_aravis-srv)


;//! \htmlinclude ReturnImageFloat-request.msg.html

(cl:defclass <ReturnImageFloat-request> (roslisp-msg-protocol:ros-message)
  ((request
    :reader request
    :initarg :request
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ReturnImageFloat-request (<ReturnImageFloat-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReturnImageFloat-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReturnImageFloat-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_aravis-srv:<ReturnImageFloat-request> is deprecated: use camera_aravis-srv:ReturnImageFloat-request instead.")))

(cl:ensure-generic-function 'request-val :lambda-list '(m))
(cl:defmethod request-val ((m <ReturnImageFloat-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_aravis-srv:request-val is deprecated.  Use camera_aravis-srv:request instead.")
  (request m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReturnImageFloat-request>) ostream)
  "Serializes a message object of type '<ReturnImageFloat-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'request) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReturnImageFloat-request>) istream)
  "Deserializes a message object of type '<ReturnImageFloat-request>"
    (cl:setf (cl:slot-value msg 'request) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReturnImageFloat-request>)))
  "Returns string type for a service object of type '<ReturnImageFloat-request>"
  "camera_aravis/ReturnImageFloatRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReturnImageFloat-request)))
  "Returns string type for a service object of type 'ReturnImageFloat-request"
  "camera_aravis/ReturnImageFloatRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReturnImageFloat-request>)))
  "Returns md5sum for a message object of type '<ReturnImageFloat-request>"
  "6cd7e2eb7fd7a9c18f076cc4104651f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReturnImageFloat-request)))
  "Returns md5sum for a message object of type 'ReturnImageFloat-request"
  "6cd7e2eb7fd7a9c18f076cc4104651f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReturnImageFloat-request>)))
  "Returns full string definition for message of type '<ReturnImageFloat-request>"
  (cl:format cl:nil "bool request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReturnImageFloat-request)))
  "Returns full string definition for message of type 'ReturnImageFloat-request"
  (cl:format cl:nil "bool request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReturnImageFloat-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReturnImageFloat-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReturnImageFloat-request
    (cl:cons ':request (request msg))
))
;//! \htmlinclude ReturnImageFloat-response.msg.html

(cl:defclass <ReturnImageFloat-response> (roslisp-msg-protocol:ros-message)
  ((shape
    :reader shape
    :initarg :shape
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ReturnImageFloat-response (<ReturnImageFloat-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReturnImageFloat-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReturnImageFloat-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_aravis-srv:<ReturnImageFloat-response> is deprecated: use camera_aravis-srv:ReturnImageFloat-response instead.")))

(cl:ensure-generic-function 'shape-val :lambda-list '(m))
(cl:defmethod shape-val ((m <ReturnImageFloat-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_aravis-srv:shape-val is deprecated.  Use camera_aravis-srv:shape instead.")
  (shape m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <ReturnImageFloat-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_aravis-srv:data-val is deprecated.  Use camera_aravis-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReturnImageFloat-response>) ostream)
  "Serializes a message object of type '<ReturnImageFloat-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'shape))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'shape))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReturnImageFloat-response>) istream)
  "Deserializes a message object of type '<ReturnImageFloat-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'shape) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'shape)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReturnImageFloat-response>)))
  "Returns string type for a service object of type '<ReturnImageFloat-response>"
  "camera_aravis/ReturnImageFloatResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReturnImageFloat-response)))
  "Returns string type for a service object of type 'ReturnImageFloat-response"
  "camera_aravis/ReturnImageFloatResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReturnImageFloat-response>)))
  "Returns md5sum for a message object of type '<ReturnImageFloat-response>"
  "6cd7e2eb7fd7a9c18f076cc4104651f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReturnImageFloat-response)))
  "Returns md5sum for a message object of type 'ReturnImageFloat-response"
  "6cd7e2eb7fd7a9c18f076cc4104651f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReturnImageFloat-response>)))
  "Returns full string definition for message of type '<ReturnImageFloat-response>"
  (cl:format cl:nil "uint16[] shape~%float32[] data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReturnImageFloat-response)))
  "Returns full string definition for message of type 'ReturnImageFloat-response"
  (cl:format cl:nil "uint16[] shape~%float32[] data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReturnImageFloat-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'shape) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReturnImageFloat-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReturnImageFloat-response
    (cl:cons ':shape (shape msg))
    (cl:cons ':data (data msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReturnImageFloat)))
  'ReturnImageFloat-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReturnImageFloat)))
  'ReturnImageFloat-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReturnImageFloat)))
  "Returns string type for a service object of type '<ReturnImageFloat>"
  "camera_aravis/ReturnImageFloat")