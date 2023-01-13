;; Auto-generated. Do not edit!


(when (boundp 'labbot::msgToLabbot)
  (if (not (find-package "LABBOT"))
    (make-package "LABBOT"))
  (shadow 'msgToLabbot (find-package "LABBOT")))
(unless (find-package "LABBOT::MSGTOLABBOT")
  (make-package "LABBOT::MSGTOLABBOT"))

(in-package "ROS")
;;//! \htmlinclude msgToLabbot.msg.html


(defclass labbot::msgToLabbot
  :super ros::object
  :slots (_motorRightSpeed _motorLeftSpeed ))

(defmethod labbot::msgToLabbot
  (:init
   (&key
    ((:motorRightSpeed __motorRightSpeed) 0.0)
    ((:motorLeftSpeed __motorLeftSpeed) 0.0)
    )
   (send-super :init)
   (setq _motorRightSpeed (float __motorRightSpeed))
   (setq _motorLeftSpeed (float __motorLeftSpeed))
   self)
  (:motorRightSpeed
   (&optional __motorRightSpeed)
   (if __motorRightSpeed (setq _motorRightSpeed __motorRightSpeed)) _motorRightSpeed)
  (:motorLeftSpeed
   (&optional __motorLeftSpeed)
   (if __motorLeftSpeed (setq _motorLeftSpeed __motorLeftSpeed)) _motorLeftSpeed)
  (:serialization-length
   ()
   (+
    ;; float32 _motorRightSpeed
    4
    ;; float32 _motorLeftSpeed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _motorRightSpeed
       (sys::poke _motorRightSpeed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _motorLeftSpeed
       (sys::poke _motorLeftSpeed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _motorRightSpeed
     (setq _motorRightSpeed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _motorLeftSpeed
     (setq _motorLeftSpeed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get labbot::msgToLabbot :md5sum-) "227e24617faca74e2a267f0d326d13ec")
(setf (get labbot::msgToLabbot :datatype-) "labbot/msgToLabbot")
(setf (get labbot::msgToLabbot :definition-)
      "#motor speed (range: 0-80)
float32 motorRightSpeed
float32 motorLeftSpeed

")



(provide :labbot/msgToLabbot "227e24617faca74e2a267f0d326d13ec")


