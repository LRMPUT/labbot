;; Auto-generated. Do not edit!


(when (boundp 'labbot::msgFromLabbot)
  (if (not (find-package "LABBOT"))
    (make-package "LABBOT"))
  (shadow 'msgFromLabbot (find-package "LABBOT")))
(unless (find-package "LABBOT::MSGFROMLABBOT")
  (make-package "LABBOT::MSGFROMLABBOT"))

(in-package "ROS")
;;//! \htmlinclude msgFromLabbot.msg.html


(defclass labbot::msgFromLabbot
  :super ros::object
  :slots (_motorRightInput _motorRightSetpoint _motorRightOutput _motorLeftInput _motorLeftSetpoint _motorLefttOutput ))

(defmethod labbot::msgFromLabbot
  (:init
   (&key
    ((:motorRightInput __motorRightInput) 0.0)
    ((:motorRightSetpoint __motorRightSetpoint) 0.0)
    ((:motorRightOutput __motorRightOutput) 0.0)
    ((:motorLeftInput __motorLeftInput) 0.0)
    ((:motorLeftSetpoint __motorLeftSetpoint) 0.0)
    ((:motorLefttOutput __motorLefttOutput) 0.0)
    )
   (send-super :init)
   (setq _motorRightInput (float __motorRightInput))
   (setq _motorRightSetpoint (float __motorRightSetpoint))
   (setq _motorRightOutput (float __motorRightOutput))
   (setq _motorLeftInput (float __motorLeftInput))
   (setq _motorLeftSetpoint (float __motorLeftSetpoint))
   (setq _motorLefttOutput (float __motorLefttOutput))
   self)
  (:motorRightInput
   (&optional __motorRightInput)
   (if __motorRightInput (setq _motorRightInput __motorRightInput)) _motorRightInput)
  (:motorRightSetpoint
   (&optional __motorRightSetpoint)
   (if __motorRightSetpoint (setq _motorRightSetpoint __motorRightSetpoint)) _motorRightSetpoint)
  (:motorRightOutput
   (&optional __motorRightOutput)
   (if __motorRightOutput (setq _motorRightOutput __motorRightOutput)) _motorRightOutput)
  (:motorLeftInput
   (&optional __motorLeftInput)
   (if __motorLeftInput (setq _motorLeftInput __motorLeftInput)) _motorLeftInput)
  (:motorLeftSetpoint
   (&optional __motorLeftSetpoint)
   (if __motorLeftSetpoint (setq _motorLeftSetpoint __motorLeftSetpoint)) _motorLeftSetpoint)
  (:motorLefttOutput
   (&optional __motorLefttOutput)
   (if __motorLefttOutput (setq _motorLefttOutput __motorLefttOutput)) _motorLefttOutput)
  (:serialization-length
   ()
   (+
    ;; float32 _motorRightInput
    4
    ;; float32 _motorRightSetpoint
    4
    ;; float32 _motorRightOutput
    4
    ;; float32 _motorLeftInput
    4
    ;; float32 _motorLeftSetpoint
    4
    ;; float32 _motorLefttOutput
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _motorRightInput
       (sys::poke _motorRightInput (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _motorRightSetpoint
       (sys::poke _motorRightSetpoint (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _motorRightOutput
       (sys::poke _motorRightOutput (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _motorLeftInput
       (sys::poke _motorLeftInput (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _motorLeftSetpoint
       (sys::poke _motorLeftSetpoint (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _motorLefttOutput
       (sys::poke _motorLefttOutput (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _motorRightInput
     (setq _motorRightInput (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _motorRightSetpoint
     (setq _motorRightSetpoint (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _motorRightOutput
     (setq _motorRightOutput (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _motorLeftInput
     (setq _motorLeftInput (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _motorLeftSetpoint
     (setq _motorLeftSetpoint (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _motorLefttOutput
     (setq _motorLefttOutput (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get labbot::msgFromLabbot :md5sum-) "4bf0f6e2fd2b507f5c1d51ceec5b93b5")
(setf (get labbot::msgFromLabbot :datatype-) "labbot/msgFromLabbot")
(setf (get labbot::msgFromLabbot :definition-)
      "# right motor parameters
float32 motorRightInput
float32 motorRightSetpoint
float32 motorRightOutput
# left motor parameters
float32 motorLeftInput
float32 motorLeftSetpoint
float32 motorLefttOutput

")



(provide :labbot/msgFromLabbot "4bf0f6e2fd2b507f5c1d51ceec5b93b5")


