;; Auto-generated. Do not edit!


(when (boundp 'lifter_ctr::lifter_mode)
  (if (not (find-package "LIFTER_CTR"))
    (make-package "LIFTER_CTR"))
  (shadow 'lifter_mode (find-package "LIFTER_CTR")))
(unless (find-package "LIFTER_CTR::LIFTER_MODE")
  (make-package "LIFTER_CTR::LIFTER_MODE"))

(in-package "ROS")
;;//! \htmlinclude lifter_mode.msg.html


(defclass lifter_ctr::lifter_mode
  :super ros::object
  :slots (_updown_mode ))

(defmethod lifter_ctr::lifter_mode
  (:init
   (&key
    ((:updown_mode __updown_mode) "")
    )
   (send-super :init)
   (setq _updown_mode (string __updown_mode))
   self)
  (:updown_mode
   (&optional __updown_mode)
   (if __updown_mode (setq _updown_mode __updown_mode)) _updown_mode)
  (:serialization-length
   ()
   (+
    ;; string _updown_mode
    4 (length _updown_mode)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _updown_mode
       (write-long (length _updown_mode) s) (princ _updown_mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _updown_mode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _updown_mode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get lifter_ctr::lifter_mode :md5sum-) "f7990a6d0129a755d88d46f3f5051aed")
(setf (get lifter_ctr::lifter_mode :datatype-) "lifter_ctr/lifter_mode")
(setf (get lifter_ctr::lifter_mode :definition-)
      "string updown_mode
")



(provide :lifter_ctr/lifter_mode "f7990a6d0129a755d88d46f3f5051aed")


