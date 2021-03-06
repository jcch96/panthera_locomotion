;; Auto-generated. Do not edit!


(when (boundp 'global_planner::CmapClear)
  (if (not (find-package "GLOBAL_PLANNER"))
    (make-package "GLOBAL_PLANNER"))
  (shadow 'CmapClear (find-package "GLOBAL_PLANNER")))
(unless (find-package "GLOBAL_PLANNER::CMAPCLEAR")
  (make-package "GLOBAL_PLANNER::CMAPCLEAR"))

(in-package "ROS")
;;//! \htmlinclude CmapClear.msg.html


(defclass global_planner::CmapClear
  :super ros::object
  :slots (_right _up _left ))

(defmethod global_planner::CmapClear
  (:init
   (&key
    ((:right __right) nil)
    ((:up __up) nil)
    ((:left __left) nil)
    )
   (send-super :init)
   (setq _right __right)
   (setq _up __up)
   (setq _left __left)
   self)
  (:right
   (&optional __right)
   (if __right (setq _right __right)) _right)
  (:up
   (&optional __up)
   (if __up (setq _up __up)) _up)
  (:left
   (&optional __left)
   (if __left (setq _left __left)) _left)
  (:serialization-length
   ()
   (+
    ;; bool _right
    1
    ;; bool _up
    1
    ;; bool _left
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _right
       (if _right (write-byte -1 s) (write-byte 0 s))
     ;; bool _up
       (if _up (write-byte -1 s) (write-byte 0 s))
     ;; bool _left
       (if _left (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _right
     (setq _right (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _up
     (setq _up (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _left
     (setq _left (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get global_planner::CmapClear :md5sum-) "d5d3676c50ca21d6bdbb6d3621aefaac")
(setf (get global_planner::CmapClear :datatype-) "global_planner/CmapClear")
(setf (get global_planner::CmapClear :definition-)
      "bool right
bool up
bool left

")



(provide :global_planner/CmapClear "d5d3676c50ca21d6bdbb6d3621aefaac")


