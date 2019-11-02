;; Auto-generated. Do not edit!


(when (boundp 'simple_parking_maneuver::ParkingManeuver)
  (if (not (find-package "SIMPLE_PARKING_MANEUVER"))
    (make-package "SIMPLE_PARKING_MANEUVER"))
  (shadow 'ParkingManeuver (find-package "SIMPLE_PARKING_MANEUVER")))
(unless (find-package "SIMPLE_PARKING_MANEUVER::PARKINGMANEUVER")
  (make-package "SIMPLE_PARKING_MANEUVER::PARKINGMANEUVER"))
(unless (find-package "SIMPLE_PARKING_MANEUVER::PARKINGMANEUVERREQUEST")
  (make-package "SIMPLE_PARKING_MANEUVER::PARKINGMANEUVERREQUEST"))
(unless (find-package "SIMPLE_PARKING_MANEUVER::PARKINGMANEUVERRESPONSE")
  (make-package "SIMPLE_PARKING_MANEUVER::PARKINGMANEUVERRESPONSE"))

(in-package "ROS")





(defclass simple_parking_maneuver::ParkingManeuverRequest
  :super ros::object
  :slots (_direction ))

(defmethod simple_parking_maneuver::ParkingManeuverRequest
  (:init
   (&key
    ((:direction __direction) "")
    )
   (send-super :init)
   (setq _direction (string __direction))
   self)
  (:direction
   (&optional __direction)
   (if __direction (setq _direction __direction)) _direction)
  (:serialization-length
   ()
   (+
    ;; string _direction
    4 (length _direction)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _direction
       (write-long (length _direction) s) (princ _direction s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _direction
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _direction (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass simple_parking_maneuver::ParkingManeuverResponse
  :super ros::object
  :slots (_info ))

(defmethod simple_parking_maneuver::ParkingManeuverResponse
  (:init
   (&key
    ((:info __info) "")
    )
   (send-super :init)
   (setq _info (string __info))
   self)
  (:info
   (&optional __info)
   (if __info (setq _info __info)) _info)
  (:serialization-length
   ()
   (+
    ;; string _info
    4 (length _info)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _info
       (write-long (length _info) s) (princ _info s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _info
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _info (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass simple_parking_maneuver::ParkingManeuver
  :super ros::object
  :slots ())

(setf (get simple_parking_maneuver::ParkingManeuver :md5sum-) "7a31466994d8db49e4cf1556345a8002")
(setf (get simple_parking_maneuver::ParkingManeuver :datatype-) "simple_parking_maneuver/ParkingManeuver")
(setf (get simple_parking_maneuver::ParkingManeuver :request) simple_parking_maneuver::ParkingManeuverRequest)
(setf (get simple_parking_maneuver::ParkingManeuver :response) simple_parking_maneuver::ParkingManeuverResponse)

(defmethod simple_parking_maneuver::ParkingManeuverRequest
  (:response () (instance simple_parking_maneuver::ParkingManeuverResponse :init)))

(setf (get simple_parking_maneuver::ParkingManeuverRequest :md5sum-) "7a31466994d8db49e4cf1556345a8002")
(setf (get simple_parking_maneuver::ParkingManeuverRequest :datatype-) "simple_parking_maneuver/ParkingManeuverRequest")
(setf (get simple_parking_maneuver::ParkingManeuverRequest :definition-)
      "string direction
---
string info
")

(setf (get simple_parking_maneuver::ParkingManeuverResponse :md5sum-) "7a31466994d8db49e4cf1556345a8002")
(setf (get simple_parking_maneuver::ParkingManeuverResponse :datatype-) "simple_parking_maneuver/ParkingManeuverResponse")
(setf (get simple_parking_maneuver::ParkingManeuverResponse :definition-)
      "string direction
---
string info
")



(provide :simple_parking_maneuver/ParkingManeuver "7a31466994d8db49e4cf1556345a8002")


