
(cl:in-package :asdf)

(defsystem "simple_parking_maneuver-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ParkingManeuver" :depends-on ("_package_ParkingManeuver"))
    (:file "_package_ParkingManeuver" :depends-on ("_package"))
  ))