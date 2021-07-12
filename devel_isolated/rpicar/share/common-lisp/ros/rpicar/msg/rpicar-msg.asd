
(cl:in-package :asdf)

(defsystem "rpicar-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :nav_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "telemetry" :depends-on ("_package_telemetry"))
    (:file "_package_telemetry" :depends-on ("_package"))
  ))