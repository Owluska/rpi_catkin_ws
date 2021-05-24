
(cl:in-package :asdf)

(defsystem "rpicar-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "camera" :depends-on ("_package_camera"))
    (:file "_package_camera" :depends-on ("_package"))
  ))