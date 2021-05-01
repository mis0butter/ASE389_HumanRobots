
(cl:in-package :asdf)

(defsystem "towr_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :xpp_msgs-msg
)
  :components ((:file "_package")
    (:file "TowrCommand" :depends-on ("_package_TowrCommand"))
    (:file "_package_TowrCommand" :depends-on ("_package"))
  ))