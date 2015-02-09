
(cl:in-package :asdf)

(defsystem "sar_localization-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Csi" :depends-on ("_package_Csi"))
    (:file "_package_Csi" :depends-on ("_package"))
    (:file "Imu" :depends-on ("_package_Imu"))
    (:file "_package_Imu" :depends-on ("_package"))
  ))