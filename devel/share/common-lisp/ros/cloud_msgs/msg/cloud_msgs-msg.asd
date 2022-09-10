
(cl:in-package :asdf)

(defsystem "cloud_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Grid" :depends-on ("_package_Grid"))
    (:file "_package_Grid" :depends-on ("_package"))
    (:file "PointXYA" :depends-on ("_package_PointXYA"))
    (:file "_package_PointXYA" :depends-on ("_package"))
    (:file "cloud_info" :depends-on ("_package_cloud_info"))
    (:file "_package_cloud_info" :depends-on ("_package"))
  ))