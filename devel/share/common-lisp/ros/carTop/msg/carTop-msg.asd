
(cl:in-package :asdf)

(defsystem "carTop-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "carTop" :depends-on ("_package_carTop"))
    (:file "_package_carTop" :depends-on ("_package"))
  ))