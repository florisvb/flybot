
(cl:in-package :asdf)

(defsystem "camera_aravis-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ReturnImageFloat" :depends-on ("_package_ReturnImageFloat"))
    (:file "_package_ReturnImageFloat" :depends-on ("_package"))
  ))