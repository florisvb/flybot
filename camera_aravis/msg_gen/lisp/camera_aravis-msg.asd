
(cl:in-package :asdf)

(defsystem "camera_aravis-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "FloatArray" :depends-on ("_package_FloatArray"))
    (:file "_package_FloatArray" :depends-on ("_package"))
  ))