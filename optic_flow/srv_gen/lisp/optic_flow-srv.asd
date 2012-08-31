
(cl:in-package :asdf)

(defsystem "optic_flow-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ReturnImageFloat" :depends-on ("_package_ReturnImageFloat"))
    (:file "_package_ReturnImageFloat" :depends-on ("_package"))
  ))