
(cl:in-package :asdf)

(defsystem "manipulation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "solution" :depends-on ("_package_solution"))
    (:file "_package_solution" :depends-on ("_package"))
  ))