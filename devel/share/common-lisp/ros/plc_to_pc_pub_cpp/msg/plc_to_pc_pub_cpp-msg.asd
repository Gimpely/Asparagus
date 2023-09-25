
(cl:in-package :asdf)

(defsystem "plc_to_pc_pub_cpp-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "double5" :depends-on ("_package_double5"))
    (:file "_package_double5" :depends-on ("_package"))
  ))