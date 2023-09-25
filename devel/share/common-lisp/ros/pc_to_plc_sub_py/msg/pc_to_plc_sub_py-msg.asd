
(cl:in-package :asdf)

(defsystem "pc_to_plc_sub_py-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "double5" :depends-on ("_package_double5"))
    (:file "_package_double5" :depends-on ("_package"))
  ))