
(cl:in-package :asdf)

(defsystem "lifter_ctr-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "lifter_mode" :depends-on ("_package_lifter_mode"))
    (:file "_package_lifter_mode" :depends-on ("_package"))
  ))