
(cl:in-package :asdf)

(defsystem "labbot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "msgFromLabbot" :depends-on ("_package_msgFromLabbot"))
    (:file "_package_msgFromLabbot" :depends-on ("_package"))
    (:file "msgToLabbot" :depends-on ("_package_msgToLabbot"))
    (:file "_package_msgToLabbot" :depends-on ("_package"))
  ))