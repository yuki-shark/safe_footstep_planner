#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")

(jaxon_red-init)
(format t "jaxon_red initialized~%")
(setq *robot* *jaxon_red*)

(send *robot* :head :neck-p :joint-angle 48)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
