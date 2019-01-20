#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/chidori-interface.l")
(load "package://safe_footstep_planner/euslisp/foothold_check_experiment.l")

(chidori-init)
(format t "chidori initialized~%")
(setq *robot* *chidori*)

(defun check-foothold(req)
  ;; #f(0.25 0.00 0.07)
  (when (and (>= (send req :target :x)  0.00) (<= (send req :target :x) 0.30)
             (>= (send req :target :y) -0.03) (<= (send req :target :y) 0.03)
             (>= (send req :target :z)  0.10) (<= (send req :target :z) 0.10))
    (format t "Start Check Foothold~%")
    (format t "x: ~A  y: ~A  z: ~A~%"
            (send req :target :x)
            (send req :target :y)
            (send req :target :z))
    ;; (setq default-pose (send *ri* :state :potentio-vector))
    (setq target-x (* (send req :target :x) 1000))
    (setq target-y (* (send req :target :y) 1000))
    (setq target-z (* (send req :target :z) 1000))
    (let ((m (send req :response)))
      (when (eq (send req :leg) 1)
        (setq force-mean (foothold-check-lleg target-x target-y target-z))
        ) ;; when leg = 1 (lleg)
      (when (eq (send req :leg) 2)
        (setq force-mean (foothold-check-rleg target-x target-y target-z))
        ) ;; when leg = 2 (rleg)
      ;; (setq min_z (* 1000 (send req :target :x)))
      (format t "Check Foothold Finished~%")
      (send m :force_z (abs force-mean))
      m) ;; let response
    ) ;; when check target foothold coords
  ) ;; defun

(ros::roseus "check_foothold_controller")
(format t "Ready to check foothold~%")
(ros::load-ros-manifest "safe_footstep_planner")
(ros::advertise-service "check_foothold" safe_footstep_planner::CheckFoothold #'check-foothold)
(do-until-key
 (ros::spin-once))
