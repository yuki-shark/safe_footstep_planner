#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/chidori-interface.l")

(chidori-init)
(print "chidori initialized")
(setq *robot* *chidori*)

(defun check-foothold(req)
  ;; #f(0.25 0.00 0.07)
  (when (and (>= (send req :target :x)  0.00) (<= (send req :target :x) 0.30)
             (>= (send req :target :y) -0.10) (<= (send req :target :y) 0.10)
             (>= (send req :target :z)  0.00) (<= (send req :target :z) 0.08))
    (print "starting check foothold")
    (format t "x: ~A   y: ~A   z: ~A~%"
            (send req :target :x)
            (send req :target :y)
            (send req :target :z))
    (let ((m (send req :response)))
      (progn
        (print "move ZMP to lleg")
        (send *ri* :stop-st)
        (send *ri* :stop-auto-balancer)
        (send *ri* :set-auto-balancer-param :leg-names '(:lleg) :transition-time 10.0)
        (send *ri* :start-auto-balancer)
        ;; (send *ri* :start-st)
        (print "step on")
        (progn
          (send *robot* :reset-pose)
          (send *robot* :rleg :move-end-pos (float-vector 0 0 95))
          (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
          (send *ri* :wait-interpolation)
          )
        (progn
          (send *robot* :reset-pose)
          (send *robot* :rleg :move-end-pos (scale 1000 (float-vector (send req :target :x) (send req :target :y) 0.095)))
          (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
          (send *ri* :wait-interpolation)
          )
        (print "watching force")
        (progn
          (send *robot* :reset-pose)
          (send *robot* :rleg :move-end-pos (scale 1000 (float-vector (send req :target :x) (send req :target :y) (- (send req :target :z) 0.005))))
          (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
          ;; (send *ri* :wait-interpolation)
          (unix:usleep 50000)
          (setq min_z 1000)
          (while (send *ri* :interpolatingp)
            (when (< (abs (elt (send *ri* :state :force-vector :rleg) 2)) min_z)
              (setq min_z (abs (elt (send *ri* :state :force-vector :rleg) 2)))
              )
            )
          (format t "min_z: ~A~%" min_z)
          )
        ;; (read-line)
        (unix:sleep 3)
        (print "step off")
        (progn
          (send *robot* :reset-pose)
          (send *robot* :rleg :move-end-pos (scale 1000 (float-vector (send req :target :x) (send req :target :y) 0.095)))
          (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
          (send *ri* :wait-interpolation)
          )
        (progn
          (send *robot* :reset-pose)
          (send *robot* :rleg :move-end-pos (float-vector 0 0 95))
          (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
          (send *ri* :wait-interpolation)
          )
        (progn
          (send *robot* :reset-pose)
          (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
          (send *robot* :rleg :move-end-pos (float-vector 0 0 0))
          (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
          (send *ri* :wait-interpolation)
          )
        ;; (read-line)
        (unix:sleep 3)
        (print "move ZMP to center of legs")
        (progn
          (send *ri* :stop-auto-balancer)
          (send *ri* :set-auto-balancer-param :leg-names '(:rleg :lleg) :transition-time 2.0)
          (send *ri* :start-auto-balancer)
          (send *ri* :start-st)
          )
        ) ;; check foothold motion
      ;; (setq min_z (* 1000 (send req :target :x)))
      (print "check foothold finished")
      (send m :force_z min_z)
      m) ;; let
    ) ;; when
  ) ;; defun

(ros::roseus "check_foothold_server")
(print "init check_foothold_server")
(ros::load-ros-manifest "safe_footstep_planner")
(ros::advertise-service "check_foothold" safe_footstep_planner::CheckFoothold #'check-foothold)
(do-until-key
 (ros::spin-once))
