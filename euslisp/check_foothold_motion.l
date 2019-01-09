(chidori-init)
(setq *robot* *chidori*)

;; step motion
(progn
  (send *ri* :stop-st)
  (send *ri* :set-log-maxlength 20000)
  (send *ri* :set-gait-generator-param
        :default-orbit-type :stair
        :default-step-height 0.075
        :default-step-time 3
        :default-double-support-ratio 0.5
        )
  (send *ri* :start-log)
  (send *ri* :set-foot-steps
        (list (make-coords :coords (send *robot* :lleg :end-coords :copy-worldcoords) :name :lleg)
              (make-coords :coords (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 250 0 70)) :name :rleg)
              ))
  (send *ri* :set-foot-steps
        (list (make-coords :coords (send *robot* :lleg :end-coords :copy-worldcoords) :name :lleg)
              (make-coords :coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 0 -200 0)) :name :rleg)
              ))
  (send *ri* :save-log "/tmp/ohmori_exp_sponge_double2")
  (send *ri* :start-st)
  )

;; 20190109
;; check motion
(progn
  (send *ri* :stop-st)
  (send *ri* :stop-auto-balancer)
  (send *ri* :set-auto-balancer-param :leg-names '(:lleg) :transition-time 10.0)
  (send *ri* :start-auto-balancer)
  ;; (send *ri* :start-st)
  (progn
    (send *robot* :reset-pose)
    (send *robot* :rleg :move-end-pos #f(0 0 95))
    (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    )
  (progn
    (send *robot* :reset-pose)
    (send *robot* :rleg :move-end-pos #f(250 0 95))
    (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    )
  (progn
    (send *robot* :reset-pose)
    (send *robot* :rleg :move-end-pos #f(250 0 65))
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    )
  ;; (read-line)
  (unix::sleep 3)
  (progn
    (send *robot* :reset-pose)
    (send *robot* :rleg :move-end-pos #f(250 0 95))
    (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    )
  (progn
    (send *robot* :reset-pose)
    (send *robot* :rleg :move-end-pos #f(0 0 95))
    (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    )
  (progn
    (send *robot* :reset-pose)
    (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
    (send *robot* :rleg :move-end-pos #f(0 0 0))
    (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    )
  ;; (read-line)
  (unix::sleep 3)
  (progn
    (send *ri* :stop-auto-balancer)
    (send *ri* :set-auto-balancer-param :leg-names '(:rleg :lleg) :transition-time 2.0)
    (send *ri* :start-auto-balancer)
    (send *ri* :start-st)
    )
  )
