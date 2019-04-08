;; This buffer is for notes you don't want to save, and for Lisp evaluation.
;; If you want to create a file, visit that file with C-x C-f,
;; then enter the text in that file's own buffer.

;; (chidori-init)
;; (format t "chidori initialized~%")
;; (setq *robot* *chidori*)

(defun foothold-check-lleg(target_x target_y target_z)
  (progn
    ;; (setq default-pose (copy-seq (send *ri* :state :potentio-vector)))
    (send *robot* :angle-vector (send *ri* :state :potentio-vector))
    (send *robot* :fix-leg-to-coords (make-coords))
    (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
    (setq default-pose (send *robot* :angle-vector))
    (format t "default-pose angle-vector~%")
    (print default-pose)
    ;; (setq default-pose (send *robot* :reset-pose))
    (format t "Move ZMP to rleg~%")
    ;; (read-line)
    (format t "Moving ZMP to rleg...~%")
    ;; (send *ri* :angle-vector (send *ri* :state :potentio-vector) 1000)
    (send *ri* :angle-vector default-pose 1000)
    (send *ri* :wait-interpolation)
    (unix:sleep 1)
    (send *ri* :stop-st)
    (send *ri* :stop-auto-balancer)
    (format t "Set abc and st~%")
    (format t "Setting abc and st...~%")
    (send *ri* :set-auto-balancer-param
          :leg-names '(:rleg)
          :transition-time 10.0
          :default-zmp-offsets (list #f(30 20 0) #f(30 -20 0))
          )
    (send *ri* :start-auto-balancer)
    (send *ri* :set-st-param
          :is-ik-enable (list t nil)
          :is-feedback-control-enable (list t t)
          :is-zmp-calc-enable (list t t)
          :eefm-use-force-difference-control nil)
    (send *ri* :start-st)
    (format t "Step On~%")
    (format t "Stepping On...~%")
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* :lleg :move-end-pos (float-vector 0 0 (+ target_z 40)))
      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* :lleg :move-end-pos (float-vector target_x target_y (+ target_z 40)))
      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* :lleg :move-end-pos (float-vector target_x target_y (+ target_z 20)))
      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
    (progn
      (send *ri* :set-reference-force-updater-param :lleg
            :motion-dir #f(0 0 1)
            :frame "world"
            :update-freq 200
            :p-gain 0.002
            )
      (send *ri* :start-reference-force-updater :lleg)
      )
    (format t "Watch Force~%")
    (format t "Watching Force...~%")
    (send *ri* :start-log)
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* :lleg :move-end-pos (float-vector target_x target_y (- target_z 10)))
      (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
      (send *ri* :wait-interpolation)
      )
    (unix:sleep 5)
    (setq num 0)
    (setq start-time (elt (unix:ptimes) 0))
    (while (< (- (elt (unix:ptimes) 0) start-time) (* 5 100))
      ;; (format t "num: ~A~%" num)
      (if (eq num 0)
          (setq force-mean (elt (send *ri* :state :force-vector :lleg) 2))
        (+ (/ force-mean (+ num 1) num)
           (elt (send *ri* :state :force-vector :lleg) 2)
           ))
      (setq num (+ num 1))
      )
    (format t "force z mean: ~A~%" (abs force-mean))
    (format t "Step Off~%")
    (format t "Stepping Off...~%")
    (send *ri* :set-reference-force-updater-param :lleg
          :p-gain 0.004
          )
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* :lleg :move-end-pos (float-vector target_x target_y (+ target_z 20)))
      (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
      (send *ri* :wait-interpolation)
      )
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* :lleg :move-end-pos (float-vector target_x target_y (+ target_z 40)))
      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
    (progn
      (send *ri* :set-ref-force (float-vector 0 0 0) 3000 :lleg)
      (unix:sleep 3)
      (send *ri* :stop-reference-force-updater :lleg)
      )
    (send *ri* :save-log "/tmp/foothold_check")
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* :lleg :move-end-pos (float-vector 0 0 (+ target_z 40)))
      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* :lleg :move-end-pos (float-vector 0 0 10))
      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
    (progn
      (send *robot* :angle-vector default-pose)
      ;; (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
      ;; (send *robot* :lleg :move-end-pos (float-vector 0 0 0))
      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
    (format t "Return ZMP to center of legs~%")
    (format t "Returning ZMP to center of legs...~%")
    (progn
      (send *ri* :stop-st)
      (send *ri* :stop-auto-balancer)
      (send *ri* :set-auto-balancer-param
            :leg-names '(:rleg :lleg)
            :transition-time 2.0
            :default-zmp-offsets (list #f(30 20 0) #f(30 -20 0))
            )
      (send *ri* :start-auto-balancer)
      (send *ri* :set-st-param
            :is-ik-enable (list t t)
            :is-feedback-control-enable (list t t)
            :is-zmp-calc-enable (list t t)
            :eefm-use-force-difference-control t)
      (send *ri* :start-st)
      )
    )
  force-mean
  )

(defun foothold-check-rleg(target_x target_y target_z)
  ;; (setq default-pose (copy-seq (send *ri* :state :potentio-vector)))
  (progn
    (send *robot* :angle-vector (send *ri* :state :potentio-vector))
    (send *robot* :fix-leg-to-coords (make-coords))
    (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
    (setq default-pose (send *robot* :angle-vector))
    ;; (setq default-pose (send *robot* :reset-pose))
    (format t "Move ZMP to lleg~%")
    (format t "Moving ZMP to lleg...~%")
    ;; (send *ri* :angle-vector (send *ri* :state :potentio-vector) 1000)
    (send *ri* :angle-vector default-pose 1000)
    (send *ri* :wait-interpolation)
    (unix:sleep 1)
    (send *ri* :stop-st)
    (send *ri* :stop-auto-balancer)
    (format t "Set abc and st~%")
    (format t "Setting abc and st...~%")
    (send *ri* :set-auto-balancer-param
          :leg-names '(:lleg)
          :transition-time 10.0
          :default-zmp-offsets (list #f(30 -20 0) #f(30 20 0))
          )
    (send *ri* :start-auto-balancer)
    (send *ri* :set-st-param
          :is-ik-enable (list nil t)
          :is-feedback-control-enable (list t t)
          :is-zmp-calc-enable (list t t)
          :eefm-use-force-difference-control nil)
    (send *ri* :start-st)
    (format t "Step On~%")
    (format t "Stepping On...~%")
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* :rleg :move-end-pos (float-vector 0 0 (+ target_z 40)))
      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* :rleg :move-end-pos (float-vector target_x target_y (+ target_z 40)))
      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* :rleg :move-end-pos (float-vector target_x target_y (+ target_z 20)))
      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
    (progn
      (send *ri* :set-reference-force-updater-param :rleg
            :motion-dir #f(0 0 1)
            :frame "world"
            :update-freq 200
            :p-gain 0.002
            )
      (send *ri* :start-reference-force-updater :rleg)
      )
    (format t "Watch Force~%")
    (format t "Watching Force...~%")
    (send *ri* :start-log)
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* :rleg :move-end-pos (float-vector target_x target_y (- target_z 10)))
      (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
      (send *ri* :wait-interpolation)
      )
    (unix:sleep 5)
    (setq num 0)
    (setq start-time (elt (unix:ptimes) 0))
    (while (< (- (elt (unix:ptimes) 0) start-time) (* 5 100))
      ;; (format t "num: ~A~%" num)
      (if (eq num 0)
          (setq force-mean (elt (send *ri* :state :force-vector :rleg) 2))
        (+ (/ force-mean (+ num 1) num)
           (elt (send *ri* :state :force-vector :rleg) 2)
           ))
      (setq num (+ num 1))
      )
    (format t "force z mean: ~A~%" (abs force-mean))
    (format t "Step Off~%")
    (format t "Stepping Off...~%")
    (send *ri* :set-reference-force-updater-param :rleg
          :p-gain 0.004
          )
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* :rleg :move-end-pos (float-vector target_x target_y (+ target_z 20)))
      (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
      (send *ri* :wait-interpolation)
      )
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* :rleg :move-end-pos (float-vector target_x target_y (+ target_z 40)))
      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
    (progn
      (send *ri* :set-ref-force (float-vector 0 0 0) 3000 :rleg)
      (unix:sleep 3)
      (send *ri* :stop-reference-force-updater :rleg)
      )
    (send *ri* :save-log "/tmp/foothold_check")
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* :rleg :move-end-pos (float-vector 0 0 (+ target_z 40)))
      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* :rleg :move-end-pos (float-vector 0 0 10))
      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
    (progn
      (send *robot* :angle-vector default-pose)
      ;; (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
      ;; (send *robot* :rleg :move-end-pos (float-vector 0 0 0))
      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
    (format t "Return ZMP to center of legs~%")
    (format t "Returning ZMP to center of legs...~%")
    (progn
      (send *ri* :stop-st)
      (send *ri* :stop-auto-balancer)
      (send *ri* :set-auto-balancer-param
            :leg-names '(:rleg :lleg)
            :transition-time 2.0
            :default-zmp-offsets (list #f(30 20 0) #f(30 -20 0))
            )
      (send *ri* :start-auto-balancer)
      (send *ri* :set-st-param
            :is-ik-enable (list t t)
            :is-feedback-control-enable (list t t)
            :is-zmp-calc-enable (list t t)
            :eefm-use-force-difference-control t)
      (send *ri* :start-st)
      )
    )
  force-mean
  )

(defun move-back()
  (progn
    (send *robot* :angle-vector default-pose)
    ;; (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
    ;; (send *robot* :lleg :move-end-pos (float-vector 0 0 0))
    (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    )
  (format t "Return ZMP to center of legs~%")
  (read-line)
  ;; (unix:sleep 3)
  (progn
    (send *ri* :stop-st)
    (send *ri* :stop-auto-balancer)
    (send *ri* :set-auto-balancer-param :leg-names '(:rleg :lleg) :transition-time 2.0)
    (send *ri* :start-auto-balancer)
    (send *ri* :set-st-param
          :is-ik-enable (list t t)
          :is-feedback-control-enable (list t t)
          :is-zmp-calc-enable (list t t)
          :eefm-use-force-difference-control t)
    (send *ri* :start-st)
    )
  )



;; (progn
;;   (send *ri* :set-ref-force #f(0 0 100) 1000 :lleg)
;;   (unix:sleep 1)
;;   (send *ri* :start-impedance :lleg :m-p 500 :d-p 10000 :k-p 1000)
;;   )

;; (send *ri* :set-ref-force #f(0 0 0) 1000 :lleg)
;; (send *ri* :stop-impedance :lleg)






;; (progn
;;   (send *ri* :set-reference-force-updater-param :lleg
;;         :motion-dir #f(0 0 1)
;;         :frame "world"
;;         :update-freq 200
;;         :p-gain 0.001
;;         )
;;   (send *ri* :start-reference-force-updater :lleg)
;;   )

;; (progn
;;   (send *ri* :set-ref-force (float-vector 0 0 0) 5000 :lleg)
;;   (unix:sleep 5)
;;   (send *ri* :stop-reference-force-updater :lleg)
;; )
