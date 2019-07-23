;; This buffer is for notes you don't want to save, and for Lisp evaluation.
;; If you want to create a file, visit that file with C-x C-f,
;; then enter the text in that file's own buffer.

;; (chidori-init)
;; (format t "chidori initialized~%")
;; (setq *robot* *chidori*)

(defun foothold-check (&key (target-x 0.0) (target-y 0.0) (target-z 0.0)
                            (fz-thresh 50) (initial-pz nil) (final-pz nil)
                            (step-on-p-gain 0.002) (step-off-p-gain 0.004)
                            (fix-limb :lleg) (move-limb :rleg))
  ;; set st param
  (progn
    (when (equal move-limb :lleg)
      (setq is-rleg-ik-enable t)
      (setq is-lleg-ik-enable nil)
      (setq fix-limb-default-zmp #f(30 20 0))
      )
    (when (equal move-limb :rleg)
      (setq is-rleg-ik-enable nil)
      (setq is-lleg-ik-enable t)
      (setq fix-limb-default-zmp #f(30 -20 0))
      )
    )

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
          :leg-names (list fix-limb)
          :transition-time 10.0
          :default-zmp-offsets (list fix-limb-default-zmp)
          )
    (send *ri* :start-auto-balancer)
    (send *ri* :set-st-param
          :is-ik-enable (list is-rleg-ik-enable is-lleg-ik-enable)
          :is-feedback-control-enable (list t t)
          :is-zmp-calc-enable (list t t)
          :eefm-use-force-difference-control nil)
    (send *ri* :start-st)
    (format t "Step On~%")
    (format t "Stepping On...~%")
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* move-limb :move-end-pos (float-vector 0 0 (+ target-z 40)))
      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* move-limb :move-end-pos (float-vector target-x target-y (+ target-z 40)))
      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* move-limb :move-end-pos (float-vector target-x target-y (+ target-z 20)))
      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
    (progn
      (send *ri* :set-reference-force-updater-param move-limb
            :motion-dir #f(0 0 1)
            :frame "world"
            :update-freq 200
            :p-gain step-on-p-gain
            )
      (send *ri* :start-reference-force-updater move-limb)
      )
    (format t "Watch Force~%")
    (format t "Watching Force...~%")
    (send *ri* :start-log)
    ;; stepping on
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* move-limb :move-end-pos (float-vector target-x target-y (- target-z 10)))
      (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
      ;; (send *ri* :wait-interpolation)
      )
    (setq start-time (elt (unix:ptimes) 0))
    (while (< (- (elt (unix:ptimes) 0) start-time) (* 5 100))
      (when (and (> (elt (send *ri* :state :off-force-vector move-limb) 2)
                    fz-thresh)
                 (not initial-pz))
        (setq initial-pz (elt (v- (send (send *robot* (read-from-string (format nil "~a-end-coords" move-limb))) :worldpos)
                                  (send (send *robot* (read-from-string (format nil "~a-end-coords" fix-limb))) :worldpos))
                              2))
        (format t "initial pos z: ~A [mm]~%" initial-pz)
        )
      )
    (if (not initial-pz)
        (progn
          (setq initial-pz (elt (v- (send (send *robot* (read-from-string (format nil "~a-end-coords" move-limb))) :worldpos)
                                    (send (send *robot* (read-from-string (format nil "~a-end-coords" fix-limb))) :worldpos))
                                2))
          (format t "initial pos z: ~A [mm]~%" initial-pz)
          )
      )
    ;; waiting
    (unix:sleep 5)
    ;; checking
    (setq num 0)
    (setq start-time (elt (unix:ptimes) 0))
    (while (< (- (elt (unix:ptimes) 0) start-time) (* 5 100))
      ;; (format t "num: ~A~%" num)
      (if (eq num 0)
          (progn
            (setq force-mean (elt (send *ri* :state :off-force-vector move-limb) 2))
            (setq pos-z-mean (elt (v- (send (send *robot* (read-from-string (format nil "~a-end-coords" move-limb))) :worldpos)
                                      (send (send *robot* (read-from-string (format nil "~a-end-coords" fix-limb))) :worldpos))
                                  2)))
        (progn
          (+ (/ force-mean (+ num 1) num)
             (elt (send *ri* :state :off-force-vector move-limb) 2))
          (+ (/ pos-z-mean (+ num 1) num)
             (elt (v- (send (send *robot* (read-from-string (format nil "~a-end-coords" move-limb))) :worldpos)
                      (send (send *robot* (read-from-string (format nil "~a-end-coords" fix-limb))) :worldpos))
                  2))
          ))
      (setq num (+ num 1))
      )
    (format t "force z mean: ~A [N]~%" (abs force-mean))
    (format t "pos z mean  : ~A [mm]~%" pos-z-mean)
    (format t "Step Off~%")
    (format t "Stepping Off...~%")
    (send *ri* :set-reference-force-updater-param move-limb
          :p-gain step-off-p-gain
          )
    ;; stepping off
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* move-limb :move-end-pos (float-vector target-x target-y (+ target-z 20)))
      (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
      ;; (send *ri* :wait-interpolation)
      )
    (setq start-time (elt (unix:ptimes) 0))
    (while (< (- (elt (unix:ptimes) 0) start-time) (* 5 100))
      (when (and (< (elt (send *ri* :state :off-force-vector move-limb) 2)
                    fz-thresh)
                 (not final-pz))
        (setq final-pz (elt (v- (send (send *robot* (read-from-string (format nil "~a-end-coords" move-limb))) :worldpos)
                                (send (send *robot* (read-from-string (format nil "~a-end-coords" fix-limb))) :worldpos))
                            2))
        (format t "final pos z: ~A [mm]~%" final-pz)
        )
      )
    (if (not final-pz)
        (progn
          (setq final-pz initial-pz)
          (format t "final pos z: ~A [mm]~%" final-pz)
          )
      )
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* move-limb :move-end-pos (float-vector target-x target-y (+ target-z 40)))
      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
    (progn
      (send *ri* :set-ref-force (float-vector 0 0 0) 3000 move-limb)
      (unix:sleep 3)
      (send *ri* :stop-reference-force-updater move-limb)
      )
    (send *ri* :save-log "/tmp/foothold_check")
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* move-limb :move-end-pos (float-vector 0 0 (+ target-z 40)))
      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
    (progn
      (send *robot* :angle-vector default-pose)
      (send *robot* move-limb :move-end-pos (float-vector 0 0 10))
      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
    (progn
      (send *robot* :angle-vector default-pose)
      ;; (send *robot* :move-centroid-on-foot :both '(move-limb fix-limb))
      ;; (send *robot* move-limb :move-end-pos (float-vector 0 0 0))
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

  ;; calc stiffness
  (setq pz (* (- (* 0.5 (+ initial-pz final-pz))
                 pos-z-mean)
              0.001))
  (setq stiffness (/ (abs force-mean) (abs pz)))
  (format t "stiffness : ~A [N/m]~%" stiffness)

  ;; force-mean
  stiffness
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
