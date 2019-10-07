#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "safe_footstep_planner")
(ros::roseus-add-msgs "hrpsys_ros_bridge")

(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(require "package://eus_qp/optmotiongen/euslisp/inverse-kinematics-statics-wrapper.l")

(unless (boundp '*jaxon_red*)
  (jaxon_red-init)
  ;; (jaxon_red)
  (setq *robot* *jaxon_red*)
  (objects (list *robot*))
  )

;; support foot relative
(defun target-callback
    (msg)
  (setq *landing-pos* (scale 1e3 (float-vector (send msg :x) (send msg :y) (send msg :z))))
  (setq *l_r* (send msg :l_r))
  )

;; support foot relative
(defun end-cog-callback
    (msg)
  (setq *end-cog-pos* (scale 1e3 (float-vector (send msg :x) (send msg :y) (send msg :z))))
  (setq *end-cog-vel* (scale 1e3 (float-vector (send msg :vx) (send msg :vy) (send msg :vz))))
  )

;; BODY relative
(defun wall-callback
    (msg)
  (let* ((max-idx 0)
         (max-norm 0)
         (box-pos)
         (box-q)
         (box-size)
         (root-rot)
         (box-pos-off #f(0 0 0))
         )
    (when (> (length (send msg :boxes)) 0)
      (dotimes (i (length (send msg :boxes)))
        (let* ((tmp-size (send (elt (send msg :boxes) i) :dimensions))
               (tmp-norm (norm (float-vector (send tmp-size :x) (send tmp-size :y) (send tmp-size :z))))
               (min-length 50) ;; {mm}
               )
          (when (> tmp-norm max-norm)
            (when (or
                   (and (> (send tmp-size :x) min-length) (> (send tmp-size :y) min-length))
                   (and (> (send tmp-size :x) min-length) (> (send tmp-size :z) min-length))
                   (and (> (send tmp-size :y) min-length) (> (send tmp-size :z) min-length))
                   )
              (setq max-idx i)
              (setq max-norm tmp-norm))
            )))
      (setq box-pos (send (elt (send msg :boxes) max-idx) :pose :position))
      (setq box-q (send (elt (send msg :boxes) max-idx) :pose :orientation))
      (setq box-size (send (elt (send msg :boxes) max-idx) :dimensions))
      (setq *box-pos* (scale 1000 (float-vector (send box-pos :x) (send box-pos :y) (send box-pos :z) )))
      (setq *box-pos* (v+ *box-pos* box-pos-off))
      (setq *box-rot* (quaternion2matrix (float-vector (send box-q :x) (send box-q :y) (send box-q :z) (send box-q :w))))
      ;; (setq *box-size* (scale 1000 (float-vector (send box-size :x) (send box-size :y) (send box-size :z))))
      (setq *box-size* (scale 1000 (float-vector (send box-size :z) (send box-size :y) (send box-size :x))))
      ;; (warn  "box-pos: ~A~%" *box-pos*)
      ;; (warn  "box-rot: ~A~%" *box-rot*)
      ;; (warn  "box-size: ~A~%" *box-size*)
      (setq *board* (make-cube (elt *box-size* 0) (elt *box-size* 1) (elt *box-size* 2)))
      (setq *box-rot* (rotate-matrix *box-rot* (deg2rad -90) :y t))
      (unless *is-stopping* (send *robot* :angle-vector (send *ri* :potentio-vector)))
      (send *robot* :fix-leg-to-coords (make-coords))
      (setq root-rot (send (send (send *robot* :torso-root-link) :copy-worldcoords) :rot))
      (setq *box-rot* (m* (transpose root-rot) *box-rot*))
      (send *board* :newcoords
            (make-coords :pos *box-pos*
                         :rot *box-rot*
                         ))
      (setq *face* (elt (send *board* :faces) 1))
      (send *face* :put :margin 20) ;; [mm]
      (send *face* :put :normal-move-target-axis :z)
      (send *face* :put :normal-tolerance 60.0)
      )))

(defun setup-wall
    ()
  (setq *board* (make-cube 1 1 1)) ;; dummy
  (objects (list *robot* *board*))
  (send *robot* :rarm :collar-y :max-angle 30)
  (send *robot* :larm :collar-y :min-angle -30)
  (send *robot* :put :larm-contact-coords
        (make-cascoords :parent (send (send *robot* :larm :end-coords) :parent)
                        :coords (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :rotate -pi/2 :y)))
  (send *robot* :put :rarm-contact-coords
        (make-cascoords :parent (send (send *robot* :rarm :end-coords) :parent)
                        :coords (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :rotate -pi/2 :y)))
  (create-support-polygon *robot* (send (send *robot* :rleg :end-coords) :parent))
  (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))
  (setq *landing-pos* (float-vector 0 0 0)) ;; dummy
  (setq *l_r* 0) ;; dummy
  )

(defun emergency-touch-wall
    ()
  (warn "cog a~%")
  (print *cog-pos*)
  (warn "cog b~%")
  (setq *cog-pos* (float-vector 200 0 (elt (send (send *robot* :rleg :end-coords) :worldpos) 2)))
  (setq *contact-list*
        (list (generate-contact-ik-arg-from-rect-face :rect-face *face* :name "larm")
              (generate-contact-ik-arg-from-rect-face :rect-face *face* :name "rarm")
              ))
  (setq *robot-env*
        (instance robot-environment :init :robot *robot* :contact-list *contact-list* :root-virtual-mode :6dof))
  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list nil
                  :only-kinematics? nil
                  :kin-target-coords-list
                  (list
                   (send (send (send *robot* :lleg :end-coords) :copy-worldcoords) :translate *lleg-trans-pos*)
                   (send (send (send *robot* :rleg :end-coords) :copy-worldcoords) :translate *rleg-trans-pos*)
                   (send *robot-env* :contact "larm" :contact-coords)
                   (send *robot-env* :contact "rarm" :contact-coords)
                   (make-coords :pos *cog-pos*)
                   )
                  :kin-attention-coords-list
                  (list
                   (send *robot* :lleg :end-coords)
                   (send *robot* :rleg :end-coords)
                   (send *robot* :get :larm-contact-coords)
                   (send *robot* :get :rarm-contact-coords)
                   :cog
                   )
                  :kin-scale-list
                  (list
                   1
                   1
                   (list 1 1 1 0 0 0)
                   (list 1 1 1 0 0 0)
                   (list 1 0 0 0 0 0) ;; only x
                   )
                  :contact-target-coords-list
                  (list
                   (if (equal *l_r* 0)
                       (send (send (send *robot* :lleg :end-coords) :copy-worldcoords) :translate *lleg-trans-pos*)
                     (send (send (send *robot* :rleg :end-coords) :copy-worldcoords) :translate *rleg-trans-pos*))
                   (send *robot-env* :contact "larm" :contact-coords)
                   (send *robot-env* :contact "rarm" :contact-coords)
                   )
                  :contact-attention-coords-list
                  (list
                   (if (equal *l_r* 0)
                       (send *robot* :lleg :end-coords)
                     (send *robot* :rleg :end-coords))
                   (send *robot* :get :larm-contact-coords)
                   (send *robot* :get :rarm-contact-coords)
                   )
                  :contact-constraint-list
                  (list
                   (instance default-contact-constraint
                             :init-from-support-polygon
                             :support-polygon (send (send (send *robot* (if (equal *l_r* 0) :lleg :rleg) :end-coords) :parent) :get :support-polygon)
                             :support-polygon-coords (send (send *robot* (if (equal *l_r* 0) :lleg :rleg) :end-coords) :worldcoords)
                             :mu-trans 0.1
                             )
                   (instance default-contact-constraint
                             :init-from-support-polygon :support-polygon nil
                             ;; :mu-trans 0.01
                             )
                   (instance default-contact-constraint
                             :init-from-support-polygon :support-polygon nil
                             ;; :mu-trans 0.01
                             )
                   )
                  ))
  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-config-task*
                  ;; :no-visualize? t
                  :no-print? t
                  ))
  (send *sqp-opt* :optimize :loop-num 10)
  (send *robot-env* :dissoc-root-virtual)
  )

(defun calc-cog-pos
    (calc-tm) ;; ik calculattion time + touch wall motion time
  (let* ((next-sup-leg-pos)
         (next-swg-leg-pos)
         (end-cog-pos) ;; world (root-link) frame
         (omega (sqrt (/ 9.80665 (elt *end-cog-pos* 2))))
         (sup-off #f(150 0 0))
         )
    (setq next-sup-leg-pos
          (send
           (if (equal *l_r* 0)
               (send (send (send *robot* :lleg :end-coords) :copy-worldcoords) :translate *lleg-trans-pos*)
             (send (send (send *robot* :rleg :end-coords) :copy-worldcoords) :translate *rleg-trans-pos*))
           :worldpos))
    (setq next-sup-leg-pos (v+ next-sup-leg-pos sup-off))
    (setq next-swg-leg-pos
          (send
           (if (equal *l_r* 0)
               (send (send (send *robot* :rleg :end-coords) :copy-worldcoords) :translate *rleg-trans-pos*)
             (send (send (send *robot* :lleg :end-coords) :copy-worldcoords) :translate *lleg-trans-pos*))
           :worldpos))
    (setq end-cog-pos (v+ *end-cog-pos* next-swg-leg-pos))
    (setq *cog-pos*
          (v+
           (v+
            (scale (cosh (* omega calc-tm)) (v- end-cog-pos next-sup-leg-pos))
            (scale (/ (sinh (* omega calc-tm)) omega) *end-cog-vel*))
           next-sup-leg-pos))

    ;; debug
    (setq *sup* next-sup-leg-pos)
    (setq *swg* next-swg-leg-pos)
    (setq *omega* omega)
    (setq *vel* *end-cog-vel*)
    (setq *cog* end-cog-pos)
    (setq *cog-orig* *end-cog-pos*)
    (setq *lleg-trans* *lleg-trans-pos*)
    (setq *rleg-trans* *rleg-trans-pos*)
    (setq *lr* *l_r*)
    (setq *lleg* (send (send *robot* :lleg :end-coords) :copy-worldcoords))
    (setq *rleg* (send (send *robot* :rleg :end-coords) :copy-worldcoords))
    ;; (setq *cog-pos*
    ;;       (v+
    ;;        (v+
    ;;         (scale (cosh (* *omega* calc-tm)) (v- *cog* *sup*))
    ;;         (scale (/ (sinh (* *omega* calc-tm)) *omega*) *vel*))
    ;;        *sup*))

    (setq *emergency-cog* *cog-pos*)
    )
  )

(defun move-robot
    ()
  (let* ((root-pos))
    ;; (when (boundp '*axis*)
    ;;   (send *axis* :dissoc *robot*))
    (send *robot* :reset-pose)
    (send *robot* :fix-leg-to-coords (make-coords))
    (setq root-pos (scale -1 (send (send (send *robot* :torso-root-link) :copy-worldcoords) :pos)))
    (send *robot* :newcoords (make-coords :pos root-pos))
    ;; (setq *axis* (make-cascoords :pos root-pos))
    ;; (send *axis* :assoc *robot*)
    ;; (setq *tmp-angle* 0)
    ;; (send *axis* :rotate (deg2rad *tmp-angle*) :y)
    ;; (send *robot* :legs :ankle-p :joint-angle (- (send *robot* :rleg :ankle-p :joint-angle) *tmp-angle*))
    )
  ;; move swing foot to landing pos
  (setq *rleg-trans-pos* #f(0 0 0))
  (setq *lleg-trans-pos* #f(0 0 0))
  (if (equal *l_r* 0)
      (progn ;; lleg is swing
        (setq *lleg-trans-pos*
              (v-
               (v+ (send *robot* :rleg :end-coords :worldpos) *landing-pos*)
               (send *robot* :lleg :end-coords :worldpos)
               ))
        )
    (progn ;; rleg is swing
      (setq *rleg-trans-pos*
            (v-
             (v+ (send *robot* :lleg :end-coords :worldpos) *landing-pos*)
             (send *robot* :rleg :end-coords :worldpos)
             ))
      )
    )
  (calc-cog-pos 0.1)
  ;; (setq *cog-pos* (float-vector 200 0 (elt (send (send *robot* :rleg :end-coords) :worldpos) 2)))
  ;; solve IK
  (emergency-touch-wall)
  ;; send angle-vector
  (send *ri* :emergencystopperservice_setEmergencyJointAngles :angles (send *robot* :angle-vector) :solved t)
  (setq *emergency-av* (send *robot* :angle-vector))
  )

(defun reset-footstep
    ()
  (let* ((off (* 1000 (elt (send (send (send *ri* :get-gait-generator-param) :leg_default_translate_pos) :data) 4))))
    (send *ri* :set-foot-steps
          (list (make-coords :coords (send *robot* :rleg :end-coords :copy-worldcoords) :name :rleg)
                (make-coords :coords (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 0 200 0)) :rotate (deg2rad 0) :y) :name :lleg)
                (make-coords :coords (send *robot* :rleg :end-coords :copy-worldcoords) :name :rleg)
                ))
    ))

(defun wo-touch-in-double
    ()
  (setq *contact-list*
        (list (instance 2d-planar-contact :init :name "swing_leg" :fix-yaw? nil)
              ))
  (setq *robot-env*
        (instance robot-environment :init :robot *robot* :contact-list *contact-list* :root-virtual-mode :6dof))
  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list nil
                  :only-kinematics? nil
                  :kin-target-coords-list
                  (list
                   (if (equal *emergency-l_r* 0)
                       (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                     (send *robot-env* :contact "swing_leg" :contact-coords))
                   (if (equal *emergency-l_r* 0)
                       (send *robot-env* :contact "swing_leg" :contact-coords)
                     (send (send *robot* :rleg :end-coords) :copy-worldcoords))
                   (send (send *robot* :larm :end-coords) :copy-worldcoords)
                   (send (send *robot* :rarm :end-coords) :copy-worldcoords)
                   (make-coords :pos *cog-pos*)
                   )
                  :kin-attention-coords-list
                  (list
                   (send *robot* :lleg :end-coords)
                   (send *robot* :rleg :end-coords)
                   (send *robot* :get :larm-contact-coords)
                   (send *robot* :get :rarm-contact-coords)
                   :cog
                   )
                  :kin-scale-list
                  (list
                   1
                   1
                   (list 1 1 1 0 0 0)
                   (list 1 1 1 0 0 0)
                   (list 1 1 0 0 0 0)
                   )
                  :contact-target-coords-list
                  (list
                   (if (equal *emergency-l_r* 0)
                       (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                     (send *robot-env* :contact "swing_leg" :contact-coords))
                   (if (equal *emergency-l_r* 0)
                       (send *robot-env* :contact "swing_leg" :contact-coords)
                     (send (send *robot* :rleg :end-coords) :copy-worldcoords))
                   )
                  :contact-attention-coords-list
                  (list
                   (send *robot* :lleg :end-coords)
                   (send *robot* :rleg :end-coords)
                   )
                  :contact-constraint-list
                  (list
                   (instance default-contact-constraint
                             :init-from-support-polygon
                             :support-polygon (send (send (send *robot* :lleg :end-coords) :parent) :get :support-polygon)
                             :support-polygon-coords (send (send *robot* :lleg :end-coords) :worldcoords)
                             )
                   (instance default-contact-constraint
                             :init-from-support-polygon
                             :support-polygon (send (send (send *robot* :rleg :end-coords) :parent) :get :support-polygon)
                             :support-polygon-coords (send (send *robot* :rleg :end-coords) :worldcoords)
                             )
                   )
                  ))
  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-config-task*
                  ))
  (send *sqp-opt* :optimize :loop-num 50)
  (send *robot-env* :dissoc-root-virtual)
  )

(defun w-touch-in-tri
    ()
  (setq *contact-list*
        (list (instance 2d-planar-contact :init :name "rleg" :fix-yaw? nil)
              ))
  (setq *robot-env*
        (instance robot-environment :init :robot *robot* :contact-list *contact-list* :root-virtual-mode :6dof))
  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list nil
                  :only-kinematics? nil
                  :kin-target-coords-list
                  (list
                   (send (send (send *robot* :lleg :end-coords) :copy-worldcoords) :translate *lleg-trans-height-off*)
                   (send (send (send *robot* :rleg :end-coords) :copy-worldcoords) :translate *rleg-trans-height-off*)
                   (send (send *robot* :get :larm-contact-coords) :copy-worldcoords)
                   (send (send *robot* :get :rarm-contact-coords) :copy-worldcoords)
                   (make-coords :pos *cog-pos*)
                   )
                  :kin-attention-coords-list
                  (list
                   (send *robot* :lleg :end-coords)
                   (send *robot* :rleg :end-coords)
                   (send *robot* :get :larm-contact-coords)
                   (send *robot* :get :rarm-contact-coords)
                   :cog
                   )
                  :kin-scale-list
                  (list
                   1
                   1
                   (list 1 1 1 0 0 0)
                   (list 1 1 1 0 0 0)
                   (list 1 1 0 0 0 0)
                   )
                  :contact-target-coords-list
                  (list
                   (send (send *robot* (if (equal *emergency-l_r* 0) :lleg :rleg) :end-coords) :copy-worldcoords)
                   (send (send *robot* :get :larm-contact-coords) :copy-worldcoords)
                   (send (send *robot* :get :rarm-contact-coords) :copy-worldcoords)
                   )
                  :contact-attention-coords-list
                  (list
                   (if (equal *emergency-l_r* 0)
                       (send *robot* :lleg :end-coords)
                     (send *robot* :rleg :end-coords))
                   (send *robot* :get :larm-contact-coords)
                   (send *robot* :get :rarm-contact-coords)
                   )
                  :contact-constraint-list
                  (list
                   (instance default-contact-constraint
                             :init-from-support-polygon
                             :support-polygon (send (send (send *robot* (if (equal *l_r* 0) :lleg :rleg) :end-coords) :parent) :get :support-polygon)
                             :support-polygon-coords (send (send *robot* (if (equal *l_r* 0) :lleg :rleg) :end-coords) :worldcoords)
                             :mu-trans 0.2
                             )
                   (instance default-contact-constraint
                             :init-from-support-polygon :support-polygon nil
                             ;; :mu-trans 0.01
                             )
                   (instance default-contact-constraint
                             :init-from-support-polygon :support-polygon nil
                             ;; :mu-trans 0.01
                             )
                   )
                  ))
  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-config-task*
                  ))
  (send *sqp-opt* :optimize :loop-num 50)
  (send *robot-env* :dissoc-root-virtual)
  )

(defun w-touch-in-tri-swing
    ()
  (setq *contact-list*
        (list (instance 2d-planar-contact :init :name "rleg" :fix-yaw? nil)
              ))
  (setq *robot-env*
        (instance robot-environment :init :robot *robot* :contact-list *contact-list* :root-virtual-mode :6dof))
  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list nil
                  :only-kinematics? nil
                  :kin-target-coords-list
                  (list
                   (send (send (send *robot* :lleg :end-coords) :copy-worldcoords) :translate *lleg-trans-height*)
                   (send (send (send *robot* :rleg :end-coords) :copy-worldcoords) :translate *rleg-trans-height*)
                   (send (send *robot* :get :larm-contact-coords) :copy-worldcoords)
                   (send (send *robot* :get :rarm-contact-coords) :copy-worldcoords)
                   (make-coords :pos *cog-pos*)
                   )
                  :kin-attention-coords-list
                  (list
                   (send *robot* :lleg :end-coords)
                   (send *robot* :rleg :end-coords)
                   (send *robot* :get :larm-contact-coords)
                   (send *robot* :get :rarm-contact-coords)
                   :cog
                   )
                  :kin-scale-list
                  (list
                   1
                   1
                   (list 1 1 1 0 0 0)
                   (list 1 1 1 0 0 0)
                   (list 1 1 0 0 0 0)
                   )
                  :contact-target-coords-list
                  (list
                   (send (send *robot* (if (equal *emergency-l_r* 0) :lleg :rleg) :end-coords) :copy-worldcoords)
                   (send (send *robot* :get :larm-contact-coords) :copy-worldcoords)
                   (send (send *robot* :get :rarm-contact-coords) :copy-worldcoords)
                   )
                  :contact-attention-coords-list
                  (list
                   (if (equal *emergency-l_r* 0)
                       (send *robot* :lleg :end-coords)
                     (send *robot* :rleg :end-coords))
                   (send *robot* :get :larm-contact-coords)
                   (send *robot* :get :rarm-contact-coords)
                   )
                  :contact-constraint-list
                  (list
                   (instance default-contact-constraint
                             :init-from-support-polygon
                             :support-polygon (send (send (send *robot* (if (equal *l_r* 0) :lleg :rleg) :end-coords) :parent) :get :support-polygon)
                             :support-polygon-coords (send (send *robot* (if (equal *l_r* 0) :lleg :rleg) :end-coords) :worldcoords)
                             :mu-trans 0.2
                             )
                   (instance default-contact-constraint
                             :init-from-support-polygon :support-polygon nil
                             ;; :mu-trans 0.01
                             )
                   (instance default-contact-constraint
                             :init-from-support-polygon :support-polygon nil
                             ;; :mu-trans 0.01
                             )
                   )
                  ))
  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-config-task*
                  ))
  (send *sqp-opt* :optimize :loop-num 50)
  (send *robot-env* :dissoc-root-virtual)
  )

(defun wo-touch-manip
    ()
  (setq *contact-list*
        (list (instance 2d-planar-contact :init :name "rleg" :fix-yaw? nil)
              ))
  (setq *robot-env*
        (instance robot-environment :init :robot *robot* :contact-list *contact-list* :root-virtual-mode :6dof))
  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list nil
                  :only-kinematics? nil
                  :kin-target-coords-list
                  (list
                   (send lleg-coords :copy-worldcoords)
                   (send rleg-coords :copy-worldcoords)
                   (send (send *robot* :get :larm-contact-coords) :copy-worldcoords)
                   (send (send *robot* :get :rarm-contact-coords) :copy-worldcoords)
                   (make-coords :pos (float-vector 0 0 0))
                   )
                  :kin-attention-coords-list
                  (list
                   (send *robot* :lleg :end-coords)
                   (send *robot* :rleg :end-coords)
                   (send *robot* :get :larm-contact-coords)
                   (send *robot* :get :rarm-contact-coords)
                   :cog
                   )
                  :kin-scale-list
                  (list
                   1
                   1
                   (list 1 1 1 0 0 0)
                   (list 1 1 1 0 0 0)
                   (list 1 1 0 0 0 0)
                   )
                  :contact-target-coords-list
                  (list
                   (send lleg-coords :copy-worldcoords)
                   (send rleg-coords :copy-worldcoords)
                   )
                  :contact-attention-coords-list
                  (list
                   (send *robot* :lleg :end-coords)
                   (send *robot* :rleg :end-coords)
                   )
                  :contact-constraint-list
                  (list
                   (instance default-contact-constraint
                             :init-from-support-polygon
                             :support-polygon (send (send (send *robot* :lleg :end-coords) :parent) :get :support-polygon)
                             :support-polygon-coords (send (send *robot* :lleg :end-coords) :worldcoords)
                             )
                   (instance default-contact-constraint
                             :init-from-support-polygon
                             :support-polygon (send (send (send *robot* :rleg :end-coords) :parent) :get :support-polygon)
                             :support-polygon-coords (send (send *robot* :rleg :end-coords) :worldcoords)
                             )
                   )
                  ))
  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-config-task*
                  ))
  (send *sqp-opt* :optimize :loop-num 50)
  (send *robot-env* :dissoc-root-virtual)
  )

(defun release-robot
    ()
  (let* ((rleg-pos)
         (lleg-pos))
    (send *ri* :set-auto-balancer-param :is-emergency-step-mode nil)
    (send *ri* :go-stop)
    (send *robot* :angle-vector *emergency-av*)
    (send *robot* :fix-leg-to-coords (make-coords) (if (equal *emergency-l_r* 0) :lleg :rleg))
    (send *ri* :angle-vector *emergency-av* 1000)
    (send *ri* :wait-interpolation)
    (send *ri* :emergency-release-motion)
    (send *ri* :autobalancerservice_releaseemergencystop)
    (setq *cog-pos* (send *robot* :centroid))
    (setq *cog-pos* (float-vector (elt *cog-pos* 0) (+ (elt *cog-pos* 1) (if (equal *emergency-l_r* 0) -100 100)) 0))
    (wo-touch-in-double)
    (setq *av3* (send *robot* :angle-vector))
    (send *robot* :fix-leg-to-coords (make-coords))
    (setq *cog-pos* (send *robot* :centroid))
    (setq *cog-pos* (float-vector (elt *cog-pos* 0) (elt *cog-pos* 1) 0))
    (setq rleg-coords (send (send *robot* :rleg :end-coords) :copy-worldcoords))
    (setq lleg-coords (send (send *robot* :lleg :end-coords) :copy-worldcoords))
    (setq *rleg-trans-height* #f(0 0 0))
    (setq *lleg-trans-height* #f(0 0 0))
    (setq *rleg-trans-height-off* #f(0 0 0))
    (setq *lleg-trans-height-off* #f(0 0 0))
    (if (equal *emergency-l_r* 0)
        (setq *rleg-trans-height* #f(0 0 30))
      (setq *lleg-trans-height* #f(0 0 30)))
    (if (equal *emergency-l_r* 0)
        (setq *rleg-trans-height-off* #f(0 0 25))
      (setq *lleg-trans-height-off* #f(0 0 25)))
    (w-touch-in-tri)
    (setq *av2* (send *robot* :angle-vector))
    (send *robot* :angle-vector *av3*)
    (send *robot* :fix-leg-to-coords (make-coords))
    (w-touch-in-tri-swing)
    (setq *av1* (send *robot* :angle-vector))
    (send *robot* :reset-manip-pose)
    (send *robot* :fix-leg-to-coords (make-coords))
    (wo-touch-manip)
    (setq *av4* (send *robot* :angle-vector))
    ;; (send *ri* :set-ref-force #f(-200 0 0) 5000 :arms)
    (send *ri* :angle-vector-sequence (list *av1* *av2* *av3* *av4*) (list 5000 3000 3000 3000))
    (send *ri* :wait-interpolation)
    (send *ri* :start-auto-balancer)
    (send *ri* :autobalancerservice_startstabilizer)
    (reset-footstep)
    ))

(defun start-touch-wall
    ()
  (setq *is-stopping* nil)
  (do-until-key
   (x::window-main-one)
   (send *ri* :state)
   (when (and (equal (send *ri* :emergency-mode) 2) (not *is-stopping*))
     (move-robot)
     (setq *is-stopping* t)
     (setq *emergency-face* *face*)
     (setq *emergency-board* *board*)
     (setq *emergency-l_r* *l_r*)
     (send *ri* :angle-vector *emergency-av* 1000)
     )
   (when (and *is-stopping* (equal (send *ri* :get-auto-balancer-controller-mode) :mode_idle))
     (objects (list *robot*))
     (release-robot)
     )
   (unless *is-stopping*
     (progn
       (let* ((root-pos))
         (send *robot* :reset-pose)
         (send *robot* :fix-leg-to-coords (make-coords))
         (setq root-pos (scale -1 (send (send (send *robot* :torso-root-link) :copy-worldcoords) :pos)))
         (send *robot* :newcoords (make-coords :pos root-pos))
         )
       (objects (list *robot* *board*))))
   (ros::spin-once))
  )

(ros::roseus "touch_wall")
;; (ros::subscribe "touchable_wall" jsk_recognition_msgs::BoundingBoxArray #'wall-callback 1)
;; (ros::subscribe "/valve_recognition/cluster_decomposer/boxes" jsk_recognition_msgs::BoundingBoxArray #'wall-callback 1)
(ros::subscribe "/valve_recognition/walls/output" jsk_recognition_msgs::BoundingBoxArray #'wall-callback 10)
(ros::subscribe "/landing_target" safe_footstep_planner::OnlineFootStep #'target-callback 1)
(ros::subscribe "/end_cog_state" hrpsys_ros_bridge::CogState #'end-cog-callback 1)
(setup-wall)

(format t "start controller~%")
(start-touch-wall)

(format t "stop controller~%")

