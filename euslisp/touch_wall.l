#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "safe_footstep_planner")

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

;; BODY relative
(defun wall-callback
    (msg)
  (let* ((max-idx 0)
         (max-norm 0)
         (box-pos)
         (box-q)
         (box-size)
         (root-rot)
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
      (send *face* :put :normal-tolerance 10.0)
      )))

(defun setup-wall
    ()
  (send *ri* :set-emergency-stopper-param :default-retrieve-time 0.5)
  (setq *board* (make-cube 1 1 1)) ;; dummy
  (objects (list *robot* *board*))
  (send *robot* :rarm :collar-y :max-angle 30)
  (send *robot* :larm :collar-y :min-angle -30)
  (setq *landing-pos* (float-vector 0 0 0)) ;; dummy
  (setq *l_r* 0) ;; dummy
  )

(defun move-robot
    ()
  (let* ((root-pos))
    (when (boundp '*axis*)
      (send *axis* :dissoc *robot*))
    (send *robot* :reset-pose)
    (send *robot* :fix-leg-to-coords (make-coords))
    (setq root-pos (scale -1 (send (send (send *robot* :torso-root-link) :copy-worldcoords) :pos)))
    (send *robot* :newcoords (make-coords :pos root-pos))
    (setq *axis* (make-cascoords :pos root-pos))
    (send *axis* :assoc *robot*)
    (setq *tmp-angle* 10)
    (send *axis* :rotate (deg2rad *tmp-angle*) :y)
    (send *robot* :legs :ankle-p :joint-angle (- (send *robot* :rleg :ankle-p :joint-angle) *tmp-angle*))
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
  ;; solve IK
  (send *robot*
        :inverse-kinematics-statics-optmotiongen
        (list
         (send (send (send *robot* :lleg :end-coords) :copy-worldcoords) :translate *lleg-trans-pos*)
         (send (send (send *robot* :rleg :end-coords) :copy-worldcoords) :translate *rleg-trans-pos*)
         *face*
         *face*
         )
        :move-target
        (list
         (send *robot* :lleg :end-coords)
         (send *robot* :rleg :end-coords)
         (send *robot* :larm :end-coords)
         (send *robot* :rarm :end-coords)
         )
        :contact-target-coords
        (list
         (if (equal *l_r* 0)
             (send (send (send *robot* :lleg :end-coords) :copy-worldcoords) :translate *lleg-trans-pos*)
           (send (send (send *robot* :rleg :end-coords) :copy-worldcoords) :translate *rleg-trans-pos*))
         *face*
         *face*
         )
        :contact-move-target
        (list
         (if (equal *l_r* 0)
             (send *robot* :lleg :end-coords)
           (send *robot* :rleg :end-coords))
         (send *robot* :larm :end-coords)
         (send *robot* :rarm :end-coords)
              )
        :contact-constraint
        (list
         (instance default-contact-constraint
                   :init-from-support-polygon :support-polygon nil)
         (instance default-contact-constraint
                   :init-from-support-polygon :support-polygon nil)
         (instance default-contact-constraint
                   :init-from-support-polygon :support-polygon nil)
         )
        ;; :collision-avoidance-link-pair
        ;; (list (list (send *robot* :link "RARM_LINK7") (send *robot* :link "LARM_LINK7")))
        ;; :collision-distance-limit 200 ;; [mm]
        ;; :posture-joint-list
        ;; (send *robot* :joint-list)
        ;; :posture-joint-angle-list
        ;; (send-all (send *robot* :joint-list) :joint-angle)
        :translation-axis (list t t t t)
        :rotation-axis (list t t nil nil)
        :debug-view t
        :root-virtual-mode :6dof
        :optimize-torque? nil
        :revert-if-fail nil
        :stop 10
        )

  ;; send angle-vector
  (send *ri* :emergencystopperservice_setEmergencyJointAngles :angles (send *robot* :angle-vector) :solved t)
  )

(defun start-touch-wall
    ()
  (setq *is-stopping* nil)
  (do-until-key
   (x::window-main-one)
   (send *ri* :state)
   (when (and (equal (send *ri* :emergency-mode) 2) (not *is-stopping*))
     (move-robot)
     (setq *is-stopping* t)
     )
   (unless *is-stopping* (objects (list *robot* *board*)))
   (ros::spin-once))
  )

(ros::roseus "touch_wall")
;; (ros::subscribe "touchable_wall" jsk_recognition_msgs::BoundingBoxArray #'wall-callback 1)
;; (ros::subscribe "/valve_recognition/cluster_decomposer/boxes" jsk_recognition_msgs::BoundingBoxArray #'wall-callback 1)
(ros::subscribe "/valve_recognition/walls/output" jsk_recognition_msgs::BoundingBoxArray #'wall-callback 1)
(ros::subscribe "/landing_target" safe_footstep_planner::OnlineFootStep #'target-callback 1)
(setup-wall)

(format t "start controller~%")
(start-touch-wall)

(format t "stop controller~%")

