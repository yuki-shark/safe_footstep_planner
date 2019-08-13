#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_recognition_msgs")

(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(require "package://eus_qp/optmotiongen/euslisp/inverse-kinematics-statics-wrapper.l")

(unless (boundp '*jaxon_red*)
  (jaxon_red-init)
  ;; (jaxon_red)
  (setq *robot* *jaxon_red*)
  (objects (list *robot*))
  )

;; BODY relative
(defun wall-callback
    (msg)
  (let* ((max-idx 0)
         (max-norm 0)
         (box-pos)
         (box-q)
         (box-size)
         )
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
    ;; (setq *box-pos* (scale 1000 (float-vector (- (send box-pos :x) (/ (send box-size :x) 2)) (- (send box-pos :y) (/ (send box-size :x) 2)) (- (send box-pos :x) (/ (send box-size :x) 2)))))
    (setq *box-pos* (scale 1000 (float-vector (send box-pos :x) (send box-pos :y) (send box-pos :z) )))
    (setq *box-rot* (quaternion2matrix (float-vector (send box-q :x) (send box-q :y) (send box-q :z) (send box-q :w))))
    ;; (setq *box-size* (scale 1000 (float-vector (send box-size :x) (send box-size :y) (send box-size :z))))
    (setq *box-size* (scale 1000 (float-vector (send box-size :z) (send box-size :y) (send box-size :x))))
    ;; (warn  "box-pos: ~A~%" *box-pos*)
    ;; (warn  "box-rot: ~A~%" *box-rot*)
    ;; (warn  "box-size: ~A~%" *box-size*)
    (setq *board* (make-cube (elt *box-size* 0) (elt *box-size* 1) (elt *box-size* 2)))
    (send *board* :newcoords
          (make-coords :pos *box-pos*
                       :rot *box-rot*))
    (send *board* :rotate (deg2rad -90) :y)

    (setq *face* (elt (send *board* :faces) 1))
    (send *face* :put :margin 20) ;; [mm]
    (send *face* :put :normal-move-target-axis :z)
    (send *face* :put :normal-tolerance 10.0)
  ))

(defun setup-wall
    ()
  (setq *board* (make-cube 1 1 1)) ;; tmp
  (objects (list *robot* *board*))
  )

(defun move-robot
    ()
  ;; (send *robot* :angle-vector (send *ri* :potentio-vector))
  (let* ((root-pos))
    (when (boundp '*axis*)
      (send *axis* :dissoc *robot*))
    (send *robot* :reset-pose)
    (send *robot* :fix-leg-to-coords (make-coords))
    (setq root-pos (scale -1 (send (send (send *robot* :torso-root-link) :copy-worldcoords) :pos)))
    (send *robot* :newcoords (make-coords :pos root-pos
                                          :rpy (list 0 0 0)))
    (setq *axis* (make-cascoords :pos root-pos))
    (send *axis* :assoc *robot*)
    (send *axis* :rotate (deg2rad 15) :y)
    )
  (setq hogea 1)
  ;; solve IK
  (send *robot*
        :inverse-kinematics-statics-optmotiongen
        (list (send (send *robot* :lleg :end-coords) :copy-worldcoords)
              (send (send *robot* :rleg :end-coords) :copy-worldcoords)
              *face*
              *face*
              )
        :move-target
        (list (send *robot* :lleg :end-coords)
              (send *robot* :rleg :end-coords)
              (send *robot* :larm :end-coords)
              (send *robot* :rarm :end-coords)
              )
        :contact-target-coords
        (list (send (send *robot* :lleg :end-coords) :copy-worldcoords)
              (send (send *robot* :rleg :end-coords) :copy-worldcoords)
              *face*
              *face*
              )
        :contact-move-target
        (list (send *robot* :lleg :end-coords)
              (send *robot* :rleg :end-coords)
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
         (instance default-contact-constraint
                   :init-from-support-polygon :support-polygon nil)
         )
        :collision-avoidance-link-pair
        (list (list (send *robot* :link "RARM_LINK7") (send *robot* :link "LARM_LINK7")))
        :collision-distance-limit 200 ;; [mm]
        :translation-axis (list t t t t)
        :rotation-axis (list t t nil nil)
        :debug-view nil
        :root-virtual-mode :6dof
        :optimize-torque? nil
        :revert-if-fail nil
        )
  (setq hogea 2)

  ;; send angle-vector
  (send *ri* :emergencystopperservice_setEmergencyJointAngles :angles (send *robot* :angle-vector) :solved t)
  )

(defun start-touch-wall
    ()
  (let* ((is-stopping nil))
    (do-until-key
     (x::window-main-one)
     (send *ri* :state)
     (when (and (eq (send *ri* :emergency-mode) 2) (not is-stopping))
       (move-robot)
       (setq is-stopping t))
     ;; (when (eq (send *ri* :emergency-mode) 0)
     ;;   (setq is-stopping nil))
     (unless is-stopping (objects (list *robot* *board*)))
     (ros::spin-once)))
  )

(ros::roseus "touch_wall")
;; (ros::subscribe "touchable_wall" jsk_recognition_msgs::BoundingBoxArray #'wall-callback 1)
(ros::subscribe "/valve_recognition/cluster_decomposer/boxes" jsk_recognition_msgs::BoundingBoxArray #'wall-callback 1)
(setup-wall)

(format t "start controller~%")
(start-touch-wall)

(format t "stop controller~%")

