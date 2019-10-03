#!/usr/bin/env roseus

(ros::roseus-add-msgs "safe_footstep_planner")

(defun callback(msg)
  (format t "callback~%")
  (setq polygons (send msg :polygons))
  (setq meshes (list))
  (dotimes (i (length polygons))
    (setq polygon (send (elt polygons i) :points))
    (setq ver (list))
    (dotimes (j (length polygon))
      (setq ver (nconc ver (list (float-vector
                                  (send (elt polygon j) :x)
                                  (send (elt polygon j) :y)
                                  (send (elt polygon j) :z)
                                  ))))
      )
    ;; for closing loop (remove if not required)
    (setq ver (nconc ver (list (float-vector
                                (send (elt polygon 0) :x)
                                (send (elt polygon 0) :y)
                                (send (elt polygon 0) :z)
                                ))))
    (setq meshes (nconc meshes (list (instance face :init :vertices ver))))
    )

  (dotimes (i (length meshes)) (send (elt meshes i) :draw-on :flush t :color #f(0 0 0) :width 100))
  ) ;; defun

;; (make-irtviewer)
(send *irtviewer* :draw-objects)
(ros::roseus "draw_polygons")
(ros::subscribe "combined_meshed_polygons" safe_footstep_planner::PolygonArray #'callback 1)
(do-until-key
 (ros::spin-once))
(format t "Shutdown Controller~%")

#|
(roseus)
(make-irtviewer)
(send *irtviewer* :change-background #f(1 1 1))
(send *viewer* :viewing :look #f(0 0  -6000) #f(0 0.0 1))
(load "draw_meshes.l")
|#
