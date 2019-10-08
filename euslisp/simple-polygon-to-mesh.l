#!/usr/bin/env roseus

(ros::roseus-add-msgs "safe_footstep_planner")

(defun callback(msg)
  ;; (format t "callback~%")
  (setq contours (send msg :polygons))
  (setq polygon-holes (list))
  (setq islands (list))
  ;; (format t "contours : ~A~%" (length contours))
  (setq margin 0.01)
  (setq inner-margin 0.05)
  ;; (format t "margin   : ~A [m]~%" margin)

  (setq triangles (list))
  (dotimes (i (length contours))
    (setq points (send (elt contours i) :points))
    (setq polygon (list))
    ;; (format t "points : ~A~%" (length points))

    ;; get centroid of the target hole
    (dotimes (j (length points))
      ;; (setq x (send (elt points (+ j 1)) :x))
      ;; (setq y (send (elt points (+ j 1)) :y))
      (setq x (send (elt points j) :x))
      (setq y (send (elt points j) :y))
      (when (= j 0)
        (setq px x)
        (setq py y)
        )
      (when (> j 0)
        (when (> (norm (v- (float-vector x y 0) (float-vector px py 0))) margin)
          (setq polygon (nconc polygon (list (float-vector x y 0))))
          (setq px x)
          (setq py y)
          )
        )
      )

    ;; generate small polygon
    ;; Please replace here
    ;; ========================================================================
    (setq f (instance face :init :vertices polygon))
    (setq xc (elt (cadr (send f :centroid)) 0))
    (setq yc (elt (cadr (send f :centroid)) 1))

    (setq small-polygon (list))
    (dotimes (j (length points))
      (setq x (send (elt points j) :x))
      (setq y (send (elt points j) :y))
      (setq vec (float-vector (- x xc) (- y yc) 0))
      (setq np (v+ (float-vector x y 0) (scale (/ (* -1 inner-margin) (norm vec)) (float-vector (elt vec 0) (elt vec 1) 0))))
      ;; (setq npi (v+ (float-vector x y 0) (scale (/ (* -1 margin) (norm vec)) (float-vector (elt vec 0) (elt vec 1) 0))))
      (setq small-polygon (nconc small-polygon (list (float-vector (elt np 0) (elt np 1) 0))))
      )

    ;; generate triangle mesh from polygon
    (setq f (instance face :init :vertices small-polygon))
    ;; ========================================================================

    (setq triangles (nconc triangles (geometry::face-to-triangle f)))

    )

  ;; (format t "triangles : ~A~%" (length triangles))

  ;; publish triangles
  (setq polygons-msg (list))
  (dotimes (i (length triangles))
    (setq ver (send (elt triangles i) :vertices))
    (setq polygon-msg (instance geometry_msgs::Polygon :init
                                :points (list (instance geometry_msgs::Point32 :init
                                                        :x (float (elt (elt ver 0) 0))
                                                        :y (float (elt (elt ver 0) 1))
                                                        :z 0)
                                              (instance geometry_msgs::Point32 :init
                                                        :x (float (elt (elt ver 1) 0))
                                                        :y (float (elt (elt ver 1) 1))
                                                        :z 0)
                                              (instance geometry_msgs::Point32 :init
                                                        :x (float (elt (elt ver 2) 0))
                                                        :y (float (elt (elt ver 2) 1))
                                                        :z 0)
                                              )))
    (setq polygons-msg (nconc polygons-msg (list polygon-msg)))
    )
  (setq polygon-array-msg (instance safe_footstep_planner::PolygonArray :init :polygons polygons-msg))
  (ros::publish "meshed_polygons" polygon-array-msg)
  ;; (format t "published~%")
  ) ;; defun

(ros::roseus "simple_polygon_to_mesh")
(format t "init~%")
(ros::advertise "meshed_polygons" safe_footstep_planner::PolygonArray 1)
;; (ros::subscribe "steppable_polygons" safe_footstep_planner::PolygonArray #'callback 3)
(ros::subscribe "fixed_steppable_polygons" safe_footstep_planner::PolygonArray #'callback 3)
(do-until-key
 (ros::spin-once))
(format t "Shutdown Controller~%")
