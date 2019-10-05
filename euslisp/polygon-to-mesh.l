#!/usr/bin/env roseus

(ros::roseus-add-msgs "safe_footstep_planner")
(ros::roseus-add-msgs "opencv_apps")

(defun callback(msg)
  (format t "callback~%")
  (setq contours (send msg :contours))
  (setq polygon-holes (list))
  (setq islands (list))
  (format t "contours : ~A~%" (length contours))
  (setq margin 10)
  (format t "margin   : ~A~%" margin)

  ;; coutours are doubled !!!
  (dotimes (i (/ (length contours) 2))
    (setq points (send (elt contours (* 2 i)) :points))
    (setq polygon-hole (list))
    (format t "points : ~A~%" (length points))

    ;; get centroid of the target hole
    (dotimes (j (- (length points) 1))
      (setq x (send (elt points (+ j 1)) :x))
      (setq y (send (elt points (+ j 1)) :y))
      (when (= j 0)
        (setq px x)
        (setq py y)
        )
      (when (> j 0)
        (when (> (norm (v- (float-vector x y 0) (float-vector px py 0))) 10)
          (setq polygon-hole (nconc polygon-hole (list (float-vector x y 0))))
          (setq px x)
          (setq py y)
          )
        )
      )
    (setq h (instance face :init :vertices polygon-hole))
    (setq xc (elt (cadr (send h :centroid)) 0))
    (setq yc (elt (cadr (send h :centroid)) 1))

    ;; make holes and islands
    (setq polygon-hole (list))
    (setq island (list))
    (dotimes (j (- (length points) 1))
      (setq x (send (elt points (+ j 1)) :x))
      (setq y (send (elt points (+ j 1)) :y))
      (setq vec (float-vector (- x xc) (- y yc) 0))
      (setq np (v+ (float-vector x y 0) (scale (/ margin (norm vec)) (float-vector (elt vec 0) (elt vec 1) 0))))
      (setq npi (v+ (float-vector x y 0) (scale (/ (* -1 margin) (norm vec)) (float-vector (elt vec 0) (elt vec 1) 0))))

      ;; set contours of holes and islands
      (setq x (elt np 0))
      (when (< x 0) (setq x 0))
      (when (> x 500) (setq x 500))
      (setq y (elt np 1))
      (when (< y 0) (setq y 0))
      (when (> y 500) (setq y 500))
      (setq xi (elt npi 0))
      (when (< xi 0) (setq xi 0))
      (when (> xi 500) (setq xi 500))
      (setq yi (elt npi 1))
      (when (< yi 0) (setq yi 0))
      (when (> yi 500) (setq yi 500))

      ;; remove near points
      (when (= j 0)
        (setq px x)
        (setq py y)
        )
      (when (> j 0)
        (when (> (norm (v- (float-vector x y 0) (float-vector px py 0))) 10)
          (setq polygon-hole (nconc polygon-hole (list (float-vector x y 0))))
          (setq island (nconc island (list (float-vector xi yi 0))))
          (setq px x)
          (setq py y)
          )
        )
      )
    (format t "polygon points : ~A~%" (length polygon-hole))
    ;; (setq polygon-holes (nconc polygon-holes
    ;;                            (list (instance hole :init :vertices (reverse polygon-hole)))))
    (setq polygon-holes (nconc polygon-holes
                               (list (instance hole :init :vertices polygon-hole))))
    (setq islands (nconc islands
                         (list (instance face :init :vertices island))))
    )
  ;;;;;; for debug ;;;;;;
  ;; (setq polygon-holes (list
  ;;                      (instance hole :init :vertices (reverse (list #f(100.0 100.0 0.0) #f(150.0 100.0 0.0) #f(150.0 150.0 0.0) #f(125.0 175.0 0.0) #f(100.0 150.0 0.0))))
  ;;                      (instance hole :init :vertices (reverse (list #f(300.0 300.0 0.0) #f(350.0 300.0 0.0) #f(350.0 350.0 0.0) #f(300.0 350.0 0.0))))))
  ;;;;;;;;;;;;;;;;;;;;;;;
  (setq img (list #f(0.0 0.0 0.0) #f(500.0 0.0 0.0) #f(500.0 500.0 0.0) #f(0.0 500.0 0.0)))
  ;; (setq img (list #f(0.0 0.0 0.0) #f(0.0 500.0 0.0) #f(500.0 500.0 0.0) #f(500.0 0.0 0.0)))
  (setq f (instance face :init :vertices img :holes polygon-holes))

  ;; generate triangle mesh from faces
  (setq triangles (list))
  (setq triangles (nconc triangles (geometry::face-to-triangle f)))
  ;; (setq triangles (geometry::face-to-triangle f))
  (dotimes (i (length islands))
    (setq triangles (nconc triangles (geometry::face-to-triangle (elt islands i))))
    )
  (format t "triangles : ~A~%" (length triangles))
  ;;;;;; viewer ;;;;;;
  ;; (setq fs (instance faceset :init :faces (list f)))
  ;; (setq fs (instance faceset :init :faces (list-insert f 0 islands)))
  ;; (objects (list fs))
  ;; (dotimes (i (length triangles)) (send (elt triangles i) :draw-on :flush t))
  ;;;;;;;;;;;;;;;;;;;;


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
  (format t "polygons-msg : ~A ~A ~A~%"
          (send (elt (send (elt polygons-msg 0) :points) 0) :x)
          (send (elt (send (elt polygons-msg 0) :points) 0) :y)
          (send (elt (send (elt polygons-msg 0) :points) 0) :z))
  (ros::publish "meshed_polygons" polygon-array-msg)
  (format t "published~%")
  ) ;; defun

(ros::roseus "polygon_to_mesh")
(format t "init~%")
(ros::advertise "meshed_polygons" safe_footstep_planner::PolygonArray 1)
(ros::subscribe "find_contours/contours" opencv_apps::ContourArrayStamped #'callback 3)
(do-until-key
 (ros::spin-once))
(format t "Shutdown Controller~%")
