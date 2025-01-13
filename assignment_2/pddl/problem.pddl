(define (problem Rosbot) 
(:domain assignment_2)
(:objects
    rob - robot
    wp1 wp2 wp3 wp4 - waypoint
)

(:init
    ;todo: put the initial state's facts and numeric values here
    (robot_at rob wp1)
    (connected wp1 wp2)
    (connected wp2 wp3)
    (connected wp3 wp4)
    (connected wp4 wp1)
    (unvisited wp1)
    (unvisited wp2)
    (unvisited wp3)
    (unvisited wp4)
    
)

(:goal (and
       ;todo: put the goal condition here
       (robot_at rob wp4)
       ;(not(unvisited wp1))
       ;(not(unvisited wp2))
       ;(not(unvisited wp3))
       ;(not(unvisited wp4))
       ))

;un-comment the following line if metric is needed
;(:metric minimize (???))
)
