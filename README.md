# robot_controllers
<h1>States</h1>


<h4>state 0: wait 10s for everything to load</h4>

<h4>state 1: turn for 3s to move next to the outer line</h4>

<h4>state 2: PID control off the outerline of outer ring</h4>

<h4>state 3: Red line detected. Wait for pedestrian</h4>

<h4>state 4: Move robot straight until red line detected again: state -> 2</h4>

<h4>state 5: Left PID for cornering the outer line of the inner ring,
         used for initial cornering as well as after vehicle has passed</h4>

<h4>state 6: Stop and wait until the vehicle has been detected</h4>

<h4>state 7: wait for 3 seconds after the vehicle is detected, then state 6 -> 5</h4>

<h4>state 8: Left PID for inner line of inner ring. Look for parked cars</h4>

<h4>state 9: Parked car has been detected, move straight for 3.5s, then state 9 -> 8</h4>
