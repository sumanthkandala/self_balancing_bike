# self_balancing_bike
ME 235 Project Self Balancing Bike

Balance Controller Added. Navigation Controller in Progress.

Config Parameters Used:

  Bike Height
  h = 0.15 

  Front Axle to CG
  l_f = 0.2

  Real Axle to CG
  l_r = 0.1 

  Wheel Diameter (Only Visualization)
  wd = 0.1 

  Wheelbase
  w = l_f + l_r

  Gravity
  g = 9.81 

  Discretization Step
  time_step = 0.01 

To run with Visualization:
python3 main.py -args

where args are:
-m (mode): 'auto' or 'manual'
-c (controller): 'pid' or 'nav'

Notes:
Manual (aka Keyboard Control mode) only works with nav controller to ensure path/position tracking. PID control is only used to demonstrate self-balancing. 
