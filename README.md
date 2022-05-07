# self_balancing_bike
ME 235 Project Self Balancing Bike

Balance Controller Added. Navigation Controller in Progress.

Config Parameters Used:

h = 0.015 #Height

l_f = 0.02 #Front Axle to CG

l_r = 0.01 #Real Axle to CG

wd = 0.01 #Wheel Diameter (Only Visualization)

w = l_f + l_r #Wheelbase

g = 9.81 #Gravity 

time_step = 0.01 #Seconds

To run with Visualization:
python3 main.py -args

where args are:
-m (mode): 'auto' or 'manual'
-c (controller): 'pid' or 'nav'
-v (visualizer): 'true' or 'false'

Code runs faster without visualizer. 

TODO:
1) Visualizer speed improve
2) Navigation controller tuning