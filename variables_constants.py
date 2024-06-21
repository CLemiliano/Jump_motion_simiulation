
from math import sqrt



#-----------------------#
#      VARIABLES
#-----------------------#

stop_pushing = False
leg_off_ground = False

last_conditions = [0,0,0,0,0]   # conditions (l,dl,d2l,d3l,d4l) of the last movement
C, c = [], []                   # system constraint and state functions

t_tab   = []     # time during the jump
H_tab   = []     # high between the leg frame and the groun (>0)

l_tab   = []     # leg extension length (<0)
dl_tab  = []     # leg extension speed
d2l_tab = []     # leg extension acceleration
d3l_tab = []     # leg extension acceleration derivate
d4l_tab = []     # leg extension acceleration 2nd derivate

q_tab   = []     # angular position of the hips and the knee joints
dq_tab  = []     # angular speed of the joints
d2q_tab = []     # angular acceleration of the joints

F_tab   = []     # pushing force of the leg on the ground
Cm_tab  = []     # hips and knee motors torque 





#-----------------------#
#      CONSTANTS
#-----------------------#
 
m = 2.2                      # mass of the leg in kg
g = 9.81                     # gravity


L1,L2 = 0.213, 0.211         # leg parts dimensions
l0 = -0.2                    # initial high

lp = -(L1+L2)*0.9            # leg deployment length at the end of the push
vp = 0                       # speed at the end of the push
Tp = 2                       # duration of the pushing phase T in seconds

Tt = 2                       # duration T in seconds of leg tuck. If we want to jump, the leg tuck must be faster than gravity : Δl = 1/2 * gt²  <=>  t = security_coeff * √(2*Δl/g)


a0,a1,a2,a3,a4 = 0,0,0,0,0   # polynom coefficients



