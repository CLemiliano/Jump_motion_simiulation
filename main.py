from math import pi, acos, cos, sin
import numpy as np
from time import perf_counter
from plot_values import plot_values, print_max_values, animate_figure
from variables_constants import *
from movement_constraints import *


error = False  # triggered if error appears in a calculous



#-----------------------#
#      FUNCTIONS
#-----------------------#

def H(t) :

    # if pushing phase or reception phase
    if (not leg_off_ground) :
        H = -l(t)[1]
    
    # if leg in the air
    else :
        H = -lp - vp*t - (1/2)*g*pow(t,2)
    
    return H



def l(t) :
    l = vc.a0 + vc.a1*t + (1/2)*vc.a2*pow(t,2) + (1/6)*vc.a3*pow(t,3) + (1/24)*vc.a4*pow(t,4)
    return [0,l]  # l < 0 when foot under frame (so basically, l<0 all the time)


def dl(t) :
    dl = vc.a1 + vc.a2*t + (1/2)*vc.a3*pow(t,2) + (1/6)*vc.a4*pow(t,3)
    return [0,dl]


def d2l(t) :
    d2l = vc.a2 + vc.a3*t + (1/2)*vc.a4*pow(t,2)
    return [0,d2l]


def d3l(t) :
    d3l = vc.a3 + vc.a4*t
    return [0,d3l]    


def d4l(t) :
    d4l = vc.a4
    return [0,d4l]    

    

def jacobian(q) :

    q1,q2 = q[0], q[1]
    #
    j11 = - L1*sin(q1) - L2*sin(q1+q2)
    j12 = - L2*sin(q1+q2)
    j21 = + L1*cos(q1) + L2*cos(q1+q2)
    j22 = + L2*cos(q1+q2)
    #
    J = np.array( [[j11, j12],
                   [j21, j22]] )
    return J



def q(l) :

    global error
    
    if ( l[1] >= 0 ) :
        print("\n\nERROR : z_l = {} > 0 (leg turned on itself)\n\n".format(l[1]))
        error = True

    elif ( l[1] <= -L1-L2 ) :
        print("\n\nERROR : z_l = {} < -L1-L2 = -0.424 (leg over extended)\n\n".format(l[1]))
        error = True

    else :
        # cosinus law        
        q1 = acos( (L2**2 - L1**2 - l[1]**2) / (-2*L1*(-l[1])) ) - pi/2  # q1 is the angle between x1 axis and x0 axis
        q2 = acos( (l[1]**2- L1**2 - L2**2) / (-2*L1*L2)  ) - pi         # q2 is the angle between x2 axis and x1 axis
        return [q1,q2]



def dq(J,dl) :

    # inverse jacobian
    J_i = np.linalg.inv(J)
    # 
    dq = J_i@dl
    return dq



def d2q(J,q,dq,d2l):
    
    q1, q2  = q[0],  q[1]
    dq1,dq2 = dq[0], dq[1]

    # inverse jacobian
    J_i = np.linalg.inv(J)

    # derivate jacobian
    dj11 = - L1*dq1*cos(q1) - L2*dq1*cos(q1+q2)
    dj12 = - L2*dq2*cos(q1+q2)
    dj21 = - L1*dq2*sin(q1) - L2*dq1*sin(q1+q2)
    dj22 = - L2*dq2*sin(q1+q2)
    #
    dJ = np.array( [[dj11, dj12],
                    [dj21, dj22]])

    # d2q = J_i * (d2l - dJ*dq)
    d2q = J_i @ (d2l - dJ@dq)
    return d2q



def F(d2l) :
    global m
    # we consider the mass as ponctual at A
    xF = 0
    if (not leg_off_ground) :
        zF = m * (-d2l[1]+g)
    else :
        zF = 0
    F = np.array([xF,zF])
    return F



def Cm(J,F) :
    
    # transpose J
    J_t = np.transpose(J)

    # joints torque Cm = J_t * F
    Cm = J_t@F
    return Cm



def update_variables(t_tot, t) :

    t_tab.append(t_tot)
    H_tab.append(H(t))

    l_tab.append(l(t))
    dl_tab.append(dl(t))
    d2l_tab.append(d2l(t))
    d3l_tab.append(d3l(t))
    d4l_tab.append(d4l(t))
    lf, dlf, d2lf = l_tab[len(l_tab)-1], dl_tab[len(dl_tab)-1], d2l_tab[len(d2l_tab)-1]

    q_tab.append(q(lf))
    qf = q_tab[len(q_tab)-1]
    dq_tab.append(dq(jacobian(qf), dlf))
    dqf = dq_tab[len(dq_tab)-1]
    d2q_tab.append(d2q(jacobian(qf), qf, dqf, d2lf))

    F_tab.append(F(d2lf))
    Ff = F_tab[len(F_tab)-1]
    Cm_tab.append(Cm(jacobian(qf), Ff))









#-----------------------#
#  PUSHING MOVEMENT LOOP
#-----------------------#

# We want the leg to reach a certain position (that we name the 'pushing position p')
# my 5 imposed conditions for the pushing phase : [l(0)=-0.2  |  v(0)=0  |  a(0)=0m/s²  |  vf=0m/s  |  lf=-(L1+L2)*90%]

f = 1/0.02                       # frequency = 5ms
dt = 0                            # marks the time of the last update
t0 = perf_counter()               # start chronometer
t = 0                             # running time
t_tot = 0                         # time since the begining of the simulation
first_iteration = True

while (perf_counter()-t0 <= Tp and not error) :

    # update phase chronometer
    t = perf_counter()-t0
    # update global chronometer
    t_tot = t

    # every 5ms, save the jump variables (which are functions of t)
    if (dt+1/f < t) :

        # set movement constraints
        if (first_iteration) :
            # set the initials conditions of the push
            update_last_conditions()
            set_movement_constraints([l0, 0, 0, vp, lp], Tp)
            set_initial_conditions()
            first_iteration = False

        update_variables(t_tot, t)  # update jump variables (leg_high, leg_deployement, motors_speed...)
        dt = t  # update dt

        print("t={:.2f}  |  perf_count={:.2f}  |  dt={:.2f}".format(t%10, perf_counter()%10, dt%10))








#-----------------------#
#  TUCK MOVEMENT LOOP
#-----------------------#

# We want the leg to reach a certain position (that we name the 'pushing position p')
# my 5 imposed conditions for the pushing phase : [l(0)=last_l  |  v(0)=last_speed  |  a(0)=last_acceleration  |  vf=0m/s  |  lf=l0]

f = 1/0.02                        # updating frequency = 20ms
dt = 0                            # marks the time of the last update
t0 = perf_counter()               # reset chronometer
t = 0                             # reset time to 0 (because new movement)
first_iteration = True

while (perf_counter()-t0 <= Tt and not error) :

    # update phase chronometer
    t = perf_counter()-t0
    # update global chronometer
    t_tot = t     # BE CAREFUL HERE : the formula 't_tot' is false; it superimposes the 2 movement curves in the graphs which display the variation of the parameters

    # every 5ms, save the jump variables (which are functions of t)
    if (dt+1/f < t) :

        # set movement constraints
        if (first_iteration) :
            # set the initials conditions of the push
            update_last_conditions()
            set_movement_constraints([l_tab[len(l_tab)-1][1], dl_tab[len(dl_tab)-1][1], d2l_tab[len(d2l_tab)-1][1], 0, l0], Tt)
            set_initial_conditions()
            first_iteration = False

        update_variables(t_tot, t)  # update jump variables (leg_high, leg_deployement, motors_speed...)
        dt = t  # update dt

        print("t={:.2f}  |  perf_count={:.2f}  |  dt={:.2f}".format(t%10, perf_counter()%10, dt%10))










#-----------------------#
#      PLOT RESULTS
#-----------------------#

plot_values()
print_max_values()
animate_figure()
