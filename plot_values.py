
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from variables_constants import t_tab,q_tab,dq_tab,d2q_tab,F_tab,Cm_tab, l_tab, dl_tab, d2l_tab, d3l_tab, d4l_tab,H_tab, L1, L2
from math import cos, sin, pi


high_tab, l_ext_tab = [], []
Fp_tab = []
T1_tab, T2_tab = [], []
Q1_tab, Q2_tab = [], []
dQ1_tab, dQ2_tab = [], []
d2Q1_tab, d2Q2_tab = [], []



def get_values() :

    for i in range (len(t_tab)) :
        #
        l_ext_tab.append(l_tab[i][1])
        high_tab.append(H_tab[i])
        #
        Q1_tab.append(q_tab[i][0])
        Q2_tab.append(q_tab[i][1])
        #
        dQ1_tab.append(dq_tab[i][0]) 
        dQ2_tab.append(dq_tab[i][1])
        #
        d2Q1_tab.append(d2q_tab[i][0]) 
        d2Q2_tab.append(d2q_tab[i][1])
        #
        Fp_tab.append(F_tab[i][1])
        T1_tab.append(Cm_tab[i][0])
        T2_tab.append(Cm_tab[i][1])



def plot_values() :

    # store each value on their own table
    get_values()
    
    plt.figure(figsize=(30,30))

    # plot HIGH variation of the mass center of the leg
    plt.subplot(2,4,1)
    plt.title("leg High during jump")
    plt.xlabel("time (s)")
    plt.ylabel("H (m)")
    plt.plot(t_tab,high_tab, label="high variation")
    plt.legend(loc="upper left")

    # plot l variation 
    plt.subplot(2,4,2)
    plt.title("leg extension during jump")
    plt.xlabel("time (s)")
    plt.ylabel("l (m)")
    plt.plot(t_tab,l_tab, label="l variation")
    plt.legend(loc="lower left")

    # plot dl variation
    plt.subplot(2,4,3)
    plt.title("leg extension velocity during jump")
    plt.xlabel("time (s)")
    plt.ylabel("dl (m)")
    plt.plot(t_tab,dl_tab, label="dl variation")
    plt.legend(loc="lower left")

    # plot d2l variation
    plt.subplot(2,4,4)
    plt.title("leg extension acceleration during jump")
    plt.xlabel("time (s)")
    plt.ylabel("d2l (m)")
    plt.plot(t_tab,d2l_tab, label="d2l variation")
    plt.legend(loc="lower left")

   # plot JOINTS ANGLE variation
    plt.subplot(2,4,5)
    plt.title("joints angles during jump")
    plt.xlabel("time (s)")
    plt.ylabel("Q (rad)")
    plt.plot(t_tab,Q1_tab, label="Q1(hips angle)", color='r')
    plt.plot(t_tab,Q2_tab, label="Q2(knee angle)", color='g')
    plt.legend(loc="lower left")

    # plot JOINTS ANGLE SPEED variation
    plt.subplot(2,4,6)
    plt.title("joints angles speed during jump")
    plt.xlabel("time (s)")
    plt.ylabel("dQ (rad/s)")
    plt.plot(t_tab,dQ1_tab, label="dQ1(hips)", color='r')
    plt.plot(t_tab,dQ2_tab, label="dQ2(knee)", color='g')
    plt.legend(loc="lower left")

    # plot JOINTS ANGLE ACCELERATION variation
    plt.subplot(2,4,7)
    plt.title("joints angles acceleration during jump")
    plt.xlabel("time (s)")
    plt.ylabel("d2Q (rad/s²)")
    plt.plot(t_tab,d2Q1_tab, label="dQ1(hips)", color='r')
    plt.plot(t_tab,d2Q2_tab, label="dQ2(knee)", color='g')
    plt.legend(loc="lower right")

    # plot VERTICAL PUSHING FORCE variation
    #plt.subplot(2,4,8)
    #plt.title("pushing force during jump")
    #plt.xlabel("time (s)")
    #plt.ylabel("F (N)")
    #plt.plot(t_tab,Fp_tab, label="pushing force", color='b')
    #plt.legend(loc="lower left")

    # plot MOTORS TORQUE variation
    plt.subplot(2,4,8)
    plt.title("motors torque during jump")
    plt.xlabel("time (s)")
    plt.ylabel("T (N.m)")
    plt.plot(t_tab,T1_tab, label="motor1 torque (T1)", color='r')
    plt.plot(t_tab,T2_tab, label="motor2 torque (T2)", color='g')
    plt.legend(loc="center left")

    #plt.show()






#-------------------------------------------------#
#     PRINT THE MAXIMAL VALUE OF EACH VARIABLE
#-------------------------------------------------#

def print_max_values() :
    
    print("\nMax high = {h:.3f}m".format(h=max(high_tab)))
    print("\nTakeoff high = {Ld:.3f}m".format(Ld=max(l_ext_tab)))
    print("\nMax motor speed = {Q:.3f}rad/s (nominal speed : 16.8 rad/s)".format( Q=max( abs(min(dQ1_tab)), max(dQ2_tab) )   ))
    print("\nMax force = {F:.3f}N".format(F=max(Fp_tab)))
    print("\nMax torque = {T:.3f}N.m  (nominal torque : 5N.m)\n".format(  T=max( abs(min(T1_tab)), abs(min(T2_tab)), max(T1_tab), max(T2_tab) )  ))







#-------------------------------------------------#
#      ANIMATION OF THE LEG DURIG THE JUMP
#-------------------------------------------------#

# creating a Figure and 2 Axis for the graphic visualisation in 2D
fig, axis  = plt.subplots()
body,      = axis.plot([],[], 'o', markersize=20, color='red')
upper_leg, = axis.plot([],[], color='black')
lower_leg, = axis.plot([],[], color='black')



def set_figure() :
    axis.set_xlim([-1,1])
    axis.set_ylim([-1,1])
    axis.set_title("leg jump movement")

    plt.grid()  # add a grid to the figure



def update_figure(frame):

    x_body, z_body = 0, H_tab[frame]
    body.set_data([x_body], [z_body])
    #
    x_ul = cos(Q1_tab[frame])*L1 + x_body
    z_ul = sin(Q1_tab[frame])*L1 + z_body
    upper_leg.set_data([x_body,x_ul], [z_body,z_ul])
    #
    x_ll = x_body + (cos(Q1_tab[frame])*L1 + cos(Q1_tab[frame]+Q2_tab[frame])*L2)
    z_ll = z_body + (sin(Q1_tab[frame])*L1 + sin(Q1_tab[frame]+Q2_tab[frame])*L2)
    #
    lower_leg.set_data([x_ul,x_ll], [z_ul,z_ll])

    return body,


animation = FuncAnimation(fig, update_figure)

def animate_figure():
    global animation
    set_figure()
    animation = FuncAnimation(fig, update_figure, frames=len(t_tab), interval=1)
    plt.show()