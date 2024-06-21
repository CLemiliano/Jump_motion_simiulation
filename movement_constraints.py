from variables_constants import *
import variables_constants as vc
import numpy as np




def update_last_conditions() :

    vc.last_conditions 

    if (len(t_tab) > 0) :
        # save the conditions of the system at one period of time t
        l   =   l_tab[len(l_tab)-1  ][1]
        dl  =  dl_tab[len(dl_tab)-1 ][1]
        d2l = d2l_tab[len(d2l_tab)-1][1]
        d3l = d3l_tab[len(d3l_tab)-1][1]
        d4l = d4l_tab[len(d4l_tab)-1][1]
        #
        vc.last_conditions = [l, dl,d2l, d3l, d4l]
    
    else :
        vc.last_conditions = [vc.a0,vc.a1,vc.a2,vc.a3,vc.a4]
             
    






# MOVEMENT CONSTRAINTS
#---------------------------

def set_movement_constraints(conditions_tab, T) :

    # verify if the condition array is correct
    if (len(conditions_tab)>5) :
        print("\nERROR : you must choose 5 CONDITIONS for your movement. Setted : {}".format(len(conditions_tab)))

    vc.c = np.array( conditions_tab )

    # my condition functions array
    vc.C = np.array(   [[1, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0],
                        [0, 0, 1, 0, 0],
                        [0, 1, T, pow(T,2)/2, pow(T,3)/6],
                        [1, T, pow(T,2)/2, pow(T,3)/6, pow(T,4)/24]] )









# SET INITIAL CONDITIONS
#---------------------------

def set_initial_conditions() :
    
    # inverse the condition functions array (if the array is inversible)
    if not np.isclose(np.linalg.det(vc.C), 0) :
        C_i = np.linalg.inv(vc.C)

        print("C_inverse = {}\n\n c = {}".format(C_i, vc.c))

        # a = C_i * c
        a = C_i@vc.c
        vc.a0,vc.a1,vc.a2,vc.a3,vc.a4 = a[0],a[1],a[2],a[3],a[4]
        print("\npolynom coeff : [{:.2f},{:.2f},{:2f},{:.2f},{:.2f}]\n".format(vc.a0,vc.a1,vc.a2,vc.a3,vc.a4))
    
    else :
        print("\n\nERROR IN THE CONSTRAINTS MATRIX\n\n")

