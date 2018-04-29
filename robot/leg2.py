import odrive.core
import time
import math

import numpy as np
import matplotlib.pyplot as plt
import matplotlib

# For symbolic processing
import sympy
from sympy import symbols
from sympy import sin, cos, asin, acos, pi, atan
from sympy.utilities.lambdify import lambdify
from sympy import Matrix
from sympy.solvers import solve
from scipy import linalg

#%matplotlib qt5 #opens new window for plotting things on the same figure

class Leg:
    """
    This is our first class in class :)

    We will define a leg class to interface with the leg and standardize 
    the kind of operations we want to perform

    """
    global l1, l2, l_base, theta0_sym, theta1_sym, alpha0_sym, alpha1_sym, encoder2angle
    #### Variables outside the init function are constants of the class
    # leg geometry
    l1 = 7  # NEED TO UPDATE units of cm
    l2 = 14  # NEED TO UPDATE units of cm
    l_base = 7.7  # NEED TO UPDATE units of cm
    theta0_sym, theta1_sym, alpha0_sym, alpha1_sym, = symbols(
            'theta0_sym theta1_sym alpha0_sym alpha1_sym' , real=True)

    # motor controller parameters
    encoder2angle = 2048 * 4

    ### Methods
    # Classes are initiated with a constructor that can take in initial parameters. At
    # a minimum it takes in a copy of itself (python... weird). The constructor
    # is one place we can define and initialize class variables

    def __init__(self, simulate = False):
        """
        This is the constructor for the leg class. Whenever you make a new leg
        this code will be called. We can optionally make a leg that will simulate
        the computations without needing to be connected to the ODrive
        """

        self.simulate = True #simulate

            

        # make the option to code without having the odrive connected
        if self.simulate == False:
            self.drv = self.connect_to_controller()
            self.m0 = self.drv.motor0  # easier handles to the motor commands
            self.m1 = self.drv.motor1

            # current positions
            m0_pos, m1_pos = self.get_joint_pos()
            self.joint_0_pos = m0_pos
            self.joint_1_pos = m1_pos

        else:
            self.drv = None
            self.joint_0_pos = 2
            self.joint_1_pos = 1.4

        # home angles
        self.joint_0_home = 0
        self.joint_1_home = 0

        

        # We will compute the jacobian and inverse just once in the class initialization.
        # This will be done symbolically so that we can use the inverse without having
        # to recompute it every time
        self.J = self.compute_jacobian()
        #print('jacobian done')
        #self.J_inv = self.J.pinv()
        #print('inverse done')

    def connect_to_controller(self):
        """
        Connects to the motor controller
        """
        drv = odrive.core.find_any(consider_usb=True, consider_serial=False)

        if drv is None:
            print('No controller found')
        else:
            print('Connected!')
        return drv

    ###
    ### Motion functions
    ###


    def get_joint_pos(self):
        """
        Get the current joint positions and store them in self.joint_0_pos and self.joint_1_pos in degrees.
        Also, return these positions using the return statement to terminate the function
        """
        # if simulating exit function
        if self.simulate == True:
            return (-1, -1)
        
        else:
            self.joint_0_pos=self.m0.encoder.pll.pos/(2048*4)
            self.joint_1_pos=self.m1.encoder.pll_pos/(2048*4)        

            return (self.joint_0_pos, self.joint_1_pos)

    def set_home(self):
        """
        This function updates the home locations of the motors so that 
        all move commands we execute are relative to this location. 
        """
        # if simulating exit function
        if self.simulate == True:
            return
        
        
        theta_0=self.joint_0_pos
        theta_1=self.joint_1_pos
        
        return(theta_0, theta_1)
        

        #
        # Your code here
        #


    def set_joint_pos(self, theta0, theta1, vel0=0, vel1=0, curr0=0, curr1=0):
        """
        Set the joint positions in units of deg, and with respect to the joint homes.
        We have the option of passing the velocity and current feedforward terms.
        """
        # if simulating exit function
        if self.simulate == True:
            return
        
        l0=l_base;
        alpha_0=theta0 - math.acos((l0^2 + 2*l1^2 - 2*l0*l1*math.cos(theta1) - 2*l1*math.cos(theta0 + math.acos((2*l0^2 - 2*l1*math.cos(theta1)*l0)/(2*l0*(l0^2 - 2*math.cos(theta1)*l0*l1 + l1^2)^(1/2))) - 157/50)*(l0^2 - 2*math.cos(theta1)*l0*l1 + l1^2)^(1/2))^(1/2)/(2*l2)) - math.acos((2*l1^2 - 2*l1*math.cos(theta0 + math.acos((2*l0^2 - 2*l1*math.cos(theta1)*l0)/(2*l0*(l0^2 - 2*math.cos(theta1)*l0*l1 + l1^2)^(1/2))) - 157/50)*(l0^2 - 2*math.cos(theta1)*l0*l1 + l1^2)^(1/2))/(2*l1*(l0^2 + 2*l1^2 - 2*l0*l1*math.cos(theta1) - 2*l1*math.cos(theta0 + math.acos((2*l0^2 - 2*l0*l1*math.cos(theta1))/(2*l0*(l0^2 - 2*math.cos(theta1)*l0*l1 + l1^2)^(1/2))) - 157/50)*(l0^2 - 2*math.cos(theta1)*l0*l1 + l1^2)^(1/2))^(1/2))) + 157/50
        
        x=l0/2+l1*math.cos(theta0)+l2*cos(alpha_0)
        y=l1*math.sin(theta0)+l2*math.sin(alpha_0)
        

        #
        # Your code here
        #


    def move_home(self):
        """
        Move the motors to the home position
        """
        # if simulating exit function
        if self.simulate == True:
            return
        
        m0.pos_setpoint=theta_0
        m1.pos_setpoint=theta_1


        #
        # Your code here
        #

    def set_foot_pos(self, x, y):
        """
        Move the foot to position x, y. This function will call the inverse kinematics 
        solver and then call set_joint_pos with the appropriate angles
        """
        # if simulating exit function
        if self.simulate == True:
            return
        
        e=1
        while e>>10^(-5)
        inverse_kinematics(self, x, y)
        
        

        #
        # Your code here
        #

    def move_trajectory(self, tt, xx, yy):
        """
        Move the foot over a cyclic trajectory to positions xx, yy in time tt. 
        This will repeatedly call the set_foot_pos function to the new foot 
        location specified by the trajectory.
        """
        # if simulating exit function
        if self.simulate == True:
            return
        
        for i in range (1,12)
        set_foot_pos(self,xx[i],yy[i])
        
        

        #
        # Your code here
        #

    ###
    ### Leg geometry functions
    ###
    def compute_internal_angles(self, theta_0, theta_1):
        """
        Return the internal angles of the robot leg 
        from the current motor angles
        """

        l0=l_base;
        l3=math.sqrt(l1**2 + l0**2-2*l1*l0*math.cos(theta_1));
        alpha2=math.acos((l0**2+l3**2-l1**2)/(2*l0*l3));
        alpha3=3.14-theta_0-alpha2;
        l4=math.sqrt(l1**2+l3**2-2*l1*l3*math.cos(alpha3));
        alpha4=math.acos((l4**2+l1**2-l3**2)/(2*l1*l4));
        alpha5=math.acos((l4**2)/(2*l4*l2));
        
        alpha_0=3.14-alpha4-alpha5+theta_0;
        
        l5=math.sqrt(l1**2+l0**2-2*l1*l0*math.cos(3.14-theta_0));
        alpha6=math.acos((l0**2+l5**2-l1**2)/(2*l0*l5));
        alpha7=theta_1-alpha6;
        l6=math.squrt(l1**2+l5**2-2*l1*l5*math.cos(alpha7));
        alpha8=math.acos((l1**2+l6**2-l5**2)/(2*l6*l2));
        alpha9=math.acos((l6**2+l2**2-l2**2)/(2*l6*l2));
        
        alpha_1=alpha8+alpha9-3.14+theta_1;
        

        return (alpha_0, alpha_1)

    def compute_jacobian(self):
        """
        This function implements the symbolic solution to the Jacobian.
        """

        # initiate the symbolic variables
        theta0_sym, theta1_sym, alpha0_sym, alpha1_sym = symbols(
            'theta1_sym theta2_sym alpha1_sym alpha2_sym', real=True)
        
        l0=l_base;
        
        alpha0_sym=theta0_sym - math.acos((l0^2 + 2*l1^2 - 2*l0*l1*math.cos(theta1_sym) - 2*l1*math.cos(theta0_sym + math.acos((2*l0^2 - 2*l1*math.cos(theta1_sym)*l0)/(2*l0*(l0^2 - 2*math.cos(theta1_sym)*l0*l1 + l1^2)^(1/2))) - 157/50)*(l0^2 - 2*math.cos(theta1_sym)*l0*l1 + l1^2)^(1/2))^(1/2)/(2*l2)) - math.acos((2*l1^2 - 2*l1*math.cos(theta0_sym + math.acos((2*l0^2 - 2*l1*math.cos(theta1_sym)*l0)/(2*l0*(l0^2 - 2*math.cos(theta1_sym)*l0*l1 + l1^2)^(1/2))) - 157/50)*(l0^2 - 2*math.cos(theta1_sym)*l0*l1 + l1^2)^(1/2))/(2*l1*(l0^2 + 2*l1^2 - 2*l0*l1*math.cos(theta1_sym) - 2*l1*math.cos(theta0_sym + math.acos((2*l0^2 - 2*l0*l1*math.cos(theta1_sym))/(2*l0*(l0^2 - 2*math.cos(theta1_sym)*l0*l1 + l1^2)^(1/2))) - 157/50)*(l0^2 - 2*math.cos(theta1_sym)*l0*l1 + l1^2)^(1/2))^(1/2))) + 157/50 
        
        x=l0/2+l1*math.cos(theta0_sym)+l2*math.cos(alpha0_sym);
        y=l1*math.sin(theta0_sym)+l2*math.sin(alpha0_sym);
        
        J=[[diff(x,theta0_sym),diff(x,theta1_sym)],[diff(y,theta0_sym),diff(y,theta1_sym)]]

        #
        # Your code here that solves J as a matrix
        #

        return J

    def inverse_kinematics(self, x, y):
        """
        This function will compute the required theta_0 and theta_1 angles to position the 
        foot to the point x, y. We will use an iterative solver to determine the angles.
        """
        
        l0=l_base;
        
        J=inv(J);
        
        #while e>>10^(-5)
        x0=l0/2+l1*math.cos(theta_0)+l2*cos(alpha_0)
        y0=l1*math.sin(theta_0)+l2*math.sin(alpha_0)
        [[theta_0],[theta_1]]=0.001*J*([[x],[y]]-[[x0],[y0]])+[[theta_0],[theta_1]]
        e=np.norm([[x],[y]]-[[x0],[y0]])
        
        
        
        
        
        

        #
        # Your code here that solves J as a matrix
        #

        return (theta_0, theta_1,e)

    ###
    ### Visualization functions
    ###
    def draw_leg(ax=False):
        """
        This function takes in the four angles of the leg and draws
        the configuration
        """

        theta1, theta2 = self.joint_0_pos, self.joint_1_pos
        link1, link2, width = self.l1, self.l2, self.l_base

        alpha1, alpha2 = self.compute_internal_angles()

        def pol2cart(rho, phi):
            x = rho * np.cos(phi)
            y = rho * np.sin(phi)
            return (x, y)

        if ax == False:
            ax = plt.gca()
            ax.cla()

        ax.plot(-width / 2, 0, 'ok')
        ax.plot(width / 2, 0, 'ok')

        ax.plot([-width / 2, 0], [0, 0], 'k')
        ax.plot([width / 2, 0], [0, 0], 'k')

        ax.plot(-width / 2 + np.array([0, link1 * cos(theta1)]), [0, link1 * sin(theta1)], 'k')
        ax.plot(width / 2 + np.array([0, link1 * cos(theta2)]), [0, link1 * sin(theta2)], 'k')

        ax.plot(-width / 2 + link1 * cos(theta1) + np.array([0, link2 * cos(alpha1)]), \
                link1 * sin(theta1) + np.array([0, link2 * sin(alpha1)]), 'k');
        ax.plot(width / 2 + link1 * cos(theta2) + np.array([0, link2 * cos(alpha2)]), \
                np.array(link1 * sin(theta2) + np.array([0, link2 * sin(alpha2)])), 'k');

        ax.plot(width / 2 + link1 * cos(theta2) + link2 * cos(alpha2), \
                np.array(link1 * sin(theta2) + link2 * sin(alpha2)), 'ro');

        ax.axis([-2, 2, -2, 2])
        ax.invert_yaxis()