#! /usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import sys

from scipy.optimize import minimize

from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


#GLOBAL CONSTANTS - conversion factors
degToRad        = np.pi/180
radToDeg        = 180.0/np.pi
PoundsToNewtons = 4.4482216282509
NewtonsTokN     = 1.0/1000
inchesToM       = 0.0254   
mToInches       = 39.3701
psiToN_msq      = 6894.76 #psi to Newtons per meter squared


#ZYX Euler Angle Rotation Matrix function
#Left-handed rotation after Z,Y,X right-handed rotation
def zyx_euler_angles_to_mat(alpha, beta, gamma):
    """
    Converts ZYX Euler angles (rotation about z, then resulting y, then resulting x)
    into a rotation matrix.

    Args:
        alpha (float): angle in radians to rotate about z
        beta  (float): angle in radians to rotate about y
        gamma (float): angle in radians to rotate about x
    Returns:
        3x3 numpy array that is a rotation matrix
    """
    # TODO: Replace following line with implementation
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    cb = np.cos(beta)
    sb = np.sin(beta)
    cg = np.cos(gamma)
    sg = np.sin(gamma)
    return np.array([[ca*cb, ca*sb*sg-sa*cg, ca*sb*cg+sa*sg],
                     [sa*cb, sa*sb*sg+ca*cg, sa*sb*cg-ca*sg],
                     [-sb  , cb*sg         , cb*cg         ]])
                    
                    

class HydraulicCylinder():
    
    def __init__(self, CylinderType):
        #choice: hydraulic cylinder brand
        assert CylinderType == "BAILEY" or CylinderType == "WOLVERINE", "The hydraulic cylinder brand is invalid."
        
        if CylinderType == "BAILEY":
        
            # Bailey Hydraulics (can be customized)
            boreD = 1.5*inchesToM                   # m
            rodD = 1.0*inchesToM                    # m
            maxP = 5000.0*psiToN_msq                # N/m^2
            wallThickness = 0.25*inchesToM;         # m
            cylinderOD = boreD + wallThickness * 2; # m
            self.retractLength = 10.0*inchesToM     # m means the shortest length for the hydraulic cylinder
            self.stroke = 4.0*inchesToM             # m
            
            #capability:
            # max hydraulic force is 39.303282 kN. (max pushing load)
            # max hydraulic force on the side of rod is 21.835157 kN. (max pulling load)
        
        else: # choice == "WOLVERINE"
        
            # WWSB1004-S bore x Stroke = 1 in. x 4 in.
            boreD = 1.0*inchesToM                 # m
            rodD = 0.625*inchesToM                # m
            maxP = 3000.0*psiToN_msq              # N/m^2
            cylinderOD = 1.5*inchesToM            # m
            self.retractLength = 10.0*inchesToM   # m. means the shortest length for the hydraulic cylinder
            self.stroke = 4.0*inchesToM           # m
            
            # capability:
            # max hydraulic force is 10.480875 kN. (max pushing load)
            # max hydraulic force on the side of rod is 6.386783 kN. (max pulling load)

        # calculate the maximum force on both sides of double acting hydraulic cylinders
        self.maxPushingForce = np.pi*boreD**2/4*maxP*NewtonsTokN   # kN
        self.maxPullingForce = np.pi*(boreD**2 - rodD**2)/4*maxP*NewtonsTokN # kN
        
        #if printFlag:
        print("\nmax hydraulic pushing force per cylinder is %f kN." % self.maxPushingForce)
        print("max hydraulic pulling force per cylinder is %f kN.\n" % self.maxPullingForce)
        

        
class StewartPlatform():
    
    def __init__(self,r_base, r_platform, seperationAngle, PlatformTwist, Cylinder):
        
        #Store cylinder object and extract properties
        self.Cylinder = Cylinder
        self.lmin =  Cylinder.retractLength #m, min length of actuators
        self.lmax =  Cylinder.retractLength + Cylinder.stroke #m, max length of actuators
        self.Fmax =  Cylinder.maxPushingForce  #kN
        self.Fmin = -Cylinder.maxPullingForce  #kN
        
        #Intrinsic for a stewart platform. Gets rid of magic numbers though
        self.numActuators = 6
        
        #Calculate actuator connection points on base (in base frame) (store as list of np vectors)
        self.base_joint_pos = []
        for i in range(self.numActuators/2):
            theta1 = (120*i*degToRad + seperationAngle/2)
            theta2 = (120*i*degToRad - seperationAngle/2)
            vec1 = np.array([r_base*np.cos(theta1), r_base*np.sin(theta1), 0])
            vec2 = np.array([r_base*np.cos(theta2), r_base*np.sin(theta2), 0])
            self.base_joint_pos.append(vec1)
            self.base_joint_pos.append(vec2)
            
        #Calculate actuator connection points on platform (in platform frame) (store as list of np vectors)
        self.platform_joint_pos = []
        for i in range(self.numActuators/2):
            theta1 = (120*i*degToRad - seperationAngle/2 + PlatformTwist)
            theta2 = (120*i*degToRad + seperationAngle/2 - PlatformTwist)
            vec1 = np.array([r_platform*np.cos(theta1), r_platform*np.sin(theta1), 0])
            vec2 = np.array([r_platform*np.cos(theta2), r_platform*np.sin(theta2), 0])
            self.platform_joint_pos.append(vec1)
            self.platform_joint_pos.append(vec2)
            
        #Initialize empty containers for other variables
        self.Jacobian = np.zeros((self.numActuators, self.numActuators))
        self.q = np.zeros(self.numActuators) #Actuator lengths (since they are prismatic) 
        self.tau = np.zeros(self.numActuators) #Forces exerted by the actuators
            
    
    '''Computes inverse kinematics and updates relevant object variables. 
    Returns True if position is within range of joint limits, otherwise returns 
    False. '''        
    def InverseKinematics(self, platformPos_b, EulerAngles):
        #Extract our Euler Angles
        Alpha = EulerAngles[0]
        Beta  = EulerAngles[1]
        Gamma = EulerAngles[2]
        
        #Compute Rotation Matrix (Left handed rotation from platform frame to base frame)
        R_p_b = zyx_euler_angles_to_mat(Alpha, Beta, Gamma)
        
        #Calculate vector from base connection point to platform connection point for each actuator (in base frame)
        feasible = True
        self.b_l = []
        for i in range(self.numActuators):
            b_li = platformPos_b+np.dot(R_p_b,self.platform_joint_pos[i]) - self.base_joint_pos[i]  #Find vector from bi to pi in base frame
            self.b_l.append(b_li)            #Store np arrays of position vectors in a list            
            self.q[i] = np.linalg.norm(b_li) #Store norm of vector
            
            #Check joint limits. If joint limits are exceeded, return false
            #and assign -1 to offending joint's position.
            if self.q[i] > self.lmax or self.q[i] < self.lmin:
                feasible = False
                self.q[i] = -1
                
        return feasible


    '''Computes Jacobian and updates relevant object variables. 
    Returns True if position is within range of joint limits, otherwise returns 
    False. '''    
    def ComputeJacobian(self, platformPos_b, EulerAngles):
        
        #Extract our Euler Angles
        Alpha = EulerAngles[0]
        Beta  = EulerAngles[1]
        Gamma = EulerAngles[2]
        
        #Compute Rotation Matrix (rotates vector from platform frame to base frame)
        R_p_b = zyx_euler_angles_to_mat(Alpha, Beta, Gamma)
        
        #Update joint positions using Inverse Kinematics
        feasible = self.InverseKinematics(platformPos_b, EulerAngles)
        
        self.Jacobian2 = np.zeros((6,6))
        
        if feasible:
            #Update each row of the Jacobian matrix
            for i in range(self.numActuators):
                self.Jacobian[i,:] = (1.0/self.q[i]*
                    np.append(self.b_l[i], np.cross(np.dot(R_p_b,self.platform_joint_pos[i]),
                    (platformPos_b-self.base_joint_pos[i])))  )
                    
                    
                #Check against alternate formulation
                unit_vec = self.b_l[i]/np.linalg.norm(self.b_l[i])
                self.Jacobian2[i,:] = np.append(unit_vec, np.cross(np.dot(R_p_b, self.platform_joint_pos[i]), unit_vec))
            return True
        else:
            return False
    
    '''Computes the workspace of the platform for a certain Euler
    Angle orientation'''     
    def GeometricWorkspace(self, yRange, zRange, resolution, EulerAngles):
        yMin, yMax = yRange
        zMin, zMax = zRange
        
        pY = np.array([])
        pZ = np.array([])
    
        for pos_y in np.arange(yMin, yMax + resolution, resolution):
            for pos_z in np.arange(zMin, zMax + resolution, resolution):
                # Platform position
                platformPos = np.array([0, pos_y, pos_z])
                
                #See if the position and orientation is feasible
                feasible = self.InverseKinematics(platformPos, EulerAngles)
      
                if feasible:
                    pY = np.append(pY, pos_y)
                    pZ = np.append(pZ, pos_z)
        return pY, pZ

    '''Calculate the max force and torque the platform can resist in a certain 
    configuration. Force direction is given, and constant torque term is given '''
    def MaxLoad(self,platformPos_b, EulerAngles, appliedForceDirection_p, appliedTorque_p):
        #Run a constrained optimization routine
        # - Magnitude of actuator forces are design variables
        # - norm of actuator torques is objective function
        # - Actuator limits are the constraints
        
        #Extract our Euler Angles
        Alpha = EulerAngles[0]
        Beta  = EulerAngles[1]
        Gamma = EulerAngles[2]
        
        #Compute Rotation Matrix (rotates vector from platform frame to base frame)
        R_p_b = zyx_euler_angles_to_mat(Alpha, Beta, Gamma)
        
        #Initial condition
        x0 = 0
        
        #Constraints
        def lowerActuatorLimit(x):
            #Put together combined force and torque vector on the platform as seen in the base frame
            forceOnPlatform_b = (np.append(np.dot(R_p_b,appliedForceDirection_p*-x), 
                                           np.dot(R_p_b,appliedTorque_p)) )
            #Compute the needed actuator torques to support the force on platform
            tau = self.ComputeTorquesUnderLoad(platformPos_b, EulerAngles, forceOnPlatform_b)
            return tau - self.Fmin
            
        def upperActuatorLimit(x):
            #Put together combined force and torque vector on the platform as seen in the base frame
            forceOnPlatform_b = (np.append(np.dot(R_p_b,appliedForceDirection_p*-x), 
                                           np.dot(R_p_b,appliedTorque_p)) )
            tau = self.ComputeTorquesUnderLoad(platformPos_b, EulerAngles, forceOnPlatform_b)
            return self.Fmax - tau
            
        ineq_cons1 = {'type': 'ineq',
                      'fun' : lowerActuatorLimit}
                     
        ineq_cons2 = {'type': 'ineq',
                      'fun' : upperActuatorLimit}
            
        #Define our objective function
        def obj(x):
            return -x
            #forceOnPlatform_b = (np.append(np.dot(R_p_b,appliedForceDirection_p*-x), 
            #                               np.dot(R_p_b,appliedTorque_p)) )
            #return -np.linalg.norm(self.ComputeTorquesUnderLoad(platformPos_b, EulerAngles, forceOnPlatform_b))
        
        #Run optimization routine
        res = minimize(obj, x0, method='SLSQP',constraints = [ineq_cons1,ineq_cons2],
                       options={'ftol': 1e-9, 'disp': False})
        
        maxF = res.x[0]
        forceOnPlatform_b = (np.append(np.dot(R_p_b,appliedForceDirection_p*-res.x[0]), 
                                           np.dot(R_p_b,appliedTorque_p)) )
        tau = self.ComputeTorquesUnderLoad(platformPos_b, EulerAngles, forceOnPlatform_b)
        
        return maxF, tau
        
        
    '''Calculate the torques on all prismatic joints necessary to 
    withstand a certain force and torque applied at the platform's center. 
    
    INPUT: forceOnPlatform_b is a np array that either has three elements (xyz 
    forces) or six elements (xyz forces and torques).'''
    def ComputeTorquesUnderLoad(self, platformPos, EulerAngles, forceOnPlatform_b):
        # J.T * tau = F -> tau = inv(J.T) * F
        # J (6, 6) matrix in base frame
    
        #If we are just given the force on the platform, augment the vector
        #to include the zero torque vector 
        if forceOnPlatform_b.shape[0] == 3:
            forceOnPlatform_b = np.append(forceOnPlatform_b, np.array((0,0,0)) )
        
        #Update Jacobian and ensure configuration is feasible
        feasible = self.ComputeJacobian(platformPos, EulerAngles)
        
        if feasible:
            invJ_T = np.linalg.pinv(self.Jacobian.T)
        
            # torques (push forces) on prismatic joints of all hydraulic cylinders
            self.tau = np.dot(invJ_T, forceOnPlatform_b)
            return self.tau
        else:
            return None
            
            
            b_li = T+np.dot(R_b_p,p[i]) - b[i]        #Find vector from bi to pi in base frame

    def WorkspaceUnderLoad(self, yRange, zRange, resolution, EulerAngles, appliedForceDirection, appliedTorque):
        # need to first do geometric workspace
        # the point should first satisfy the geometric constraints and then meet the needs of hydraulic forces
        pYGeometric, pZGeometric = self.GeometricWorkspace(yRange, zRange, resolution, EulerAngles)
        
        assert pYGeometric.shape[0] == pZGeometric.shape[0], "The shape of pYGeometric and the shape of pZGeometric are not the same."
        
        #Stack points of interest into 2xn array
        p = np.vstack((pYGeometric, pZGeometric))
        
        MaxForces = []
        
        for i in range(p.shape[1]):
             platformPos = np.array([0, p[0,i], p[1,i]])
             maxF, tau = self.MaxLoad(platformPos, EulerAngles, appliedForceDirection, appliedTorque)
             MaxForces.append(maxF)
        
        pY = p[0,:]
        pZ = p[1,:]
        return pY, pZ, MaxForces


if __name__ == '__main__':
    
    
    ########################################
    #----------Problem Parameters----------#
    ########################################
    
    #Plotting Parameters
    PlotArrangement = 0
    PlotWorkspace = 0
    
    #Computing Parameters
    ComputeLoadWorkspace = 0
    
    #Type of hydraulic cylinder (BAILEY or WOLVERINE)
    HydraulicType = "BAILEY" 
    
    #---Stewart Platform parameters----
    r_base = 6.54*inchesToM      #radius of base mounting circle (m)
    r_platform = 6.54/2*inchesToM  #radius of platform mounting circle (m)
    s = 46.0*degToRad #Seperation Angle between actuators on triangle corners
    PlatformTwist = 60.0*degToRad 
    
    
    
    ###########################################################################
    #----------Applied Force and Configuration Parameters----------------------
    ###########################################################################
    
    #----(1) Determine max force magnitude in given direction with a given torque---
    #Force direction angles
    alpha = 0.0*degToRad            #Rotation about z in platform frame
    beta =  (90.0+0.0)*degToRad     #Rotation about x in platform frame
    #Unit vector of applied force (in platform frame)
    appliedForceDirection_p = np.array([np.cos(alpha)*np.cos(beta),np.sin(alpha)*np.cos(beta),-np.sin(beta)])
    #Applied torque vector (in platform frame)
    appliedTorque_p = np.array([0,0,600])*NewtonsTokN #Need to convert to kN.m to keep things consistent
    
    
    #---(2) Determine actuator forces needed to balanced a given WOB, TOQ, and Disturbance force/torque---
    WOB = np.array([0,0,-10])
    TOB = np.array([0,0,-0.6])
    Disturbance = np.array([-1,0,0])

    #----Configuration To Test------
    #Translational different between base frame and platform frame (T) (Seen in base frame)
    T = np.array([1.0, 0 , 12.5])*inchesToM #(x,y,z), m
    
    #Euler angles between base frame and platform frame (z,y,x) (From base frame)
    Alpha = 0.0*degToRad    #Z rotation (twist)
    Beta =  0.0*degToRad   #Y rotation (tilt)
    Gamma = 7.0*degToRad    #X rotation (tilt)
    EulerAngles = np.array([Alpha,Beta,Gamma])
    ###########################################################################
    ###########################################################################
    
    
    #-----Workspace Analysis parameters-----
    yRange = np.array([-5, 5])*inchesToM # m
    zRange = np.array([9, 14.5])*inchesToM   # m
    resolution = 0.15*inchesToM             # m
    
    
    #############################################
    ##---------------TESTING-------------------##
    #############################################
    
    #Max force that can be applied in zero configuration 
    MAX_F = 235.3
    
    #Define our cylinder and Stewart Platform objects
    Cylinder = HydraulicCylinder(HydraulicType)
    SP = StewartPlatform(r_base, r_platform, s, PlatformTwist, Cylinder)
    
    #Determine if configuration defined by T and EulerAngles is feasible
    configFeasible = SP.InverseKinematics(T,EulerAngles)

    #Compute workspace for Constant platform orientation
    pY, pZ = SP.GeometricWorkspace(yRange, zRange, resolution, EulerAngles)
    
    #Determine actuator forces needed to balanced a given WOB, TOQ, and disturbance/disturbance induced torque 
    R_p_b = zyx_euler_angles_to_mat(Alpha, Beta, Gamma)     
    forceOnPlatform_b = np.append(np.dot(R_p_b,(WOB+Disturbance)), np.dot(R_p_b,(TOB-np.array([0,Disturbance[0]*(2*inchesToM),0]))))
    tau = SP.ComputeTorquesUnderLoad(T, EulerAngles, -forceOnPlatform_b)
    
    #Compute the max Force magnitude (F) that the given configuration can balance.
    #Compute the actuators forces needed to do this.
    maxF, maxTau = SP.MaxLoad(T,EulerAngles,appliedForceDirection_p, appliedTorque_p)
    
    
    #############################################
    ##--------------PRINT RESULTS--------------##
    #############################################
    np.set_printoptions(precision=2)
    print "\n--------TEST-----------\n"
    print("Base to platform displacement (x,y,z) (in):")
    print(T*mToInches)
    print("\nEuler Angles, (z,y,x) (deg):")
    print(EulerAngles*radToDeg)
    print("\nConfiguration feasible?")
    print(configFeasible)
    
    print("\nApplied force orientation (in platform frame):")
    print("alpha (z rotation, deg)")
    print(alpha*radToDeg)
    print("beta (x rotation, deg)")
    print(90-beta*radToDeg)
    
    print("\nmaxF in configuration is (kN):")
    print(np.array([maxF]))
    print(np.array([maxF])/MAX_F) #Fraction of maxF at zero configuration
    
    print("\nActuator forces to achieve maxF are (kN):")
    print(maxTau)
    
    #Print tau as the fraction of total available force in that directoin
    fractionTau = []
    for force in maxTau:
        if force > 0:
            fractionTau.append(force/Cylinder.maxPushingForce)
        elif force < 0:
            fractionTau.append(force/Cylinder.maxPullingForce)
    print(np.array(fractionTau))
        
    print("\n--------------------\n")
    print("WOB, TOB, Disturbance (kN): ")
    print(WOB)
    print(TOB)
    print(Disturbance)
    print("Total Force/Torque on platform (p_frame)")
    print(forceOnPlatform_b)
    print("Actuator Torques")
    print(tau)
    fractionTau = []
    for force in tau:
        if force > 0:
            fractionTau.append(force/Cylinder.maxPushingForce)
        elif force < 0:
            fractionTau.append(force/Cylinder.maxPullingForce)
    print(np.array(fractionTau))
    
    #############################################
    ##---------ERROR CHECKING------------------##
    #############################################
    #Constraint - Moments must be balanced around the point where the force is applied
    p_l_hat = []
    R_p_b = zyx_euler_angles_to_mat(Alpha, Beta, Gamma)
    for i in range(6):
        b_li_hat = SP.b_l[i]/np.linalg.norm(SP.b_l[i])     #Convert to unit vector
        p_li_hat = np.dot(R_p_b.T, b_li_hat)     #Rotate into platform frame
        p_l_hat.append(p_li_hat)                 #Add to our list
        
    #Obtain actuator forces on platform (Force magnitude (xi) times force unit vectors (p_li_hat))
    ActuatorForces = [np.dot(Fi,p_li_hat) for Fi,p_li_hat in zip(maxTau,p_l_hat)] 
    
    #Compute the moments about the point of applied force
    Moments = [np.cross(pi,Li) for pi,Li in zip(SP.platform_joint_pos,ActuatorForces)] 
    
    #Return the sum of x moments (should be zero at equilibrium)
    print ("\nThe sum of moments is: ")
    print (sum(Moments))
    
    
    #-------------------------------------------------------------------
    if(ComputeLoadWorkspace):
        #Compute the Load workspace 
        pY_F, pZ_F, MaxForces = SP.WorkspaceUnderLoad(yRange, zRange, resolution, EulerAngles, appliedForceDirection_p, appliedTorque_p)

        
    #%%############################
    ##------PLOTTING-------------##
    ###############################
    yPlotLimit = np.array([-5, 5])
    zPlotLimit = np.array([9, 14.5])
    
    #Plot the base actuator points
    if(PlotArrangement):
        plt.figure(1)
        plt.clf()
        B = np.array(SP.base_joint_pos)*mToInches
        P = np.array(SP.platform_joint_pos)*mToInches
        plt.scatter(B[:,0],B[:,1], LineWidth = 3)
        plt.scatter(P[:,0],P[:,1], LineWidth = 3)
        plt.xlabel('x (in)')
        plt.ylabel('y (in)')
        plt.title('Top view of Stewart Platform in Zero Configuration')
        plt.legend(('base','platform',''),loc=10)
        
        #Plot actuator connections
        for i in range(SP.numActuators):
            plt.plot((B[i,0],P[i,0]),(B[i,1],P[i,1]), 'r')
            
        plt.grid(True)
        plt.axis('square')
        
    #Plot Constant Orientation Workspace
    if(PlotWorkspace):
        plt.figure(2)
        plt.clf()
        plt.title('Constant Orientation Workspace')
        plt.scatter(pY*mToInches, pZ*mToInches, marker="o")
        plt.xlim(yPlotLimit) # -10.5, 10.5
        plt.ylim(zPlotLimit) #6, 14
        plt.gca().set_aspect('equal', adjustable='box')
        plt.ylabel('z (in)')
        plt.xlabel('y (in)')

#%%        
    if(ComputeLoadWorkspace):
        plt.figure(3)
        plt.clf()
        plt.title('Load Capacity of Constant Orientation Workspace')
        sc1 = plt.scatter(pY_F*mToInches, pZ_F*mToInches, c = MaxForces, marker="o")
        plt.title("Max Pushing Load")
        plt.xlim(yPlotLimit) # yPlotLimit -10.5, 10.5
        plt.ylim(zPlotLimit) # zPlotLimit 6, 14
        plt.gca().set_aspect('equal', adjustable='box')
        plt.xlabel('Y (in)')
        plt.ylabel('Z (in)')
        clb = plt.colorbar(sc1,fraction=0.03)
        clb.ax.set_title('tau (kN)')
        
            
#%% Plot platform, base, actuator forces and applied force in 3d 
    if(1):
        SP.InverseKinematics(T,EulerAngles)
        fig = plt.figure(4)
        fig.clf()
        ax = fig.add_subplot(2,1,1,projection = '3d')
        
        B = np.array(SP.base_joint_pos).T*mToInches
        R_p_b = zyx_euler_angles_to_mat(Alpha, Beta, Gamma)
        #Rotate the P points to the base frame orientation
        P = np.dot(R_p_b,np.array(SP.platform_joint_pos).T*mToInches)
        
        #Add the offset to the platform frame points
        T_in = T*mToInches
        P[0,:] = P[0,:] + T_in[0]
        P[1,:] = P[1,:] + T_in[1]
        P[2,:] = P[2,:] + T_in[2]
        
        #Plot Joint connections
        ax.scatter(B[0,:],B[1,:],B[2,:], linewidth = 3)
        ax.scatter(P[0,:],P[1,:],P[2,:], linewidth = 3)
        
        #Plot leg lines in red
        for i in range(6):
            b_l = SP.b_l[i]*mToInches
            E = B[:,i] + b_l
            ax.plot((B[0,i],E[0]),(B[1,i],E[1]),(B[2,i],E[2]),'r')
        
        #Plot retracted leg lines in blue
        r = Cylinder.retractLength*mToInches
        for i in range(6):
            R = (P[:,i] - B[:,i])
            R = R/np.linalg.norm(R)*r
            R = B[:,i]+R
            ax.plot((B[0,i],R[0]),(B[1,i],R[1]),(B[2,i],R[2]),'b', linewidth = 3.5)
        
        #Plot line from center of base to center of platform
        ax.plot((0,T_in[0]),(0,T_in[1]),(0,T_in[2]),'b--')  
        
        #Plot actuator force vectors
        ActVecs = SP.b_l/np.linalg.norm(SP.b_l)
        scale = 0.4
        ActVecs[:,0] = ActVecs[:,0]*SP.tau*scale
        ActVecs[:,1] = ActVecs[:,1]*SP.tau*scale
        ActVecs[:,2] = ActVecs[:,2]*SP.tau*scale
        for i in range(6):
            ax.quiver(P[0,i],P[1,i],P[2,i],ActVecs[i,0],ActVecs[i,1],ActVecs[i,2],color='g')
       
        #Reorder B and P for plotting
        permutation = [1,0,3,2,5,4]
        i = np.argsort(permutation)
        B_re = np.hstack((B[:,i],B[:,1].reshape(3,1)))
        P_re = np.hstack((P[:,i],P[:,1].reshape(3,1)))
        
        #Plot base and platform perimeter
        ax.plot(B_re[0,:],B_re[1,:],B_re[2,:], 'grey', linewidth = 3)
        ax.plot(P_re[0,:],P_re[1,:],P_re[2,:], 'grey', linewidth = 3)
        
        #Plot applied force vector (not to scale)
        a = 3
        u = np.dot(R_p_b,appliedForceDirection_p)*a
        T_f = T_in-u
        ax.quiver(T_f[0],T_f[1],T_f[2],u[0],u[1],u[2])
        
         
        #Plot base and platform polygons
        PolyCollections = []
        PolyCollections.append(Poly3DCollection([B_re.T],facecolor = 'grey', lw = 2,alpha = 0.2))
        PolyCollections.append(Poly3DCollection([P_re.T],facecolor = 'grey', lw = 2,alpha = 0.2))

        ax.add_collection3d(PolyCollections[0])
        ax.add_collection3d(PolyCollections[1])
        
        #Set axis labels
        ax.set_title("Stewart Platform")
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.axis('square')
        ax.view_init(25,-15)
        
        #-----------Second subplot (XZ Plane)----------------------------
        ax = fig.add_subplot(2,1,2)
        
        ax1 = 1 #Choose 0 for x, 1 for y
        
        ax.scatter(B[ax1,:],B[2,:], linewidth = 3)
        ax.scatter(P[ax1,:],P[2,:], linewidth = 3)
        
        #Plot actuator lines
        for i in range(6):
            ax.plot((B[ax1,i],P[ax1,i]),(B[2,i],P[2,i]),'r')
        
        #Plot rectracted lengths
        for i in range(6):
            R = (P[:,i] - B[:,i])
            R = R/np.linalg.norm(R)*r
            R = B[:,i]+R
            ax.plot((B[ax1,i],R[ax1]),(B[2,i],R[2]),'b', LineWidth = 3.5)
        
        
        #Plot joint connections
        ax.plot(B_re[ax1,:],B_re[2,:], 'grey', LineWidth = 3)
        ax.plot(P_re[ax1,:],P_re[2,:], 'grey', LineWidth = 3)
        
        #Plot line from center of base to center of platform
        ax.plot((0,T_in[ax1]),(0,T_in[2]),'b--')  
        
        #Plot actuator force vectors
        for i in range(6):
            ax.quiver(P[ax1,i],P[2,i],ActVecs[i,ax1],ActVecs[i,2],scale = 16,color='g')
        
        #Plot applied force vector
        ax.quiver(T_f[ax1],T_f[2],u[ax1],u[2],scale = 14)
        
        if ax1 == 0:
            ax.set_xlabel('X')
        else:
            ax.set_xlabel('Y')
            
        ax.set_ylabel('Z')
        ax.axis('square')
        ax.axis([-7,7,-1,19.5])
        
        #ax.view_init(0,90)
        #plt.show()
        
