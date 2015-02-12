#!/usr/bin/env python
"""
Created on Feb 9 2015   

@author: Ruoyu Tan
"""
import rospy
import numpy 
import scipy.io
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped

class UnscentedKF():
    """Uncented KalmanFilter 

    Attributes:
        likes_spam: A boolean indicating if we like SPAM or not.
        eggs: An integer count of the eggs we have laid.
    """
    
    def __init__(self,sys_dyn,mea_dyn,Q_EUL,P_EUL,P_INIT,X_INIT):
        """Initiates Uncented Kalman Filter """
        self.x_hat=X_INIT
        CONST_ALPHA=1   #Give a value for the spread of sigma point
        CONST_BETA=2   #Give a value for the prior knowledge of distribution
        self.CONST_N=self.x_hat.size   #Give a value for scaling parameter
        CONST_LAMBDA=CONST_ALPHA*numpy.sqrt(CONST_N)
        # Define Wm Matrix
        self.W_M=numpy.zeors([2*CONST_N+1,1],dtype=float)
        self.W_M[0][0]=((CONST_ALPHA**2)*(CONST_N)-CONST_N)/(CONST_N+CONST_LAMBDA)
        for i in range(0,2*CONST_N+1):
            self.W_M[i]=1/(2*CONST_N+2*CONST_LAMBDA)
        # Define Wc Matrix        
        self.W_C=numpy.eye(2*CONST_N+1,dtype=float)
        self.W_C[0][0]=CONST_LAMBDA/(CONST_N+CONST_LAMBDA)+(1-CONST_ALPHA**2+CONST_BETA)
        for i in range(0,2*CONST_N+1):
            self.W_C[i][i]=self.W_M[i]
        self.p_qua #Covariance matrix in quaternion form
        self.P0_QUA
    
    def time_update(self,x_pre,q_qua,x_input):
        """Time Update Part in UKF

        Attributes:
            likes_spam: A boolean indicating if we like SPAM or not.
            eggs: An integer count of the eggs we have laid.
        """
        # find cholesky but check for non positive definite matrices
        # need to calculate eignvalue!!! show eigenvalue greater than one
        w,v=numpy.linalg.eig(self.p_qua)
        for i in range(0,CONST_N+1):
            if w[i]<0:
                self.p_qua=self.P0_QUA
                break
        l_chol = numpy.linalg.cholesky(self.p_qua) # returns lower
        # Calculate sigma points
        x_plus=numpy.tile(self.CONST_N,numpy.array([x_pre+CONST_LAMBDA*L_chol]))
        x_mius=numpy.tile(self.CONST_N,numpy.array([x_pre-CONST_LAMBDA*L_chol]))
        x_self=numpy.array([x_pre])
        x_chi_prior=numpy.concatenate((x_self,x_plus,x_minus))
        self.x_chi_current=self.sys_dym(x_chi_prior)
        self.x_hat_prior=numpy.dot(x_chi_current,self.W_M)
        temp_prior = x_chi_current-numpy.tile(self.x_hat_prior, (1, (2*self.n+1)))
        p_prior=numpy.dot(temp_prior,numpy.dot(W_C,temp_prior.T))+q_qua
        
    def measurement_update(self,mea_data):
         """Measurement Update Part in UKF

        Attributes:
            likes_spam: A boolean indicating if we like SPAM or not.
            eggs: An integer count of the eggs we have laid.
        """
        y_chi_prior=self.mea_dym(mea_data,self.x_chi_current)
        self.y_hat_prior=y_chi_prior*self.W_M
        temp_x = self.x_chi_current-numpy.tile(self.x_hat_prior, (1, (2*self.n+1)))
        temp_y = self.y_chi_current-numpy.tile(self.y_hat_prior, (1, (2*self.n+1)))
        p_yy=numpy.dot(temp_y,numpy.dot(W_C,temp_y.T))+r_qua
        p_xy=numpy.dot(temp_x,numpy.dot(W_C,temp_y.T))
        k_temp = numpy.dot(p_xy, numpy.linalg.inv(p_yy))
        self.x_hat=self.x_hat_prior+numpy.dot(k_temp,(y_input-self.y_hat_prior))
        p_estimate=p_prior-numpy.dot(k_temp,numpy.dot(p_yy,k_temp.T))
        
class AttitudeFilter():
    """Attitude estimator for AUV by fusing IMU data 

    Attributes:
        likes_spam: A boolean indicating if we like SPAM or not.
        eggs: An integer count of the eggs we have laid.
    """

    def __init__(self):
        """Initiates AttitudeFilter """
        P_INIT=1000.0*numpy.eye(4,dtype=float)   # Initialize covariance matrix
        # Define process noise matrix in Euler form
        Q_EUL=numpy.eye(3,dtype=float)   
        Q_EUL[0][0]=0.01   # Uncertainty in pitch angle when system subject flucturation
        Q_EUL[1][1]=0.01   # Uncertainty in roll angle when system subject flucturation
        Q_EUL[2][2]=0.01   # Uncertainty in yaw angle when system subject flucturation
        R_EUL=numpy.eye(6,dtype=float)  
        # Define measurement noise matrix in Euler form
        R_ACC[0][0]=0.01   # Covariance error for acclometer in x direction
        R_ACC[1][1]=0.01   # Covariance error for acclometer in y direction
        R_ACC[2][2]=0.01   # Covariance error for acclometer in z direction
        R_MAG[0][0]=0.01   # Covariance error for magnometer in x direction
        R_MAG[1][1]=0.01   # Covariance error for magnometer in y direction
        R_EUL[2][2]=0.01   # Covariance error for magnometer in z direction
        # Define initial states
        X_INIT=numpy.zeros([4,1],dtype=float)
        # initialize a UKF with this class's members
        self.uncented_kf= UnscentedKF(self.system_dynamics, self.measurement_dynamics, Q, R, P0, x0)
    
    def sys_dym(self,x_input,x_val):
        system_matrix=numpy.matrix([[1,-x_input[0]*self.dt/2,-x_input[1]*self.dt/2,-x_input[2]*self.dt/2]...,[0,1,dt,0],[0,0,1,0],[0,0,0,1]])
        x_chi=        
        return 
    
    def mea_dym(self,)
    
    def acc_update(self,data):
        
    def gyro_update(self,data):
        
    def mag_update(self,data):
        mag_mea=[data.vector.x,data.vector.y,data.vector.z]
        mea_type=2;
        self.uncented_kf=measurement_update(meag_mea,mea_type)        
        
def mainloop():
    # Starts the node
    rospy.init_node('att_fus')
    pub = rospy.Publisher('/energy_state', EnergyState)
    i_filter = AttitudeFilter()   # Define a variable as AttitudeFilter class
    # Subscribes accelometer, gyro and magnometer data from IMU for our filter
    sub_accel = rospy.Subscriber('/auv_accel',Vector3Stamped, i_filter.acc_update)
    sub_pose = rospy.Subscriber('/auv_pose', PoseStamped, i_filter.gyro_update)
    sub_mag = rospy.Subscriber('/auv_mag', Vector3Stamped, i_filter.mag_update)    
    while not rospy.is_shutdown():
        rospy.sleep(0.01)

# Main node function
if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException: pass
