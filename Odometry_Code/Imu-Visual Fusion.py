#!/usr/bin/env python


import pickle
import numpy as np
import matplotlib.pyplot as plt
with open('imu_readings.pkl','rb') as f:
    data=pickle.load(f)





time=data['time']
quaternion=data['Quaternion']
angular_vel=data['Angular Velocity']
acc=data['Acceleration']
rot_vel_cov=data['Angular Vel Cov'][0]    #Covariance matrix for an IMU is constant, hence taking matrix recorded 
quaternion_cov=data['Orientation Cov'][0] #at the first reading
acc_cov=data['linear_acc_cov'][0]




class Quaternion():
    def __init__(self,w=1.,x=0.,y=0.,z=0.,axis_angle=None,euler=None):
        if euler is None:
            self.w=w
            self.x=x
            self.y=y
            self.z=z

        else:
            roll=euler[0]
            pitch=euler[1]
            yaw=euler[2]
            
            cr=np.cos(roll*0.5)
            sr=np.sin(roll*0.5)
            cp=np.cos(pitch*0.5)
            sp=np.sin(pitch*0.5)
            cy=np.cos(yaw*0.5)
            sy=np.sin(yaw*0.5)
            
            self.w=cr * cp * cy + sr * sy * sp
            self.x=sr * cp * cy - cr * sp * sy
            self.y=cr * sp * cy + sr * cp * sy
            self.z=cr * cp * sy - sr * sp * cy
 
    def to_mat(self):
        q=np.array([self.x,self.y,self.z]).reshape(3,1)
        i=(self.w**2-np.dot(q.T,q))*np.identity(3)
        j=2*np.dot(q,q.T)
        k=2*self.w*skew_symmetric(q)
        return i+j+k
    def to_numpy(self):
        return np.array([self.w,self.x,self.y,self.z])
    def normalize(self):
        mag=np.linalg.norm([self.w,self.x,self.y,self.z])
        return Quaternion(self.w/mag,self.x/mag,self.y/mag,self.z/mag)
    def quat_mult_right(self,q,out='np'):
        quat=np.array([self.x,self.y,self.z]).reshape(3,1)
        s=np.zeros([4,4])
        s[0,1:]=-quat[:,0]
        s[1:,0]=quat[:,0]
        s[1:,1:]=-skew_symmetric(quat)
        sigma=self.w*np.identity(4)+s
        
        if type(q).__name__=='Quaternion':
            prod_np=np.dot(sigma,q.to_numpy())
        else:
            prod_np=np.dot(sigma,q)
        if out=='np':
            return prod_np
        elif out=='Quaternion':
            prod=Quaternion(prod_np[0],prod_np[1],prod_np[2],prod_np[3])
            return prod
    def quat_mult_left(self,q,out='np'):
        quat=np.array([self.x,self.y,self.z]).reshape(3,1)
        s=np.zeros([4,4])
        s[0,1:]=-quat[:,0]
        s[1:,0]=quat[:,0]
        s[1:,1:]=skew_symmetric(quat)
        sigma=self.w*np.identity(4)+s
        
        if type(q).__name__=='Quaternion':
            prod_np=np.dot(sigma,q.to_numpy())
        else:
            prod_np=np.dot(sigma,q)
        if out=='np':
            return prod_np
        elif out=='Quaternion':
            prod=Quaternion(prod_np[0],prod_np[1],prod_np[2],prod_np[3])
            return prod




def angle_normalize(a):
    #Returns angle in the range of [-pi,pi]
    return np.arctan2(np.sin(float(x)),np.cos(float(x)))
def skew_symmetric(v):

    skew=np.array([[0,  -v[2], v[1]],
                    [ v[2], 0,  -v[0]],
                    [-v[1],v[0], 0]],dtype=np.float64)
    return skew




gravity=np.array([0,0,-9.81]) #gravity
motion_jac=np.zeros([9,6]) #motion model noise jacobian
motion_jac[3:,:]=np.identity(6) 
meas_jac=np.zeros([3,9])
meas_jac[:,:3]=np.identity(3) #measurement model jacobian


position_est = np.zeros([len(time), 3])  # position estimates
velocity_est = np.zeros([len(time), 3])  # velocity estimates
quaternion_est = np.zeros([len(time), 4])  # orientation estimates as quaternions
p_covariance = np.zeros([len(time), 9, 9])  # covariance matrices at each timestep




position_est[0]=np.array([0.000399,0.000498,0.189597])
velocity_est[0]=np.array([0,0,0])
e=[-0.000001,-0.001190,0.000994]
quaternion_est[0]= np.array(Quaternion(euler=np.array(e)).to_numpy())

l_k=np.zeros([9,6])
l_k[3:9,:]=np.identity(6)
Q=np.identity(6)
Q[0:3,0:3]*=np.array(acc_cov).reshape(3,3)
Q[3:6,3:6]*=np.array(rot_vel_cov).reshape(3,3)



for k in range(1,len(time)):
    delta_t=time[k]-time[k-1]
    if delta_t==0:
        position_est[k]=position_est[k-1]
        velocity_est[k]=velocity_est[k-1]
        quaternion_est[k]=quaternion_est[k-1]
    else:    
        Q_k=Q*delta_t**2
        C_ns=Quaternion(*quaternion_est[k-1]).to_mat()
        #Update with IMU Inputs
        position_est[k]=position_est[k-1]+delta_t*velocity_est[k-1]+delta_t**2*(C_ns@np.array(acc[k-1])+gravity)/2
        velocity_est[k]=velocity_est[k-1]+delta_t*(C_ns@np.array(acc[k-1])+gravity)
        q=Quaternion(euler=(angular_vel[k-1]*delta_t)).quat_mult_right(quaternion_est[k-1])
        quaternion_est[k]=Quaternion(*q).normalize().to_numpy()




est_traj_fig = plt.figure()
ax = est_traj_fig.add_subplot(111, projection='3d')
ax.plot(position_est[:,0], position_est[:,1], position_est[:,2], label='Estimated')
ax.set_xlabel('Easting [m]')
ax.set_ylabel('Northing [m]')
ax.set_zlabel('Up [m]')
ax.set_xlim(0, 200)
ax.set_ylim(0, 200)
ax.set_zlim(-2, 2)
ax.set_xticks([0, 50, 100, 150, 200])
ax.set_yticks([0, 50, 100, 150, 200])
ax.set_zticks([-2, -1, 0, 1, 2])
ax.legend(loc=(0.62,0.77))
ax.view_init(elev=45, azim=-50)
plt.show()






