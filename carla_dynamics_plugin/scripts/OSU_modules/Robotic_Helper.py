class _RoboticHelper():
    
    #return numpy representation of SE3 transform matrix
    #
    # param carlaTransform: carla object representing a transformation
    # return: numpy transform matrix (as defined in reference[1] modern_robotics import textbook)
    #
    #   [[Calpha*Cbeta Calpha*Sbeta*Sgamma-Salpha*Cgamma Calpha*Sbeta*Cgamma+SalphaS*gamma x],
    #    [Salpha*Cbeta Salpha*Sbeta*Sgamma+Calpha*Cgamma Salpha*Sbeta*Cgamma-Calpha*Sgamma y],
    #    [-Sbeta       Cbeta*Sgamma                      Cbeta*Cgamm                       z],
    #    [0            0                                 0                                 1]]
    # 

    #TODO: update function
    def to_transform(self,carlaTransform):
        rotation = self.to_rotation(carlaTransform.rotation)
        location = carlaTransform.location

        x = location.x
        y = location.y
        z = location.z

        transform = np.array([])
        
        return transform
    
    #return numpy representation of SO3 rotation matrix
    #
    # param carlaTransform: carla object representing a transformation
    # return: numpy transform matrix (as defined in reference[1] modern_robotics import textbook)
    #
    #   [[Calpha*Cbeta Calpha*Sbeta*Sgamma-Salpha*Cgamma Calpha*Sbeta*Cgamma+SalphaS*gamma],
    #    [Salpha*Cbeta Salpha*Sbeta*Sgamma+Calpha*Cgamma Salpha*Sbeta*Cgamma-Calpha*Sgamma],
    #    [-Sbeta       Cbeta*Sgamma                      Cbeta*Cgamm                      ]]
    # 
    def to_rotation(self,carlaRotation):
        roll = math.radians(carlaRotation.roll)
        pitch = math.radians(carlaRotation.pitch)
        yaw = math.radians(carlaRotation.yaw)

        if (roll == 360) or (roll ==-360):
            roll = 0
        if (pitch == 360) or (pitch == -360):
            pitch = 0
        if (yaw == 360) or (yaw == -360):
            yaw = 0

        yaw_matrix = np.array([[math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]])

        pitch_matrix = np.array([[math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]])

        roll_matrix = np.array([[1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]])

        return np.matmul(yaw_matrix, pitch_matrix, roll_matrix)

    #adjust frame of refrence
    #
    # param rotation_matrix: SO3 representaion of another frame of refeennce 
    # param array: current frame of reference frame
    #
    # return: roation_matrix*array -> the representation of array in new frame
    #
    def convert_orientation(self,rotation_matrix,array):
        return np.matmul(rotation_matrix,array)

    # SO3 to unit quanternion
    def rot_to_quaternion(self,SO3):
        omg, theta = self.log_rot(SO3)
        q = np.zeros((4,1))
        q[0,0] = math.cos(theta/2)
        q[1:4] = omg*math.sin(theta/2)

        return q

    # Logrithm of Rotation Calculation
    def log_rot(self,R):
        trace = np.trace(R)
        omega_skew = np.zeros((3,3))

        if np.array_equal(R,np.eye(3,3)):
            theta = 0

        elif trace == -1:
            theta = math.pi
            if R[2,2] != -1:
                omega_skew = 1/math.sqrt(2*(1+R[2,2]))*np.transpose(np.array(R[0,2], R[1,2],1+R[2,2]))
            elif R[1,1] != -1:
                omega_skew = 1/math.sqrt(2*(1+R[1,1]))*np.transpose(np.array(R[0,1], 1+ R[1,1],R[2,1]))
            elif R[0,0] != -1:
                omega_skew = 1/math.sqrt(2*(1+R[0,0]))*np.transpose(np.array(1+R[0,0], R[1,0],R[2,0]))

        else:
            theta = math.acos((0.5)*(trace-1))
            if theta > 0:
                omega_skew = (1/(2*math.sin(theta))*np.subtract(R,np.transpose(R))) 

        omg = np.zeros((3,1))
        omg = mr.so3ToVec(omega_skew)       
        omg = omg.reshape((3,1))

        if omg[2] > 0:
            theta = -theta

        return omg, theta
