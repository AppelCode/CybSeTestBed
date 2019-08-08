class _VehicleVelocityControl():
    #TODO: add proper intialization for orientation 
    def __init__(self,id):
        self.car_id = id 
        self.original_sigint = signal.getsignal(signal.SIGINT)
        self.manager = multiprocessing.Manager()
        self.velocity_set_args = self.manager.dict()    
        self.velocity_set_args['direction_x'] = 0
        self.velocity_set_args['direction_y'] = 0
        self.velocity_set_args['direction_z'] = 0
        self.velocity_set_args['magnitude'] = 0         #only used for testing
        self.velocity_set_args['x_angular_vel'] = 0
        self.velocity_set_args['y_angular_vel'] = 0
        self.velocity_set_args['z_angular_vel'] = 0
        self.mutex = Lock()

        #start parallel process for vehicle velocity control
        numjobs = 1 
        jobs = []
        for i in range(numjobs):
            p = multiprocessing.Process(target=self._vehicle_control)
            jobs.append(p)
            p.start()

    #process used to move vehicle in carla
    #TODO: handle smooth exit
    #TODO: update inputs for host and port
    
    def _vehicle_control(self):

        def exit_child_process(signum, frame):
            signal.signal(signal.SIGINT, self.original_sigint)
            exit(1)
        
        client = carla.Client(args.host,args.port)
        world = client.get_world()                  #grab the world
        actors = world.get_actors()                 #grab all actors
        car = actors.find(self.car_id)                   #find a specific car
        
        #process exit request
        signal.signal(signal.SIGINT, exit_child_process)

        #run vehicle in x direction
        while True:

            self.mutex.acquire()
            try:  
                magnitude = self.velocity_set_args['magnitude']
                x_dir = self.velocity_set_args['direction_x']
                y_dir = self.velocity_set_args['direction_y']
                z_dir = self.velocity_set_args['direction_z']

                #used for testing ros communication 
                #TODO: remove magnitude used in testing
                x_dir = magnitude*x_dir
                y_dir = magnitude*y_dir
                z_dir = magnitude*z_dir

                x_ang = self.velocity_set_args['x_angular_vel']
                y_ang = self.velocity_set_args['y_angular_vel']
                z_ang = self.velocity_set_args['z_angular_vel']

            finally:
                self.mutex.release()

                run_velocity_dir = carla.Vector3D(x=x_dir,y=y_dir,z=z_dir)
                run_ang_velocity = carla.Vector3D(x=x_ang,y=y_ang,z=z_ang)
                car.set_velocity(run_velocity_dir)

                #TODO: add angular velocity input when lat control is finished
                car.set_angular_velocity(run_ang_velocity)
        
    def _update_vehicle_velocity(self,twist):

        self.velocity_set_args['direction_x'] = np.asscalar(twist[0])
        self.velocity_set_args['direction_y'] = np.asscalar(twist[1])
        self.velocity_set_args['direction_z'] = np.asscalar(twist[2])
        self.velocity_set_args['x_angular_vel'] = np.asscalar(twist[3])
        self.velocity_set_args['y_angular_vel'] = np.asscalar(twist[4])
        self.velocity_set_args['z_angular_vel'] = np.asscalar(twist[5])
