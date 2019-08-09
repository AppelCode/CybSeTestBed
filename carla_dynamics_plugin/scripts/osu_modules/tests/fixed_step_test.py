import carla
import time
import rospy
import datetime
import multiprocessing
import os

sim_start_time = 0
ros_start_time_s = 0
real_start_time = 0

def print_time(timestamp):
    sim_time = timestamp.elapsed_seconds - sim_start_time
    ros_time = rospy.Time.now().to_sec() - ros_start_time_s
    real_time = time.time() - real_start_time

    #print('Ros_time: {} Sim_time: {} Real_time: {} \n'.format( \
        #datetime.timedelta(seconds=int(ros_time)), \
        #datetime.timedelta(seconds=int(sim_time)), \
        #datetime.timedelta(seconds=int(real_time))))

    #print(sim_time - ros_time)

def control_tick():

    #start up ros
    client = carla.Client('127.0.0.1',2000)
    world = client.get_world()

    rospy.init_node("yoyo",anonymous=True)

    world.tick()

    snapshot = world.get_snapshot()
    sim_start_time = snapshot.elapsed_seconds
    ros_start_time_s = rospy.Time.now().to_sec()

    #print(sim_start_time)

    KP = 1e9
    KI = 0
    KD = 0

    #init conditions
    e_int = 0
    u=0
    u0 = 0.05*1e9
    previous_time = rospy.Time.now().to_nsec()

    while True:

        if(int(rospy.Time.now().to_nsec() - previous_time) < int(u+u0)):
            continue
        world.tick()

        snapshot = world.get_snapshot()
        cur_sim_time = snapshot.elapsed_seconds - sim_start_time
        cur_ros_time = rospy.Time.now().to_sec() - ros_start_time_s

        e = cur_sim_time - cur_ros_time

        u = KP*e + KI*(e_int)

        print('error: {}, u: {}'.format(e,u))

        #print('tick')
        previous_time = rospy.Time.now().to_nsec()
        

        e_int = e + e_int

if __name__ == '__main__':

    #create connection to carla
    client = carla.Client('127.0.0.1',2000)
    world = client.get_world()

    #adjust world settings
    settings = world.get_settings()
    settings.fixed_delta_seconds = 0.05
    settings.synchronous_mode = True
    world.apply_settings(settings)

    #set up tick callback
    world.on_tick(print_time)
    
    #start up ros
    rospy.init_node("yoyo",anonymous=True)

    print(rospy.Time.now().to_sec())

    #start parallel process for server
    numjobs = 1 
    jobs = []
    for i in range(numjobs):
        p = multiprocessing.Process(target=control_tick)
        jobs.append(p)
        p.start()

    #wait for tick process to start
    #world.wait_for_tick()

    #grab ros start time
    ros_start_time = rospy.Time.now()
    ros_start_time_s = ros_start_time.to_sec()

    #grab real start time
    real_start_time = time.time()

    #grab sim start time
    snapshot = world.get_snapshot()
    sim_start_time = snapshot.timestamp.elapsed_seconds

    while True:
        world.wait_for_tick()


        #os.system('clear')
        #print('#######################################################')
        #print('####            Test of OSU CybSe plugin           ####')
        #print('#######################################################')
        


