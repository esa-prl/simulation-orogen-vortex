/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace vortex;

Task::Task(std::string const& name):
    TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine):
    TaskBase(name, engine)
{
}

Task::~Task()
{
}

bool Task::configureHook()
{
    start_time = base::Time::now();
    if (! TaskBase::configureHook())
         return false;

    udp = new udp::UDP();

 
    num_motors = _num_motors.get();
    num_joints = _num_joints.get();
	manipulator_num_joints = _manipulator_num_joints.get();

	PORTS = _PORTS.get();
	PORTC = _PORTC.get();

	joint_readings_names = _joint_readings_names.get();

	joints_readings.resize(num_joints);
	manipulator_readings.resize(manipulator_num_joints);

	for (int i = 0; i < num_joints; i++)
        joints_readings.names[i] = joint_readings_names[i];
   
    // Driving joints; steering joints; walking joints; arm joints 
	dataSend = new double[num_motors+manipulator_num_joints];
    for (int i = 0; i < num_motors+manipulator_num_joints; i++)
        dataSend[i] = 0;

    // rover joints position and speed; manipulator position;
    // [x y z roll pitch yaw] rover pose (6); [x y yaw] goal pose (3)
	dataReceived = new double[num_joints*2 + manipulator_num_joints + nPose + nOrientation + nGoalWayPoint];
    for (int i = 0; i < num_joints*2 + manipulator_num_joints + nPose + nOrientation + nGoalWayPoint; i++)
        dataReceived[i] = 0;

    /// ---Configure UDP connection--- ///

    // Create sockets
    sockS = udp->createSocket();
    sockC = udp->createSocket();

    // Load addresses
    addrS = _addrS.get();
    addrC = _addrC.get();
   
    // Bind & connect sockets with addresses
    udp->bindSocket(sockS, addrS, PORTS);
    udp->connectSocket(sockC, addrC, PORTC);
   	return true;
}

bool Task::startHook()
{
    if(!TaskBase::startHook())
        return false;
	std::cout << "Starting UDP interface execution..." << std::endl; 
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();
    //base::Time current_time = getCurrentTime();
    
	/// ---Receive from Vortex Studio & prepare the data to publish it to RoCK--- ///

	// RECEIVE PACKET FROM VORTEX STUDIO
    n = udp->udpReceive(sockS, dataReceived);

	// JOINTS READINGS VALUES

	for(int i = 0; i < num_joints; i=i+1)
	{
		joints_readings.elements[i].position = dataReceived[2*i];
		joints_readings.elements[i].speed = dataReceived[2*i+1];
	}
	    
    // LOCALIZATION
    // Rover position
    pose.position.x() = dataReceived[num_joints*2];
    pose.position.y() = dataReceived[num_joints*2+1];
    pose.position.z() = dataReceived[num_joints*2+2];
    // Rover orientation
	double cy = cos(dataReceived[num_joints*2+5] * 0.5);
    double sy = sin(dataReceived[num_joints*2+5] * 0.5);
    double cp = cos(dataReceived[num_joints*2+4] * 0.5);
    double sp = sin(dataReceived[num_joints*2+4] * 0.5);
    double cr = cos(dataReceived[num_joints*2+3] * 0.5);
    double sr = sin(dataReceived[num_joints*2+3] * 0.5);

    pose.orientation.w() = cy * cp * cr + sy * sp * sr;
    pose.orientation.x() = cy * cp * sr - sy * sp * cr;
    pose.orientation.y() = sy * cp * sr + cy * sp * cr;
    pose.orientation.z() = sy * cp * cr - cy * sp * sr;

    // Sample position (x, y) and orientation(gamma)
	goalWaypoint.position[0] = dataReceived[num_joints*2+6];
	goalWaypoint.position[1] = dataReceived[num_joints*2+7];
	goalWaypoint.heading = dataReceived[num_joints*2+11];


    // Sample position (x, y,z) and orientation(alpha, beta, gama) for the kinova arm planning
    //Defining a vector that will contain the sample position 
    sample_position.resize(3);
    
    //Defining a vector that will contain the sample orientation 
    sample_orientation.resize(3);
	
    //Storing sample position received form Vortex
    sample_position[0] = dataReceived[num_joints*2+6];
    sample_position[1] = dataReceived[num_joints*2+7];
    sample_position[2] = dataReceived[num_joints*2+8];

    //Storing sample orientation received form Vortex
    sample_orientation[0] = dataReceived[num_joints*2+9];
    sample_orientation[1] = dataReceived[num_joints*2+10];
    sample_orientation[2] = dataReceived[num_joints*2+11];
 
    for(int i = 0; i < manipulator_num_joints; i++)
    	manipulator_readings.at(i) = dataReceived[num_joints*2+nPose+nOrientation+nGoalWayPoint+i];

	// PUBLISH TO ROCK
	_pose.write(pose);
	_goalWaypoint.write(goalWaypoint);
	_joints_readings.write(joints_readings);
	_manipulator_readings.write(manipulator_readings);
    //Sending sample position and orientation to kinova planning
    _sample_position_port.write(sample_position);
    _sample_orientation_port.write(sample_orientation);

	/// ---Receive the data from RoCK and pack it for Vortex Studio--- ///
    
	if(_joints_commands.read(joints_commands) == RTT::NewData)
	{
		// JOINTS COMMANDS
		for(int i = 0; i < num_motors; i++)
		{
			if (joints_commands[i].isPosition()) // Steering drives and walking drives
			{
				//vrep->enableControlLoop(joints_handles[i]);
				if(!isnan(joints_commands.elements[i].position))
					dataSend[i] = joints_commands.elements[i].position;
			}
			else if (joints_commands[i].isSpeed()) // Wheel drives
			{
				//vrep->disableControlLoop(joints_handles[i]);
				if(!isnan(joints_commands.elements[i].speed))
					dataSend[i] = joints_commands.elements[i].speed;
				if(!isnan(joints_commands.elements[num_motors].speed)) //Driving Group is activated (all wheel drive signal set to driving group value)
					dataSend[i] = joints_commands.elements[num_motors].speed;
			}
			else
			{
				//For some reason, isPosition() and isSpeed() can be both false, though driving group speed is set to zero
				if(!isnan(joints_commands.elements[num_motors].speed)) //Driving Group
					dataSend[i] = joints_commands.elements[num_motors].speed;
			}
		}
    }


    if(_manipulator_commands.read(manipulator_commands) == RTT::NewData)
    {
        for(int i = num_motors; i < num_motors+manipulator_num_joints; i++)
        {
            dataSend[i] = manipulator_commands[i-num_motors];
        }	
    }
    /*else
    {
        for(int i = num_motors; i < num_motors+manipulator_num_joints-1; i++)
        {
            dataSend[i] = manipulator_readings.at(i-num_motors+1);
        }	
    }*/
    // SEND PACKET TO VORTEX STUDIO
    n = udp->udpSend(sockC, dataSend, num_motors+manipulator_num_joints);
	

}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
