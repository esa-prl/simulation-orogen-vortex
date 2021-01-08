/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef VORTEX_TASK_TASK_HPP
#define VORTEX_TASK_TASK_HPP

#include "vortex/TaskBase.hpp"
#include <udp/udp.hpp>
#include <time.h>
#include <base/samples/Joints.hpp>
#include <base/commands/Joints.hpp>
#include <base/commands/Motion2D.hpp>
#include <vector>

namespace vortex{
	class Task : public TaskBase
		{
			friend class TaskBase;
			protected:
				udp::UDP *udp;
				base::samples::RigidBodyState pose;
				int sockS;
				int sockC;
				int num_joints;
				int num_motors;
				int manipulator_num_joints;
				int nPose = 3;
				int nOrientation = 3;
				int nGoalWayPoint = 6;
				int n;
				int PORTC;
				int PORTS;
				std::string addrS;
				std::string addrC;
				double *dataReceived;
				double *dataSend;
                std::vector<double> sample_orientation;
                std::vector<double> sample_position;
            	std::vector<double> manipulator_commands;
				std::vector<double> manipulator_readings;
				base::samples::Joints joints_commands;
				base::samples::Joints joints_readings;
				std::vector<std::string> joint_readings_names;
				base::Waypoint goalWaypoint;
				base::Time start_time;
				base::Time getCurrentTime();
			public:
        		Task(std::string const& name = "vortex::Task");
		    	Task(std::string const& name, RTT::ExecutionEngine* engine);
				~Task();
				bool configureHook();
				bool startHook();
				void updateHook();
				void errorHook();
				void stopHook();
				void cleanupHook();
    };
}

#endif

