
#include <iostream>
#include <rtt/deployment/ComponentLoader.hpp>
#include <rtt/plugin/PluginLoader.hpp>
#include <ocl/DeploymentComponent.hpp>

#include <ocl/TaskBrowser.hpp>
#include <rtt/os/main.h>

#include <rtt/RTT.hpp>
#include <rtt_actionlib/rtt_actionlib.h>
#include <rtt_actionlib/rtt_simple_action_server.h>
#include <rtt/scripting/Scripting.hpp>

#include <actionlib/action_definition.h>
#include <rtt_actionlib_examples/SomeActionAction.h>
#include <rtt_rosclock/rtt_rosclock.h>

class SomeComponent : public RTT::TaskContext 
{
	private:
		ACTION_DEFINITION(rtt_actionlib_examples::SomeActionAction);

	private:
		rtt_actionlib::RTTSimpleActionServer<rtt_actionlib_examples::SomeActionAction> action_server;
		Goal goal;
		ros::Time goal_timestamp;


	public:

		// Component constructor
		SomeComponent(std::string name) :
			TaskContext(name, RTT::base::TaskCore::PreOperational),
			action_server(this->provides())
		{ 
		}

		// RTT configure hook
		bool configureHook() {
			return true;
		}

		// RTT start hook
		bool startHook() {
			// Start action server
			return action_server.start();
		}
		
		// RTT stop hook
		void stopHook() {
			Result result;
			if (action_server.isActive()) {
				result.actual_delay_time = rtt_rosclock::host_now() - goal_timestamp;
			}
			action_server.abortActive(result);
			action_server.rejectPending(Result());
		}

		// RTT update hook
		void updateHook() {
			if (action_server.isPending()) {
				// Accept pending goal
				Result result;
				if (action_server.isActive()) {
					// result of active goal if it is preemted
					result.actual_delay_time = rtt_rosclock::host_now() - goal_timestamp;
				}
				action_server.acceptPending(result);
				goal = *(action_server.getActiveGoal());
				goal_timestamp = rtt_rosclock::host_now();
			}
			if (action_server.isActive()) {
				// Pursue goal...
				double percent_complete = 
					100.0 *
					(rtt_rosclock::host_now() - goal_timestamp).toSec() /
					(goal.delay_time.toSec());

				// Publish feedback
				Feedback feedback;
				feedback.percent_complete = percent_complete;
				action_server.publishFeedback(feedback);

				// Set succeded if complete
				if (percent_complete >= 100.0) {
					Result result;
					result.actual_delay_time = rtt_rosclock::host_now() - goal_timestamp;
					action_server.succeedActive(result);
				}
			}
			else if (action_server.isPreemting()) {
				// Preempt active goal
				Result result;
				result.actual_delay_time = rtt_rosclock::host_now() - goal_timestamp;
				action_server.cancelActive(result);
			}
				
		}

};


using namespace RTT::corba;
using namespace RTT;

int ORO_main(int argc, char** argv)
{
	// Get a pointer to the component above
	OCL::DeploymentComponent deployer("deployer");

	RTT::Logger::Instance()->setLogLevel(RTT::Logger::Debug);

	// Import the necessary plugins
	deployer.import("rtt_ros");
	deployer.getProvider<RTT::Scripting>("scripting")->eval("ros.import(\"rtt_actionlib_examples\");");

	// Create an instance of our component
	SomeComponent some_component("some_component");
	deployer.addPeer(&some_component);

	// Connect the component to some ros topics
	std::string script = "\
						  loadService(\"some_component\",\"actionlib\");\
						  some_component.actionlib.connect(\"/some/ros/namespace/my_action\");";
	deployer.getProvider<RTT::Scripting>("scripting")->eval(script);

	// Create an activity for processing the action
	Activity activity( 15, 0.05 ); // priority=15, period=20Hz
	some_component.setActivity(&activity);

	// Start the server
	some_component.configure();
	some_component.start();

	// Interactive task browser
	OCL::TaskBrowser browse( &deployer );
	browse.loop();

	return 0;
} 



