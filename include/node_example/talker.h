#ifndef NODE_EXAMPLE_TALKER_H
#define NODE_EXAMPLE_TALKER_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

// Custom message includes. Auto-generated from msg/ directory.
#include "node_example/NodeExampleData.h"

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <node_example/nodeExampleConfig.h>

namespace node_example
{

class ExampleTalker
{
public:
  //! Constructor.
  ExampleTalker(ros::NodeHandle nh);

  //! Callback function for dynamic reconfigure server.
  void configCallback(node_example::nodeExampleConfig& config, uint32_t level);

  //! Timer callback for publishing message.
  void timerCallback(const ros::TimerEvent& event);

private:
  //! The timer variable used to go to callback function at specified rate.
  ros::Timer timer_;

  //! Message publisher.
  ros::Publisher pub_;

  //! Dynamic reconfigure server.
  dynamic_reconfigure::Server<node_example::nodeExampleConfig> dr_srv_;

  //! The actual message.
  std::string message_;

  //! The first integer to use in addition.
  int a_;

  //! The second integer to use in addition.
  int b_;
};

}

#endif // NODE_EXAMPLE_TALKER_H
