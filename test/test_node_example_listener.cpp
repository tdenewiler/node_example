#include <gtest/gtest.h>
#include <ros/ros.h>

#include <rosgraph_msgs/Log.h>
#include <node_example/NodeExampleData.h>

#include <boost/thread.hpp>

class Helper
{
 public:
  Helper() : got_msg_(false)
  {
    rosout_sub_ = nh_.subscribe("rosout", 1, &Helper::rosoutCallback, this);
    example_pub_ = nh_.advertise<node_example::NodeExampleData>("example", 1, true);

    spinForTime(0.5);
    join();
  }

  void spinForTime(double secs)
  {
    spin_secs_ = secs;
    spin_thread_.reset(new boost::thread(boost::bind(&Helper::spinThread, this)));
  }

  void join()
  {
    spin_thread_->join();
  }

  bool gotMsg()
  {
    return got_msg_;
  }

  void sendData(double a, double b, const std::string &message)
  {
    node_example::NodeExampleData msg;
    msg.a = a;
    msg.b = b;
    msg.message = message;
    example_pub_.publish(msg);
    spinForTime(1.0);
    join();
  }

 private:
  void spinThread()
  {
    ros::Rate r(10.0);
    ros::Time start = ros::Time::now();

    while ((ros::Time::now() - start) < ros::Duration(spin_secs_))
    {
      ros::spinOnce();
      r.sleep();
    }
  }

  void rosoutCallback(const rosgraph_msgs::LogConstPtr &msg __attribute__((unused)))
  {
    got_msg_ = true;
  }

 private:
  ros::NodeHandle nh_;
  ros::Subscriber rosout_sub_;
  ros::Publisher example_pub_;

  bool got_msg_;
  double spin_secs_;
  boost::shared_ptr<boost::thread> spin_thread_;
};

TEST(NodeExampleTest, rosoutPopulated)
{
  Helper h;
  h.sendData(1, 1, "test");
  EXPECT_TRUE(h.gotMsg());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test_node_example_listener");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
