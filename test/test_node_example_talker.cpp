#include <gtest/gtest.h>
#include <ros/ros.h>

#include <node_example/NodeExampleData.h>

#include <boost/thread.hpp>

class Helper
{
 public:
  Helper() : got_msg_(false)
  {
    example_sub_ = nh_.subscribe("example", 1, &Helper::exampleCallback, this);

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

  void exampleCallback(const node_example::NodeExampleDataConstPtr &msg)
  {
    got_msg_ = true;
  }

 private:
  ros::NodeHandle nh_;
  ros::Subscriber example_sub_;

  bool got_msg_;
  double spin_secs_;
  boost::shared_ptr<boost::thread> spin_thread_;
};

TEST(NodeExampleTest, getMessage)
{
  Helper h;
  EXPECT_TRUE(h.gotMsg());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test_node_example_talker");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
