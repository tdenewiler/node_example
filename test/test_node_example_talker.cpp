#include <gtest/gtest.h>
#include <ros/ros.h>

#include <node_example/NodeExampleData.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <boost/thread.hpp>

class Helper
{
 public:
  Helper() : got_msg_(false)
  {
    example_sub_ = nh_.subscribe("example", 1, &Helper::exampleCallback, this);
  }

  void spinForTime(double secs)
  {
    spin_secs_ = secs;
    spin_thread_.reset(new boost::thread(boost::bind(&Helper::spinThread, this)));
    join();
  }

  void join()
  {
    spin_thread_->join();
  }

  bool gotMsg()
  {
    return got_msg_;
  }

  void reset()
  {
    got_msg_ = false;
  }

  void setEnable(bool enable)
  {
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::BoolParameter bool_param;
    dynamic_reconfigure::Config conf;

    bool_param.name = "enable";
    bool_param.value = enable;
    conf.bools.push_back(bool_param);

    srv_req.config = conf;

    ros::service::call("/talker/set_parameters", srv_req, srv_resp);
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

  void exampleCallback(const node_example::NodeExampleDataConstPtr &msg __attribute__((unused)))
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
  h.spinForTime(1.0);
  EXPECT_TRUE(h.gotMsg());
}

TEST(NodeExampleTest, stopNode)
{
  Helper h;
  h.setEnable(false);
  h.reset();
  h.spinForTime(1.0);
  EXPECT_FALSE(h.gotMsg());
}

TEST(NodeExampleTest, restartNode)
{
  Helper h;
  h.setEnable(false);
  h.spinForTime(0.5);
  h.setEnable(true);
  h.reset();
  h.spinForTime(1.0);
  EXPECT_TRUE(h.gotMsg());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test_node_example_talker");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
