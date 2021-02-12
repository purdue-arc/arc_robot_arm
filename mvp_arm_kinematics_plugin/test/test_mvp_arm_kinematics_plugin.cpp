/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Southwest Research Institute
 *  All rights reserved.
*********************************************************************/
#include <gtest/gtest.h>
#include <memory>
#include <boost/bind.hpp>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <xmlrpcpp/XmlRpcValue.h>

// MoveIt
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const double DEFAULT_SEARCH_DISCRETIZATION = 0.01f;
const double EXPECTED_SUCCESS_RATE = 0.8;
const double DEFAULT_TOLERANCE = 1e-5;

template <typename T>
inline bool getParam(const std::string& param, T& val)
{
  // first look within private namespace
  ros::NodeHandle pnh("~");
  if (pnh.getParam(param, val))
    return true;

  // then in local namespace
  ros::NodeHandle nh;
  return nh.getParam(param, val);
}

// As loading of parameters is quite slow, we share them across all tests
class SharedData
{
  friend class KinematicsTest;
  typedef pluginlib::ClassLoader<kinematics::KinematicsBase> KinematicsLoader;

  moveit::core::RobotModelPtr robot_model_;
  std::unique_ptr<KinematicsLoader> kinematics_loader_;
  std::string root_link_;
  std::string tip_link_;
  std::string group_name_;
  std::vector<std::string> joints_;
  std::vector<double> seed_;
  std::vector<double> consistency_limits_;
  double timeout_;
  double tolerance_;
  int num_fk_tests_;
  int num_ik_cb_tests_;
  int num_ik_tests_;
  int num_ik_multiple_tests_;
  int num_nearest_ik_tests_;

  SharedData(SharedData const&) = delete;  // this is a singleton
  SharedData()
  {
    initialize();
  }

  void initialize()
  {
    ROS_INFO_STREAM("Loading robot model from " << ros::this_node::getNamespace() << "/" << ROBOT_DESCRIPTION_PARAM);
    // load robot model
    rdf_loader::RDFLoader rdf_loader(ROBOT_DESCRIPTION_PARAM);
    robot_model_ = std::make_shared<moveit::core::RobotModel>(rdf_loader.getURDF(), rdf_loader.getSRDF());
    ASSERT_TRUE(bool(robot_model_)) << "Failed to load robot model";

    // init ClassLoader
    kinematics_loader_ = std::make_unique<KinematicsLoader>("moveit_core", "kinematics::KinematicsBase");
   ASSERT_TRUE(bool(kinematics_loader_)) << "Failed to instantiate ClassLoader";

    // load parameters
    ASSERT_TRUE(getParam("group", group_name_));
    ASSERT_TRUE(getParam("tip_link", tip_link_));
    ASSERT_TRUE(getParam("root_link", root_link_));
    ASSERT_TRUE(getParam("joint_names", joints_));
    getParam("seed", seed_);
    ASSERT_TRUE(seed_.empty() || seed_.size() == joints_.size());
    getParam("consistency_limits", consistency_limits_);
    if (!getParam("ik_timeout", timeout_) || timeout_ < 0.0)
      timeout_ = 1.0;
    if (!getParam("tolerance", tolerance_) || tolerance_ < 0.0)
      tolerance_ = DEFAULT_TOLERANCE;
    ASSERT_TRUE(consistency_limits_.empty() || consistency_limits_.size() == joints_.size());
    ASSERT_TRUE(getParam("num_fk_tests", num_fk_tests_));
    ASSERT_TRUE(getParam("num_ik_cb_tests", num_ik_cb_tests_));
    ASSERT_TRUE(getParam("num_ik_tests", num_ik_tests_));
    ASSERT_TRUE(getParam("num_ik_multiple_tests", num_ik_multiple_tests_));
    ASSERT_TRUE(getParam("num_nearest_ik_tests", num_nearest_ik_tests_));

    ASSERT_TRUE(robot_model_->hasJointModelGroup(group_name_));
    ASSERT_TRUE(robot_model_->hasLinkModel(root_link_));
    ASSERT_TRUE(robot_model_->hasLinkModel(tip_link_));
  }

public:
  auto createUniqueInstance(const std::string& name) const
  {
    return kinematics_loader_->createUniqueInstance(name);
  }

  static const SharedData& instance()
  {
    static SharedData instance;
    return instance;
  }
  static void release()
  {
    SharedData& shared = const_cast<SharedData&>(instance());
    shared.kinematics_loader_.reset();
  }
};

class KinematicsTest : public ::testing::Test
{
	protected:
		void operator=(const SharedData& data)
		{
			robot_model_ = data.robot_model_;
			jmg_ = robot_model_->getJointModelGroup(data.group_name_);
			root_link_ = data.root_link_;
			tip_link_ = data.tip_link_;
			group_name_ = data.group_name_;
			joints_ = data.joints_;
			seed_ = data.seed_;
			consistency_limits_ = data.consistency_limits_;
			timeout_ = data.timeout_;
			tolerance_ = data.tolerance_;
			num_fk_tests_ = data.num_fk_tests_;
			num_ik_cb_tests_ = data.num_ik_cb_tests_;
			num_ik_tests_ = data.num_ik_tests_;
			num_ik_multiple_tests_ = data.num_ik_multiple_tests_;
			num_nearest_ik_tests_ = data.num_nearest_ik_tests_;
		}
		
	void SetUp() override
		{
			*this = SharedData::instance();

			std::string plugin_name;
			ASSERT_TRUE(getParam("ik_plugin_name", plugin_name));
			ROS_INFO_STREAM("Loading " << plugin_name);
			kinematics_solver_ = SharedData::instance().createUniqueInstance(plugin_name);
			ASSERT_TRUE(bool(kinematics_solver_)) << "Failed to load plugin: " << plugin_name;

			// initializing plugin
			ASSERT_TRUE(kinematics_solver_->initialize(*robot_model_, group_name_, root_link_, { tip_link_ },
																								 DEFAULT_SEARCH_DISCRETIZATION) ||
									kinematics_solver_->initialize(ROBOT_DESCRIPTION_PARAM, group_name_, root_link_, { tip_link_ },
																								 DEFAULT_SEARCH_DISCRETIZATION))
					<< "Solver failed to initialize";

			jmg_ = robot_model_->getJointModelGroup(kinematics_solver_->getGroupName());
			ASSERT_TRUE(jmg_);

			// Validate chain information
			ASSERT_EQ(root_link_, kinematics_solver_->getBaseFrame());
			ASSERT_FALSE(kinematics_solver_->getTipFrames().empty());
			ASSERT_EQ(tip_link_, kinematics_solver_->getTipFrame());
			ASSERT_EQ(joints_, kinematics_solver_->getJointNames());
		}
	public:
		moveit::core::RobotModelPtr robot_model_;
		moveit::core::JointModelGroup* jmg_;
		kinematics::KinematicsBasePtr kinematics_solver_;
		random_numbers::RandomNumberGenerator rng_{ 42 };
		std::string root_link_;
		std::string tip_link_;
		std::string group_name_;
		std::vector<std::string> joints_;
		std::vector<double> seed_;
		std::vector<double> consistency_limits_;
		double timeout_;
		double tolerance_;
		unsigned int num_fk_tests_;
		unsigned int num_ik_cb_tests_;
		unsigned int num_ik_tests_;
		unsigned int num_ik_multiple_tests_;
		unsigned int num_nearest_ik_tests_;
};

TEST_F(KinematicsTest, initialize) {
  ASSERT_EQ(robot_model_->getURDF()->getName(), "mvp_arm_4dof");
  ASSERT_EQ(robot_model_->getSRDF()->getName(), "mvp_arm_4dof");
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
	ROSCONSOLE_AUTOINIT;

  ros::init(argc, argv, "test_mvp_arm_kinematics_plugin");
  ros::NodeHandle nh;

  int result = RUN_ALL_TESTS();
	SharedData::release();
	return result;
}
