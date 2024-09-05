#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("Task2_2");
namespace mtc = moveit::task_constructor;

class ReachTarget
{
public:
  ReachTarget(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr ReachTarget::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

ReachTarget::ReachTarget(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

void ReachTarget::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.06, 0.01 };
  

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.15;
  pose.position.y = 0.725;
  pose.position.z = 0.2;
  pose.orientation.w = 1.0;
  object.pose = pose;

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Add obstacle to planning scene!
  //box1
  moveit_msgs::msg::CollisionObject box1;
  box1.id = "box1";
  box1.header.frame_id = "world";
  box1.primitives.resize(1);
  box1.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  box1.primitives[0].dimensions = { 0.2, 0.2, 0.5 };

  geometry_msgs::msg::Pose Pose_box1;
  Pose_box1.position.x = 0.2;
  Pose_box1.position.y = 0.5;
  Pose_box1.position.z = 0.25;
  Pose_box1.orientation.w = 1.0;
  box1.pose = Pose_box1;

  // box2
  moveit_msgs::msg::CollisionObject box2;
  box2.id = "box2";
  box2.header.frame_id = "world";
  box2.primitives.resize(1);
  box2.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  box2.primitives[0].dimensions = { 0.25, 0.25, 1.75 };

  geometry_msgs::msg::Pose Pose_box2;
  Pose_box2.position.x = -0.55;
  Pose_box2.position.y = -0.55;
  Pose_box2.position.z = 0.0;
  Pose_box2.orientation.w = 1.0;
  box2.pose = Pose_box2;

  // box3
  moveit_msgs::msg::CollisionObject box3;
  box3.id = "box3";  
  box3.header.frame_id = "world";
  box3.primitives.resize(1);
  box3.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  box3.primitives[0].dimensions = { 0.28, 0.28, 0.22 };

  geometry_msgs::msg::Pose Pose_box3;  
  Pose_box3.position.x = 0.5;
  Pose_box3.position.y = -0.55;
  Pose_box3.position.z = 0.14;
  Pose_box3.orientation.w = 1.0;
  box3.pose = Pose_box3;

  // box4
  moveit_msgs::msg::CollisionObject box4;
  box4.id = "box4";  
  box4.header.frame_id = "world";
  box4.primitives.resize(1);
  box4.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  box4.primitives[0].dimensions = { 0.25, 0.25, 1.1 };

  geometry_msgs::msg::Pose Pose_box4; 
  Pose_box4.position.x = -0.4;
  Pose_box4.position.y = 0.4;
  Pose_box4.position.z = 0.5;
  Pose_box4.orientation.w = 1.0;
  box4.pose = Pose_box4;

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
  psi.applyCollisionObject(box1);
  psi.applyCollisionObject(box2);
  psi.applyCollisionObject(box3);
  psi.applyCollisionObject(box4);
}

void ReachTarget::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

mtc::Task ReachTarget::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.001);

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    "move to pick",
    mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  // mtc::Stage* attach_object_stage =
  //   nullptr;  // Forward attach_object_stage to place pose generator

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state

      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.1;


      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    {
      auto stage =
      std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
                        task.getRobotModel()
                            ->getJointModelGroup(hand_group_name)
                            ->getLinkModelNamesWithCollisionGeometry(),
                        true);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      // attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }
    
    task.add(std::move(grasp));
  }

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<ReachTarget>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}