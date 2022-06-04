#include "mj_sim_impl.h"
#include "mj_utils.h"

#include <cassert>
#include <chrono>
#include <type_traits>
#include <iomanip>

#include "MujocoClient.h"
#include "config.h"

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include "implot.h"

#include "ImGuizmo.h"

#include "MujocoClient.h"

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <mc_rtc/version.h>

#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// IF YOU DO NOT SET THIS TO TRUE IT WILL NOT USE THE VARIABLE STIFFNESS
bool VARIABLE_STIFFNESS_ACTIVE = true;


namespace mc_mujoco
{

double MjRobot::PD(double jnt_id, double q_ref, double q, double qdot_ref, double qdot)
{
  double p_error = q_ref - q;
  double v_error = qdot_ref - qdot;
  double ret = (kp[jnt_id] * p_error + kd[jnt_id] * v_error);
  return ret;
}

/* Load PD gains from file (taken from RobotHardware/robot.cpp) */
bool MjRobot::loadGain(const std::string & path_to_pd, const std::vector<std::string> & joints)
{
  std::ifstream strm(path_to_pd.c_str());
  if(!strm.is_open())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] Cannot open PD gains file for {} at {}", name,
                                                     path_to_pd);
  }

  int num_joints = joints.size();
  if(!num_joints)
  {
    return false;
  }
  std::vector<double> default_pgain(num_joints, 0);
  std::vector<double> default_dgain(num_joints, 0);
  for(int i = 0; i < num_joints; i++)
  {
    std::string str;
    bool getlinep;
    while((getlinep = !!(std::getline(strm, str))))
    {
      if(str.empty())
      {
        continue;
      }
      if(str[0] == '#')
      {
        continue;
      }
      double tmp;
      std::istringstream sstrm(str);
      sstrm >> tmp;
      default_pgain[i] = tmp;
      if(sstrm.eof()) break;

      sstrm >> tmp;
      default_dgain[i] = tmp;
      if(sstrm.eof()) break;
      break;
    }
    if(!getlinep)
    {
      if(i < num_joints)
      {
        mc_rtc::log::error(
            "[mc_mujoco] loadGain error: size of gains reading from file ({}) does not match size of joints",
            path_to_pd);
      }
      break;
    }
  }

  strm.close();
  mc_rtc::log::info("[mc_mujoco] Gains for {}", name);
  for(unsigned int i = 0; i < num_joints; i++)
  {
    mc_rtc::log::info("[mc_mujoco] {}, pgain = {}, dgain = {}", joints[i], default_pgain[i], default_dgain[i]);
    // push to kp and kd
    kp.push_back(default_pgain[i]);
    kd.push_back(default_dgain[i]);
  }
  return true;
}

MjSimImpl::MjSimImpl(const MjConfiguration & config)
: controller(std::make_unique<mc_control::MCGlobalController>(config.mc_config)), config(config)
{
  auto get_robot_cfg_path = [&](const std::string & robot_name) -> std::string {
    if(bfs::exists(bfs::path(mc_mujoco::USER_FOLDER) / (robot_name + ".yaml")))
    {
      return (bfs::path(mc_mujoco::USER_FOLDER) / (robot_name + ".yaml")).string();
    }
    else if(bfs::exists(bfs::path(mc_mujoco::SHARE_FOLDER) / (robot_name + ".yaml")))
    {
      return (bfs::path(mc_mujoco::SHARE_FOLDER) / (robot_name + ".yaml")).string();
    }
    else
    {
      return "";
    }
  };

  std::vector<std::string> mujRobots;
  std::vector<std::string> xmlFiles;
  std::vector<std::string> pdGainsFiles;
#if MC_RTC_VERSION_MAJOR > 1
  for(const auto & r_ptr : controller->robots())
  {
    const auto & r = *r_ptr;
#else
  for(const auto & r : controller->robots())
  {
#endif
    const auto & robot_cfg_path = get_robot_cfg_path(r.module().name);
    if(robot_cfg_path.size())
    {
      auto robot_cfg = mc_rtc::Configuration(robot_cfg_path);
      if(!robot_cfg.has("xmlModelPath"))
      {
        mc_rtc::log::error_and_throw<std::runtime_error>("Missing xmlModelPath in {}", robot_cfg_path);
      }
      mujRobots.push_back(r.name());
      xmlFiles.push_back(static_cast<std::string>(robot_cfg("xmlModelPath")));
      pdGainsFiles.push_back(robot_cfg("pdGainsPath", std::string("")));
      if(!bfs::exists(xmlFiles.back()))
      {
        mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] XML model cannot be found at {}",
                                                         xmlFiles.back());
      }
    }
  }

  if(!xmlFiles.size())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No Mujoco model associated to any robots in the controller");
  }

  // initial mujoco here and load XML model
  bool initialized = mujoco_init(this, mujRobots, xmlFiles);
  if(!initialized)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] Initialized failed.");
  }

  // read PD gains from file
  for(size_t i = 0; i < robots.size(); ++i)
  {
    auto & r = robots[i];
    bool has_motor =
        std::any_of(r.mj_mot_names.begin(), r.mj_mot_names.end(), [](const std::string & m) { return m.size() != 0; });
    const auto & robot = controller->robot(r.name);
    if(robot.mb().nrDof() == 0 || (robot.mb().nrDof() == 6 && robot.mb().joint(0).dof() == 6) || !has_motor)
    {
      continue;
    }
    if(!bfs::exists(pdGainsFiles[i]))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] PD gains file for {} cannot be found at {}", r.name,
                                                       pdGainsFiles.back());
    }
    r.loadGain(pdGainsFiles[i], controller->robots().robot(r.name).module().ref_joint_order());
  }

  if(config.with_visualization)
  {
    mujoco_create_window(this);
    if(config.with_mc_rtc_gui)
    {
      client = std::make_unique<MujocoClient>();
    }
  }
  mc_rtc::log::info("[mc_mujoco] Initialized successful.");
}

void MjSimImpl::cleanup()
{
  mujoco_cleanup(this);
}

void MjRobot::initialize(mjModel * model, const mc_rbdyn::Robot & robot)
{
  mj_jnt_ids.resize(0);
  for(const auto & j : mj_jnt_names)
  {
    mj_jnt_ids.push_back(mj_name2id(model, mjOBJ_JOINT, j.c_str()));
  }
  auto fill_acuator_ids = [&](const std::vector<std::string> & names, std::vector<int> & ids) {
    ids.resize(0);
    for(const auto & n : names)
    {
      if(n.size())
      {
        ids.push_back(mj_name2id(model, mjOBJ_ACTUATOR, n.c_str()));
      }
      else
      {
        ids.push_back(-1);
      }
    }
  };
  fill_acuator_ids(mj_mot_names, mj_mot_ids);
  fill_acuator_ids(mj_pos_act_names, mj_pos_act_ids);
  fill_acuator_ids(mj_vel_act_names, mj_vel_act_ids);
  if(root_body.size())
  {
    root_body_id = mj_name2id(model, mjOBJ_BODY, root_body.c_str());
  }
  auto init_sensor_id = [&](const char * mj_name, const char * mc_name, const std::string & sensor_name,
                            const char * suffix, mjtSensor type, std::unordered_map<std::string, int> & mapping) {
    auto mj_sensor = prefixed(fmt::format("{}_{}", sensor_name, suffix));
    auto sensor_id = mujoco_get_sensor_id(*model, mj_sensor, type);
    if(sensor_id == -1)
    {
      mc_rtc::log::error("[mc_mujoco] No MuJoCo {} for {} {} in {}, expected to find a {} named {}", mj_name,
                         sensor_name, mc_name, name, mj_name, mj_sensor);
    }
    mapping[sensor_name] = sensor_id;
  };
  for(const auto & fs : robot.module().forceSensors())
  {
    wrenches[fs.name()] = sva::ForceVecd(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
    init_sensor_id("force sensor", "force sensor", fs.name(), "fsensor", mjSENS_FORCE, mc_fs_to_mj_fsensor_id);
    init_sensor_id("torque sensor", "force sensor", fs.name(), "tsensor", mjSENS_TORQUE, mc_fs_to_mj_tsensor_id);
  }
  for(const auto & bs : robot.bodySensors())
  {
    if(bs.name() == "FloatingBase" || bs.name().empty())
    {
      continue;
    }
    gyros[bs.name()] = Eigen::Vector3d::Zero();
    accelerometers[bs.name()] = Eigen::Vector3d::Zero();
    init_sensor_id("gyro sensor", "body sensor", bs.name(), "gyro", mjSENS_GYRO, mc_bs_to_mj_gyro_id);
    init_sensor_id("accelerometer sensor", "body sensor", bs.name(), "accelerometer", mjSENS_ACCELEROMETER,
                   mc_bs_to_mj_accelerometer_id);
  }




  //****************************************************Start: Range Sensors******************************************************

  // get of range sensors
  auto getRangeSensors = [] (mc_rbdyn::Robot & robot) -> std::vector<mc_mujoco::RangeSensor *>
  {
    std::vector<mc_mujoco::RangeSensor *> out;
    const auto & module = robot.module();
    mc_rtc::log::error("[mc_mujoco] Robot {} Devices {}", robot.name(), module.devices().size());
    for(const auto & s : module.devices())
    {
      mc_rtc::log::error("[mc_mujoco] Device {}", s->name());
      auto sensor = dynamic_cast<mc_mujoco::RangeSensor *>(s.get());
      if(sensor)
      {
        auto & robot_sensor = robot.sensor<mc_mujoco::RangeSensor>(sensor->name());
        out.push_back(&robot_sensor);
      }
    }
    return out;
  };

  mc_rtc::log::error("[mc_mujoco] Initialization of range sensors");
  for(mc_mujoco::RangeSensor * rs : getRangeSensors(const_cast<mc_rbdyn::Robot &>(robot)))
  {
    mc_rtc::log::error("[mc_mujoco] Range sensor name {}", rs->name());
    ranges[rs->name()] = 0;
    ranges_ptr[rs->name()] = rs;
    init_sensor_id("range sensor", "range sensor", rs->name(), "ranger", mjSENS_RANGEFINDER, mc_rs_to_mj_ranger_id);
  }

  //****************************************************End: Range Sensors******************************************************

  reset(robot);
}

void MjRobot::reset(const mc_rbdyn::Robot & robot)
{
  const auto & mbc = robot.mbc();
  const auto & rjo = robot.module().ref_joint_order();



  //****************************************************Start: Passive Joints******************************************************

  // Create passive joints vector name

  const rbd::MultiBody & multiBody = robot.mb(); // MultiBody object
  const std::vector<rbd::Joint> & joints = multiBody.joints();
  std::vector<std::string> totJoints;
  for (int i = 0; i < joints.size(); i++)
  {
    const std::string & jointName = joints[i].name(); // All joints name
    totJoints.push_back(jointName);
    // std::cout << "  joint_name:   " << jointName << std::endl; 
  }
  for (int i = 0; i < rjo.size(); i++)
  {
    const std::string & ActivejointName = rjo[i];
    // std::cout << "  active_joint_name:   " << ActivejointName << std::endl;
  }

  for (int i = 0; i < mj_jnt_names.size(); i++)
  {
    // std::cout << "  mj_jnt_names:   " << mj_jnt_names[i] << std::endl;
  }

  //****************************************************End: Passive Joints******************************************************

  mj_to_mbc.resize(0);
  mj_prev_ctrl_q.resize(0);
  mj_prev_ctrl_alpha.resize(0);
  mj_jnt_to_rjo.resize(0);
  mj_jnt_to_passive.resize(0);
  encoders = std::vector<double>(rjo.size(), 0.0);
  alphas = std::vector<double>(rjo.size(), 0.0);
  torques = std::vector<double>(rjo.size(), 0.0);
  for(const auto & mj_jn : mj_jnt_names)
  {
    const auto & jn = [&]() {
      if(prefix.size())
      {
        return mj_jn.substr(prefix.size() + 1);
      }
      return mj_jn;
    }();

//****************************************************Lines  relative to passive joints******************************************************
    auto rjo_it = std::find(rjo.begin(), rjo.end(), jn);
    auto passive_it = std::find(totJoints.begin(), totJoints.end(), jn); //*
    int rjo_idx = -1;
    int passive_idx = -1; //*
    if(rjo_it != rjo.end())
    {
      rjo_idx = std::distance(rjo.begin(), rjo_it);
    }
    else
    {
      passive_idx = std::distance(totJoints.begin(), passive_it);
    }
    mj_jnt_to_rjo.push_back(rjo_idx);
    mj_jnt_to_passive.push_back(passive_idx); //*

    const auto motor = std::string(robot.name()+"_"+jn+"_motor"); //*
    auto it = std::find(mj_mot_names.begin(), mj_mot_names.end(), motor); //*
    if(robot.hasJoint(jn) && it != mj_mot_names.end()) //*
    {
      auto jIndex = robot.jointIndexByName(jn);
      mj_mot_to_rjo.push_back(rjo_idx); //*
      mj_to_mbc.push_back(jIndex);
      if(robot.mb().joint(jIndex).dof() != 1)
      {
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "[mc_mujoco] Only support revolute and prismatic joint for control");
      }
      mj_prev_ctrl_q.push_back(robot.mbc().q[jIndex][0]);
      mj_prev_ctrl_alpha.push_back(robot.mbc().alpha[jIndex][0]);
      if(rjo_idx != -1)
      {
        encoders[rjo_idx] = mj_prev_ctrl_q.back();
        alphas[rjo_idx] = mj_prev_ctrl_alpha.back();
      }
    }
    else
    {
      mj_mot_to_rjo.push_back(-1); //*
      mj_to_mbc.push_back(-1);
    }
  }

  mj_ctrl = mj_prev_ctrl_q;
  mj_next_ctrl_q = mj_prev_ctrl_q;
  mj_next_ctrl_alpha = mj_prev_ctrl_alpha;
   
}

//****************************************************End: Lines relative to passive joints******************************************************

void MjSimImpl::setSimulationInitialState()
{
  if(controller)
  {
    qInit.resize(0);
    alphaInit.resize(0);
    for(auto & r : robots)
    {
      const auto & robot = controller->robots().robot(r.name);
      r.initialize(model, robot);
      for(const auto & rs : r.ranges_ptr) //*
      { //*
        rs.second->addToLogger(controller->controller().logger(), robot.name()); //*
      } //*
      if(r.root_joint.size())
      {
        r.root_qpos_idx = qInit.size();
        r.root_qvel_idx = alphaInit.size();
        if(robot.mb().joint(0).dof() == 6)
        {
          const auto & t = robot.posW().translation();
          for(size_t i = 0; i < 3; ++i)
          {
            qInit.push_back(t[i]);
            // push linear/angular velocities
            alphaInit.push_back(0);
            alphaInit.push_back(0);
          }
          Eigen::Quaterniond q = Eigen::Quaterniond(robot.posW().rotation()).inverse();
          qInit.push_back(q.w());
          qInit.push_back(q.x());
          qInit.push_back(q.y());
          qInit.push_back(q.z());
        }
      }
      else if(r.root_body_id != -1)
      {
        const auto & t = robot.posW().translation();
        model->body_pos[3 * r.root_body_id + 0] = t.x();
        model->body_pos[3 * r.root_body_id + 1] = t.y();
        model->body_pos[3 * r.root_body_id + 2] = t.z();
        Eigen::Quaterniond q = Eigen::Quaterniond(robot.posW().rotation()).inverse();
        model->body_quat[4 * r.root_body_id + 0] = q.w();
        model->body_quat[4 * r.root_body_id + 1] = q.x();
        model->body_quat[4 * r.root_body_id + 2] = q.y();
        model->body_quat[4 * r.root_body_id + 3] = q.z();
      }
      for(size_t i = 0; i < r.mj_jnt_names.size(); ++i)
      {
        qInit.push_back(r.encoders[r.mj_jnt_to_rjo[i]]);
        alphaInit.push_back(r.alphas[r.mj_jnt_to_rjo[i]]);
      }
    }
  }
  // set initial qpos, qvel in mujoco
  if(!mujoco_set_const(model, data, qInit, alphaInit))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] Set inital state failed.");
  }
  mj_forward(model, data);
}

void MjSimImpl::startSimulation()
{
  //****************************************************Start: Passive forces components******************************************************

  auto qpos = [this](int i) -> double
  {
    return data->qpos[i+7];
  };

  auto qpos_spring = [this](int i) -> double
  {
    return model->qpos_spring[i+7];
  };

  auto qvel = [this](int i) -> double
  {
    return data->qvel[i+6];
  };

  auto qfrc_passive = [this](int i) -> double
  {
    return data->qfrc_passive[i+6];
  };

  auto damping = [this](int i) -> double
  {
    return model->dof_damping[i+6];
  };

  auto stiffness = [this](int i) -> double
  {
    return model->jnt_stiffness[i+1];
  };

  const auto & mj_jnt_names = robots[0].mj_jnt_names;
  for(int i = 0; i < mj_jnt_names.size(); ++i)
  {
    controller->controller().logger().addLogEntry(mj_jnt_names[i]+"_stiffness", this, [stiffness, i]() { return stiffness(i); });
    controller->controller().logger().addLogEntry(mj_jnt_names[i]+"_damping", this, [damping, i]() { return damping(i); });
    controller->controller().logger().addLogEntry(mj_jnt_names[i]+"_qpos", this, [qpos, i]() { return qpos(i); });
    controller->controller().logger().addLogEntry(mj_jnt_names[i]+"_qvel", this, [qvel, i]() { return qvel(i); });
    controller->controller().logger().addLogEntry(mj_jnt_names[i]+"_qfrc_passive", this, [qfrc_passive, i]() { return qfrc_passive(i); });
    controller->controller().logger().addLogEntry(mj_jnt_names[i]+"_qpos_spring", this, [qpos_spring, i]() { return qpos_spring(i); });

    controller->controller().logger().addLogEntry(mj_jnt_names[i]+"_own_qfrc_passive_stiffness", this, [qpos, stiffness, i]() { return -stiffness(i) * qpos(i); });
    controller->controller().logger().addLogEntry(mj_jnt_names[i]+"_own_qfrc_passive_damping", this, [qvel, damping, i]() { return -damping(i) * qvel(i); });
    controller->controller().logger().addLogEntry(mj_jnt_names[i]+"_own_qfrc_passive", this, [qpos, stiffness, qvel, damping, i]() { return -stiffness(i) * qpos(i) -damping(i) * qvel(i); });
    // mc_rtc::log::warning("ID {}  \t name {} \t stiffness {:4f}\t damping {:4f} \t qpos {:.6f} \t qvel {:.6f} \t passive {:.6f}",
    //   i, mj_jnt_names[i], stiffness(i), damping(i), qpos(i), qvel(i), qfrc_passive(i)); 
  }

  auto stiffnessToAngle = [this](double VarStiff) 
  {
    double angle_low = 0;
    double angle_high = 1;
    double stiffness_low = 0;
    double stiffness_high = 100;
    return angle_low+(VarStiff-stiffness_low)*(angle_high-angle_low)/(stiffness_high-stiffness_low);
  };

  int idx_phalanx = 0;
  for(int k = 0; k < mj_jnt_names.size(); ++k)
  {      
    if(mj_jnt_names[k] == "hrp4j_soft_R_PHALANX_10")
    {
      idx_phalanx = k;
      break;
    }
  }
  
  controller->robot().q()[controller->robot().jointIndexByName("R_VARSTIFF")][0] = stiffnessToAngle(stiffness(idx_phalanx));
  controller->robot().q()[controller->robot().jointIndexByName("L_VARSTIFF")][0] = stiffnessToAngle(stiffness(idx_phalanx));

  //****************************************************End: Passive force components******************************************************

  setSimulationInitialState();
  if(!config.with_controller)
  {
    controller.reset();
    return;
  }

  // get sim timestep and set the frameskip parameter
  double simTimestep = model->opt.timestep;
  frameskip_ = std::round(controller->timestep() / simTimestep);
  mc_rtc::log::info("[mc_mujoco] MC-RTC timestep: {}. MJ timestep: {}", controller->timestep(), simTimestep);
  mc_rtc::log::info("[mc_mujoco] Hence, Frameskip: {}", frameskip_);

  for(const auto & r : robots)
  {
    controller->setEncoderValues(r.name, r.encoders);
  }
  controller->init(robots[0].encoders);

  controller->running = true;

}

void MjRobot::updateSensors(mc_control::MCGlobalController * gc, mjModel * model, mjData * data)
{
  for(size_t i = 0; i < mj_jnt_ids.size(); ++i)
  {
    if(mj_jnt_to_rjo[i] == -1)
    {
      continue;
    }
    encoders[mj_jnt_to_rjo[i]] = data->qpos[model->jnt_qposadr[mj_jnt_ids[i]]];
    alphas[mj_jnt_to_rjo[i]] = data->qvel[model->jnt_dofadr[mj_jnt_ids[i]]];
  }

  for(size_t i = 0; i < mj_jnt_ids.size(); ++i) //*
  { //*
    if(mj_jnt_to_passive[i] == -1) //*
    { //*
      continue; //*
    } //* 
    gc->realRobot(name).mbc().q[mj_jnt_to_passive[i]][0]= data->qpos[model->jnt_qposadr[mj_jnt_ids[i]]]; //*
  } //*


  for(size_t i = 0; i < mj_mot_ids.size(); ++i)
  {
    if(mj_jnt_to_rjo[i] == -1)
    {
      continue;
    }
    torques[mj_jnt_to_rjo[i]] = data->qfrc_actuator[model->jnt_dofadr[mj_jnt_ids[i]]];
  }
  if(!gc)
  {
    return;
  }
  auto & robot = gc->controller().robots().robot(name);

  // Body sensor updates
  if(root_qpos_idx != -1)
  {
    root_pos = Eigen::Map<Eigen::Vector3d>(&data->qpos[root_qpos_idx]);
    root_ori.w() = data->qpos[root_qpos_idx + 3];
    root_ori.x() = data->qpos[root_qpos_idx + 4];
    root_ori.y() = data->qpos[root_qpos_idx + 5];
    root_ori.z() = data->qpos[root_qpos_idx + 6];
    root_ori = root_ori.inverse();
    root_linvel = Eigen::Map<Eigen::Vector3d>(&data->qvel[root_qvel_idx]);
    root_angvel = Eigen::Map<Eigen::Vector3d>(&data->qvel[root_qvel_idx + 3]);
    root_linacc = Eigen::Map<Eigen::Vector3d>(&data->qacc[root_qvel_idx]);
    root_angacc = Eigen::Map<Eigen::Vector3d>(&data->qacc[root_qvel_idx + 3]);
    if(robot.hasBodySensor("FloatingBase"))
    {
      gc->setSensorPositions(name, {{"FloatingBase", root_pos}});
      gc->setSensorOrientations(name, {{"FloatingBase", root_ori}});
      gc->setSensorLinearVelocities(name, {{"FloatingBase", root_linvel}});
      gc->setSensorAngularVelocities(name, {{"FloatingBase", root_angvel}});
      gc->setSensorLinearAccelerations(name, {{"FloatingBase", root_linacc}});
      // FIXME Not implemented in mc_rtc
      // gc->setSensorAngularAccelerations(name, {{"FloatingBase", root_angacc}});
    }
  }

  // Gyro update
  for(auto & gyro : gyros)
  {
    mujoco_get_sensordata(*model, *data, mc_bs_to_mj_gyro_id[gyro.first], gyro.second.data());
  }
  gc->setSensorAngularVelocities(name, gyros);

  // Accelerometers update
  for(auto & accelerometer : accelerometers)
  {
    mujoco_get_sensordata(*model, *data, mc_bs_to_mj_accelerometer_id[accelerometer.first],
                          accelerometer.second.data());
  }
  gc->setSensorLinearAccelerations(name, accelerometers);

  // Force sensor update
  for(auto & fs : wrenches)
  {
    mujoco_get_sensordata(*model, *data, mc_fs_to_mj_fsensor_id[fs.first], fs.second.force().data());
    mujoco_get_sensordata(*model, *data, mc_fs_to_mj_tsensor_id[fs.first], fs.second.couple().data());
    fs.second *= -1;
  }
  gc->setWrenches(name, wrenches);

//****************************************************Start: Range Sensors******************************************************
  // Range sensor update
  for(auto & rs : ranges)
  {
    mujoco_get_sensordata(*model, *data, mc_rs_to_mj_ranger_id[rs.first], &rs.second);
    ranges_ptr[rs.first]->update(rs.second);
  }

//****************************************************End: Range Sensors******************************************************

  // Joint sensor updates
  gc->setEncoderValues(name, encoders);
  gc->setEncoderVelocities(name, alphas);
  gc->setJointTorques(name, torques);

}

void MjSimImpl::updateData()
{
  for(auto & r : robots)
  {
    r.updateSensors(controller.get(), model, data);
  }
}

void MjRobot::updateControl(const mc_rbdyn::Robot & robot)
{
  mj_prev_ctrl_q = mj_next_ctrl_q;
  mj_prev_ctrl_alpha = mj_next_ctrl_alpha;
  size_t ctrl_idx = 0;
  for(size_t i = 0; i < mj_to_mbc.size(); ++i)
  {
    auto jIndex = mj_to_mbc[i];
    if(jIndex != -1)
    {
      mj_next_ctrl_q[ctrl_idx] = robot.mbc().q[jIndex][0];
      mj_next_ctrl_alpha[ctrl_idx] = robot.mbc().alpha[jIndex][0];
      ctrl_idx++;
    }
  }

}

void MjRobot::sendControl(const mjModel & model, mjData & data, size_t interp_idx, size_t frameskip_)
{
  for(size_t i = 0, j = 0; j < mj_mot_ids.size(); ++j)
  {
    auto mot_id = mj_mot_ids[j];
    auto rjo_id = mj_mot_to_rjo[j];
    if(rjo_id == - 1) 
    { 
      continue; 
    } 
    auto pos_act_id = mj_pos_act_ids[i];
    auto vel_act_id = mj_vel_act_ids[i];
    // compute desired q using interpolation
    double q_ref = (interp_idx + 1) * (mj_next_ctrl_q[i] - mj_prev_ctrl_q[i]) / frameskip_;
    q_ref += mj_prev_ctrl_q[i];
    // compute desired alpha using interpolation
    double alpha_ref = (interp_idx + 1) * (mj_next_ctrl_alpha[i] - mj_prev_ctrl_alpha[i]) / frameskip_;
    alpha_ref += mj_prev_ctrl_alpha[i];
    if(mot_id != -1)
    {
      // compute desired torque using PD control
      mj_ctrl[i] = PD(i, q_ref, encoders[rjo_id], alpha_ref, alphas[rjo_id]);
      double ratio = model.actuator_gear[6 * mot_id];
      data.ctrl[mot_id] = mj_ctrl[i] / ratio;
    }
    if(pos_act_id != -1)
    {
      data.ctrl[pos_act_id] = q_ref;
    }
    if(vel_act_id != -1)
    {
      data.ctrl[vel_act_id] = alpha_ref;
    }
    ++ i;
  }
}

bool MjSimImpl::controlStep()
{
  auto interp_idx = iterCount_ % frameskip_;
  // After every frameskip iters
  if(config.with_controller && interp_idx == 0)
  {
    // run the controller
    if(!controller->run())
    {
      return true;
    }
    for(auto & r : robots)
    {
      r.updateControl(controller->robots().robot(r.name));



// **********************************************Start: Update variable stiffness**********************************************************      
      if(r.name == "hrp4j_soft" && VARIABLE_STIFFNESS_ACTIVE)
      {   
        const auto & mj_jnt_names = r.mj_jnt_names;
        const auto & mj_jnt_ids = r.mj_jnt_ids;

        const double stiffness_low = 0;
        const double stiffness_high = 100; 
        const double angle_low = 0;
        const double angle_high = 1;

        auto qpos = [this](int i) -> double
        {
          return data->qpos[i+7];
        };

        auto qvel = [this](int i) -> double
        {
          return data->qvel[i+6];
        };

        auto stiffness = [this](int i) -> double
        {
          return model->jnt_stiffness[i+1];
        };

        auto SetJointStiffness = [this](int i, double VarStiff) 
        {
          model->jnt_stiffness[i+1] = VarStiff;
        };

        // Create a matrix softIndices with alle the indices of phalanges joints
        int NUMBER_SOFT_JOINTS = 10;
        int softIndices[2][NUMBER_SOFT_JOINTS];
        // Create a matrix varIndices with alle the indices of variable stiffness joint
        int NUMBER_VAR_JOINTS = 2;
        int VarStiffIndices[NUMBER_VAR_JOINTS];
        const std::vector<std::string> prefix = {"R", "L"};

        for(int k = 0; k < mj_jnt_names.size(); ++k)
        {      
          for(int i = 0; i < NUMBER_VAR_JOINTS; ++ i)
          {
            std::string varStiff_name = "hrp4j_soft_"+prefix[i]+"_VARSTIFF";
            if(mj_jnt_names[k] == varStiff_name)
            {
              VarStiffIndices[i] = mj_jnt_ids[k]-1;
            }
            else
            {
              for(int j = 1; j < NUMBER_SOFT_JOINTS+1; ++j)
              {
                std::string phalanx_name = "hrp4j_soft_"+prefix[i]+"_PHALANX_"+std::to_string(j);
                if(mj_jnt_names[k] == phalanx_name)
                {
                  softIndices[i][j-1] = mj_jnt_ids[k]-1;
                }
              }
            }
          }
          // mc_rtc::log::warning("ID {}  \t name {} \t stiffness {:4f}\t qpos {:.6f}", k, mj_jnt_names[k], stiffness(k), qpos(k)); 
        }

        // Update the stiffness of the phalanxes
        for(int i = 0; i < NUMBER_VAR_JOINTS; ++ i)
        {
          double springSoft = stiffness_low + (qpos(VarStiffIndices[i]) - angle_low)*(stiffness_high-stiffness_low)/(angle_high-angle_low);
          for(int j = 1; j < NUMBER_SOFT_JOINTS+1; ++j)
          {
            SetJointStiffness(softIndices[i][j-1], springSoft);
          }
        }
      }




// **********************************************End: Update variable stiffness**********************************************************

    }

  }

  // On each control iter
  for(auto & r : robots)
  {
    r.sendControl(*model, *data, interp_idx, frameskip_);
  }
  iterCount_++;
  return false;
}

void MjSimImpl::simStep()
{
  // clear old perturbations, apply new
  mju_zero(data->xfrc_applied, 6 * model->nbody);
  mjv_applyPerturbPose(model, data, &pert, 0); // move mocap bodies only
  mjv_applyPerturbForce(model, data, &pert);

  // take one step in simulation
  // model.opt.timestep will be used here
  mj_step(model, data);

  // To display contacts
  // for (unsigned int i = 0; i < data->ncon; ++i)
  // {
  //   int body1_id = model->geom_bodyid[data->contact[i].geom1];
  //   int body2_id = model->geom_bodyid[data->contact[i].geom2];
  //   const char * body1_c =  mj_id2name(model, mjOBJ_BODY, body1_id);
  //   const char * body2_c =  mj_id2name(model, mjOBJ_BODY, body2_id);
  //   std::cout << "Body1: " << body1_c << "\t" << "Body2: " << body2_c << std::endl;
  // }

  // mj_printData(model, data, "/tmp/ntm.txt");
}

void MjSimImpl::resetSimulation(const std::map<std::string, std::vector<double>> & reset_qs,
                                const std::map<std::string, sva::PTransformd> & reset_pos)
{
  iterCount_ = 0;
  reset_simulation_ = false;
  if(controller)
  {
    controller->reset(reset_qs, reset_pos);
    for(auto & robot : robots)
    {
      robot.reset(controller->robot(robot.name));
    }
  }
  mj_resetData(model, data);
  setSimulationInitialState();
}

bool MjSimImpl::stepSimulation()
{
  if(reset_simulation_)
  {
    resetSimulation({}, {});
  }
  auto start_step = clock::now();
  // Only run the GUI update if the simulation is paused

  if(config.step_by_step && rem_steps == 0)
  {
    if(controller)
    {
      controller->running = false;
      controller->run();
      controller->running = true;
    }
    mj_sim_start_t = start_step;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return false;
  }
  if(iterCount_ > 0)
  {
    duration_us dt = start_step - mj_sim_start_t;
    mj_sync_delay += duration_us(1e6 * model->opt.timestep) - dt;
    mj_sim_dt[(iterCount_ - 1) % mj_sim_dt.size()] = dt.count();
  }
  mj_sim_start_t = start_step;
  auto do_step = [this, &start_step]() {
    {
      std::lock_guard<std::mutex> lock(rendering_mutex_);
      simStep();
    }
    updateData();
    return controlStep();
  };
  bool done = false;
  if(!config.step_by_step)
  {
    done = do_step();
  }
  if(config.step_by_step && rem_steps > 0)
  {
    done = do_step();
    rem_steps--;
  }
  if(config.sync_real_time)
  {
    std::this_thread::sleep_until(start_step + duration_us(1e6 * model->opt.timestep) + mj_sync_delay);
  }
  return done;
}

void MjSimImpl::updateScene()
{
  // update scene and render
  std::lock_guard<std::mutex> lock(rendering_mutex_);

  mjv_updateScene(model, data, &options, &pert, &camera, mjCAT_ALL, &scene);

  // If you do not want to record geometries elements, you have to let this comment.
  // You should push on git.
  // if(client)
  // {
  //   client->updateScene(scene);
  // }

  // process pending GUI events, call GLFW callbacks
  glfwPollEvents();
}

bool MjSimImpl::render()
{
  if(!config.with_visualization)
  {
    return true;
  }

  // mj render
  mjr_render(uistate.rect[0], &scene, &context);

  // @@@@@@@@@@@@@Here only the robot and the steps on the ground are displayed
  // call this to record the video
  // record();

  // This is called here to display some elements as markers and to do not record them
  // If you do not want them you have to comment those lines
  // --> From here
  if(client)
  {
    client->updateScene(scene);
  }
  mjr_render(uistate.rect[0], &scene, &context);
  // --> Until here

  // Render ImGui
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  ImGuizmo::BeginFrame();
  ImGuiIO & io = ImGui::GetIO();
  ImGuizmo::AllowAxisFlip(false);
  ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
  if(client)
  {
    client->update();
    client->draw2D(window);
    client->draw3D();
    // @@@@@@@@@@@@@@Here the robot, and geometries elements
    record();
  }
  {
    auto right_margin = 5.0f;
    auto top_margin = 5.0f;
    auto width = io.DisplaySize.x - 2 * right_margin;
    auto height = io.DisplaySize.y - 2 * top_margin;
    ImGui::SetNextWindowPos({0.8f * width - right_margin, top_margin}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({0.2f * width, 0.3f * height}, ImGuiCond_FirstUseEver);
    ImGui::Begin(fmt::format("mc_mujoco (MuJoCo {})", mj_version()).c_str());
    size_t nsamples = std::min(mj_sim_dt.size(), iterCount_);
    mj_sim_dt_average = 0;
    for(size_t i = 0; i < nsamples; ++i)
    {
      mj_sim_dt_average += mj_sim_dt[i] / nsamples;
    }
    ImGui::Text("Average sim time: %.2fμs", mj_sim_dt_average);
    ImGui::Text("Simulation/Real time: %.2f", mj_sim_dt_average / (1e6 * model->opt.timestep));
    if(ImGui::Checkbox("Sync with real-time", &config.sync_real_time))
    {
      if(config.sync_real_time)
      {
        mj_sync_delay = duration_us(0);
      }
    }
    ImGui::Checkbox("Step-by-step", &config.step_by_step);
    if(config.step_by_step)
    {
      auto doNStepsButton = [&](size_t n, bool final_) {
        size_t n_ms = std::ceil(n * 1000 * (controller ? controller->timestep() : model->opt.timestep));
        if(ImGui::Button(fmt::format("+{}ms", n_ms).c_str()))
        {
          rem_steps = n;
        }
        if(!final_)
        {
          ImGui::SameLine();
        }
      };
      doNStepsButton(1, false);
      doNStepsButton(5, false);
      doNStepsButton(10, false);
      doNStepsButton(50, false);
      doNStepsButton(100, true);
    }
    ImGui::Checkbox("Record", &config.recording);
    auto flag_to_gui = [&](const char * label, mjtVisFlag flag) {
      bool show = options.flags[flag];
      if(ImGui::Checkbox(label, &show))
      {
        options.flags[flag] = show;
      }
    };
    flag_to_gui("Show contact points [C]", mjVIS_CONTACTPOINT);
    flag_to_gui("Show contact forces [F]", mjVIS_CONTACTFORCE);
    flag_to_gui("Show rangefinder [R]", mjVIS_RANGEFINDER);
    auto group_to_checkbox = [&](size_t group, bool last) {
      bool show = options.geomgroup[group];
      if(ImGui::Checkbox(fmt::format("{}", group).c_str(), &show))
      {
        options.geomgroup[group] = show;
      }
      if(!last)
      {
        ImGui::SameLine();
      }
    };
    ImGui::Text("%s", fmt::format("Visible layers [0-{}]", mjNGROUP).c_str());
    for(size_t i = 0; i < mjNGROUP; ++i)
    {
      group_to_checkbox(i, i == mjNGROUP - 1);
    }
    if(ImGui::Button("Reset simulation"))
    {
      reset_simulation_ = true;
    }
    ImGui::End();
  }
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  // @@@@@@@@@@@@@@@@@@@Here everything is displayed
  // record();
  
  // swap OpenGL buffers (blocking call due to v-sync)
  glfwSwapBuffers(window);

  return !glfwWindowShouldClose(window);
}

void MjSimImpl::stopSimulation() {}

void MjSimImpl::saveGUISettings()
{
  auto user_path = bfs::path(USER_FOLDER);
  if(!bfs::exists(user_path))
  {
    if(!bfs::create_directories(user_path))
    {
      mc_rtc::log::critical("Failed to create the user directory: {}. GUI configuration will not be saved",
                            user_path.string());
      return;
    }
  }
  mc_rtc::Configuration config;
  auto camera_c = config.add("camera");
  auto lookat = camera_c.array("lookat", 3);
  for(size_t i = 0; i < 3; ++i)
  {
    lookat.push(camera.lookat[i]);
  }
  camera_c.add("distance", camera.distance);
  camera_c.add("azimuth", camera.azimuth);
  camera_c.add("elevation", camera.elevation);
  auto visualize_c = config.add("visualize");
  visualize_c.add("collisions", static_cast<bool>(options.geomgroup[0]));
  visualize_c.add("visuals", static_cast<bool>(options.geomgroup[1]));
  visualize_c.add("contact-points", static_cast<bool>(options.flags[mjVIS_CONTACTPOINT]));
  visualize_c.add("contact-forces", static_cast<bool>(options.flags[mjVIS_CONTACTFORCE]));
  auto path_out = (user_path / "mc_mujoco.yaml").string();
  config.save(path_out);
  mc_rtc::log::success("[mc_mujoco] Configuration saved to {}", path_out);
}

void MjSimImpl::record()
{
  // save frames for video
  if(config.recording && !config.step_by_step && ((data->time - frametime_) > 1.0 / 30.0 || frametime_== 0 ))
  {
    mjrRect viewport =  mjr_maxViewport(&context);
    int W = viewport.width;
    int H = viewport.height;

    // allocate rgb and depth buffers
    unsigned char* rgb = (unsigned char*)malloc(3*W*H);
    float* depth = (float*)malloc(sizeof(float)*W*H);
    if( !rgb || !depth )
    {
      mju_error("Could not allocate buffers");
    }

    mjr_readPixels(rgb, depth, viewport, &context);

    stbi_flip_vertically_on_write(true);
    std::stringstream filename;
    filename << "record_" <<  std::setfill('0') << std::setw(5) << framecount_ ++ << ".png"; 
    stbi_write_png(filename.str().c_str(), W, H, 3, rgb, 3*W);
    // std::cout << data->time - frametime_ << " " << 1.0 / 60.0 << std::endl;
    frametime_ = data->time;

    free(rgb);
    free(depth);
  }
}

MjSim::MjSim(const MjConfiguration & config) : impl(new MjSimImpl(config))
{
  impl->startSimulation();
}

MjSim::~MjSim()
{
  impl->cleanup();
}

bool MjSim::stepSimulation()
{
  return impl->stepSimulation();
}

void MjSim::stopSimulation()
{
  impl->stopSimulation();
}

void MjSim::updateScene()
{
  impl->updateScene();
}

void MjSim::resetSimulation(const std::map<std::string, std::vector<double>> & reset_qs,
                            const std::map<std::string, sva::PTransformd> & reset_pos)
{
  impl->resetSimulation(reset_qs, reset_pos);
}

bool MjSim::render()
{
  return impl->render();
}

const MjConfiguration & MjSim::config() const
{
  return impl->config;
}

mc_control::MCGlobalController * MjSim::controller() noexcept
{
  return impl->get_controller();
}

} // namespace mc_mujoco
