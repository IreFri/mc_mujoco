#pragma once

#include <mc_mujoco/devices/api.h>

#include <mc_rbdyn/Device.h>
#include <mc_rtc/log/Logger.h>

#include <vector>

namespace mc_mujoco
{

struct RangeSensor;

struct MC_MUJOCO_DEVICES_DLLAPI RangeSensor : public mc_rbdyn::Device
{
  RangeSensor(const std::string & name, const std::string & parent, const sva::PTransformd & X_p_s);

  struct Configuration
  {
    /** Minimum scannable angle in radians */
    double minAngle;
    /** Maximum scannable angle in radians */
    double maxAngle;
    /** Angular resolution in radians */
    double angularRes;
    /** Minimum scannable range in meters */
    double minRange;
    /** Maximum scannable range in meters */
    double maxRange;
    /** Range resolution in meters */
    double rangeRes;
    /** Scanning frequency */
    double frequency;
  };

  /** Return the latest range sensing data, might be empty */
  inline const double & data() const noexcept
  {
    return data_;
  }

  /** Return the latest range sensing time, might be empty */
  inline const double & time() const noexcept
  {
    return time_;
  }

  /** Access the RangeSensor configuration
   *
   * This configuration is invalid before data has been received
   */
  inline const Configuration config() const noexcept
  {
    return config_;
  }

  /** Update the sensor configuration and data from an OpenRTM message */
  void update(const double & data, const double & time);

  /** Add the sensor to a logger */
  void addToLogger(mc_rtc::Logger & logger, const std::string & prefix);

  /** Remove the sensor from a logger */
  void removeFromLogger(mc_rtc::Logger & logger);

  mc_rbdyn::DevicePtr clone() const override;

private:
  Configuration config_;
  double data_;
  double time_;
};

} // namespace mc_openrtm
