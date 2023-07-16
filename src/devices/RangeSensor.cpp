#include <mc_mujoco/devices/RangeSensor.h>

namespace mc_mujoco
{

RangeSensor::RangeSensor(const std::string & name, const std::string & parent, const sva::PTransformd & X_p_s)
: mc_rbdyn::Device(name, parent, X_p_s)
{
}

mc_rbdyn::DevicePtr RangeSensor::clone() const
{
  return mc_rbdyn::DevicePtr(new RangeSensor(name_, parent_, X_p_s_));
}

void RangeSensor::update(const double & data, const double & time)
{
  // config_.minAngle = data.config.minAngle;
  // config_.maxAngle = data.config.maxAngle;
  // config_.angularRes = data.config.angularRes;
  // config_.minRange = data.config.minRange;
  // config_.maxRange = data.config.maxRange;
  // config_.rangeRes = data.config.rangeRes;
  // config_.frequency = data.config.frequency;
  data_ = data;
  time_ = time;
}

void RangeSensor::addToLogger(mc_rtc::Logger & logger, const std::string & prefixIn)
{
  auto prefix = prefixIn.size() ? fmt::format("{}_{}_", prefixIn, name_) : name_ + "_";
  logger.addLogEntry(prefix + "config_minAngle", this, [this]() { return config_.minAngle; });
  logger.addLogEntry(prefix + "config_maxAngle", this, [this]() { return config_.maxAngle; });
  logger.addLogEntry(prefix + "config_angularRes", this, [this]() { return config_.angularRes; });
  logger.addLogEntry(prefix + "config_minRange", this, [this]() { return config_.minRange; });
  logger.addLogEntry(prefix + "config_maxRange", this, [this]() { return config_.maxRange; });
  logger.addLogEntry(prefix + "config_rangeRes", this, [this]() { return config_.rangeRes; });
  logger.addLogEntry(prefix + "config_frequency", this, [this]() { return config_.frequency; });
  logger.addLogEntry(prefix + "data", this, [this]() -> const double & { return data_; });
}

void RangeSensor::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(this);
}

} // namespace mc_openrtm
