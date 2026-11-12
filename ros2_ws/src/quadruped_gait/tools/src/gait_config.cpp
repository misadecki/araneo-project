#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include "gait_config.hh"

GaitConfig loadGaitConfig(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);

  YAML::Node n = root["gait_node"];
  if (!n) throw std::runtime_error("No gait_node in file " + path);
  n = n["ros__parameters"];
  if (!n) throw std::runtime_error("No gait_node/ros__parameters in file " + path);

  auto get = [&](const std::string& key) {
    if (!n[key]) throw std::runtime_error("Lack of key: " + key);
    return n[key].as<double>();
  };

  GaitConfig c = {};
  const std::string name = n["gait"].as<std::string>("walk");
  if (name == "walk") c.gait = WALK;
  else if (name == "trot") c.gait = TROT;
  else throw std::runtime_error("Gait: trot | walk. (" + name + ")");

  c.params.q1_m = get("q1_m");
  c.params.q2_st = get("q2_st");
  c.params.Tsw = get("Tsw");
  c.params.dq2 = get("dq2");
  c.a_max = get("a_max");

  return c;
}
