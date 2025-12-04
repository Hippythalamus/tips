#pragma once

#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>


// qos profile struct 
struct QosProfile
{
    rmw_qos_profile_t rmw{};
    int frequency_hz = 0;   

    QosProfile()
    {
        rmw = rmw_qos_profile_default;
    }
};

// Loading parameters from yaml
inline QosProfile load_qos(const YAML::Node & node)
{
    QosProfile profile;

    auto & rmw = profile.rmw;

    // reliability
    if (node["reliability"])
    {
        std::string v = node["reliability"].as<std::string>();
        if (v == "reliable")
            rmw.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        else
            rmw.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    }

    // durability
    if (node["durability"])
    {
        std::string v = node["durability"].as<std::string>();
        if (v == "transient_local")
            rmw.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        else
            rmw.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    }

    // history
    if (node["history"])
    {
        std::string v = node["history"].as<std::string>();
        if (v == "keep_all")
            rmw.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
        else
            rmw.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    }

    // depth
    if (node["depth"])
    {
        rmw.depth = node["depth"].as<int>();
    }

    // frequency 
    if (node["frequency"])
    {
        profile.frequency_hz = node["frequency"].as<int>();
    }

    return profile;
}


// qos profile to rclcpp QOS
inline rclcpp::QoS make_qos_from_rmw(const rmw_qos_profile_t & rmw)
{
    rclcpp::QoSInitialization init = rclcpp::QoSInitialization::from_rmw(rmw);
    rclcpp::QoS qos(init);

    qos.reliability(rclcpp::ReliabilityPolicy(rmw.reliability));
    qos.durability(rclcpp::DurabilityPolicy(rmw.durability));
    qos.liveliness(rclcpp::LivelinessPolicy(rmw.liveliness));

    qos.deadline(rmw.deadline);
    qos.lifespan(rmw.lifespan);
    qos.liveliness_lease_duration(rmw.liveliness_lease_duration);

    return qos;
}

// Load all parameters from yaml
class QosLoader
{
public:
    explicit QosLoader(const std::string & yaml_path)
    {
        YAML::Node yaml = YAML::LoadFile(yaml_path);
        YAML::Node root = yaml["qos_profiles"];

        if (!root)
            throw std::runtime_error("Processing error");

        for (auto it : root)
        {
            const std::string name = it.first.as<std::string>();
            profiles_[name] = load_qos(it.second);
        }
    }

    // Get rclcpp::QoS
    rclcpp::QoS qos(const std::string & name) const
    {
        return make_qos_from_rmw(get(name).rmw);
    }

    // get profile rmw 
    const QosProfile & get(const std::string & name) const
    {
        if (profiles_.count(name) == 0)
            throw std::runtime_error("QoS profile not found: " + name);

        return profiles_.at(name);
    }

    /// qet frequency
    int frequency(const std::string & name) const
    {
        return get(name).frequency_hz;
    }

private:
    std::unordered_map<std::string, QosProfile> profiles_;
};

