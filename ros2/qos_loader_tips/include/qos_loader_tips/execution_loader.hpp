#pragma once
#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

struct ExecutionProfile
{
    std::string executor;          // single_threaded multi_threaded  dedicated (custom)
    std::string callback_group;    // reentrant / mutually_exclusive
    //int cpu_affinity = -1;         // -1 = no pinning
};

class ExecutionLoader
{
public:
    explicit ExecutionLoader(const std::string & yaml_path)
    {
        YAML::Node root = YAML::LoadFile(yaml_path)["execution_profiles"];
        if (!root)
            throw std::runtime_error("execution_profiles not found");

        for (auto it : root) {
            const std::string name = it.first.as<std::string>();
            profiles_[name] = parse(it.second);
        }
    }

    const ExecutionProfile & get(const std::string & name) const
    {
        return profiles_.at(name);
    }

private:
    ExecutionProfile parse(const YAML::Node & n)
    {
        ExecutionProfile p;
        p.executor       = n["executor"].as<std::string>("multi_threaded");
        p.callback_group = n["callback_group"].as<std::string>("reentrant");
      //  p.cpu_affinity   = n["cpu_affinity"].as<int>(-1);            TODO: I have better idea,  but I have no time
        return p;
    }

    std::unordered_map<std::string, ExecutionProfile> profiles_;
};
