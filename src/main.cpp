#include "ur3_variant_generalisation/variant_geometry.hpp"
#include "ur3_variant_generalisation/variant_trajectory_generator.hpp"
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;
namespace vg = ur3_variant_generalisation;

int main(int argc, char ** argv) {
  try {
    if (argc < 3 || argc > 4) {
      std::cerr << "Usage: variant_offline_generator <variant_motion_parameters.csv> <output_dir> [fixed|speed]\n";
      return 2;
    }
    const std::string policy_arg = argc == 4 ? argv[3] : "fixed";
    const auto policy = policy_arg == "fixed" ? vg::TimePolicy::FixedDuration
      : policy_arg == "speed" ? vg::TimePolicy::ConstantNominalTipSpeed
      : throw std::runtime_error("Unknown time policy (use fixed or speed)");
    auto variants = vg::readVariantCSV(argv[1]);
    vg::SyntheticV1Skill skill;
    if (!skill.train()) throw std::runtime_error("Synthetic DMP training failed");
    fs::create_directories(argv[2]);
    std::cout << "OFFLINE SYNTHETIC CDMP TEST — NOT EXECUTABLE ON ROBOT\n"
              << "Frame: fitting-local x/y; z relative to V1 fixture top.\n"
              << "Initial orientation: identity; yaw is local about +Z.\n"
              << "Time policy: " << policy_arg << "\n\n";
    for (const auto & v : variants) {
      const auto samples = skill.generate(v, policy);
      const fs::path output = fs::path(argv[2]) /
        ("ANALYSIS_ONLY_" + v.id + "_" + policy_arg + ".csv");
      vg::saveAnalysisCSV(output.string(), samples);
      const auto & last = samples.back();
      std::cout << v.id << ": n=" << samples.size()
                << ", final z relative V1 top=" << std::fixed << std::setprecision(2)
                << 1000.0*last.fixture_relative_z_m << " mm"
                << ", yaw=" << (180.0/3.14159265358979323846)*last.relative_yaw_rad
                << " deg, duration=" << last.time_s << " s\n"
                << "    -> " << output << '\n';
    }
    std::cout << "\nNo ROS node started, no controller touched, no base-frame trajectory generated.\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "ERROR: " << e.what() << '\n';
    return 1;
  }
}
