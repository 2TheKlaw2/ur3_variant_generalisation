#include "ur3_variant_generalisation/variant_trajectory_generator.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace ur3_variant_generalisation {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kReferenceDepthM = 0.005;
constexpr double kReferenceYawRad = -10.0 * kPi / 180.0;
constexpr double kTabRadiusM = 0.0288;
constexpr double kReferenceInsertDurationS = 1.5;
constexpr double kReferenceRotateDurationS = 1.5;
constexpr double kSampleDt = 0.01;

// Controlled test motion, NOT a real freedrive recording.
double smoothStep(double s) { return s*s*(3.0 - 2.0*s); }
AnalysisSample sample(double time, int phase, double top_m, double insertion_m,
                      double rotation_progress_rad, double coupled_drop_m) {
  AnalysisSample result;
  result.time_s = time;
  result.phase = phase;
  result.fixture_relative_z_m = top_m - insertion_m - coupled_drop_m;
  result.relative_yaw_rad = rotation_progress_rad;
  result.qz = std::sin(rotation_progress_rad / 2.0);
  result.qw = std::cos(rotation_progress_rad / 2.0);
  return result;
}
} // namespace

bool SyntheticV1Skill::train() {
  std::vector<double> t1, z, t2, yaw;
  for (int i=0; i<=150; ++i) {
    const double u = static_cast<double>(i) / 150.0;
    t1.push_back(kReferenceInsertDurationS*u);
    z.push_back(kReferenceDepthM*smoothStep(u));
    t2.push_back(kReferenceRotateDurationS*u);
    yaw.push_back(kReferenceYawRad*smoothStep(u));
  }
  return insertion_.train(t1,z) && rotation_.train(t2,yaw);
}

std::vector<AnalysisSample> SyntheticV1Skill::generate(
  const VariantGeometry & v, TimePolicy policy) const {
  validateVariantGeometry(v);
  if (!insertion_.trained() || !rotation_.trained()) throw std::runtime_error("Skill not trained");

  const double insertion_goal_m = v.insertion_depth_mm / 1000.0;
  const double rotation_goal_rad = v.rotation_deg * kPi / 180.0;
  const double extra_drop_m = v.rotation_drop_mm / 1000.0;
  const double top_m = v.top_offset_mm / 1000.0;
  double insertion_scale = 1.0, rotation_scale = 1.0;
  if (policy == TimePolicy::ConstantNominalTipSpeed) {
    insertion_scale = insertion_goal_m / kReferenceDepthM;
    // For V5 account for the combined circumferential + axial travel.
    const double new_arc_m = kTabRadiusM * std::abs(rotation_goal_rad);
    const double ref_arc_m = kTabRadiusM * std::abs(kReferenceYawRad);
    rotation_scale = std::hypot(new_arc_m, extra_drop_m) / ref_arc_m;
  }

  const auto depths = insertion_.rollout(insertion_goal_m, insertion_scale, kSampleDt);
  const auto yaws = rotation_.rollout(rotation_goal_rad, rotation_scale, kSampleDt);
  if (depths.size()<2 || yaws.size()<2) throw std::runtime_error("DMP rollout failed");
  std::vector<AnalysisSample> result;
  result.reserve(depths.size() + yaws.size()-1);
  // The DMP implementation pins its last output to the goal; inspect continuity
  // before using such a trajectory for physical planning.
  const double insert_dt = insertion_.duration()*insertion_scale/(depths.size()-1);
  const double rotate_dt = rotation_.duration()*rotation_scale/(yaws.size()-1);
  for (std::size_t i=0; i<depths.size(); ++i) {
    result.push_back(sample(i*insert_dt, 1, top_m, depths[i], 0.0, 0.0));
  }
  const double phase2_start = result.back().time_s;
  for (std::size_t i=1; i<yaws.size(); ++i) {
    // Coupled descent follows measured angular progress, not elapsed wall time.
    const double progress = std::clamp(yaws[i]/rotation_goal_rad, 0.0, 1.0);
    result.push_back(sample(phase2_start+i*rotate_dt, 2, top_m,
      insertion_goal_m, yaws[i], extra_drop_m*progress));
  }
  return result;
}

void saveAnalysisCSV(const std::string & file,
                     const std::vector<AnalysisSample> & samples) {
  std::ofstream out(file);
  if (!out) throw std::runtime_error("Cannot create output: " + file);
  // Intentionally INCOMPATIBLE with ur3_lfd DemoIO / MoveIt executor CSV.
  out << "time_s,phase,local_x_m,local_y_m,fixture_relative_z_m,relative_yaw_rad,qx,qy,qz,qw\n";
  out << std::setprecision(12);
  for (const auto & s : samples) {
    out << s.time_s << ',' << s.phase << ',' << s.local_x_m << ',' << s.local_y_m
        << ',' << s.fixture_relative_z_m << ',' << s.relative_yaw_rad
        << ',' << s.qx << ',' << s.qy << ',' << s.qz << ',' << s.qw << '\n';
  }
}
} // namespace ur3_variant_generalisation
