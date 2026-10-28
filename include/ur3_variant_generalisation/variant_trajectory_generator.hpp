#pragma once
#include <string>
#include <vector>
#include "ur3_lfd/dmp.hpp"
#include "ur3_variant_generalisation/variant_geometry.hpp"

namespace ur3_variant_generalisation {

// Fixture-relative, offline-only sample. Coordinates deliberately NOT robot-base
// poses. These files must never be passed directly to a robot executor.
struct AnalysisSample {
  double time_s{0.0};
  int phase{0}; // 1 = insertion, 2 = engagement rotation
  double local_x_m{0.0};
  double local_y_m{0.0};
  double fixture_relative_z_m{0.0};
  double relative_yaw_rad{0.0};
  double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
};

enum class TimePolicy { FixedDuration, ConstantNominalTipSpeed };

class SyntheticV1Skill {
public:
  bool train();
  std::vector<AnalysisSample> generate(
    const VariantGeometry & variant, TimePolicy time_policy) const;
  double insertionDuration() const { return insertion_.duration(); }
  double rotationDuration() const { return rotation_.duration(); }
private:
  ur3_lfd::DiscreteDMP1D insertion_{30};
  ur3_lfd::DiscreteDMP1D rotation_{30};
};

void saveAnalysisCSV(const std::string & file,
                     const std::vector<AnalysisSample> & samples);

} // namespace ur3_variant_generalisation
