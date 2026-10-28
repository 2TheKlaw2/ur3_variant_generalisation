#pragma once
#include <string>
#include <vector>

namespace ur3_variant_generalisation {

struct VariantGeometry {
  std::string id;
  double insertion_depth_mm{0.0};
  double rotation_deg{0.0}; // - = clockwise viewed from above, pending fixture-axis verification
  double rotation_drop_mm{0.0};
  double top_offset_mm{0.0};
  // NaN when fixture position has NOT been calibrated. We do not use these yet.
  double fixture_center_x_mm{0.0};
  double fixture_center_y_mm{0.0};
  double fixture_yaw_deg{0.0};
};

// Reads the existing 8-column variant_motion_parameters.csv.
// Empty fixture columns remain uncalibrated, not implicitly zero.
std::vector<VariantGeometry> readVariantCSV(const std::string & file);
void validateVariantGeometry(const VariantGeometry & v);

} // namespace ur3_variant_generalisation
