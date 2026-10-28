#pragma once

#include <string>
#include <vector>
#include "ur3_variant_generalisation/variant_geometry.hpp"
#include "ur3_variant_generalisation/variant_trajectory_generator.hpp"

namespace ur3_variant_generalisation {

// Plate coordinates are measured relative to the centre/top of V1 on EACH
// respective plate. Local +X follows the row; it is NOT robot-base +X.
struct PlateSlot {
  std::string variant_id;
  double row_offset_mm{0.0};
  double height_offset_mm{0.0};
  double initial_yaw_deg{0.0};
};
struct PlateLayout {
  std::string name;
  std::vector<PlateSlot> slots;
};
struct PlateLocalPose {
  double x_m{0.0}, y_m{0.0}, z_m{0.0};
  double yaw_rad{0.0};
};

// A 3D calibration pose for the V1 centre/top of the specified plate.
// We deliberately refuse base-frame conversion until a verified calibration
// has been supplied. Not a robot-controller pose or executable plan.
struct PlateRegistration {
  std::string plate_name;
  bool verified{false};
  double x_m{0.0}, y_m{0.0}, z_m{0.0};
  double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
};
struct Pose3D {
  double x_m{0.0}, y_m{0.0}, z_m{0.0};
  double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
};
struct PlateLocalSample {
  double time_s{0.0};
  int phase{0};
  double x_m{0.0}, y_m{0.0}, z_m{0.0};
  double yaw_rad{0.0};
  double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
};

std::vector<PlateRegistration> readPlateRegistrationsCSV(const std::string & path);

PlateLayout readPlateLayoutCSV(const std::string & path, const std::string & name);
const PlateSlot & findPlateSlot(const PlateLayout & plate, const std::string & variant_id);
void validatePlateAndMotion(const PlateLayout & pickup,
                            const PlateLayout & placement,
                            const std::vector<VariantGeometry> & variants);
PlateLocalPose localTopReference(const PlateSlot & slot);
std::vector<PlateLocalSample> toPlacementPlateLocal(
    const PlateSlot & placement_slot,
    const VariantGeometry & variant,
    const std::vector<AnalysisSample> & fitting_local_path);
void savePlateLocalReferencesCSV(const std::string & filename,
                                 const PlateLayout & pickup,
                                 const PlateLayout & placement);
void savePlateLocalTrajectoryCSV(const std::string & filename,
                                 const std::vector<PlateLocalSample> & samples);

// Applies an independently calibrated rigid plate-to-base transformation.
// Used only for unit tests/previews, never to create robot commands.
Pose3D transformToBase(const PlateLocalPose & local,
                       const PlateRegistration & registration);

}  // namespace ur3_variant_generalisation
