#include "ur3_variant_generalisation/plate_layout.hpp"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace fs=std::filesystem;
namespace vg=ur3_variant_generalisation;

int main(int argc, char ** argv) {
  try {
    if (argc<5 || argc>6) {
      std::cerr<<"Usage: plate_layout_preview <pickup_plate.csv> <placement_plate.csv> "
                 "<variant_motion_parameters.csv> <output_dir> [fixed|speed]\n";
      return 2;
    }
    const std::string policy=argc==6?argv[5]:"fixed";
    const auto time_policy=policy=="fixed" ? vg::TimePolicy::FixedDuration
        : policy=="speed" ? vg::TimePolicy::ConstantNominalTipSpeed
        : throw std::runtime_error("Policy must be 'fixed' or 'speed'");
    const auto pickup=vg::readPlateLayoutCSV(argv[1],"pickup_local");
    const auto placement=vg::readPlateLayoutCSV(argv[2],"placement_local");
    const auto variants=vg::readVariantCSV(argv[3]);
    vg::validatePlateAndMotion(pickup,placement,variants);
    // Registration is deliberately OPTIONAL for this offline analysis tool;
    // any future robot-base export must refuse unverified registrations.
    const auto registration_csv=fs::path(argv[1]).parent_path()/"plate_base_registration.csv";
    if (fs::exists(registration_csv)) {
      const auto registrations=vg::readPlateRegistrationsCSV(registration_csv.string());
      for (const auto & frame: registrations) {
        std::cout<<"Plate robot-base registration ("<<frame.plate_name<<"): "
                 <<(frame.verified?"VERIFIED IN FILE (not independently validated)":"MISSING")<<'\n';
      }
    }
    vg::SyntheticV1Skill skill;
    if (!skill.train()) throw std::runtime_error("Synthetic demonstration training failed");
    fs::create_directories(argv[4]);
    const fs::path base(argv[4]);
    vg::savePlateLocalReferencesCSV((base/"ANALYSIS_ONLY_plate_references.csv").string(),
                                    pickup,placement);
    std::cout<<"ANALYSIS-ONLY PLATE-LAYOUT TEST; no robot-base calibration\n"
             <<"Pickup offsets: 40 mm; placement offsets: 50 mm (plate-local +X).\n"
             <<"Plate +X is a CAD convention, NOT UR3e base +X.\n"
             <<"'Top reference' is NOT a calibrated RG2 gripping TCP position.\n\n";
    for (const auto & variant: variants) {
      const auto & pickup_slot=vg::findPlateSlot(pickup,variant.id);
      const auto & place_slot=vg::findPlateSlot(placement,variant.id);
      const auto path=skill.generate(variant,time_policy);
      const auto transformed=vg::toPlacementPlateLocal(place_slot,variant,path);
      vg::savePlateLocalTrajectoryCSV((base/("ANALYSIS_ONLY_"+variant.id+
                                     "_placement_local_"+policy+".csv")).string(),transformed);
      const auto initial=vg::localTopReference(pickup_slot);
      const auto dest=vg::localTopReference(place_slot);
      std::cout<<variant.id<<": pickup x="<<std::fixed<<std::setprecision(1)
               <<1000*initial.x_m<<" mm z="<<1000*initial.z_m
               <<" mm; placement x="<<1000*dest.x_m<<" mm z="<<1000*dest.z_m
               <<" mm; assembly final z="<<1000*transformed.back().z_m
               <<" mm (placement-V1-top reference)\n";
    }
    std::cout<<"\nNo physical plate origins or tool offsets are known. "
                "Nothing here is safe to send to MoveIt/UR3e.\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr<<"ERROR: "<<e.what()<<'\n';
    return 1;
  }
}
