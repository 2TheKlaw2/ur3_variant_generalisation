#include "ur3_variant_generalisation/plate_layout.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace vg=ur3_variant_generalisation;
namespace {
constexpr double kPi=3.14159265358979323846;
void check(bool ok,const std::string & name) {
  if (!ok) throw std::runtime_error(name);
}
void near(double actual,double expected,double eps,const std::string & label) {
  check(std::abs(actual-expected)<eps,label+": expected "+std::to_string(expected)+
                                    " actual "+std::to_string(actual));
}
}
int main(int argc, char ** argv) {
  try {
    if (argc!=4) throw std::runtime_error("Test needs pickup, placement, motion CSV paths");
    auto pickup=vg::readPlateLayoutCSV(argv[1],"pickup_local");
    auto placement=vg::readPlateLayoutCSV(argv[2],"placement_local");
    const auto motion=vg::readVariantCSV(argv[3]);
    vg::validatePlateAndMotion(pickup,placement,motion);
    const auto file=std::string(argv[1]);
    const auto registration_path=file.substr(0,file.find_last_of('/'))+"/plate_base_registration.csv";
    const auto frames=vg::readPlateRegistrationsCSV(registration_path);
    check(frames.size()==2 && !frames[0].verified && !frames[1].verified,
          "unregistered plates must remain explicitly unverified");
    const std::array<int,5> pick_x{0,40,80,120,160};
    const std::array<int,5> place_x{0,50,100,150,200};
    const std::array<int,5> heights{0,0,5,5,5};
    vg::SyntheticV1Skill skill;
    check(skill.train(),"synthetic V1 training");
    for (int i=0;i<5;++i) {
      const std::string id="V"+std::to_string(i+1);
      const auto & p=vg::findPlateSlot(pickup,id);
      const auto & d=vg::findPlateSlot(placement,id);
      near(vg::localTopReference(p).x_m,pick_x[i]*0.001,1e-12,"pickup x");
      near(vg::localTopReference(d).x_m,place_x[i]*0.001,1e-12,"placement x");
      near(vg::localTopReference(p).z_m,heights[i]*0.001,1e-12,"pickup height");
      near(vg::localTopReference(d).z_m,heights[i]*0.001,1e-12,"placement height");
      near(vg::localTopReference(d).yaw_rad,0,1e-12,"placement initial yaw");
      auto generated=skill.generate(motion.at(i),vg::TimePolicy::FixedDuration);
      auto transformed=vg::toPlacementPlateLocal(d,motion.at(i),generated);
      check(transformed.size()==generated.size(),"trajectory size");
      near(transformed.front().x_m,place_x[i]*0.001,1e-12,"starting fitting x");
      near(transformed.back().x_m,place_x[i]*0.001,1e-12,"ending fitting x");
      near(transformed.front().z_m,heights[i]*0.001,1e-12,"starting fitting z");
      near(transformed.back().z_m,-0.005,1e-9,"ending fitting z");
      near(transformed.back().yaw_rad,motion.at(i).rotation_deg*kPi/180,1e-10,
           "ending fitting yaw");
      near(transformed.at(generated.size()/2).z_m,
           generated.at(generated.size()/2).fixture_relative_z_m,1e-12,
           "no double application of height offset");
    }
    // Registration must be explicitly verified; an uncalibrated plate frame
    // must NOT silently be interpreted as a base-frame origin.
    const vg::PlateRegistration missing{"pickup",false};
    bool refused=false;
    try { (void)vg::transformToBase({0.040,0,0.005,0},missing); }
    catch (const std::runtime_error &) { refused=true; }
    check(refused,"must reject unverified base transform");

    // Synthetic-only verification of the full 3-D rigid transformation.
    // Rotate +90 deg around world Z; local +X becomes world +Y.
    const vg::PlateRegistration mock{"test",true,1.0,2.0,0.8,
      0,0,std::sin(kPi/4),std::cos(kPi/4)};
    const auto target=vg::transformToBase({0.040,0,0.005,0.1},mock);
    near(target.x_m,1.0,1e-10,"registered x");
    near(target.y_m,2.040,1e-10,"registered y");
    near(target.z_m,0.805,1e-10,"registered z");
    near(target.qz,std::sin((kPi/2+0.1)/2),1e-10,"registered yaw quaternion");

    // Fixture yaw changes the orientation/XY mapping, but not the height.
    vg::PlateSlot rotated{"V1",50,5,90};
    vg::VariantGeometry synthetic{"V1",5,-10,0,5};
    auto fake_path=skill.generate(synthetic,vg::TimePolicy::FixedDuration);
    fake_path.front().local_x_m=0.001;
    auto after=vg::toPlacementPlateLocal(rotated,synthetic,fake_path);
    near(after.front().x_m,0.050,1e-10,"slot yaw local x");
    near(after.front().y_m,0.001,1e-10,"slot yaw local y");
    near(after.front().yaw_rad,kPi/2,1e-10,"slot yaw orientation");
    std::cout<<"PASS: pickup/placement offsets, height changes, yaw, "
                "five fitted trajectories, uncalibrated guard and rigid transform.\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr<<"FAIL: "<<e.what()<<'\n';
    return 1;
  }
}
