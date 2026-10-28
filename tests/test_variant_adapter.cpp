#include <algorithm>
#include "ur3_variant_generalisation/variant_geometry.hpp"
#include "ur3_variant_generalisation/variant_trajectory_generator.hpp"
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace vg = ur3_variant_generalisation;
constexpr double PI = 3.14159265358979323846;

void expect(bool condition, const std::string & message) {
  if (!condition) throw std::runtime_error(message);
}
void near(double a, double b, double tol, const std::string & msg) {
  expect(std::abs(a-b)<tol, msg + " expected=" + std::to_string(b) +
         " got=" + std::to_string(a));
}

int main() {
  try {
    vg::SyntheticV1Skill skill;
    expect(skill.train(), "synthetic training should succeed");
    const std::array<vg::VariantGeometry,5> variants{{
      {"V1",5,-10,0,0}, {"V2",5,10,0,0}, {"V3",10,-10,0,5},
      {"V4",10,10,0,5}, {"V5",5,-10,5,5}
    }};
    const std::array<double,5> expected_z{{-5,-5,-5,-5,-5}};
    for (std::size_t i=0;i<variants.size();++i) {
      for (const auto policy : {vg::TimePolicy::FixedDuration, vg::TimePolicy::ConstantNominalTipSpeed}) {
        const auto path=skill.generate(variants[i],policy);
        expect(path.size()>50, variants[i].id + " trajectory too short");
        expect(path.front().phase==1 && path.back().phase==2, "phase markers incorrect");
        near(path.front().fixture_relative_z_m, variants[i].top_offset_mm/1000., 1e-9, "top height");
        near(path.back().fixture_relative_z_m,expected_z[i]/1000.,1e-8,"final z");
        near(path.back().relative_yaw_rad,variants[i].rotation_deg*PI/180.,1e-9,"yaw endpoint");
        int transitions=0;
        double max_insertion_z=path.front().fixture_relative_z_m;
        for (std::size_t j=0;j<path.size();++j) {
          const auto & p=path[j];
          if (j>0) {
            expect(p.time_s>path[j-1].time_s,"time must rise");
            if (p.phase != path[j-1].phase) {
              transitions++;
              near(path[j-1].fixture_relative_z_m,
                   (variants[i].top_offset_mm-variants[i].insertion_depth_mm)/1000.,
                   1e-8,"bend insertion depth");
            }
            // A large last-step jump indicates a poorly converged DMP rollout.
            const double dz=std::abs(p.fixture_relative_z_m-path[j-1].fixture_relative_z_m);
            const double da=std::abs(p.relative_yaw_rad-path[j-1].relative_yaw_rad);
            expect(dz<0.0010,"excessive axial single-sample step");
            expect(da<0.035,"excessive angular single-sample step");
          }
          const double norm=p.qx*p.qx+p.qy*p.qy+p.qz*p.qz+p.qw*p.qw;
          near(norm,1.0,1e-9,"quaternion norm");
          max_insertion_z=std::min(max_insertion_z,p.fixture_relative_z_m);
        }
        expect(transitions==1,"exactly one insertion-to-rotation switch");
        if (variants[i].id!="V5") {
          for (const auto & p : path) if (p.phase==2) {
            near(p.fixture_relative_z_m,(variants[i].top_offset_mm-variants[i].insertion_depth_mm)/1000.,1e-8,
                 "no phase-2 descent except V5");
          }
        }
        if (variants[i].id=="V5") {
          near(max_insertion_z,-0.005,1e-7,"V5 minimum z");
          for (const auto & p : path) if(p.phase==2) {
            const double progress=std::clamp(p.relative_yaw_rad/(variants[i].rotation_deg*PI/180.),0.,1.);
            near(p.fixture_relative_z_m,-0.005*progress,1e-8,"V5 rotation/descent synchronisation");
          }
        }
        if (policy==vg::TimePolicy::ConstantNominalTipSpeed && variants[i].id=="V3") {
          expect(path.back().time_s>3.8,"double insertion should take longer");
        }
        if (policy==vg::TimePolicy::ConstantNominalTipSpeed && variants[i].id=="V5") {
          expect(path.back().time_s>3.5,"helical coupling should extend rotation duration");
        }
      }
    }
    near(0.0288*(10.*PI/180.),0.005026548,1e-7,"tab radius and arc length");
    std::cout << "PASS: 5 variants x 2 timing policies; endpoints, quaternion, coupling and continuity\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
