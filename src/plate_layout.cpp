#include "ur3_variant_generalisation/plate_layout.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace ur3_variant_generalisation {
namespace {
constexpr double kPi=3.14159265358979323846;
constexpr double kDegToRad=kPi/180.0;
constexpr double kMmToM=1.0/1000.0;
std::string trim(const std::string & text) {
  const auto start=text.find_first_not_of(" \t\r\n");
  if (start==std::string::npos) return "";
  const auto end=text.find_last_not_of(" \t\r\n");
  return text.substr(start,end-start+1);
}
std::vector<std::string> csvFields(const std::string & line) {
  std::stringstream ss(line);
  std::vector<std::string> fields;
  std::string field;
  while (std::getline(ss,field,',')) fields.push_back(trim(field));
  if (!line.empty() && line.back()==',') fields.push_back("");
  return fields;
}
double requiredNumber(const std::string & value, const std::string & field) {
  if (value.empty()) throw std::runtime_error("Missing "+field);
  std::size_t parsed=0;
  const double result=std::stod(value,&parsed);
  if (parsed!=value.size() || !std::isfinite(result))
    throw std::runtime_error("Invalid number in "+field);
  return result;
}
struct Quaternion {double x,y,z,w;};
Quaternion multiply(Quaternion a, Quaternion b) {
  return {a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
          a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
          a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w,
          a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z};
}
Quaternion normalize(Quaternion q) {
  const double n=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
  if (!std::isfinite(n) || n<1e-10) throw std::runtime_error("Invalid plate orientation");
  return {q.x/n,q.y/n,q.z/n,q.w/n};
}
Quaternion yawQuaternion(double yaw_rad) {
  return {0.0,0.0,std::sin(yaw_rad/2),std::cos(yaw_rad/2)};
}
std::array<double,3> rotateVector(Quaternion q, std::array<double,3> v) {
  const Quaternion p{v[0],v[1],v[2],0.0};
  const Quaternion conj{-q.x,-q.y,-q.z,q.w};
  const auto result=multiply(multiply(q,p),conj);
  return {result.x,result.y,result.z};
}
void ensureFive(const PlateLayout & layout) {
  if (layout.slots.size()!=5)
    throw std::runtime_error(layout.name+": expected exactly five slots");
  for (int i=0;i<5;++i) {
    const auto & v=layout.slots.at(i);
    if (v.variant_id!="V"+std::to_string(i+1))
      throw std::runtime_error(layout.name+": expected ordered V1 through V5");
    if (!std::isfinite(v.row_offset_mm) || !std::isfinite(v.height_offset_mm) ||
        !std::isfinite(v.initial_yaw_deg))
      throw std::runtime_error(layout.name+": invalid numeric field");
  }
  // This fixture-family contract assumes a linear row with identical
  // orientation. A different family needs a different layout description.
  if (std::abs(layout.slots.front().row_offset_mm)>1e-8 ||
      std::abs(layout.slots.front().height_offset_mm)>1e-8)
    throw std::runtime_error(layout.name+": V1 must define the local origin/top");
  for (int i=1;i<5;++i) {
    if (layout.slots[i].row_offset_mm<=layout.slots[i-1].row_offset_mm)
      throw std::runtime_error(layout.name+": row offsets must increase");
    if (std::abs(layout.slots[i].initial_yaw_deg-layout.slots[0].initial_yaw_deg)>1e-7)
      throw std::runtime_error(layout.name+": orientation must be identical across the plate");
  }
}
}  // namespace

std::vector<PlateRegistration> readPlateRegistrationsCSV(const std::string & path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("Cannot read plate registration CSV: "+path);
  std::string line;
  if (!std::getline(file,line) || csvFields(line)!=std::vector<std::string>{
      "plate_id","base_origin_x_m","base_origin_y_m","base_origin_z_m",
      "roll_deg","pitch_deg","yaw_deg","verified"})
    throw std::runtime_error("Unexpected registration CSV columns: "+path);
  std::vector<PlateRegistration> frames;
  while (std::getline(file,line)) {
    if (trim(line).empty()) continue;
    const auto fields=csvFields(line);
    if (fields.size()!=8) throw std::runtime_error("Expected eight registration fields");
    if (fields[0]!="pickup" && fields[0]!="placement")
      throw std::runtime_error("Unknown plate registration name: "+fields[0]);
    if (fields[7]!="false" && fields[7]!="true")
      throw std::runtime_error("verified must be 'true' or 'false'");
    if (std::any_of(frames.begin(),frames.end(),[&](const auto & f){
        return f.plate_name==fields[0];}))
      throw std::runtime_error("Duplicate registration for "+fields[0]);
    PlateRegistration reg;
    reg.plate_name=fields[0];
    reg.verified=fields[7]=="true";
    if (reg.verified) {
      const double x=requiredNumber(fields[1],"base_origin_x_m");
      const double y=requiredNumber(fields[2],"base_origin_y_m");
      const double z=requiredNumber(fields[3],"base_origin_z_m");
      const double roll=requiredNumber(fields[4],"roll_deg")*kDegToRad;
      const double pitch=requiredNumber(fields[5],"pitch_deg")*kDegToRad;
      const double yaw=requiredNumber(fields[6],"yaw_deg")*kDegToRad;
      const Quaternion qx{std::sin(roll/2),0,0,std::cos(roll/2)};
      const Quaternion qy{0,std::sin(pitch/2),0,std::cos(pitch/2)};
      const auto q=normalize(multiply(multiply(yawQuaternion(yaw),qy),qx));
      reg.x_m=x;reg.y_m=y;reg.z_m=z;
      reg.qx=q.x;reg.qy=q.y;reg.qz=q.z;reg.qw=q.w;
    } else {
      for (int i=1;i<7;++i) {
        if (!fields[i].empty())
          throw std::runtime_error("Unverified registration coordinates must be blank");
      }
    }
    frames.push_back(reg);
  }
  if (frames.size()!=2) throw std::runtime_error("Expected pickup and placement registrations");
  return frames;
}

PlateLayout readPlateLayoutCSV(const std::string & path, const std::string & name) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("Cannot read plate CSV: "+path);
  std::string line;
  if (!std::getline(file,line) || csvFields(line)!=std::vector<std::string>{
      "variant_id","offset_mm","height_offset_mm","yaw_deg"})
    throw std::runtime_error("Unexpected plate CSV columns: "+path);
  PlateLayout out;
  out.name=name;
  while (std::getline(file,line)) {
    if (trim(line).empty()) continue;
    const auto fields=csvFields(line);
    if (fields.size()!=4) throw std::runtime_error("Expected 4 plate columns in: "+line);
    out.slots.push_back({fields[0],requiredNumber(fields[1],"offset_mm"),
       requiredNumber(fields[2],"height_offset_mm"),requiredNumber(fields[3],"yaw_deg")});
  }
  ensureFive(out);
  return out;
}

const PlateSlot & findPlateSlot(const PlateLayout & plate, const std::string & variant_id) {
  const auto found=std::find_if(plate.slots.begin(),plate.slots.end(),
                              [&](const PlateSlot & slot){return slot.variant_id==variant_id;});
  if (found==plate.slots.end()) throw std::runtime_error("Unknown "+plate.name+" slot: "+variant_id);
  return *found;
}

void validatePlateAndMotion(const PlateLayout & pickup, const PlateLayout & placement,
                            const std::vector<VariantGeometry> & variants) {
  ensureFive(pickup);
  ensureFive(placement);
  if (variants.size()!=5) throw std::runtime_error("Expected five assembly variants");
  for (const auto & variant:variants) {
    const auto & pick=findPlateSlot(pickup,variant.id);
    const auto & place=findPlateSlot(placement,variant.id);
    if (std::abs(place.height_offset_mm-variant.top_offset_mm)>1e-7)
      throw std::runtime_error(variant.id+": placement top height disagrees with motion CSV");
    // Current five-variant CAD: pickup blocks and fixture tops share height
    // offsets. Check explicitly so different plates are never silently conflated.
    if (std::abs(pick.height_offset_mm-place.height_offset_mm)>1e-7)
      throw std::runtime_error(variant.id+": pickup and placement height offsets differ; verify CAD");
  }
}

PlateLocalPose localTopReference(const PlateSlot & slot) {
  return {slot.row_offset_mm*kMmToM,0.0,slot.height_offset_mm*kMmToM,
          slot.initial_yaw_deg*kDegToRad};
}

std::vector<PlateLocalSample> toPlacementPlateLocal(
    const PlateSlot & slot, const VariantGeometry & variant,
    const std::vector<AnalysisSample> & fitting_local_path) {
  if (slot.variant_id!=variant.id) throw std::runtime_error("Variant/plate slot mismatch");
  if (std::abs(slot.height_offset_mm-variant.top_offset_mm)>1e-7)
    throw std::runtime_error("Plate height mismatches trajectory top reference");
  const auto origin=localTopReference(slot);
  const double c=std::cos(origin.yaw_rad),s=std::sin(origin.yaw_rad);
  std::vector<PlateLocalSample> output;
  output.reserve(fitting_local_path.size());
  for (const auto & p:fitting_local_path) {
    // p.fixture_relative_z_m ALREADY includes variant.top_offset_mm, so do
    // not add the plate height offset a second time.
    const double local_yaw=origin.yaw_rad+p.relative_yaw_rad;
    const auto q=yawQuaternion(local_yaw);
    output.push_back({p.time_s,p.phase,
       origin.x_m+c*p.local_x_m-s*p.local_y_m,
       origin.y_m+s*p.local_x_m+c*p.local_y_m,
       p.fixture_relative_z_m,local_yaw,q.x,q.y,q.z,q.w});
  }
  return output;
}

Pose3D transformToBase(const PlateLocalPose & pose,
                       const PlateRegistration & registration) {
  if (!registration.verified)
    throw std::runtime_error("Plate "+registration.plate_name+
                             " has NO VERIFIED robot-base registration");
  for (double n:{registration.x_m,registration.y_m,registration.z_m,
                 registration.qx,registration.qy,registration.qz,registration.qw,
                 pose.x_m,pose.y_m,pose.z_m,pose.yaw_rad}) {
    if (!std::isfinite(n)) throw std::runtime_error("Non-finite coordinate in frame transform");
  }
  const auto q=normalize({registration.qx,registration.qy,
                          registration.qz,registration.qw});
  const auto rotated=rotateVector(q,{pose.x_m,pose.y_m,pose.z_m});
  const auto orientation=normalize(multiply(q,yawQuaternion(pose.yaw_rad)));
  return {registration.x_m+rotated[0],registration.y_m+rotated[1],
          registration.z_m+rotated[2],orientation.x,orientation.y,
          orientation.z,orientation.w};
}

void savePlateLocalReferencesCSV(const std::string & filename,
                                 const PlateLayout & pickup,
                                 const PlateLayout & placement) {
  std::ofstream file(filename);
  if (!file) throw std::runtime_error("Cannot write: "+filename);
  file<<"plate,variant_id,plate_local_x_m,plate_local_y_m,plate_local_z_m,plate_local_yaw_rad,note\n";
  file<<std::setprecision(12);
  for (const auto & plate:{pickup,placement}) {
    for (const auto & slot:plate.slots) {
      const auto p=localTopReference(slot);
      file<<plate.name<<','<<slot.variant_id<<','<<p.x_m<<','<<p.y_m<<','<<p.z_m
          <<','<<p.yaw_rad<<",REFERENCE_ONLY_NOT_GRASP_POSE\n";
    }
  }
}

void savePlateLocalTrajectoryCSV(const std::string & filename,
                                 const std::vector<PlateLocalSample> & samples) {
  std::ofstream file(filename);
  if (!file) throw std::runtime_error("Cannot write: "+filename);
  // Deliberately unlike existing ur3_lfd MoveIt trajectory CSV format.
  file<<"time_s,phase,plate_local_x_m,plate_local_y_m,plate_local_z_m,"
         "plate_local_yaw_rad,qx,qy,qz,qw,analysis_only\n";
  file<<std::setprecision(12);
  for (const auto & p:samples) {
    file<<p.time_s<<','<<p.phase<<','<<p.x_m<<','<<p.y_m<<','<<p.z_m
        <<','<<p.yaw_rad<<','<<p.qx<<','<<p.qy<<','<<p.qz<<','<<p.qw
        <<",NOT_ROBOT_EXECUTABLE\n";
  }
}
}  // namespace ur3_variant_generalisation
