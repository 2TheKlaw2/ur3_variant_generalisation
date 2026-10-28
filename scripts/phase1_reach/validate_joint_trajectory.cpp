// Offline-only C++17 numeric trajectory checker. Never creates ROS nodes or moves hardware.
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr std::size_t N = 6;
constexpr std::array<const char *,N> JOINTS = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
constexpr std::array<double,N> HOME = {
    -0.675926510488, -1.34777768076, -0.527696251869,
    -2.8273965321, 1.55948805809, -3.87329799334};
struct Point { double t{}; std::array<double,N> q{}; };

std::vector<std::string> split(const std::string & line) {
  std::vector<std::string> out;
  std::stringstream stream(line);
  std::string s;
  while(std::getline(stream,s,',')) {
    if (!s.empty() && s.back()=='\r') s.pop_back();
    out.push_back(s);
  }
  return out;
}

double number(const std::string & s) {
  std::size_t pos=0;
  const double n=std::stod(s,&pos);
  if(pos!=s.size()||!std::isfinite(n)) throw std::runtime_error("Invalid numeric value: " + s);
  return n;
}

std::vector<Point> load(const std::string & path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("Unable to read: " + path);
  std::string line;
  if (!std::getline(f,line)) throw std::runtime_error("Empty file");
  const auto header=split(line);
  if(header.size()!=7 || header[0]!="time") throw std::runtime_error("Unexpected CSV header");
  for(std::size_t i=0;i<N;i++)
    if(header[i+1]!=JOINTS[i]) throw std::runtime_error("Unexpected joint name or order");
  std::vector<Point> out;
  while(std::getline(f,line)) {
    if(line.empty()) continue;
    auto col=split(line);
    if(col.size()!=7) throw std::runtime_error("Malformed CSV row");
    Point p;
    p.t=number(col[0]);
    for(std::size_t i=0;i<N;i++) p.q[i]=number(col[i+1]);
    out.push_back(p);
  }
  if(out.size()<3) throw std::runtime_error("Fewer than three trajectory points");
  return out;
}

void check(const std::vector<Point> & path, double slowdown) {
  if (!std::isfinite(slowdown) || slowdown<1.0 || slowdown>5.0)
    throw std::runtime_error("Slowdown factor must be in [1,5]");
  if(path.front().t < -1e-9) throw std::runtime_error("Negative start time");
  double max_start=0, max_step=0;
  for(std::size_t j=0;j<N;j++) max_start=std::max(max_start,std::abs(path.front().q[j]-HOME[j]));
  if(max_start>0.015) throw std::runtime_error("Trajectory start does not match saved home");
  std::array<double,N> vmax{}, amax{};
  std::array<std::vector<double>,N> velocities;
  std::vector<double> dt;
  for(std::size_t i=1;i<path.size();i++) {
    const double duration=(path[i].t-path[i-1].t)*slowdown;
    if(!(duration>1e-4)) throw std::runtime_error("Non-increasing or too-small timestep");
    dt.push_back(duration);
    for(std::size_t j=0;j<N;j++) {
      const double dq=path[i].q[j]-path[i-1].q[j];
      max_step=std::max(max_step,std::abs(dq));
      const double v=dq/duration;
      velocities[j].push_back(v);
      vmax[j]=std::max(vmax[j], std::abs(v));
    }
  }
  if(max_step>0.05) throw std::runtime_error("Consecutive joint position jump exceeds 0.05 rad");
  for(std::size_t j=0;j<N;j++) {
    for(std::size_t i=1;i<velocities[j].size();i++) {
      const double a=(velocities[j][i]-velocities[j][i-1]) / (0.5*(dt[i]+dt[i-1]));
      amax[j]=std::max(amax[j],std::abs(a));
    }
    // Conservative project-specific review thresholds, NOT controller limits.
    if(vmax[j]>0.22 || amax[j]>0.35)
      throw std::runtime_error(std::string(JOINTS[j])+" exceeds review velocity/acceleration limits");
  }
  std::cout<<std::fixed<<std::setprecision(6);
  std::cout<<"PHASE 1 OFFLINE C++ VALIDATION: PASS (NOT execution approval)\n";
  std::cout<<"Waypoints: "<<path.size()<<", slowed duration: "
           <<(path.back().t-path.front().t)*slowdown<<" s\n";
  std::cout<<"Start/home max error: "<<max_start<<" rad\n";
  std::cout<<"Max consecutive joint step: "<<max_step<<" rad\n";
  for(std::size_t j=0;j<N;j++)
    std::cout<<JOINTS[j]<<"  v="<<vmax[j]<<" rad/s  a="<<amax[j]<<" rad/s^2\n";
  std::cout<<"Wrist 3 start: "<<path.front().q[5]<<", end: "<<path.back().q[5]<<" rad\n";
  std::cout<<"NO HARDWARE OR SIMULATOR COMMAND WAS SENT.\n";
}
} // namespace

int main(int argc, char**argv) {
  if(argc!=2 && argc!=3) {
    std::cerr<<"Usage: validate_joint_trajectory TRAJECTORY.csv [SLOWDOWN=2.0]\n";
    return 2;
  }
  try {
    const double slowdown=argc==3?number(argv[2]):2.0;
    check(load(argv[1]),slowdown);
    return 0;
  } catch(const std::exception& e) {
    std::cerr<<"PHASE 1 OFFLINE VALIDATION FAILED: "<<e.what()<<'\n';
    return 1;
  }
}
