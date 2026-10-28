#include "ur3_variant_generalisation/variant_geometry.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace ur3_variant_generalisation {
namespace {
std::string trim(std::string s) {
  const auto first = s.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return "";
  const auto last = s.find_last_not_of(" \t\r\n");
  return s.substr(first, last - first + 1);
}
std::vector<std::string> split(const std::string & s) {
  std::stringstream stream(s);
  std::vector<std::string> fields;
  std::string cell;
  while (std::getline(stream, cell, ',')) fields.push_back(trim(cell));
  if (!s.empty() && s.back() == ',') fields.push_back("");
  return fields;
}
double parseRequired(const std::string & field, const std::string & name) {
  if (field.empty()) throw std::runtime_error("Missing " + name);
  std::size_t pos = 0;
  const double x = std::stod(field, &pos);
  if (pos != field.size() || !std::isfinite(x)) throw std::runtime_error("Invalid " + name);
  return x;
}
double parseOptional(const std::string & field, const std::string & name) {
  return field.empty() ? std::numeric_limits<double>::quiet_NaN() : parseRequired(field, name);
}
} // namespace
void validateVariantGeometry(const VariantGeometry & v) {
  if (v.id.empty() || !(v.insertion_depth_mm > 0.0) ||
      !(std::abs(v.rotation_deg) > 0.01 && std::abs(v.rotation_deg) <= 360.0) ||
      !(v.rotation_drop_mm >= 0.0) || !std::isfinite(v.top_offset_mm)) {
    throw std::runtime_error("Invalid variant geometry for " + v.id);
  }
  const bool x_known = std::isfinite(v.fixture_center_x_mm);
  const bool y_known = std::isfinite(v.fixture_center_y_mm);
  if (x_known != y_known) throw std::runtime_error("Only one fixture centre coordinate supplied for " + v.id);
}
std::vector<VariantGeometry> readVariantCSV(const std::string & file) {
  std::ifstream in(file);
  if (!in) throw std::runtime_error("Cannot open CAD parameters CSV: " + file);
  std::string line;
  if (!std::getline(in, line)) throw std::runtime_error("Empty parameters CSV");
  if (split(line) != std::vector<std::string>({
    "variant_id", "insertion_depth_mm", "rotation_deg", "rotation_drop_mm",
    "top_offset_mm", "fixture_center_x_mm", "fixture_center_y_mm", "fixture_yaw_deg"})) {
    throw std::runtime_error("CSV header does not match expected eight CAD fields");
  }
  std::vector<VariantGeometry> variants;
  while (std::getline(in, line)) {
    if (trim(line).empty()) continue;
    const auto f = split(line);
    if (f.size() != 8) throw std::runtime_error("Expected 8 CSV columns: " + line);
    VariantGeometry v;
    v.id = f[0];
    v.insertion_depth_mm = parseRequired(f[1], "insertion_depth_mm");
    v.rotation_deg = parseRequired(f[2], "rotation_deg");
    v.rotation_drop_mm = parseRequired(f[3], "rotation_drop_mm");
    v.top_offset_mm = parseRequired(f[4], "top_offset_mm");
    v.fixture_center_x_mm = parseOptional(f[5], "fixture_center_x_mm");
    v.fixture_center_y_mm = parseOptional(f[6], "fixture_center_y_mm");
    v.fixture_yaw_deg = parseOptional(f[7], "fixture_yaw_deg");
    validateVariantGeometry(v);
    if (std::any_of(variants.begin(), variants.end(), [&](const auto & old) { return old.id == v.id; })) {
      throw std::runtime_error("Duplicate variant: " + v.id);
    }
    variants.push_back(v);
  }
  if (variants.size() != 5) throw std::runtime_error("Expected five variants V1–V5");
  for (int i=1; i<=5; ++i) {
    if (std::none_of(variants.begin(), variants.end(), [&](const auto & v) {
        return v.id == "V" + std::to_string(i); })) {
      throw std::runtime_error("Missing variant V" + std::to_string(i));
    }
  }
  return variants;
}
} // namespace ur3_variant_generalisation
