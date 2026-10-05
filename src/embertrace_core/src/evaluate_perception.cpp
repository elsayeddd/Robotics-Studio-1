/** Evaluate the exact live filter on labelled CSV (recorded or synthetic). */
#include "embertrace_core/perception_filter.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
namespace
{
std::vector<std::string> split(const std::string & line)
{
  std::vector<std::string> result;
  std::istringstream stream(line); std::string field;
  while (std::getline(stream, field, ',')) {result.push_back(field);}
  return result;
}
double number(const std::string & s)
{
  std::size_t consumed{}; const double value = std::stod(s, &consumed);
  if (consumed != s.size()) {throw std::runtime_error("Invalid numeric CSV field: " + s);}
  return value;
}
bool boolean(const std::string & s)
{
  if (s != "0" && s != "1") {throw std::runtime_error("Boolean must be 0 or 1");}
  return s == "1";
}
struct Metrics
{
  std::size_t tp{}, fp{}, tn{}, fn{}, invalid{}, events{}, detected{};
  std::vector<double> latencies;
};
}
int main(int argc, char ** argv)
{
  try {
    if (argc < 2) {
      std::cerr << "Usage: evaluate_perception trials.csv [--alpha N] [--smoke-enter N] [--heat-enter N]\n";
      return 2;
    }
    embertrace_core::PerceptionConfig config;
    for (int i = 2; i < argc; i += 2) {
      if (i + 1 >= argc) {throw std::runtime_error("Missing threshold option value");}
      const std::string option = argv[i]; const double value = number(argv[i + 1]);
      if (option == "--alpha") {config.alpha = value;}
      else if (option == "--smoke-enter") {config.smoke_enter = value;}
      else if (option == "--heat-enter") {config.heat_enter = value;}
      else {throw std::runtime_error("Unknown option: " + option);}
    }
    embertrace_core::PerceptionFilter filter(config);
    std::ifstream input(argv[1]);
    if (!input) {throw std::runtime_error("Cannot open input CSV");}
    std::string line;
    std::getline(input, line);
    if (!line.empty() && line.back() == '\r') {line.pop_back();}
    if (line != "trial,scenario,now_s,stamp_s,smoke_ppm,temperature_c,wind_speed_mps,wind_direction_rad,visual_stamp_s,visual_confidence,visual_valid,environment_available,fire_present")
    {throw std::runtime_error("CSV header does not match documented schema");}
    std::map<std::string, Metrics> metrics;
    std::string previous_trial, previous_scenario;
    bool previous_label = false, detected = false;
    double onset = 0, previous_clock = -1;
    int confirmations = 0;
    std::size_t row = 1;
    while (std::getline(input, line)) {
      ++row;
      if (line.empty()) {continue;}
      if (line.back() == '\r') {line.pop_back();}
      const auto f = split(line);
      if (f.size() != 13) {throw std::runtime_error("Wrong CSV field count at row " + std::to_string(row));}
      if (f[0].empty() || f[1].empty()) {throw std::runtime_error("Missing trial/scenario");}
      const double clock = number(f[2]);
      if (f[0] != previous_trial || f[1] != previous_scenario || clock < previous_clock) {
        filter = embertrace_core::PerceptionFilter(config);
        previous_label = false; detected = false; confirmations = 0;
      }
      previous_trial = f[0]; previous_scenario = f[1]; previous_clock = clock;
      auto & m = metrics[f[1]];
      const bool label = boolean(f[12]);
      if (label && !previous_label) {onset = clock; detected = false; ++m.events;}
      filter.set_visual(number(f[8]), number(f[9]), boolean(f[10]), clock);
      std::optional<embertrace_core::PerceptionResult> result;
      if (boolean(f[11])) {
        result = filter.update({number(f[3]), number(f[4]), number(f[5]), number(f[6]), number(f[7])}, clock);
      }
      if (!result || !result->valid) {++m.invalid; confirmations = 0;}
      else {
        confirmations = result->confirmed_candidate ? confirmations + 1 : 0;
        const bool positive = confirmations >= 3;
        if (label) {if (positive) {++m.tp;} else {++m.fn;}}
        else {if (positive) {++m.fp;} else {++m.tn;}}
        if (label && positive && !detected) {
          m.latencies.push_back(clock - onset); ++m.detected; detected = true;
        }
      }
      previous_label = label;
    }
    if (metrics.empty()) {throw std::runtime_error("No trial rows");}
    std::cout << "scenario,tp,fp,tn,fn,invalid,detection_rate,false_positive_rate,availability,events,detected_events,median_latency_s,p95_latency_s\n";
    std::cout << std::fixed << std::setprecision(4);
    for (auto & entry : metrics) {
      auto & m = entry.second;
      std::sort(m.latencies.begin(), m.latencies.end());
      const auto rate = [](std::size_t a, std::size_t b) {
        return b ? std::to_string(static_cast<double>(a) / b) : std::string("NA");
      };
      const std::size_t valid = m.tp + m.fp + m.tn + m.fn;
      std::cout << entry.first << ',' << m.tp << ',' << m.fp << ',' << m.tn << ',' << m.fn << ',' << m.invalid << ','
        << rate(m.tp, m.tp + m.fn) << ',' << rate(m.fp, m.fp + m.tn) << ','
        << rate(valid, valid + m.invalid) << ',' << m.events << ',' << m.detected << ',';
      if (m.latencies.empty()) {std::cout << "NA,NA\n";}
      else {
        const auto n = m.latencies.size();
        const double median = (m.latencies[(n - 1) / 2] + m.latencies[n / 2]) / 2;
        const auto p95 = static_cast<std::size_t>(std::ceil(0.95 * n)) - 1;
        std::cout << median << ',' << m.latencies[p95] << '\n';
      }
    }
    return 0;
  } catch (const std::exception & error) {std::cerr << error.what() << '\n'; return 1;}
}
