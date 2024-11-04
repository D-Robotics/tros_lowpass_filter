// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>  
#include <fstream>  
#include <string>  
#include <vector>  
#include <unistd.h> // For sleep  

namespace tros {
  
class Tools {
public:
  Tools() {}
  virtual ~Tools() {}

private:
  // 读取并解析/proc/stat文件中的CPU时间  
  std::vector<long long> read_cpu_stat(int cpu_num) {  
      std::ifstream file("/proc/stat");  
      std::string line;  
    
      // 跳过非CPU行  
      while (getline(file, line) && line.substr(0, 3) != "cpu");  
    
      std::vector<long long> cpu_times(cpu_num); // 假设最多有8个字段（如cpu, cpu0, ..., cpuN-1）  
      std::istringstream iss(line.substr(5)); // 跳过"cpu"和空格  
    
      int i = 0;  
      while (iss >> cpu_times[i] && i < cpu_times.size()) {  
          ++i;  
      }  
    
      return cpu_times;  
  }  
    
  // 计算CPU使用率  
  double calculate_cpu_usage(const std::vector<long long>& prev_cpu_times,  
                            const std::vector<long long>& curr_cpu_times) {  
      long long prev_idle = prev_cpu_times[3] + prev_cpu_times[4]; // idle + iowait  
      long long curr_idle = curr_cpu_times[3] + curr_cpu_times[4];  
      long long prev_non_idle = 0;  
      long long curr_non_idle = 0;  
    
      // 计算非空闲时间  
      for (size_t i = 0; i < prev_cpu_times.size(); ++i) {  
          if (i != 3 && i != 4) { // 跳过idle和iowait  
              prev_non_idle += prev_cpu_times[i];  
              curr_non_idle += curr_cpu_times[i];  
          }  
      }  
    
      long long total_prev = prev_idle + prev_non_idle;  
      long long total_curr = curr_idle + curr_non_idle;  
    
      // 防止除以0  
      if (total_prev == total_curr) {  
          return 0.0;  
      }  
    
      // long long idle_diff = curr_idle - prev_idle;  
      long long non_idle_diff = curr_non_idle - prev_non_idle;  
    
      double usage = (double)(non_idle_diff) / (total_curr - total_prev) * 100.0;  
      return usage;  
  }

public:
  float GetCPUUsage(int cpu_num) {
    // CPU占用
    std::vector<long long> prev_cpu_times = read_cpu_stat(cpu_num);  
    sleep(1); // 等待1秒以获取可测量的变化  
    std::vector<long long> curr_cpu_times = read_cpu_stat(cpu_num);  
    return calculate_cpu_usage(prev_cpu_times, curr_cpu_times);
  }

  std::string GetCPUTemperature() {
    // 温度
    std::string temp_file = "/sys/class/thermal/thermal_zone0/temp";
    std::ifstream ifs;
    std::stringstream ss;
    std::string str;
    ifs.open(temp_file.c_str());
    ss << ifs.rdbuf();
    ss >> str;
    if (str.size() > 3) {
      // 转成以摄氏度为单位，保留3位有效数字
      str = str.substr(0, 3);
      str.insert(2, ".");
    }
    return str;
  }
  
  // 默认保留小数点后两位
  std::string FloatToString(float value, int precision = 2) {
    if (precision <= 0) precision = 2;
    std::ostringstream oss;  
    oss << std::fixed << std::setprecision(precision) << value;  
    return oss.str();  
  }

};

}