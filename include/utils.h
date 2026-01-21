#ifndef _LL_LOCALIZER_UTILS_H_
#define _LL_LOCALIZER_UTILS_H_

#include <glog/logging.h>
#include <chrono>
#include <fstream>
#include <map>
#include <numeric>
#include <string>

namespace ll_localizer {

/// timer
class Timer {
   public:
    struct TimerRecord {
        TimerRecord() = default;
        TimerRecord(const std::string& name, double time_usage) {
            func_name = name;
            time_usage_in_ms.emplace_back(time_usage);
        }
        std::string func_name;
        std::vector<double> time_usage_in_ms;
    };

    /**
     * call F and save its time usage
     * @tparam F
     * @param func
     * @param func_name
     */
    template <class F>
    static void Evaluate(F&& func, const std::string& func_name) {
        auto t1 = std::chrono::high_resolution_clock::now();
        std::forward<F>(func)();
        auto t2 = std::chrono::high_resolution_clock::now();
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;

        if (records.find(func_name) != records.end()) {
            records[func_name].time_usage_in_ms.emplace_back(time_used);
        } else {
            records.insert({func_name, TimerRecord(func_name, time_used)});
        }
    }

    /// print the run time
    static void PrintAll() {
        LOG(INFO) << ">>> ===== Printing run time =====";
        for (const auto& r : records) {
            LOG(INFO) << "> [ " << r.first << " ] average time usage: "
                      << std::accumulate(r.second.time_usage_in_ms.begin(), r.second.time_usage_in_ms.end(), 0.0) /
                             double(r.second.time_usage_in_ms.size())
                      << " ms , called times: " << r.second.time_usage_in_ms.size();
        }
        LOG(INFO) << ">>> ===== Printing run time end =====";
    }

    /// dump to a log file
    static void DumpIntoFile(const std::string& file_name) {
        std::ofstream ofs(file_name, std::ios::out);
        if (!ofs.is_open()) {
            LOG(ERROR) << "Failed to open file: " << file_name;
            return;
        } else {
            LOG(INFO) << "Dump Time Records into file: " << file_name;
        }

        size_t max_length = 0;
        for (const auto& iter : records) {
            ofs << iter.first << ", ";
            if (iter.second.time_usage_in_ms.size() > max_length) {
                max_length = iter.second.time_usage_in_ms.size();
            }
        }
        ofs << std::endl;

        for (size_t i = 0; i < max_length; ++i) {
            for (const auto& iter : records) {
                if (i < iter.second.time_usage_in_ms.size()) {
                    ofs << iter.second.time_usage_in_ms[i] << ",";
                } else {
                    ofs << ",";
                }
            }
            ofs << std::endl;
        }
        ofs.close();
    }

    /// get the average time usage of a function
    static double GetMeanTime(const std::string& func_name) {
        if (records.find(func_name) == records.end()) {
            return 0.0;
        }

        auto r = records[func_name];
        return std::accumulate(r.time_usage_in_ms.begin(), r.time_usage_in_ms.end(), 0.0) /
               double(r.time_usage_in_ms.size());
    }

    /// clean the records
    static void Clear() { records.clear(); }

   private:
    static std::map<std::string, TimerRecord> records;
};

}  // namespace ll_localizer

#endif  // _LL_LOCALIZER_UTILS_H_
