// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Capture what the SDK's singleton logger emits at warn and above, for tests
// whose subject is a warning (or its absence).

#pragma once

#include <taccap/log.hpp>

#include <spdlog/sinks/ringbuffer_sink.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace taccap_test {

// Attach a ring-buffer sink to the SDK's singleton logger for one test.
class CaptureLog {
public:
    CaptureLog() : sink_(std::make_shared<spdlog::sinks::ringbuffer_sink_mt>(256)) {
        sink_->set_level(spdlog::level::warn);
        xense::taccap::logger()->sinks().push_back(sink_);
    }
    ~CaptureLog() {
        auto& s = xense::taccap::logger()->sinks();
        s.erase(std::remove(s.begin(), s.end(), sink_), s.end());
    }
    CaptureLog(const CaptureLog&) = delete;
    CaptureLog& operator=(const CaptureLog&) = delete;

    std::vector<std::string> lines() { return sink_->last_formatted(); }
    size_t count(const std::string& needle) {
        auto v = lines();
        return static_cast<size_t>(std::count_if(v.begin(), v.end(), [&](const std::string& l) {
            return l.find(needle) != std::string::npos;
        }));
    }
private:
    std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt> sink_;
};

}  // namespace taccap_test
