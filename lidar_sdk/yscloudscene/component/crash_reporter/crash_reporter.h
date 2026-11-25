#ifndef CRASH_REPORTER_H
#define CRASH_REPORTER_H

#include <map>
#include <string>
#include <memory>

// Forward declarations only - no direct includes
class CrashReporter {
public:
    CrashReporter();
    ~CrashReporter();

    // Disable copying and moving
    CrashReporter(const CrashReporter&) = delete;
    CrashReporter& operator=(const CrashReporter&) = delete;
    CrashReporter(CrashReporter&&) = delete;
    CrashReporter& operator=(CrashReporter&&) = delete;

    // Static interface for simpler use
    static bool Initialize(
        const std::string& server_url = "",
        const std::map<std::string, std::string>& annotations = {});

    static void AddAnnotation(const std::string& key, const std::string& value);

    // For testing only
    static void TestCrash();

private:
    // Private implementation details
    class Impl;
    static std::unique_ptr<Impl> s_impl;
};

#endif // CRASH_REPORTER_H