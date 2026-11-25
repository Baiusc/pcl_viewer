#include "crash_reporter.h"
#include "tiny_hut/common_toolset.h"

// Include all Crashpad headers only in this cpp file
#include "crashpad/client/crashpad_client.h"
#include "crashpad/client/crash_report_database.h"
#include "crashpad/client/settings.h"
#include "crashpad/client/crashpad_info.h"

// Define the implementation class
class CrashReporter::Impl {
public:
    Impl() = default;
    ~Impl() = default;

    bool Initialize(const std::string& server_url,
        const std::map<std::string, std::string>& annotations);

    bool Initialize(const std::wstring& handler_path,
        const std::wstring& database_path,
        const std::string& server_url,
        const std::map<std::string, std::string>& annotations);

    void AddAnnotation(const std::string& key, const std::string& value);
    void TestCrash();

private:
    crashpad::CrashpadClient client;
};

// Initialize the static instance
std::unique_ptr<CrashReporter::Impl> CrashReporter::s_impl;

// Constructor and destructor
CrashReporter::CrashReporter() = default;
CrashReporter::~CrashReporter() = default;

// Static interface implementation
bool CrashReporter::Initialize(const std::string& server_url,
    const std::map<std::string, std::string>& annotations) {
    if (!s_impl) {
        s_impl = std::make_unique<Impl>();
    }
    return s_impl->Initialize(server_url, annotations);
}

void CrashReporter::AddAnnotation(const std::string& key, const std::string& value) {
    if (s_impl) {
        s_impl->AddAnnotation(key, value);
    }
}

void CrashReporter::TestCrash() {
    if (s_impl) {
        s_impl->TestCrash();
    }
}

// Implementation methods
bool CrashReporter::Impl::Initialize(const std::string& server_url,
    const std::map<std::string, std::string>& annotations) {
    // 获取执行目录
    std::string exe_dir = you_see::toolset::GetExecuteDirectory();

    // 构造handler路径（假设crashpad_handler.exe与程序在同一目录）
    std::wstring handler_path(exe_dir.begin(), exe_dir.end());
    handler_path += L"\\crashpad_handler.exe";

    // 创建crash_reports子目录用于存储崩溃报告
    std::string reports_dir = exe_dir + "\\crash_reports";
    you_see::toolset::CreateDirectories(reports_dir);

    std::wstring database_path(reports_dir.begin(), reports_dir.end());

    // 调用原始的Initialize方法
    return Initialize(handler_path, database_path, server_url, annotations);
}

bool CrashReporter::Impl::Initialize(const std::wstring& handler_path,
    const std::wstring& database_path,
    const std::string& server_url,
    const std::map<std::string, std::string>& annotations) {
    // 转换为FilePath对象
    base::FilePath handler_file_path(handler_path);
    base::FilePath database_file_path(database_path);

    // 确保数据库目录存在
    std::string path_str;
    path_str = std::string(database_path.begin(), database_path.end());;
    if (!you_see::toolset::DirectoryExist(path_str)) {
        return false;
    }

    // 初始化数据库
    std::unique_ptr<crashpad::CrashReportDatabase> database =
        crashpad::CrashReportDatabase::Initialize(database_file_path);
    if (!database) {
        return false;
    }

    // 配置设置
    crashpad::Settings* settings = database->GetSettings();
    if (settings) {
        // 启用崩溃报告上传
        settings->SetUploadsEnabled(true);
    }

    // 设置注释
    crashpad::CrashpadInfo* crashpad_info = crashpad::CrashpadInfo::GetCrashpadInfo();
    static crashpad::SimpleStringDictionary simple_annotations;
    crashpad_info->set_simple_annotations(&simple_annotations);

    for (const auto& pair : annotations) {
        simple_annotations.SetKeyValue(pair.first.c_str(), pair.second.c_str());
    }

    // 设置额外的命令行参数
    std::vector<std::string> arguments;
    arguments.push_back("--no-rate-limit");  // 禁用速率限制

    // 附件为空
    std::vector<base::FilePath> attachments;

    // 启动handler
    bool success = client.StartHandler(
        handler_file_path,
        database_file_path,
        database_file_path,
        server_url,
        annotations,
        arguments,
        true,  // 立即启动
        false  // 不以附加模式启动
    );

    return success;
}

void CrashReporter::Impl::AddAnnotation(const std::string& key, const std::string& value) {
    // 获取当前进程的CrashpadInfo
    crashpad::CrashpadInfo* crashpad_info = crashpad::CrashpadInfo::GetCrashpadInfo();

    // 获取或创建SimpleStringDictionary
    crashpad::SimpleStringDictionary* annotations = crashpad_info->simple_annotations();
    if (!annotations) {
        // 如果不存在则创建
        static crashpad::SimpleStringDictionary simple_annotations;
        crashpad_info->set_simple_annotations(&simple_annotations);
        annotations = crashpad_info->simple_annotations();
    }

    // 设置键值对
    annotations->SetKeyValue(key.c_str(), value.c_str());
}

void CrashReporter::Impl::TestCrash() {
    volatile int* ptr = nullptr;
    *ptr = 42;  // 这将导致访问冲突
}