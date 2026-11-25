#include "management_client.h"
#include <QJsonDocument>
#include <QUrl>
#include <curl/curl.h>
#include <mutex>

namespace ys_scene {
namespace {
struct Reply
{
    long status = 0;
    QByteArray body;
    QString error;
};

size_t Receive(char *data, size_t size, size_t count, void *context)
{
    auto *body = static_cast<QByteArray *>(context);
    const size_t length = size * count;
    if (length > 65536 || body->size() + length > 65536)
        return 0;
    body->append(data, static_cast<int>(length));
    return length;
}

Reply Perform(const QString &address, int mode, const std::shared_ptr<std::atomic<bool>> &cancel)
{
    static const auto initialized = curl_global_init(CURL_GLOBAL_DEFAULT);
    Reply r;
    if (initialized != CURLE_OK) {
        r.error = "libcurl initialization failed";
        return r;
    }
    CURL *curl = curl_easy_init();
    if (!curl) {
        r.error = "libcurl allocation failed";
        return r;
    }
    const QByteArray url = (address + (mode < 0 ? "/cgi-bin/lidar_status.cgi" : "/cgi-bin/lidar_switch.cgi")).toUtf8();
    const QByteArray body = QJsonDocument(QJsonObject{{"work_mode", mode}}).toJson(QJsonDocument::Compact);
    curl_slist *headers = curl_slist_append(nullptr, "Content-Type: application/json");
    headers = curl_slist_append(headers, "Cache-Control: no-cache");
    curl_easy_setopt(curl, CURLOPT_URL, url.constData());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_PROXY, "");
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, 3000L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 12000L);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, Receive);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &r.body);
    curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0L);
    curl_easy_setopt(
        curl, CURLOPT_XFERINFOFUNCTION, +[](void *p, curl_off_t, curl_off_t, curl_off_t, curl_off_t) -> int {
            return static_cast<std::atomic<bool> *>(p)->load() ? 1 : 0;
        });
    curl_easy_setopt(curl, CURLOPT_XFERINFODATA, cancel.get());
    if (mode >= 0) {
        curl_easy_setopt(curl, CURLOPT_POST, 1L);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.constData());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
    }
    const auto code = curl_easy_perform(curl);
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &r.status);
    if (code != CURLE_OK)
        r.error = QString::fromUtf8(curl_easy_strerror(code));
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    return r;
}
} // namespace

ManagementClient::ManagementClient(QObject *parent)
    : QObject(parent),
      worker_(new QObject)
{
    worker_->moveToThread(&thread_);
    connect(&thread_, &QThread::finished, worker_, &QObject::deleteLater);
    thread_.start();
    timer_.setSingleShot(true);
    connect(&timer_, &QTimer::timeout, this, &ManagementClient::Refresh);
}

ManagementClient::~ManagementClient()
{
    if (cancel_)
        cancel_->store(true);
    thread_.quit();
    thread_.wait();
}

void ManagementClient::SetAddress(const QString &address)
{
    QString normalized = address.trimmed();
    if (!normalized.contains("://"))
        normalized.prepend("http://");
    QUrl url(normalized);
    if ((url.scheme() != "http" && url.scheme() != "https") || url.host().isEmpty() || !url.userInfo().isEmpty() ||
        url.hasQuery() || url.hasFragment() || (!url.path().isEmpty() && url.path() != "/")) {
        if (cancel_)
            cancel_->store(true);
        ++generation_;
        timer_.stop();
        address_.clear();
        snapshot_ = {};
        busy_ = fresh_ = uncertain_ = false;
        requested_ = -1;
        emit AddressChanged();
        emit Changed();
        emit Message(QStringLiteral("请输入有效的设备 IP 或 HTTP 地址"));
        return;
    }
    url.setPath({});
    normalized = url.toString();
    if (normalized == address_)
        return;
    if (cancel_)
        cancel_->store(true);
    ++generation_;
    timer_.stop();
    address_ = normalized;
    emit AddressChanged();
    snapshot_ = {};
    busy_ = fresh_ = uncertain_ = false;
    requested_ = -1;
    operation_.clear();
    emit Message(QStringLiteral("正在读取管理状态"));
    emit Changed();
    Refresh();
}

void ManagementClient::SetActive(bool active)
{
    active_ = active;
    if (active)
        Refresh();
    else
        timer_.stop();
    emit Changed();
}

bool ManagementClient::CanSwitch() const
{
    return active_ && fresh_ && !busy_ && requested_ < 0 && !uncertain_ && snapshot_.phase == "COMMITTED";
}

void ManagementClient::Refresh()
{
    if (active_ && !busy_ && !address_.isEmpty())
        Request(-1);
}

void ManagementClient::Switch(int mode)
{
    if (!CanSwitch() || !snapshot_.modes.contains(mode))
        return;
    requested_ = mode;
    operation_.clear();
    observed_.start();
    emit Switching();
    Request(mode);
}

void ManagementClient::Request(int mode)
{
    timer_.stop();
    busy_ = true;
    const auto generation = generation_;
    const auto address = address_;
    cancel_ = std::make_shared<std::atomic<bool>>(false);
    const auto cancel = cancel_;
    emit Changed();
    QMetaObject::invokeMethod(
        worker_,
        [this, generation, address, mode, cancel] {
            const auto reply = Perform(address, mode, cancel);
            QMetaObject::invokeMethod(
                this,
                [this, generation, mode, reply] {
                    if (generation != generation_)
                        return;
                    busy_ = false;
                    const auto object = QJsonDocument::fromJson(reply.body).object();
                    ManagementSnapshot next;
                    const bool valid = reply.error.isEmpty() && object["code"].toInt(-1) == reply.status &&
                                       ParseManagementSnapshot(object, next);
                    fresh_ = valid && reply.status == 200;
                    if (mode >= 0 && valid && reply.status == 202) {
                        operation_ = next.operation_id;
                        emit Message(QStringLiteral("已受理，正在切换"));
                    }
                    else if (mode >= 0 && (!valid || reply.status >= 400)) {
                        uncertain_ = !valid || reply.status == 504 || reply.status == 502;
                        requested_ = -1;
                        emit Message(uncertain_ ? QStringLiteral("请求结果待确认，只查询，不重复提交")
                                                : QStringLiteral("切换被拒绝：%1").arg(object["message"].toString()));
                    }
                    if (valid) {
                        snapshot_ = next;
                        if (requested_ >= 0 && (mode < 0 || reply.status == 200)) {
                            if (!operation_.isEmpty() && operation_ != next.operation_id) {
                                requested_ = -1;
                                emit Message(QStringLiteral("操作标识已变化，原操作结果无法确认"));
                            }
                            else {
                                const auto result = ManagementOutcome(next, requested_);
                                if (!result.isEmpty()) {
                                    requested_ = -1;
                                    emit Message(result);
                                }
                            }
                        }
                        if (next.phase == "FAULT")
                            emit Message(QStringLiteral("设备故障：%1").arg(next.error));
                        if (fresh_ && next.phase == "COMMITTED") {
                            if (uncertain_) {
                                uncertain_ = false;
                                emit Message(QStringLiteral("已同步当前模式；先前请求结果无法可靠关联"));
                            }
                            emit Stable(next.actual);
                        }
                    }
                    else {
                        fresh_ = false;
                        QString error = reply.error.isEmpty()
                                            ? object["message"].toString(QStringLiteral("响应格式错误"))
                                            : reply.error;
                        if (reply.status == 404)
                            error = QStringLiteral("管理接口未部署或路径不可用");
                        emit Message(QStringLiteral("管理状态非实时：%1%2")
                                         .arg(error, uncertain_ ? QStringLiteral("；切换结果待确认") : QString()));
                    }
                    if (requested_ >= 0 && observed_.elapsed() > 60000) {
                        requested_ = -1;
                        uncertain_ = true;
                        emit Message(QStringLiteral("等待超时，请查看设备状态；未取消设备操作"));
                    }
                    emit Changed();
                    if (active_)
                        timer_.start(snapshot_.phase == "COMMITTED" && requested_ < 0 ? 2000 : 500);
                },
                Qt::QueuedConnection);
        },
        Qt::QueuedConnection);
}
} // namespace ys_scene
