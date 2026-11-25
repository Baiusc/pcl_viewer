#include "sn_discovery.h"

#ifdef Q_OS_WIN

#include <memory>
#include <vector>

#include <QElapsedTimer>
#include <QHash>
#include <QHostAddress>
#include <QMutex>
#include <QNetworkInterface>
#include <QSet>
#include <QThread>
#include <QtEndian>

#include <winsock2.h>

#include <windows.h>

#include <windns.h>

namespace ys_scene::lidar_sn {
namespace {

constexpr wchar_t kServiceType[] = L"_lidar._tcp.local";
constexpr auto kStatusPath = "/cgi-bin/lidar_network.cgi";

struct ResolverResults
{
    QMutex mutex;
    QSet<QString> names;
    DiscoveryCheck result;
    QHostAddress address;
    QString sn;
    int port = 0;
    int remaining = 0;
};

QMutex callback_mutex;
QHash<quintptr, std::shared_ptr<ResolverResults>> callbacks;
quintptr next_context = 0;

// The system may finish a canceled request after the caller's wait ends.
// Callbacks use a token so late results never dereference a destroyed request.
class CallbackContext
{
  public:
    explicit CallbackContext(const std::shared_ptr<ResolverResults> &results)
    {
        QMutexLocker lock(&callback_mutex);
        id_ = ++next_context;
        callbacks.insert(id_, results);
    }

    ~CallbackContext()
    {
        QMutexLocker lock(&callback_mutex);
        callbacks.remove(id_);
    }

    PVOID Token() const
    {
        return reinterpret_cast<PVOID>(id_);
    }
  private:
    quintptr id_;
};

std::shared_ptr<ResolverResults> Results(PVOID token)
{
    QMutexLocker lock(&callback_mutex);
    return callbacks.value(reinterpret_cast<quintptr>(token));
}

void WINAPI Browsed(DWORD status, PVOID token, PDNS_RECORD records)
{
    const auto results = Results(token);
    if (results && status == ERROR_SUCCESS) {
        QMutexLocker lock(&results->mutex);
        for (auto *record = records; record && results->names.size() < 128; record = record->pNext) {
            if (record->wType == DNS_TYPE_PTR && record->dwTtl > 0 && record->Data.PTR.pNameHost) {
                results->names.insert(QString::fromWCharArray(record->Data.PTR.pNameHost));
            }
        }
    }
    if (records) {
        DnsRecordListFree(records, DnsFreeRecordList);
    }
}

void WINAPI Resolved(DWORD status, PVOID token, PDNS_SERVICE_INSTANCE instance)
{
    const auto results = Results(token);
    if (results) {
        QMutexLocker lock(&results->mutex);
        --results->remaining;
        if (status == ERROR_SUCCESS && instance && instance->pszHostName && instance->ip4Address && instance->keys &&
            instance->values) {
            QString sn;
            QString path;
            for (DWORD i = 0; i < instance->dwPropertyCount; ++i) {
                if (!instance->keys[i] || !instance->values[i]) {
                    continue;
                }
                const auto key = QString::fromWCharArray(instance->keys[i]);
                if (key.compare(QStringLiteral("sn"), Qt::CaseInsensitive) == 0) {
                    sn = QString::fromWCharArray(instance->values[i]);
                }
                else if (key.compare(QStringLiteral("path"), Qt::CaseInsensitive) == 0) {
                    path = QString::fromWCharArray(instance->values[i]);
                }
            }
            if (sn == results->sn) {
                const QHostAddress address(qFromBigEndian(*instance->ip4Address));
                if (address != results->address) {
                    results->result.conflicting_address = true;
                }
                else if (instance->wPort == results->port && (path.isEmpty() || path == kStatusPath)) {
                    results->result.host = QString::fromWCharArray(instance->pszHostName);
                }
            }
        }
    }
    if (instance) {
        DnsServiceFreeInstance(instance);
    }
}

} // namespace

DiscoveryCheck VerifyDiscovery(const lidar_network::Endpoint &endpoint, const QString &sn)
{
    const auto nic = QNetworkInterface::interfaceFromName(endpoint.interface_id);
    if (!nic.isValid()) {
        return {};
    }
    const auto results = std::make_shared<ResolverResults>();
    results->address = QHostAddress(endpoint.address);
    results->port = endpoint.port;
    results->sn = sn;
    const CallbackContext context(results);
    DNS_SERVICE_CANCEL browse_cancel{};
    DNS_SERVICE_BROWSE_REQUEST browse{};
    browse.Version = DNS_QUERY_REQUEST_VERSION1;
    browse.InterfaceIndex = nic.index();
    browse.QueryName = kServiceType;
    browse.pBrowseCallback = Browsed;
    browse.pQueryContext = context.Token();
    if (DnsServiceBrowse(&browse, &browse_cancel) != DNS_REQUEST_PENDING) {
        return {};
    }
    QThread::msleep(2000);
    DnsServiceBrowseCancel(&browse_cancel);

    struct ResolveRequest
    {
        std::wstring name;
        DNS_SERVICE_RESOLVE_REQUEST request{};
        DNS_SERVICE_CANCEL cancel{};
        bool pending = false;
    };

    const auto names = [&] {
        QMutexLocker lock(&results->mutex);
        results->remaining = results->names.size();
        return results->names;
    }();
    std::vector<ResolveRequest> queries(names.size());
    size_t i = 0;
    for (const auto &name : names) {
        auto &query = queries[i++];
        query.name = name.toStdWString();
        query.request = {DNS_QUERY_REQUEST_VERSION1, ULONG(nic.index()), query.name.data(), Resolved, context.Token()};
        query.pending = DnsServiceResolve(&query.request, &query.cancel) == DNS_REQUEST_PENDING;
        if (!query.pending) {
            QMutexLocker lock(&results->mutex);
            --results->remaining;
        }
    }

    QElapsedTimer timeout;
    timeout.start();
    while (timeout.elapsed() < 4000) {
        {
            QMutexLocker lock(&results->mutex);
            if (results->remaining == 0) {
                return results->result;
            }
        }
        QThread::msleep(50);
    }
    for (auto &query : queries) {
        if (query.pending) {
            DnsServiceResolveCancel(&query.cancel);
        }
    }
    QMutexLocker lock(&results->mutex);
    return results->result;
}

} // namespace ys_scene::lidar_sn

#else

ys_scene::lidar_sn::DiscoveryCheck ys_scene::lidar_sn::VerifyDiscovery(const lidar_network::Endpoint &, const QString &)
{
    return {};
}

#endif
