#include "network_access.h"

#include <memory>

#include <QJsonDocument>
#include <QNetworkDatagram>
#include <curl/curl.h>

namespace ys_scene::lidar_network {

QString SourceAddress(const Endpoint &endpoint)
{
    const QHostAddress target(endpoint.address);
    const auto nic = QNetworkInterface::interfaceFromName(endpoint.interface_id);
    QString source;
    int longest = -1;
    for (const auto &entry : nic.addressEntries()) {
        if (entry.ip().protocol() == QAbstractSocket::IPv4Protocol && entry.prefixLength() > longest &&
            entry.prefixLength() > 0 && target.isInSubnet(entry.ip(), entry.prefixLength())) {
            source = entry.ip().toString();
            longest = entry.prefixLength();
        }
    }
    return source;
}

Announcement ParseAnnouncement(const QNetworkDatagram &packet, const QNetworkInterface &nic)
{
    if (!nic.isValid() || packet.interfaceIndex() != uint(nic.index()) || packet.data().size() > 4096) {
        return {};
    }
    const auto json = QJsonDocument::fromJson(packet.data()).object();
    const auto mac = NormalizeMac(json.value("mac").toString());
    const int port = json.value("port").toInt();
    quint32 address;
    if (json.value("protocol") != QJsonValue("lidar-network") || json.value("schema_version") != QJsonValue(1) ||
        json.value("service") != QJsonValue("http") || json.value("path") != QJsonValue("/cgi-bin/lidar_network.cgi") ||
        mac.isEmpty() || port <= 0 || port > 65535 || !json.value("identity").isString() ||
        !json.value("ready").isBool() || !json.value("recovery").isBool() ||
        !ParseIpv4(json.value("address").toString(), address) || QHostAddress(address) != packet.senderAddress()) {
        return {};
    }
    return {{nic.name(), QHostAddress(address).toString(), port},
            mac,
            json.value("identity").toString(),
            json.value("sn").toString()};
}

Reply Query(Endpoint endpoint, QString source, QJsonObject request, bool diagnostic)
{
    Reply reply;
    static const auto initialized = curl_global_init(CURL_GLOBAL_DEFAULT);
    std::unique_ptr<CURL, decltype(&curl_easy_cleanup)> curl(initialized == CURLE_OK ? curl_easy_init() : nullptr,
                                                             curl_easy_cleanup);
    if (!curl) {
        reply.error = QStringLiteral("无法初始化 HTTP 客户端");
        return reply;
    }
    const auto url = QString("http://%1:%2/cgi-bin/%3")
                         .arg(endpoint.address)
                         .arg(endpoint.port)
                         .arg(diagnostic ? "lidar_network_status.cgi" : "lidar_network.cgi")
                         .toLatin1();
    const auto bind = QString("host!%1").arg(source).toLatin1();
    const auto payload = QJsonDocument(request).toJson(QJsonDocument::Compact);
    QByteArray body;
    auto *headers = curl_slist_append(nullptr, "Cache-Control: no-store");
    if (!request.isEmpty()) {
        headers = curl_slist_append(headers, "Content-Type: application/json");
        curl_easy_setopt(curl.get(), CURLOPT_POST, 1L);
        curl_easy_setopt(curl.get(), CURLOPT_POSTFIELDS, payload.constData());
        curl_easy_setopt(curl.get(), CURLOPT_POSTFIELDSIZE, long(payload.size()));
    }
    curl_easy_setopt(curl.get(), CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl.get(), CURLOPT_URL, url.constData());
    curl_easy_setopt(curl.get(), CURLOPT_INTERFACE, bind.constData());
    curl_easy_setopt(curl.get(), CURLOPT_PROXY, "");
    curl_easy_setopt(curl.get(), CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(curl.get(), CURLOPT_FOLLOWLOCATION, 0L);
    curl_easy_setopt(curl.get(), CURLOPT_FRESH_CONNECT, 1L);
    curl_easy_setopt(curl.get(), CURLOPT_TIMEOUT_MS, request.isEmpty() ? 2000L : 5000L);
    curl_easy_setopt(curl.get(), CURLOPT_WRITEDATA, &body);
    curl_easy_setopt(
        curl.get(), CURLOPT_WRITEFUNCTION, +[](char *data, size_t size, size_t count, void *context) -> size_t {
            auto &bytes = *static_cast<QByteArray *>(context);
            if (size * count > 65536 || bytes.size() + size * count > 65536) {
                return 0;
            }
            bytes.append(data, int(size * count));
            return size * count;
        });
    const auto code = curl_easy_perform(curl.get());
    curl_easy_getinfo(curl.get(), CURLINFO_RESPONSE_CODE, &reply.status);
    curl_slist_free_all(headers);
    reply.snapshot = QJsonDocument::fromJson(body).object();
    if (code != CURLE_OK) {
        reply.error = QString::fromLatin1(curl_easy_strerror(code));
    }
    return reply;
}

} // namespace ys_scene::lidar_network
