#include "cloud_viewer/io/Loaders.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <unordered_map>

#include "pulse_data_processor.h"

namespace cloud_viewer
{
namespace
{

constexpr std::size_t kCsvHeaderScanLines = 200;
constexpr std::size_t kNotFound = std::numeric_limits<std::size_t>::max();

std::string trim(const std::string &text)
{
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos)
    {
        return {};
    }
    const auto end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

std::string toLower(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return text;
}

bool contains(const std::string &haystack, const std::string &needle)
{
    return haystack.find(needle) != std::string::npos;
}

std::string stripBom(std::string text)
{
    if (text.size() >= 3 &&
        static_cast<unsigned char>(text[0]) == 0xEF &&
        static_cast<unsigned char>(text[1]) == 0xBB &&
        static_cast<unsigned char>(text[2]) == 0xBF)
    {
        text.erase(0, 3);
    }
    return text;
}

/// 支持引号转义的 CSV 行拆分。
std::vector<std::string> splitCsv(const std::string &line)
{
    std::vector<std::string> fields;
    std::string field;
    bool quoted = false;
    for (std::size_t i = 0; i < line.size(); ++i)
    {
        const char ch = line[i];
        if (ch == '"')
        {
            if (quoted && i + 1 < line.size() && line[i + 1] == '"')
            {
                field.push_back('"');
                ++i;
            }
            else
            {
                quoted = !quoted;
            }
        }
        else if (ch == ',' && !quoted)
        {
            fields.push_back(field);
            field.clear();
        }
        else
        {
            field.push_back(ch);
        }
    }
    fields.push_back(field);
    return fields;
}

std::vector<std::string> splitWhitespace(const std::string &line)
{
    std::vector<std::string> fields;
    std::istringstream stream(line);
    std::string token;
    while (stream >> token)
    {
        fields.push_back(token);
    }
    return fields;
}

std::vector<std::string> readHead(const std::filesystem::path &path, std::size_t maxLines)
{
    std::vector<std::string> lines;
    std::ifstream input(path);
    if (!input.is_open())
    {
        return lines;
    }
    std::string line;
    while (lines.size() < maxLines && std::getline(input, line))
    {
        if (lines.empty())
        {
            line = stripBom(line);
        }
        lines.push_back(line);
    }
    return lines;
}

std::string readFirstLine(const std::filesystem::path &path)
{
    std::ifstream input(path);
    std::string line;
    if (!input.is_open() || !std::getline(input, line))
    {
        return {};
    }
    return stripBom(line);
}

std::string extensionLower(const std::filesystem::path &path)
{
    return toLower(path.extension().string());
}

float toFloat(const std::string &text, float fallback = 0.0f)
{
    const std::string trimmed = trim(text);
    if (trimmed.empty())
    {
        return fallback;
    }
    try
    {
        return std::stof(trimmed);
    }
    catch (...)
    {
        return fallback;
    }
}

int toInt(const std::string &text, int fallback = 0)
{
    const std::string trimmed = trim(text);
    if (trimmed.empty())
    {
        return fallback;
    }
    try
    {
        return std::stoi(trimmed);
    }
    catch (...)
    {
        return fallback;
    }
}

std::uint64_t toU64(const std::string &text, std::uint64_t fallback = 0)
{
    const std::string trimmed = trim(text);
    if (trimmed.empty())
    {
        return fallback;
    }
    try
    {
        return static_cast<std::uint64_t>(std::stoull(trimmed));
    }
    catch (...)
    {
        return fallback;
    }
}

struct CsvColumns
{
    std::vector<std::string> header;
    std::unordered_map<std::string, std::size_t> index;

    bool has(const std::string &name) const { return index.count(name) != 0; }

    std::size_t findAny(std::initializer_list<const char *> names) const
    {
        for (const char *name : names)
        {
            const auto it = index.find(name);
            if (it != index.end())
            {
                return it->second;
            }
        }
        return kNotFound;
    }

    static std::string value(const std::vector<std::string> &fields, std::size_t column)
    {
        return (column != kNotFound && column < fields.size()) ? trim(fields[column])
                                                              : std::string{};
    }
};

CsvColumns parseColumns(const std::vector<std::string> &header)
{
    CsvColumns columns;
    columns.header = header;
    for (std::size_t i = 0; i < header.size(); ++i)
    {
        columns.index[trim(header[i])] = i;
    }
    return columns;
}

bool looksLikePcd(const std::vector<std::string> &head)
{
    bool hasFields = false;
    bool hasData = false;
    for (const auto &line : head)
    {
        if (contains(line, "FIELDS"))
        {
            hasFields = true;
        }
        if (contains(line, "DATA"))
        {
            hasData = true;
        }
    }
    return hasFields && hasData;
}

bool looksLike6031Csv(const std::string &headerLine)
{
    return contains(headerLine, "center_bin") &&
           contains(headerLine, "timing_bin") &&
           contains(headerLine, "algorithm") &&
           contains(headerLine, "echo_bins");
}

bool looksLike6532Csv(const std::string &headerLine)
{
    const bool hasFrameOrRoi = contains(headerLine, "frame") || contains(headerLine, "roi");
    const bool hasRow = contains(headerLine, "row") || contains(headerLine, "行row");
    const bool hasCol = contains(headerLine, "col") || contains(headerLine, "列col");
    const bool hasDistance = contains(headerLine, "dist_m") || contains(headerLine, "distance_m") ||
                             contains(headerLine, "p0_dist_rp");
    return hasFrameOrRoi && hasRow && hasCol && hasDistance;
}

// ---------------------------------------------------------------------------
// FL6532 行数据
// ---------------------------------------------------------------------------

struct Row6532
{
    int row = 0;
    int col = 0;
    int peak = 0;
    int valid = 1;
    float slotAng = 0.0f;
    float theta = 0.0f;
    float dist = 0.0f;
    float reflect = 0.0f;
    float p0DistRaw = 0.0f;
    float p0Fwhm = 0.0f;
    float p0Conv = 0.0f;
    float p1DistRaw = 0.0f;
    float p1Fwhm = 0.0f;
    float p1Conv = 0.0f;
};

// ---------------------------------------------------------------------------
// FL6031 解析
// ---------------------------------------------------------------------------

struct Candidate6031
{
    float center = -1.0f;
    float timing = -1.0f;
    float amplitude = 0.0f;
    float width = 0.0f;
};

struct Row6031
{
    int row = 0;
    int col = 0;
    int valid = 0;
    std::uint32_t timestamp = 0;
    float horizontal = 0.0f;
    float vertical = 0.0f;
    float binWidthPs = 1000.0f;
    float timeZeroPs = 0.0f;
    float distance = 0.0f;
    float centerBin = 0.0f;
    float width = 0.0f;
    float timing = -1.0f;
    int candidateCount = 0;
    std::vector<Candidate6031> candidates;
};

bool parse6031(const std::filesystem::path &path,
               algo_lidar::PointMatrix &mat,
               std::string &algorithmName,
               std::string &error,
               std::size_t &dataLines,
               std::size_t &badLines,
               int &maxRowOut,
               int &maxColOut)
{
    std::ifstream input(path);
    if (!input.is_open())
    {
        error = "无法打开 FL6031 CSV";
        return false;
    }

    std::string line;
    if (!std::getline(input, line))
    {
        error = "FL6031 CSV 为空";
        return false;
    }
    line = stripBom(line);
    const auto header = splitCsv(line);
    const CsvColumns columns = parseColumns(header);

    const std::vector<std::string> required = {
        "timestamp", "row", "col", "horizontal_deg", "vertical_deg",
        "bin_width_ps", "time_zero_ps", "valid", "distance_m",
        "center_bin", "width_bin", "timing_bin", "candidate_count", "candidates"};
    for (const auto &name : required)
    {
        if (!columns.has(name))
        {
            error = "FL6031 CSV 缺少列: " + name;
            return false;
        }
    }

    const std::size_t cTimestamp = columns.index.at("timestamp");
    const std::size_t cRow = columns.index.at("row");
    const std::size_t cCol = columns.index.at("col");
    const std::size_t cHorizontal = columns.index.at("horizontal_deg");
    const std::size_t cVertical = columns.index.at("vertical_deg");
    const std::size_t cBinWidth = columns.index.at("bin_width_ps");
    const std::size_t cTimeZero = columns.index.at("time_zero_ps");
    const std::size_t cValid = columns.index.at("valid");
    const std::size_t cDistance = columns.index.at("distance_m");
    const std::size_t cCenterBin = columns.index.at("center_bin");
    const std::size_t cWidthBin = columns.index.at("width_bin");
    const std::size_t cTiming = columns.index.at("timing_bin");
    const std::size_t cCandidateCount = columns.index.at("candidate_count");
    const std::size_t cCandidates = columns.index.at("candidates");
    const std::size_t cAlgorithm = columns.findAny({"algorithm"});

    std::vector<Row6031> rows;
    int maxRow = -1;
    int maxCol = -1;
    dataLines = 0;
    badLines = 0;

    auto splitBy = [](const std::string &text, char delimiter)
    {
        std::vector<std::string> values;
        std::stringstream stream(text);
        std::string value;
        while (std::getline(stream, value, delimiter))
        {
            values.push_back(value);
        }
        return values;
    };

    while (std::getline(input, line))
    {
        if (trim(line).empty())
        {
            continue;
        }
        ++dataLines;
        const auto fields = splitCsv(line);

        Row6031 row;
        try
        {
            row.timestamp = static_cast<std::uint32_t>(
                toU64(CsvColumns::value(fields, cTimestamp), 0));
            row.row = std::stoi(CsvColumns::value(fields, cRow));
            row.col = std::stoi(CsvColumns::value(fields, cCol));
            row.horizontal = std::stof(CsvColumns::value(fields, cHorizontal));
            row.vertical = std::stof(CsvColumns::value(fields, cVertical));
            row.binWidthPs = std::stof(CsvColumns::value(fields, cBinWidth));
            row.timeZeroPs = std::stof(CsvColumns::value(fields, cTimeZero));
            row.valid = std::stoi(CsvColumns::value(fields, cValid));
            row.distance = std::stof(CsvColumns::value(fields, cDistance));
            row.centerBin = std::stof(CsvColumns::value(fields, cCenterBin));
            row.width = std::stof(CsvColumns::value(fields, cWidthBin));
            row.timing = std::stof(CsvColumns::value(fields, cTiming));
            row.candidateCount = std::stoi(CsvColumns::value(fields, cCandidateCount));
        }
        catch (...)
        {
            ++badLines;
            continue;
        }

        if (row.row < 0 || row.col < 0)
        {
            ++badLines;
            continue;
        }

        for (const auto &encoded : splitBy(CsvColumns::value(fields, cCandidates), ';'))
        {
            if (trim(encoded).empty())
            {
                continue;
            }
            const auto values = splitBy(encoded, '|');
            Candidate6031 candidate;
            try
            {
                if (values.size() >= 8)
                {
                    candidate.center = std::stof(values[0]);
                    candidate.timing = std::stof(values[1]);
                    candidate.amplitude = std::stof(values[2]);
                    candidate.width = std::stof(values[4]);
                }
                else if (values.size() >= 2)
                {
                    candidate.center = std::stof(values[0]);
                    candidate.timing = std::stof(values[1]);
                    candidate.amplitude = row.centerBin;
                    candidate.width = row.width;
                }
                else
                {
                    continue;
                }
            }
            catch (...)
            {
                continue;
            }
            row.candidates.push_back(candidate);
        }

        if (row.candidates.empty() && row.valid != 0 && row.timing >= 0.0f)
        {
            row.candidates.push_back(
                Candidate6031{row.centerBin, row.timing, row.centerBin, row.width});
        }

        if (algorithmName.empty() && cAlgorithm != kNotFound)
        {
            algorithmName = CsvColumns::value(fields, cAlgorithm);
        }

        maxRow = std::max(maxRow, row.row);
        maxCol = std::max(maxCol, row.col);
        rows.push_back(std::move(row));
    }

    if (rows.empty() || maxRow < 0 || maxCol < 0)
    {
        error = "FL6031 CSV 无有效数据";
        return false;
    }

    mat.assign(static_cast<std::size_t>(maxRow + 1),
               std::vector<algo_lidar::PointData>(static_cast<std::size_t>(maxCol + 1)));

    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kLightSpeed = 299792458.0f;

    for (const auto &row : rows)
    {
        algo_lidar::PointData point{};
        point.pixel_idx = static_cast<std::uint16_t>(row.row);
        point.slot_idx = static_cast<std::uint16_t>(row.col);
        point.timestamp = row.timestamp;
        point.slot_ang = row.horizontal;
        point.omega_rad = row.horizontal * kPi / 180.0f;
        point.theta = row.vertical;
        point.theta_rad = row.vertical * kPi / 180.0f;

        float distance = row.distance;
        if (!(distance > 0.0f) && row.timing >= 0.0f)
        {
            const float tofSeconds = (row.timeZeroPs + row.timing * row.binWidthPs) * 1.0e-12f;
            distance = tofSeconds * kLightSpeed * 0.5f;
        }
        point.r = distance;

        point.peak = static_cast<std::uint16_t>(
            std::clamp(std::lround(row.centerBin), 0L, 65535L));
        const float intensity = std::clamp(row.centerBin / 4095.0f * 255.0f, 0.0f, 255.0f);
        point.intensity = static_cast<std::uint8_t>(std::lround(intensity));
        point.reflect = intensity / 255.0f;

        point.is_valid = row.valid != 0 && std::isfinite(point.r) &&
                         point.r > 0.0f && point.r < 1000.0f;
        point.select_idx = 0;

        if (point.is_valid)
        {
            point.x = point.r * std::cos(point.theta_rad) * std::cos(point.omega_rad);
            point.y = point.r * std::cos(point.theta_rad) * std::sin(point.omega_rad);
            point.z = point.r * std::sin(point.theta_rad);
        }

        for (const auto &candidate : row.candidates)
        {
            if (candidate.timing < 0.0f)
            {
                continue;
            }
            const float tofPs = row.timeZeroPs + candidate.timing * row.binWidthPs;
            if (tofPs <= 0.0f)
            {
                continue;
            }
            point.list_pulse.emplace_back(tofPs, candidate.width, candidate.amplitude);
        }
        point.is_tail_pt = row.width > 0.0f && row.width < 2.0f;

        mat[static_cast<std::size_t>(row.row)][static_cast<std::size_t>(row.col)] =
            std::move(point);
    }

    maxRowOut = maxRow;
    maxColOut = maxCol;
    return true;
}

// ---------------------------------------------------------------------------
// PCD 解析（自含 ASCII / binary）
// ---------------------------------------------------------------------------

struct PcdHeader
{
    std::vector<std::string> fields;
    std::vector<int> sizes;
    std::vector<char> types;
    std::vector<int> counts;
    int width = 0;
    int height = 1;
    std::size_t points = 0;
    std::string dataType;
    std::streamoff dataOffset = 0;
};

bool parsePcdHeader(const std::filesystem::path &path, PcdHeader &header, std::string &error)
{
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open())
    {
        error = "无法打开 PCD";
        return false;
    }

    std::string line;
    while (std::getline(input, line))
    {
        line = stripBom(line);
        std::istringstream stream(line);
        std::string key;
        stream >> key;
        if (key == "FIELDS")
        {
            std::string name;
            while (stream >> name)
            {
                header.fields.push_back(name);
            }
        }
        else if (key == "SIZE")
        {
            int value = 0;
            while (stream >> value)
            {
                header.sizes.push_back(value);
            }
        }
        else if (key == "TYPE")
        {
            char value = 0;
            while (stream >> value)
            {
                header.types.push_back(value);
            }
        }
        else if (key == "COUNT")
        {
            int value = 0;
            while (stream >> value)
            {
                header.counts.push_back(value);
            }
        }
        else if (key == "WIDTH")
        {
            stream >> header.width;
        }
        else if (key == "HEIGHT")
        {
            stream >> header.height;
        }
        else if (key == "POINTS")
        {
            stream >> header.points;
        }
        else if (key == "DATA")
        {
            stream >> header.dataType;
            header.dataOffset = input.tellg();
            break;
        }
    }

    if (header.fields.empty() || header.dataType.empty())
    {
        error = "PCD 头部不完整";
        return false;
    }
    if (header.counts.empty())
    {
        header.counts.assign(header.fields.size(), 1);
    }
    if (header.points == 0)
    {
        header.points = static_cast<std::size_t>(std::max(header.width, 0)) *
                        static_cast<std::size_t>(std::max(header.height, 1));
    }
    return true;
}

std::size_t fieldIndex(const PcdHeader &header, const std::string &name)
{
    for (std::size_t i = 0; i < header.fields.size(); ++i)
    {
        if (header.fields[i] == name)
        {
            return i;
        }
    }
    return kNotFound;
}

template <typename T>
T readBinaryValue(const char *buffer, std::size_t offset)
{
    T value{};
    std::memcpy(&value, buffer + offset, sizeof(T));
    return value;
}

bool decodePcdPoints(const std::filesystem::path &path,
                     const PcdHeader &header,
                     float scale,
                     algo_lidar::PointMatrix &mat,
                     std::string &error)
{
    const std::size_t ix = fieldIndex(header, "x");
    const std::size_t iy = fieldIndex(header, "y");
    const std::size_t iz = fieldIndex(header, "z");
    const std::size_t ii = fieldIndex(header, "intensity");
    if (ix == kNotFound || iy == kNotFound || iz == kNotFound)
    {
        error = "PCD 缺少 x/y/z 字段";
        return false;
    }

    const int rows = header.height > 1 ? header.height : 1;
    const int cols = header.height > 1 ? header.width
                                       : static_cast<int>(header.points);
    mat.assign(static_cast<std::size_t>(rows),
               std::vector<algo_lidar::PointData>(static_cast<std::size_t>(std::max(cols, 0))));

    auto makePoint = [&](float x, float y, float z, float intensity) -> algo_lidar::PointData
    {
        algo_lidar::PointData point{};
        point.x = x * scale;
        point.y = y * scale;
        point.z = z * scale;
        point.r = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
        {
            point.is_valid = false;
            return point;
        }
        point.is_valid = true;
        const float clamped = std::clamp(intensity, 0.0f, 255.0f);
        point.intensity = static_cast<std::uint8_t>(clamped);
        point.reflect = clamped / 255.0f;
        point.peak = static_cast<std::uint16_t>(clamped);
        point.theta = std::asin(point.z / (point.r + 1e-6f)) * 180.0f / 3.14159265358979323846f;
        point.theta_rad = point.theta * 3.14159265358979323846f / 180.0f;
        point.slot_ang = std::atan2(point.y, point.x) * 180.0f / 3.14159265358979323846f;
        point.omega_rad = point.slot_ang * 3.14159265358979323846f / 180.0f;
        return point;
    };

    if (header.dataType == "ascii")
    {
        std::ifstream input(path);
        std::string line;
        std::size_t index = 0;
        bool reachedData = false;
        while (std::getline(input, line))
        {
            if (!reachedData)
            {
                if (contains(line, "DATA"))
                {
                    reachedData = true;
                }
                continue;
            }
            if (trim(line).empty())
            {
                continue;
            }
            if (index >= header.points)
            {
                break;
            }
            const auto values = splitWhitespace(line);
            if (values.size() < header.fields.size())
            {
                ++index;
                continue;
            }
            const float x = toFloat(values[ix]);
            const float y = toFloat(values[iy]);
            const float z = toFloat(values[iz]);
            const float intensity = (ii != kNotFound) ? toFloat(values[ii]) : 128.0f;

            const std::size_t row = header.height > 1 ? index / header.width : 0;
            const std::size_t col = header.height > 1 ? index % header.width : index;
            if (row < static_cast<std::size_t>(rows) && col < static_cast<std::size_t>(cols))
            {
                mat[row][col] = makePoint(x, y, z, intensity);
            }
            ++index;
        }
    }
    else if (header.dataType == "binary")
    {
        std::ifstream input(path, std::ios::binary);
        input.seekg(header.dataOffset, std::ios::beg);

        std::size_t stride = 0;
        for (std::size_t i = 0; i < header.fields.size(); ++i)
        {
            stride += static_cast<std::size_t>(header.sizes[i]) *
                      static_cast<std::size_t>(header.counts[i]);
        }

        std::vector<char> buffer(stride);
        auto fieldOffset = [&](std::size_t field) -> std::size_t
        {
            std::size_t offset = 0;
            for (std::size_t i = 0; i < field; ++i)
            {
                offset += static_cast<std::size_t>(header.sizes[i]) *
                          static_cast<std::size_t>(header.counts[i]);
            }
            return offset;
        };

        auto readAsFloat = [&](const char *data, std::size_t field) -> float
        {
            const std::size_t offset = fieldOffset(field);
            const char type = header.types[field];
            const int size = header.sizes[field];
            if (type == 'F')
            {
                if (size == 4)
                {
                    return readBinaryValue<float>(data, offset);
                }
                if (size == 8)
                {
                    return static_cast<float>(readBinaryValue<double>(data, offset));
                }
            }
            else if (type == 'U')
            {
                if (size == 4)
                {
                    return static_cast<float>(readBinaryValue<std::uint32_t>(data, offset));
                }
                if (size == 1)
                {
                    return static_cast<float>(readBinaryValue<std::uint8_t>(data, offset));
                }
            }
            else if (type == 'I')
            {
                if (size == 4)
                {
                    return static_cast<float>(readBinaryValue<std::int32_t>(data, offset));
                }
                if (size == 2)
                {
                    return static_cast<float>(readBinaryValue<std::int16_t>(data, offset));
                }
            }
            return 0.0f;
        };

        for (std::size_t index = 0; index < header.points; ++index)
        {
            input.read(buffer.data(), static_cast<std::streamsize>(stride));
            if (!input)
            {
                break;
            }
            const float x = readAsFloat(buffer.data(), ix);
            const float y = readAsFloat(buffer.data(), iy);
            const float z = readAsFloat(buffer.data(), iz);
            const float intensity = (ii != kNotFound) ? readAsFloat(buffer.data(), ii) : 128.0f;

            const std::size_t row = header.height > 1 ? index / header.width : 0;
            const std::size_t col = header.height > 1 ? index % header.width : index;
            if (row < static_cast<std::size_t>(rows) && col < static_cast<std::size_t>(cols))
            {
                mat[row][col] = makePoint(x, y, z, intensity);
            }
        }
    }
    else
    {
        error = "不支持的 PCD DATA 类型: " + header.dataType;
        return false;
    }

    return true;
}

/// 统计矩阵有效点，并构造 FrameMetadata。
FrameMetadata buildMetadata(const std::filesystem::path &path,
                            RadarModel model,
                            const algo_lidar::PointMatrix &mat,
                            std::uint64_t frameNumber,
                            int roiId,
                            const std::string &algorithm)
{
    FrameMetadata metadata;
    metadata.sourcePath = path;
    metadata.model = model;
    metadata.frameNumber = frameNumber;
    metadata.roiId = roiId;
    metadata.algorithmId = algorithm;
    metadata.rows = static_cast<int>(mat.size());
    metadata.cols = mat.empty() ? 0 : static_cast<int>(mat.front().size());

    std::size_t valid = 0;
    std::size_t total = 0;
    for (const auto &row : mat)
    {
        for (const auto &point : row)
        {
            ++total;
            if (point.is_valid)
            {
                ++valid;
            }
        }
    }
    metadata.totalSamples = total;
    metadata.validSamples = valid;
    metadata.invalidSamples = total - valid;
    return metadata;
}

} // namespace

const char *loaderIdName(LoaderId id)
{
    switch (id)
    {
    case LoaderId::RotatingTxt:
        return "rotating_txt";
    case LoaderId::SolidTxt:
        return "solid_txt";
    case LoaderId::FrameCsv6532:
        return "frame_csv_6532";
    case LoaderId::FrameCsv6031:
        return "frame_csv_6031";
    case LoaderId::Pcd:
        return "pcd";
    case LoaderId::Unknown:
    default:
        return "unknown";
    }
}

// ---------------------------------------------------------------------------
// RotatingTxtLoader
// ---------------------------------------------------------------------------

ProbeResult RotatingTxtLoader::probe(const std::filesystem::path &path) const
{
    ProbeResult result;
    if (extensionLower(path) != ".txt" && extensionLower(path) != "")
    {
        return result;
    }
    const auto head = readHead(path, kCsvHeaderScanLines);
    bool hasMirrorAngle = false;
    for (const auto &line : head)
    {
        if (contains(line, "start of col") || (contains(line, "cols:") && contains(line, "rows:")))
        {
            return result; // 固态格式优先
        }
        if (contains(line, "rotate mirror angle"))
        {
            hasMirrorAngle = true;
        }
    }
    for (const auto &line : head)
    {
        if (contains(line, "start of slot") ||
            (contains(line, "start of") && hasMirrorAngle))
        {
            result.loader = LoaderId::RotatingTxt;
            result.model = RadarModel::Generic;
            result.formatName = "转镜雷达 TXT";
            result.reason = "检测到 start of slot 与 rotate mirror angle 数据块";
            return result;
        }
    }
    return result;
}

bool RotatingTxtLoader::buildIndex(const std::filesystem::path &path,
                                   const LoadOptions &options,
                                   FileFrameIndex &index,
                                   std::string &error) const
{
    algo_lidar::PointMatrix mat;
    algo_lidar::readPulseData(path.string(), mat);
    if (mat.empty())
    {
        error = "转镜 TXT 解析结果为空";
        return false;
    }

    index.path = path;
    index.loader = LoaderId::RotatingTxt;
    index.model = (options.modelHint != RadarModel::Unknown) ? options.modelHint
                                                            : RadarModel::AlphaRotating;
    index.rows = static_cast<int>(mat.size());
    index.cols = mat.empty() ? 0 : static_cast<int>(mat.front().size());
    index.dataLines = static_cast<std::size_t>(index.rows) * static_cast<std::size_t>(index.cols);

    std::size_t valid = 0;
    for (const auto &row : mat)
    {
        for (const auto &point : row)
        {
            if (point.is_valid)
            {
                ++valid;
            }
        }
    }
    index.validSamples = valid;
    index.invalidSamples = index.dataLines - valid;

    FrameDescriptor descriptor;
    descriptor.id = 1;
    descriptor.sourcePath = path;
    descriptor.frameNumber = 0;
    index.frames.push_back(descriptor);
    return true;
}

FrameHandle RotatingTxtLoader::decode(const FrameDescriptor &descriptor,
                                      const LoadOptions &options,
                                      std::string &error) const
{
    algo_lidar::PointMatrix mat;
    algo_lidar::readPulseData(descriptor.sourcePath.string(), mat);
    if (mat.empty())
    {
        error = "转镜 TXT 解码结果为空";
        return nullptr;
    }
    const RadarModel model = (options.modelHint != RadarModel::Unknown)
                                 ? options.modelHint
                                 : RadarModel::AlphaRotating;
    return makeFrame(std::move(mat),
                     buildMetadata(descriptor.sourcePath, model, mat, descriptor.frameNumber, -1, ""),
                     descriptor.id);
}

// ---------------------------------------------------------------------------
// SolidTxtLoader
// ---------------------------------------------------------------------------

ProbeResult SolidTxtLoader::probe(const std::filesystem::path &path) const
{
    ProbeResult result;
    if (extensionLower(path) != ".txt" && extensionLower(path) != "")
    {
        return result;
    }
    const auto head = readHead(path, kCsvHeaderScanLines);
    bool hasColsRows = false;
    bool hasStartOfCol = false;
    for (const auto &line : head)
    {
        if (contains(line, "cols:") && contains(line, "rows:"))
        {
            hasColsRows = true;
        }
        if (contains(line, "start of col"))
        {
            hasStartOfCol = true;
        }
    }
    if (hasColsRows || hasStartOfCol)
    {
        result.loader = LoaderId::SolidTxt;
        result.model = RadarModel::Generic;
        result.formatName = "固态雷达 TXT";
        result.reason = hasColsRows ? "检测到 cols/rows 头部" : "检测到 start of col 数据块";
    }
    return result;
}

bool SolidTxtLoader::buildIndex(const std::filesystem::path &path,
                                const LoadOptions &options,
                                FileFrameIndex &index,
                                std::string &error) const
{
    algo_lidar::PointMatrix mat;
    if (!algo_lidar::readSolidLidarData(path.string(), mat) || mat.empty())
    {
        error = "固态 TXT 解析失败";
        return false;
    }

    index.path = path;
    index.loader = LoaderId::SolidTxt;
    index.model = (options.modelHint != RadarModel::Unknown) ? options.modelHint
                                                            : RadarModel::Fl6031;
    index.rows = static_cast<int>(mat.size());
    index.cols = mat.empty() ? 0 : static_cast<int>(mat.front().size());
    index.dataLines = static_cast<std::size_t>(index.rows) * static_cast<std::size_t>(index.cols);

    std::size_t valid = 0;
    for (const auto &row : mat)
    {
        for (const auto &point : row)
        {
            if (point.is_valid)
            {
                ++valid;
            }
        }
    }
    index.validSamples = valid;
    index.invalidSamples = index.dataLines - valid;

    FrameDescriptor descriptor;
    descriptor.id = 1;
    descriptor.sourcePath = path;
    descriptor.frameNumber = 0;
    index.frames.push_back(descriptor);
    return true;
}

FrameHandle SolidTxtLoader::decode(const FrameDescriptor &descriptor,
                                   const LoadOptions &options,
                                   std::string &error) const
{
    algo_lidar::PointMatrix mat;
    if (!algo_lidar::readSolidLidarData(descriptor.sourcePath.string(), mat) || mat.empty())
    {
        error = "固态 TXT 解码失败";
        return nullptr;
    }
    const RadarModel model = (options.modelHint != RadarModel::Unknown)
                                 ? options.modelHint
                                 : RadarModel::Fl6031;
    return makeFrame(std::move(mat),
                     buildMetadata(descriptor.sourcePath, model, mat, descriptor.frameNumber, -1, ""),
                     descriptor.id);
}

// ---------------------------------------------------------------------------
// FrameCsv6532Loader
// ---------------------------------------------------------------------------

ProbeResult FrameCsv6532Loader::probe(const std::filesystem::path &path) const
{
    ProbeResult result;
    if (extensionLower(path) != ".csv")
    {
        return result;
    }
    const std::string header = readFirstLine(path);
    if (looksLike6031Csv(header))
    {
        return result; // 6031 优先
    }
    if (looksLike6532Csv(header))
    {
        result.loader = LoaderId::FrameCsv6532;
        result.model = RadarModel::Fl6532;
        result.formatName = "FL6532 Frame CSV";
        result.reason = "表头包含 frame/roi/row/col 与距离字段";
    }
    return result;
}

bool FrameCsv6532Loader::buildIndex(const std::filesystem::path &path,
                                    const LoadOptions &options,
                                    FileFrameIndex &index,
                                    std::string &error) const
{
    (void)options;
    struct FrameStat
    {
        int maxRow = -1;
        int maxCol = -1;
        std::size_t rows = 0;
        std::size_t valid = 0;
        std::size_t invalid = 0;
        int roi = -1;
    };

    std::map<std::uint64_t, FrameStat> stats;
    std::size_t dataLines = 0;
    std::size_t badLines = 0;

    // 需要先读取 frame 列；scan6532 不暴露 frame，这里单独扫描。
    std::ifstream input(path);
    if (!input.is_open())
    {
        error = "无法打开 FL6532 CSV";
        return false;
    }
    std::string line;
    if (!std::getline(input, line))
    {
        error = "FL6532 CSV 为空";
        return false;
    }
    const auto header = splitCsv(stripBom(line));
    const CsvColumns columns = parseColumns(header);
    const std::size_t cFrame = columns.findAny({"frame"});
    const std::size_t cRow = columns.findAny({"row", "行row"});
    const std::size_t cCol = columns.findAny({"col", "列col"});
    const std::size_t cValid = columns.findAny({"is_valid", "valid"});
    const std::size_t cRoi = columns.findAny({"roi"});

    if (cRow == kNotFound || cCol == kNotFound)
    {
        error = "FL6532 CSV 缺少 row/col 列";
        return false;
    }

    while (std::getline(input, line))
    {
        if (trim(line).empty())
        {
            continue;
        }
        const auto fields = splitCsv(line);
        const int row = toInt(CsvColumns::value(fields, cRow), -1);
        const int col = toInt(CsvColumns::value(fields, cCol), -1);
        if (row < 0 || col < 0)
        {
            ++badLines;
            continue;
        }
        ++dataLines;
        const std::uint64_t frame = (cFrame != kNotFound)
                                        ? toU64(CsvColumns::value(fields, cFrame), 0)
                                        : 0;
        FrameStat &stat = stats[frame];
        stat.maxRow = std::max(stat.maxRow, row);
        stat.maxCol = std::max(stat.maxCol, col);
        ++stat.rows;
        if (cValid != kNotFound)
        {
            (toInt(CsvColumns::value(fields, cValid), 1) != 0) ? ++stat.valid : ++stat.invalid;
        }
        if (cRoi != kNotFound && stat.roi < 0)
        {
            stat.roi = toInt(CsvColumns::value(fields, cRoi), -1);
        }
    }

    if (stats.empty())
    {
        error = "FL6532 CSV 无有效数据行";
        return false;
    }

    index.path = path;
    index.loader = LoaderId::FrameCsv6532;
    index.model = RadarModel::Fl6532;
    index.dataLines = dataLines;
    index.badLines = badLines;

    FrameId id = 1;
    for (const auto &entry : stats)
    {
        index.rows = std::max(index.rows, entry.second.maxRow + 1);
        index.cols = std::max(index.cols, entry.second.maxCol + 1);
        index.validSamples += entry.second.valid;
        index.invalidSamples += entry.second.invalid;

        FrameDescriptor descriptor;
        descriptor.id = id++;
        descriptor.sourcePath = path;
        descriptor.frameNumber = entry.first;
        descriptor.roiId = entry.second.roi;
        index.frames.push_back(descriptor);
    }
    return true;
}

FrameHandle FrameCsv6532Loader::decode(const FrameDescriptor &descriptor,
                                       const LoadOptions &options,
                                       std::string &error) const
{
    (void)options;
    std::vector<Row6532> rows;
    std::size_t dataLines = 0;
    std::size_t badLines = 0;

    // 只保留目标帧。
    std::ifstream input(descriptor.sourcePath);
    if (!input.is_open())
    {
        error = "无法打开 FL6532 CSV";
        return nullptr;
    }
    std::string line;
    if (!std::getline(input, line))
    {
        error = "FL6532 CSV 为空";
        return nullptr;
    }
    const auto header = splitCsv(stripBom(line));
    const CsvColumns columns = parseColumns(header);
    const std::size_t cFrame = columns.findAny({"frame"});
    const std::size_t cRow = columns.findAny({"row", "行row"});
    const std::size_t cCol = columns.findAny({"col", "列col"});
    const std::size_t cDist = columns.findAny({"dist_m", "distance_m"});
    const std::size_t cSlot = columns.findAny({"slot_ang", "horizontal_deg"});
    const std::size_t cTheta = columns.findAny({"theta", "vertical_deg"});
    const std::size_t cPeak = columns.findAny({"peak"});
    const std::size_t cReflect = columns.findAny({"reflect"});
    const std::size_t cValid = columns.findAny({"is_valid", "valid"});
    const std::size_t c0Raw = columns.findAny({"p0_dist_rp", "p0_dist_rp_raw"});
    const std::size_t c0Fwhm = columns.findAny({"p0_fwhm"});
    const std::size_t c0Conv = columns.findAny({"p0_conv_intensity", "p0_conv"});
    const std::size_t c1Raw = columns.findAny({"p1_dist_rp", "p1_dist_rp_raw"});
    const std::size_t c1Fwhm = columns.findAny({"p1_fwhm"});
    const std::size_t c1Conv = columns.findAny({"p1_conv_intensity", "p1_conv"});

    if (cRow == kNotFound || cCol == kNotFound)
    {
        error = "FL6532 CSV 缺少 row/col 列";
        return nullptr;
    }

    int maxRow = -1;
    int maxCol = -1;
    while (std::getline(input, line))
    {
        if (trim(line).empty())
        {
            continue;
        }
        const auto fields = splitCsv(line);
        const std::uint64_t frame = (cFrame != kNotFound)
                                        ? toU64(CsvColumns::value(fields, cFrame), 0)
                                        : 0;
        if (frame != descriptor.frameNumber)
        {
            continue;
        }
        Row6532 row;
        row.row = toInt(CsvColumns::value(fields, cRow), -1);
        row.col = toInt(CsvColumns::value(fields, cCol), -1);
        if (row.row < 0 || row.col < 0)
        {
            ++badLines;
            continue;
        }
        row.dist = toFloat(CsvColumns::value(fields, cDist), 0.0f);
        row.slotAng = toFloat(CsvColumns::value(fields, cSlot), 0.0f);
        row.theta = toFloat(CsvColumns::value(fields, cTheta), 0.0f);
        row.peak = toInt(CsvColumns::value(fields, cPeak), 0);
        row.reflect = toFloat(CsvColumns::value(fields, cReflect), -1.0f);
        row.valid = toInt(CsvColumns::value(fields, cValid), 1);
        row.p0DistRaw = toFloat(CsvColumns::value(fields, c0Raw), 0.0f);
        row.p0Fwhm = toFloat(CsvColumns::value(fields, c0Fwhm), 0.0f);
        row.p0Conv = toFloat(CsvColumns::value(fields, c0Conv), 0.0f);
        row.p1DistRaw = toFloat(CsvColumns::value(fields, c1Raw), 0.0f);
        row.p1Fwhm = toFloat(CsvColumns::value(fields, c1Fwhm), 0.0f);
        row.p1Conv = toFloat(CsvColumns::value(fields, c1Conv), 0.0f);

        maxRow = std::max(maxRow, row.row);
        maxCol = std::max(maxCol, row.col);
        rows.push_back(row);
        ++dataLines;
    }

    if (rows.empty())
    {
        error = "指定帧无数据";
        return nullptr;
    }

    algo_lidar::PointMatrix mat(static_cast<std::size_t>(maxRow + 1),
                                std::vector<algo_lidar::PointData>(
                                    static_cast<std::size_t>(maxCol + 1)));

    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kLightSpeed = 3.0e8f;
    constexpr float kRawScale = 25600.0f;
    constexpr float kInvalidRaw = 0xFFFFFF;

    for (const auto &row : rows)
    {
        algo_lidar::PointData point{};
        point.pixel_idx = static_cast<std::uint16_t>(row.row);
        point.slot_idx = static_cast<std::uint16_t>(row.col);
        point.slot_ang = row.slotAng;
        point.omega_rad = row.slotAng * kPi / 180.0f;
        point.theta = row.theta;
        point.theta_rad = row.theta * kPi / 180.0f;
        point.r = row.dist;
        point.peak = static_cast<std::uint16_t>(std::min(row.peak, 65535));

        // reflect 列优先；缺失时把 14 bit 峰值归一化到 0~255。
        const float intensity = (row.reflect >= 0.0f)
                                    ? std::clamp(row.reflect * 255.0f, 0.0f, 255.0f)
                                    : std::clamp(static_cast<float>(row.peak) / 16383.0f * 255.0f,
                                                 0.0f, 255.0f);
        point.intensity = static_cast<std::uint8_t>(std::lround(intensity));
        point.reflect = (row.reflect >= 0.0f) ? row.reflect : intensity / 255.0f;

        point.is_valid = row.valid != 0 && row.dist > 0.0f && row.dist < 1000.0f;
        if (point.is_valid)
        {
            point.x = point.r * std::cos(point.theta_rad) * std::cos(point.omega_rad);
            point.y = point.r * std::cos(point.theta_rad) * std::sin(point.omega_rad);
            point.z = point.r * std::sin(point.theta_rad);
        }

        point.select_idx = 0;
        if (row.p0DistRaw > 0.0f && row.p0DistRaw < kInvalidRaw)
        {
            const float distM = row.p0DistRaw / kRawScale;
            const float tofPs = distM * 2.0f / kLightSpeed * 1e12f;
            point.list_pulse.emplace_back(tofPs, row.p0Fwhm / 256.0f, row.p0Conv);
            if (row.p0Fwhm / 256.0f < 2.0f)
            {
                point.is_tail_pt = true;
            }
        }
        if (row.p1DistRaw > 0.0f && row.p1DistRaw < kInvalidRaw)
        {
            const float distM = row.p1DistRaw / kRawScale;
            const float tofPs = distM * 2.0f / kLightSpeed * 1e12f;
            point.list_pulse.emplace_back(tofPs, row.p1Fwhm / 256.0f, row.p1Conv);
        }

        mat[static_cast<std::size_t>(row.row)][static_cast<std::size_t>(row.col)] =
            std::move(point);
    }

    return makeFrame(std::move(mat),
                     buildMetadata(descriptor.sourcePath, RadarModel::Fl6532, mat,
                                   descriptor.frameNumber, descriptor.roiId, descriptor.algorithmId),
                     descriptor.id);
}

// ---------------------------------------------------------------------------
// FrameCsv6031Loader
// ---------------------------------------------------------------------------

ProbeResult FrameCsv6031Loader::probe(const std::filesystem::path &path) const
{
    ProbeResult result;
    if (extensionLower(path) != ".csv")
    {
        return result;
    }
    if (looksLike6031Csv(readFirstLine(path)))
    {
        result.loader = LoaderId::FrameCsv6031;
        result.model = RadarModel::Fl6031;
        result.formatName = "FL6031 Frame CSV";
        result.reason = "表头包含 center_bin/timing_bin/algorithm/echo_bins";
    }
    return result;
}

bool FrameCsv6031Loader::buildIndex(const std::filesystem::path &path,
                                    const LoadOptions &options,
                                    FileFrameIndex &index,
                                    std::string &error) const
{
    (void)options;
    algo_lidar::PointMatrix mat;
    std::string algorithm;
    std::size_t dataLines = 0;
    std::size_t badLines = 0;
    int maxRow = -1;
    int maxCol = -1;
    if (!parse6031(path, mat, algorithm, error, dataLines, badLines, maxRow, maxCol))
    {
        return false;
    }

    index.path = path;
    index.loader = LoaderId::FrameCsv6031;
    index.model = RadarModel::Fl6031;
    index.rows = maxRow + 1;
    index.cols = maxCol + 1;
    index.dataLines = dataLines;
    index.badLines = badLines;
    index.validSamples = 0;
    for (const auto &row : mat)
    {
        for (const auto &point : row)
        {
            if (point.is_valid)
            {
                ++index.validSamples;
            }
        }
    }
    index.invalidSamples = index.dataLines - index.validSamples;

    FrameDescriptor descriptor;
    descriptor.id = 1;
    descriptor.sourcePath = path;
    descriptor.frameNumber = 0;
    descriptor.algorithmId = algorithm;
    index.frames.push_back(descriptor);
    return true;
}

FrameHandle FrameCsv6031Loader::decode(const FrameDescriptor &descriptor,
                                       const LoadOptions &options,
                                       std::string &error) const
{
    (void)options;
    algo_lidar::PointMatrix mat;
    std::string algorithm;
    std::size_t dataLines = 0;
    std::size_t badLines = 0;
    int maxRow = -1;
    int maxCol = -1;
    if (!parse6031(descriptor.sourcePath, mat, algorithm, error, dataLines, badLines,
                   maxRow, maxCol))
    {
        return nullptr;
    }

    FrameMetadata metadata = buildMetadata(descriptor.sourcePath, RadarModel::Fl6031, mat,
                                           descriptor.frameNumber, -1, algorithm);
    metadata.algorithmId = algorithm;
    return makeFrame(std::move(mat), metadata, descriptor.id);
}

// ---------------------------------------------------------------------------
// PcdLoader
// ---------------------------------------------------------------------------

ProbeResult PcdLoader::probe(const std::filesystem::path &path) const
{
    ProbeResult result;
    if (extensionLower(path) != ".pcd")
    {
        return result;
    }
    if (looksLikePcd(readHead(path, 40)))
    {
        result.loader = LoaderId::Pcd;
        result.model = RadarModel::Generic;
        result.formatName = "PCD";
        result.reason = "检测到 PCD 头部 FIELDS 与 DATA";
    }
    return result;
}

bool PcdLoader::buildIndex(const std::filesystem::path &path,
                           const LoadOptions &options,
                           FileFrameIndex &index,
                           std::string &error) const
{
    (void)options;
    PcdHeader header;
    if (!parsePcdHeader(path, header, error))
    {
        return false;
    }

    index.path = path;
    index.loader = LoaderId::Pcd;
    index.model = (options.modelHint != RadarModel::Unknown) ? options.modelHint
                                                            : RadarModel::Generic;
    index.rows = header.height > 1 ? header.height : 1;
    index.cols = header.height > 1 ? header.width : static_cast<int>(header.points);
    index.dataLines = header.points;

    FrameDescriptor descriptor;
    descriptor.id = 1;
    descriptor.sourcePath = path;
    descriptor.frameNumber = 0;
    index.frames.push_back(descriptor);
    return true;
}

FrameHandle PcdLoader::decode(const FrameDescriptor &descriptor,
                              const LoadOptions &options,
                              std::string &error) const
{
    PcdHeader header;
    if (!parsePcdHeader(descriptor.sourcePath, header, error))
    {
        return nullptr;
    }

    algo_lidar::PointMatrix mat;
    if (!decodePcdPoints(descriptor.sourcePath, header, options.pcdScale, mat, error))
    {
        return nullptr;
    }

    FrameMetadata metadata = buildMetadata(descriptor.sourcePath, RadarModel::Generic, mat,
                                           descriptor.frameNumber, -1, "");
    metadata.calibrationState = "pcd";
    return makeFrame(std::move(mat), metadata, descriptor.id);
}

// ---------------------------------------------------------------------------
// LoaderRegistry
// ---------------------------------------------------------------------------

LoaderRegistry &LoaderRegistry::instance()
{
    static LoaderRegistry registry;
    return registry;
}

LoaderRegistry::LoaderRegistry()
{
    // 注册顺序即嗅探优先级：PCD > 6031 CSV > 6532 CSV > 固态 TXT > 转镜 TXT。
    loaders_.push_back(std::make_shared<PcdLoader>());
    loaders_.push_back(std::make_shared<FrameCsv6031Loader>());
    loaders_.push_back(std::make_shared<FrameCsv6532Loader>());
    loaders_.push_back(std::make_shared<SolidTxtLoader>());
    loaders_.push_back(std::make_shared<RotatingTxtLoader>());
}

void LoaderRegistry::registerLoader(const std::shared_ptr<IPointCloudLoader> &loader)
{
    if (loader)
    {
        loaders_.push_back(loader);
    }
}

const IPointCloudLoader *LoaderRegistry::selectLoader(const std::filesystem::path &path,
                                                      const LoadOptions &options,
                                                      ProbeResult &probe) const
{
    (void)options;
    for (const auto &loader : loaders_)
    {
        ProbeResult current = loader->probe(path);
        if (current.recognized())
        {
            probe = current;
            return loader.get();
        }
    }
    probe = ProbeResult{};
    return nullptr;
}

bool LoaderRegistry::buildIndex(const std::filesystem::path &path,
                                const LoadOptions &options,
                                FileFrameIndex &index,
                                std::string &error) const
{
    ProbeResult probe;
    const IPointCloudLoader *loader = selectLoader(path, options, probe);
    if (!loader)
    {
        error = "无法识别文件格式";
        return false;
    }
    index = FileFrameIndex{};
    if (!loader->buildIndex(path, options, index, error))
    {
        return false;
    }
    if (index.model == RadarModel::Unknown)
    {
        index.model = probe.model;
    }
    return true;
}

FrameHandle LoaderRegistry::decode(const FrameDescriptor &descriptor,
                                   const LoadOptions &options,
                                   const IPointCloudLoader *loader,
                                   std::string &error) const
{
    if (!loader)
    {
        error = "加载器为空";
        return nullptr;
    }
    return loader->decode(descriptor, options, error);
}

} // namespace cloud_viewer
