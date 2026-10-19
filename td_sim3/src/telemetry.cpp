#include "tdsim/telemetry.hpp"

#include <iomanip>
#include <stdexcept>

namespace tdsim {

TelemetryLogger::TelemetryLogger(std::vector<Column> columns, std::string path)
    : columns_(std::move(columns)) {
    out_.open(path, std::ios::out | std::ios::trunc);
    if (!out_) {
        throw std::runtime_error("Telemetry: cannot open output file '" + path + "'");
    }
    header_written_ = false;
}

void TelemetryLogger::record(double t) {
    if (!out_.is_open()) return;

    if (!header_written_) {
        out_ << "t";
        for (const auto& c : columns_) {
            out_ << "," << c.name;
        }
        out_ << "\n";
        header_written_ = true;
    }

    out_ << std::fixed << std::setprecision(9) << t;
    for (const auto& c : columns_) {
        out_ << "," << c.get();
    }
    out_ << "\n";
}

void TelemetryLogger::close() {
    if (out_.is_open()) {
        out_.flush();
        out_.close();
    }
}

} // namespace tdsim
