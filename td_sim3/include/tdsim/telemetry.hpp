#pragma once
// td_sim3 — telemetry.hpp
//
// Telemetry is nearly free because every ODE slot and every signal already
// carries a human name (required for config wiring anyway). The logger is
// given an ordered list of named columns (name + getter). At each sample it
// writes one CSV row. Default is a synchronous inline write (cheap at
// <= ~1 kHz); the row format is fixed-width CSV for easy ingestion.

#include <functional>
#include <fstream>
#include <string>
#include <vector>

namespace tdsim {

class TelemetryLogger {
public:
    // A named column. The getter is a std::function so callers can capture
    // spans/signals by reference.
    struct Column {
        std::string name;
        std::function<double()> get;
    };

    TelemetryLogger() = default;

    // Opens the CSV for writing. The file is opened with trunc semantics and
    // closed (flushed) automatically when the logger is destroyed, so the
    // caller does not need to remember a close() on every exit path — in
    // particular not on exceptions unwinding out of the driver loop.
    TelemetryLogger(std::vector<Column> columns, std::string path);

    // Move-only: the logger owns the open file handle.
    TelemetryLogger(TelemetryLogger&&) noexcept = default;
    TelemetryLogger& operator=(TelemetryLogger&&) noexcept = default;
    TelemetryLogger(const TelemetryLogger&) = delete;
    TelemetryLogger& operator=(const TelemetryLogger&) = delete;

    ~TelemetryLogger() { close(); }

    void record(double t);

    std::size_t num_columns() const { return columns_.size() + 1; } // +t

private:
    void close();

    std::vector<Column> columns_;
    std::ofstream out_;
    bool header_written_ = false;
};

} // namespace tdsim
