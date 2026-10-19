// td_sim3 — config.cpp
//
// TOML -> VehicleSpec loader. Strict validation: unknown top-level sections,
// unknown sim_params keys, missing required fields, and wrong value types are
// all hard errors that name the offending file/line. This load step is the
// config's "compilation" — anything that reaches the factory is structurally
// valid TOML-shaped input (semantic checks like unknown model types, unknown
// signal ports, and unknown parameters are the factory's job).
//
// Built against toml++ 3.4.0: the public namespace is `toml` (the v3 inline
// abi namespace is transparent), `toml::parse()` returns a `parse_result`
// (succeeded/failed + table()/error()), node accessors are as_table()/
// as_array()/as_string()/as_integer()/as_floating_point(), and table
// iteration yields (const toml::key&, const toml::node&) pairs.

#include "tdsim/spec.hpp"

#include <toml++/toml.hpp>

#include <spdlog/spdlog.h>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace tdsim {

namespace {

// Human-readable name for a toml node type (used in error messages).
std::string type_name(const toml::node& n) {
    switch (n.type()) {
        case toml::node_type::table: return "table";
        case toml::node_type::array: return "array";
        case toml::node_type::string: return "string";
        case toml::node_type::integer: return "integer";
        case toml::node_type::floating_point: return "float";
        case toml::node_type::boolean: return "boolean";
        case toml::node_type::date: return "date";
        case toml::node_type::time: return "time";
        case toml::node_type::date_time: return "date-time";
        default: return "none";
    }
}

// Per-file error context: attaches "<file>:<line>:<col>" to every validation
// error. source_position is 1-based.
struct Ctx {
    const std::string& file;
    explicit Ctx(const std::string& f) : file(f) {}
    [[noreturn]] void fail(const toml::node& n, const std::string& msg) const {
        const auto src = n.source().begin;
        const std::string loc =
            (src.line > 0)
                ? std::to_string(src.line) + ":" + std::to_string(src.column)
                : "?";
        throw std::runtime_error(file + ":" + loc + ": " + msg);
    }
};

// Strict number: node must be an integer or floating point; else fail.
double require_double(const toml::node& n, const std::string& what, const Ctx& ctx) {
    if (const auto* d = n.as_floating_point()) return d->get();
    if (const auto* i = n.as_integer()) return static_cast<double>(i->get());
    ctx.fail(n, "expected a number for " + what + " (got " + type_name(n) + ")");
}

// Strict string: node must be a string; else fail.
std::string require_string(const toml::node& n, const std::string& what, const Ctx& ctx) {
    if (const auto* s = n.as_string()) return s->get();
    ctx.fail(n, "expected a string for " + what + " (got " + type_name(n) + ")");
}

// Strict array of strings: node must be an array whose elements are strings;
// else fail (what names the field for the error).
std::vector<std::string> require_string_array(const toml::node& n, const std::string& what,
                                              const Ctx& ctx) {
    const auto* arr = n.as_array();
    if (!arr) {
        ctx.fail(n, "expected an array of strings for " + what + " (got " + type_name(n) + ")");
    }
    std::vector<std::string> out;
    for (const auto& e : *arr) {
        const auto* s = e.as_string();
        if (!s) ctx.fail(e, "expected a string element in " + what + " (got " + type_name(e) + ")");
        out.push_back(s->get());
    }
    return out;
}

// Strict table of numbers: node must be a table whose values are numbers; else fail.
std::map<std::string, double> require_double_table(const toml::node& n, const std::string& what,
                                                   const Ctx& ctx) {
    const auto* tbl = n.as_table();
    if (!tbl) {
        ctx.fail(n, "expected a table of numbers for " + what + " (got " + type_name(n) + ")");
    }
    std::map<std::string, double> out;
    for (const auto& [k, v] : *tbl) {
        if (const auto* d = v.as_floating_point()) out.emplace(std::string(k), d->get());
        else if (const auto* i = v.as_integer()) out.emplace(std::string(k), static_cast<double>(i->get()));
        else ctx.fail(v, "expected a number for " + what + "." + std::string(k) +
                             " (got " + type_name(v) + ")");
    }
    return out;
}

// table::at() returns a node; unwrap it as a table (failing via ctx).
const toml::table& tbl_at(const toml::table& tbl, const char* key, const Ctx& ctx) {
    const auto* t = tbl.at(key).as_table();
    if (!t) ctx.fail(tbl.at(key), "expected '" + std::string(key) + "' to be a table");
    return *t;
}

const toml::array& arr_at(const toml::table& tbl, const char* key, const Ctx& ctx) {
    const auto* a = tbl.at(key).as_array();
    if (!a) ctx.fail(tbl.at(key), "expected '" + std::string(key) + "' to be an array");
    return *a;
}

SignalKind parse_kind(const toml::node& n, const Ctx& ctx) {
    auto s = require_string(n, "kind", ctx);
    if (s == "Signal") return SignalKind::Signal;
    if (s == "Accumulator") return SignalKind::Accumulator;
    if (s == "Latched") return SignalKind::Latched;
    ctx.fail(n, "unknown signal kind '" + s + "' (expected Signal, Accumulator, or Latched)");
}

SimParams parse_sim_params(const toml::table& tbl, const Ctx& ctx) {
    SimParams p;
    if (!tbl.contains("t_end")) {
        ctx.fail(tbl, "missing required key 't_end' in [sim_params]");
    }
    p.t_end = require_double(tbl.at("t_end"), "sim_params.t_end", ctx);
    if (p.t_end <= 0.0) ctx.fail(tbl.at("t_end"), "sim_params.t_end must be > 0");
    if (tbl.contains("dt_macro")) {
        p.dt_macro = require_double(tbl.at("dt_macro"), "sim_params.dt_macro", ctx);
        if (p.dt_macro <= 0.0) ctx.fail(tbl.at("dt_macro"), "sim_params.dt_macro must be > 0");
    }
    if (tbl.contains("dt_min")) {
        p.dt_min = require_double(tbl.at("dt_min"), "sim_params.dt_min", ctx);
        if (p.dt_min <= 0.0) ctx.fail(tbl.at("dt_min"), "sim_params.dt_min must be > 0");
    }
    if (tbl.contains("dt_max")) {
        p.dt_max = require_double(tbl.at("dt_max"), "sim_params.dt_max", ctx);
        if (p.dt_max <= 0.0) ctx.fail(tbl.at("dt_max"), "sim_params.dt_max must be > 0");
    }
    if (p.dt_max > p.dt_macro) ctx.fail(tbl.at("dt_max"), "sim_params.dt_max must be <= dt_macro");
    if (tbl.contains("eps_abs")) {
        p.eps_abs = require_double(tbl.at("eps_abs"), "sim_params.eps_abs", ctx);
        if (p.eps_abs <= 0.0) ctx.fail(tbl.at("eps_abs"), "sim_params.eps_abs must be > 0");
    }
    if (tbl.contains("eps_rel")) {
        p.eps_rel = require_double(tbl.at("eps_rel"), "sim_params.eps_rel", ctx);
        if (p.eps_rel <= 0.0) ctx.fail(tbl.at("eps_rel"), "sim_params.eps_rel must be > 0");
    }
    if (tbl.contains("telemetry_rate")) {
        p.telemetry_rate = require_double(tbl.at("telemetry_rate"), "sim_params.telemetry_rate", ctx);
        if (p.telemetry_rate < 0.0) ctx.fail(tbl.at("telemetry_rate"), "sim_params.telemetry_rate must be >= 0");
    }
    if (tbl.contains("telemetry_file")) {
        p.telemetry_file = require_string(tbl.at("telemetry_file"), "sim_params.telemetry_file", ctx);
    }
    static const std::vector<std::string> known = {
        "t_end", "dt_macro", "dt_min", "dt_max", "eps_abs", "eps_rel",
        "telemetry_rate", "telemetry_file"};
    for (const auto& [k, v] : tbl) {
        bool ok = false;
        for (const auto& kn : known) if (k.str() == kn) { ok = true; break; }
        if (!ok) ctx.fail(v, "unknown key '" + std::string(k) + "' in [sim_params]");
    }
    return p;
}

std::vector<ModelSpec> parse_models(const toml::table& tbl, const Ctx& ctx) {
    std::vector<ModelSpec> out;
    for (const auto& [id, entry] : tbl) {
        const auto* et = entry.as_table();
        if (!et) ctx.fail(entry, "[models." + std::string(id) + "] must be a table");
        ModelSpec ms;
        ms.id = std::string(id);
        const std::string what = "[models." + ms.id + "]";
        if (!et->contains("type")) ctx.fail(entry, what + ": missing required key 'type'");
        ms.type = require_string(et->at("type"), what + " type", ctx);
        if (et->contains("stage")) {
            ms.stage = require_string(et->at("stage"), what + " stage", ctx);
        }
        for (const auto& [k, v] : *et) {
            if (k.str() == "type" || k.str() == "stage") continue;
            if (const auto* dd = v.as_floating_point()) ms.params.emplace(std::string(k), dd->get());
            else if (const auto* ii = v.as_integer()) ms.params.emplace(std::string(k), static_cast<double>(ii->get()));
            else ctx.fail(v, what + " param '" + std::string(k) + "must be a number (got " + type_name(v) + ")");
        }
        out.push_back(std::move(ms));
    }
    return out;
}

std::vector<ConnectionSpec> parse_connections(const toml::array& arr, const Ctx& ctx) {
    std::vector<ConnectionSpec> out;
    std::size_t i = 0;
    for (const auto& entry : arr) {
        ++i;
        const auto* et = entry.as_table();
        if (!et) ctx.fail(entry, "[[connections]] entry #" + std::to_string(i) + " must be a table");
        ConnectionSpec cs;
        const std::string what = "[[connections]] entry #" + std::to_string(i);
        if (!et->contains("signal")) ctx.fail(entry, what + ": missing required key 'signal'");
        cs.signal = require_string(et->at("signal"), what + " signal", ctx);
        if (!et->contains("kind")) ctx.fail(entry, what + ": missing required key 'kind'");
        cs.kind = parse_kind(et->at("kind"), ctx);
        if (!et->contains("producers")) ctx.fail(entry, what + ": missing required key 'producers'");
        cs.producers = require_string_array(et->at("producers"), what + " producers", ctx);
        if (!et->contains("consumers")) ctx.fail(entry, what + ": missing required key 'consumers'");
        cs.consumers = require_string_array(et->at("consumers"), what + " consumers", ctx);
        // Optional: consumer-side port name (defaults to the producer port name).
        if (et->contains("to")) {
            cs.to_port = require_string(et->at("to"), what + " to", ctx);
        }
        for (const auto& [k, v] : *et) {
            const auto ks = k.str();
            if (ks != "signal" && ks != "kind" && ks != "producers" &&
                ks != "consumers" && ks != "to") {
                ctx.fail(v, what + ": unknown key '" + std::string(ks) + "'");
            }
        }
        out.push_back(std::move(cs));
    }
    return out;
}

std::vector<EventSpec> parse_events(const toml::table& tbl, const Ctx& ctx) {
    std::vector<EventSpec> out;
    for (const auto& [id, entry] : tbl) {
        const auto* et = entry.as_table();
        if (!et) ctx.fail(entry, "[events." + std::string(id) + "] must be a table");
        EventSpec es;
        es.id = std::string(id);
        const std::string what = "[events." + es.id + "]";
        if (!et->contains("type")) ctx.fail(entry, what + ": missing required key 'type'");
        es.type = require_string(et->at("type"), what + " type", ctx);
        if (!et->contains("stage")) ctx.fail(entry, what + ": missing required key 'stage'");
        es.stage = require_string(et->at("stage"), what + " stage", ctx);
        if (es.type == "fixed_time") {
            if (!et->contains("t")) ctx.fail(entry, what + ": fixed_time trigger requires key 't'");
            es.t = require_double(et->at("t"), what + " t", ctx);
            if (es.t < 0.0) ctx.fail(et->at("t"), what + ": t must be >= 0");
        } else {
            ctx.fail(et->at("type"), what + ": unknown event type '" + es.type +
                                          "' (supported: fixed_time)");
        }
        for (const auto& [k, v] : *et) {
            const auto ks = k.str();
            if (ks != "type" && ks != "stage" && ks != "t") {
                ctx.fail(v, what + ": unknown key '" + std::string(ks) + "'");
            }
        }
        out.push_back(std::move(es));
    }
    return out;
}

FlightComputerSpec parse_flight_computer(const toml::table& et, const Ctx& ctx) {
    const std::string what = "[flight_computer]";
    FlightComputerSpec fc;
    if (et.contains("model")) fc.model = require_string(et.at("model"), what + " model", ctx);
    if (et.contains("software")) fc.software = require_string(et.at("software"), what + " software", ctx);
    if (et.contains("sensors")) fc.sensors = require_string_array(et.at("sensors"), what + " sensors", ctx);
    if (et.contains("commands")) fc.commands = require_string_array(et.at("commands"), what + " commands", ctx);
    if (et.contains("params")) fc.params = require_double_table(et.at("params"), what + " params", ctx);
    for (const auto& [k, v] : et) {
        const auto ks = k.str();
        if (ks != "model" && ks != "software" && ks != "sensors" && ks != "commands" && ks != "params") {
            ctx.fail(v, what + ": unknown key '" + std::string(ks) + "'");
        }
    }
    return fc;
}

} // namespace

VehicleSpec parse_spec(const std::string& toml_text, const std::string& source_name) {
    toml::table doc;
    try {
        // With exceptions enabled (the default) toml::parse_result IS toml::table
        // and a malformed document throws toml::parse_error.
        doc = toml::parse(toml_text, source_name);
    } catch (const toml::parse_error& e) {
        throw std::runtime_error(source_name + ": TOML parse error: " +
                                 std::string(e.description()));
    }

    Ctx ctx{source_name};
    VehicleSpec spec;

    // Strict top-level section whitelist.
    static const std::vector<std::string> known_sections = {
        "sim_params", "models", "connections", "events", "flight_computer"};
    for (const auto& [k, v] : doc) {
        bool ok = false;
        for (const auto& ks : known_sections) if (k.str() == ks) { ok = true; break; }
        if (!ok) ctx.fail(v, "unknown top-level section '" + std::string(k) + "'");
    }

    if (!doc.contains("sim_params")) {
        ctx.fail(doc, "missing required [sim_params] table");
    }
    spec.sim = parse_sim_params(tbl_at(doc, "sim_params", ctx), ctx);

    if (doc.contains("models")) {
        spec.models = parse_models(tbl_at(doc, "models", ctx), ctx);
    }
    if (spec.models.empty()) ctx.fail(doc, "no models declared in [models]");

    if (doc.contains("connections")) {
        spec.connections = parse_connections(arr_at(doc, "connections", ctx), ctx);
    }

    if (doc.contains("events")) {
        spec.events = parse_events(tbl_at(doc, "events", ctx), ctx);
    }

    if (doc.contains("flight_computer")) {
        spec.flight_computer = parse_flight_computer(tbl_at(doc, "flight_computer", ctx), ctx);
    }

    return spec;
}

VehicleSpec load_spec(const std::string& file_path) {
    std::ifstream in(file_path);
    if (!in) {
        throw std::runtime_error("Config Error: cannot open file '" + file_path + "'");
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return parse_spec(ss.str(), file_path);
}

} // namespace tdsim
