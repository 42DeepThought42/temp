#pragma once
// td_sim3 — signal.hpp
//
// The three signal types are the ONLY channel through which models exchange
// algebraic (non-integrated) data. Their interfaces are intentionally distinct
// so that misuse is a compile error, not a runtime assertion:
//
//   Signal<T>      single producer, publish() once per Pass 2; read-only to all
//   Accumulator<T> any number of producers, add() only (+=); cleared in Pass 1
//   Latched<T>     single producer, propose() during Pass 2; consumers read the
//                  last COMMITTED value. commit() is called by the driver once
//                  per accepted macro-window boundary. Because reads never see
//                  'next', a rejected speculative step is a non-event: no
//                  discard hook is required.

#include <cstddef>
#include <string>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace tdsim {

enum class SignalKind { Signal, Accumulator, Latched };

// ---------------------------------------------------------------------------
// Flat typed slot storage. The framework allocates one contiguous block per
// type; each slot is a raw element in it. Models never see this class — they
// hold Signal<T>/Accumulator<T>/Latched<T> views handed to them by the builder.
// ---------------------------------------------------------------------------

class SlotStore {
public:
    explicit SlotStore(std::size_t capacity) : data_(capacity) {}
    // Returns the address of slot i, throwing if the store is exhausted.
    // (The framework allocates a fixed capacity up front; a vehicle config that
    // declares more ports than capacity would otherwise be a silent heap
    // overflow here.)
    void* slot(std::size_t i) {
        if (i >= data_.size()) {
            throw std::runtime_error("SlotStore exhausted: requested slot " +
                                     std::to_string(i) + " but capacity is " +
                                     std::to_string(data_.size()));
        }
        return data_.data() + i;
    }
    std::size_t size() const { return data_.size(); }

private:
    std::vector<double> data_; // 8 bytes per slot; every T we support is a double
};

// ---------------------------------------------------------------------------

template <typename T>
class Signal {
public:
    Signal() = default;
    Signal(const Signal&) = default;
    Signal& operator=(const Signal&) = default;

    // Producer side — exactly one model may hold the producer view.
    void publish(const T& v) { *slot_ = v; }

    // Consumer side — legal in Pass 2 (after the producer, per topology) or
    // Pass 3.
    const T& read() const { return *slot_; }

    const std::string& name() const { return name_; }

    // Framework-only wiring (builder uses these before the simulation starts).
    void bind(T* slot, std::string name) {
        slot_ = slot;
        name_ = std::move(name);
    }

private:
    T* slot_ = nullptr;
    std::string name_;
};

template <typename T>
class Accumulator {
public:
    Accumulator() = default;
    Accumulator(const Accumulator&) = default;
    Accumulator& operator=(const Accumulator&) = default;

    // Producer side — any number of models may hold a producer view. Order of
    // add() calls is mathematically irrelevant (commutative sum).
    void add(const T& v) { *slot_ += v; }

    // Consumer side — legal in Pass 2 only after ALL producers of this
    // accumulator have executed (enforced by builder-generated DAG edges),
    // or in Pass 3.
    const T& read() const { return *slot_; }

    // Framework-only: zeroed at the start of Pass 1, every system-functor call.
    void clear() { *slot_ = T{}; }

    const std::string& name() const { return name_; }

    void bind(T* slot, std::string name) {
        slot_ = slot;
        name_ = std::move(name);
    }

private:
    T* slot_ = nullptr;
    std::string name_;
};

template <typename T>
class Latched {
public:
    Latched() = default;
    Latched(const Latched&) = default;
    Latched& operator=(const Latched&) = default;

    // Producer side — exactly one model may hold the producer view.
    void propose(const T& v) { *next_ = v; }

    // Consumer side — reads the last committed value, i.e. the value that was
    // stable at the end of the previous accepted macro window. This is the
    // sanctioned way to break instantaneous algebraic loops (1-step delay).
    const T& read() const { return *current_; }

    // Framework-only lifecycle:
    //   init(v)      — called once from config `initial:` before t = 0
    //   commit()     — called by the driver when this latch's period is due
    //
    // Multi-rate cadence: period_ticks (in base-tick units) is the number of
    // base ticks between commits. 1 (the default) commits every base tick —
    // the original single-rate behavior. A latch with period_ticks = N commits
    // only on every Nth base tick, so its committed value is a zero-order hold
    // sampled at the base rate / N. The producer still propose()s every Pass 2
    // (it re-derives the value from the live ODE state each derivative call);
    // only the commit cadence is slowed, so the held value is the quantity
    // sampled fresh at each commit tick. The driver decides WHEN to call
    // commit() (k % period_ticks == 0); this type only stores the period.
    std::size_t period_ticks = 1;
    void init(const T& v) { *current_ = v; *next_ = v; }
    void commit() { *current_ = *next_; }

    const std::string& name() const { return name_; }

    void bind(T* current, T* next, std::string name) {
        current_ = current;
        next_ = next;
        name_ = std::move(name);
    }

private:
    T* current_ = nullptr;
    T* next_ = nullptr;
    std::string name_;
};

} // namespace tdsim
