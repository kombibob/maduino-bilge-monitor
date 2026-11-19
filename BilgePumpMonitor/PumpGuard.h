// PumpGuard.h
// Bilge pump run-time guard for the Maduino bilge monitor.
//
// Pure logic with no Arduino dependencies, so it can be unit-tested on a PC.
// The sketch feeds it raw inputs every loop and applies cut() to the K1
// interrupt relay (K1 energised = pump held off).
//
// Inputs (all passed in as "true = active"):
//   demand : Water Witch is calling for the pump (K2 closed)
//   feed   : voltage present on the pump feed (PC817 module output LOW)
//
// Behaviour:
//   IDLE     no demand.
//   RUNNING  demand present and the pump is allowed to run. If this lasts
//            longer than runLimitMs the guard trips (enforce mode) or just
//            reports that it would have (log-only mode).
//   LOCKOUT  K1 energised, pump held off for an escalating time (10/30/60/120
//            min by default). Demand clearing during lockout releases the pump
//            at once. When the lockout ends the pump gets another try.
//
// Independent fault checks (reported in both modes):
//   FEED_DEAD        demand present, pump not cut, but no voltage at the pump
//   LIVE_WHILE_CUT   pump cut but voltage still at the pump (K1/R1 stuck, or
//                    the manual override is on)
//   LIVE_NO_DEMAND   no demand, not cut, but voltage at the pump (override on
//                    or R1 stuck closed)
//
// All time maths uses unsigned subtraction, so millis() rollover is safe.

#ifndef PUMP_GUARD_H
#define PUMP_GUARD_H

#include <stdint.h>

class PumpGuard {
public:
  enum State : uint8_t { IDLE = 0, RUNNING = 1, LOCKOUT = 2 };

  enum EventType : uint8_t {
    EV_NONE = 0,
    EV_TRIP,                  // a = run ms, b = lockout ms, n = trip number
    EV_WOULD_TRIP,            // log-only: a = run ms
    EV_RETRY,                 // lockout ended, pump released: n = trip number
    EV_CLEARED,               // run finished normally after trips: a = run ms, n = trips
    EV_CLEARED_EARLY,         // demand cleared during lockout: n = trips
    EV_FAULT_FEED_DEAD,       // a = 1 if this is a reminder
    EV_FAULT_LIVE_WHILE_CUT,  // a = 1 if this is a reminder
    EV_FAULT_LIVE_NO_DEMAND,  // a = 1 if this is a reminder
    EV_FAULT_RECOVERED        // n = EventType of the fault that cleared
  };

  struct Event {
    EventType type;
    uint32_t  a;
    uint32_t  b;
    uint8_t   n;
  };

  static const uint8_t MAX_LOCKOUTS = 6;

  struct Config {
    uint32_t runLimitMs;
    uint32_t lockoutMs[MAX_LOCKOUTS];
    uint8_t  lockoutCount;
    float    lowBattV;        // below this, trips jump to the longest lockout
    uint32_t debounceMs;
    uint32_t faultConfirmMs;
    uint32_t faultRemindMs;
    bool     enforce;         // false = log-only, K1 is never energised
  };

  static Config defaultConfig() {
    Config c;
    c.runLimitMs     = 180UL * 1000UL;
    c.lockoutMs[0]   = 10UL * 60UL * 1000UL;
    c.lockoutMs[1]   = 30UL * 60UL * 1000UL;
    c.lockoutMs[2]   = 60UL * 60UL * 1000UL;
    c.lockoutMs[3]   = 120UL * 60UL * 1000UL;
    c.lockoutMs[4]   = 0;
    c.lockoutMs[5]   = 0;
    c.lockoutCount   = 4;
    c.lowBattV       = 11.8f;
    c.debounceMs     = 250;
    c.faultConfirmMs = 10UL * 1000UL;
    c.faultRemindMs  = 6UL * 60UL * 60UL * 1000UL;
    c.enforce        = false;
    return c;
  }

  PumpGuard() : cfg_(defaultConfig()) { begin(0, false, false); }

  void configure(const Config& c) {
    cfg_ = c;
    if (cfg_.lockoutCount < 1) cfg_.lockoutCount = 1;
    if (cfg_.lockoutCount > MAX_LOCKOUTS) cfg_.lockoutCount = MAX_LOCKOUTS;
  }

  // Call once after configure(), passing the current raw input states.
  void begin(uint32_t now, bool demandRaw, bool feedRaw) {
    dDemand_.init(demandRaw, now);
    dFeed_.init(feedRaw, now);
    demand_ = demandRaw;
    feed_ = feedRaw;
    state_ = demand_ ? RUNNING : IDLE;
    runStart_ = now;
    lockoutStart_ = 0;
    lockoutDur_ = 0;
    cut_ = false;
    tripCount_ = 0;
    wouldTrip_ = false;
    lastRunMs_ = 0;
    maxRunMs_ = 0;
    qHead_ = 0;
    qCount_ = 0;
    fDead_.init();
    fCut_.init();
    fNoDemand_.init();
  }

  // Call every loop. battV <= 0 means "unknown" and is ignored.
  void update(uint32_t now, bool demandRaw, bool feedRaw, float battV) {
    demand_ = dDemand_.update(demandRaw, now, cfg_.debounceMs);
    feed_   = dFeed_.update(feedRaw, now, cfg_.debounceMs);

    // Enforcement switched off while holding the pump: release immediately.
    if (state_ == LOCKOUT && !cfg_.enforce) {
      cut_ = false;
      state_ = demand_ ? RUNNING : IDLE;
      runStart_ = now;
      wouldTrip_ = false;
    }

    switch (state_) {
      case IDLE:
        if (demand_) {
          state_ = RUNNING;
          runStart_ = now;
          wouldTrip_ = false;
        }
        break;

      case RUNNING: {
        uint32_t runMs = now - runStart_;
        if (!demand_) {
          lastRunMs_ = runMs;
          if (runMs > maxRunMs_) maxRunMs_ = runMs;
          if (tripCount_ > 0) {
            push(EV_CLEARED, runMs, 0, tripCount_);
            tripCount_ = 0;
          }
          state_ = IDLE;
        } else if (runMs >= cfg_.runLimitMs) {
          if (cfg_.enforce) {
            trip(now, runMs, battV);
          } else if (!wouldTrip_) {
            wouldTrip_ = true;
            push(EV_WOULD_TRIP, runMs, 0, 0);
          }
        }
        break;
      }

      case LOCKOUT:
        if (!demand_) {
          cut_ = false;
          state_ = IDLE;
          push(EV_CLEARED_EARLY, 0, 0, tripCount_);
          tripCount_ = 0;
        } else if ((uint32_t)(now - lockoutStart_) >= lockoutDur_) {
          cut_ = false;
          state_ = RUNNING;
          runStart_ = now;
          wouldTrip_ = false;
          push(EV_RETRY, 0, 0, tripCount_);
        }
        break;
    }

    faultCheck(fDead_,     !cut_ && demand_ && !feed_,  EV_FAULT_FEED_DEAD,      now);
    faultCheck(fCut_,      cut_ && feed_,               EV_FAULT_LIVE_WHILE_CUT, now);
    faultCheck(fNoDemand_, !cut_ && !demand_ && feed_,  EV_FAULT_LIVE_NO_DEMAND, now);
  }

  // Release the pump and forget the trip history (SMS "GUARD CLEAR").
  void clear(uint32_t now) {
    cut_ = false;
    tripCount_ = 0;
    wouldTrip_ = false;
    state_ = demand_ ? RUNNING : IDLE;
    runStart_ = now;
  }

  bool popEvent(Event& e) {
    if (qCount_ == 0) return false;
    e = q_[qHead_];
    qHead_ = (uint8_t)((qHead_ + 1) % QUEUE_SIZE);
    qCount_--;
    return true;
  }

  // Runtime setters (not persisted across a reboot)
  void setEnforce(bool on) { cfg_.enforce = on; }
  void setRunLimitMs(uint32_t ms) { cfg_.runLimitMs = ms; }

  // Getters
  State    state() const { return state_; }
  bool     cut() const { return cut_; }
  bool     demand() const { return demand_; }
  bool     feed() const { return feed_; }
  bool     enforce() const { return cfg_.enforce; }
  uint32_t runLimitMs() const { return cfg_.runLimitMs; }
  uint8_t  tripCount() const { return tripCount_; }
  uint32_t lastRunMs() const { return lastRunMs_; }
  uint32_t maxRunMs() const { return maxRunMs_; }
  uint32_t runElapsedMs(uint32_t now) const {
    return state_ == RUNNING ? (uint32_t)(now - runStart_) : 0;
  }
  uint32_t lockoutRemainingMs(uint32_t now) const {
    if (state_ != LOCKOUT) return 0;
    uint32_t used = now - lockoutStart_;
    return used >= lockoutDur_ ? 0 : lockoutDur_ - used;
  }
  // bit0 feed dead, bit1 live while cut, bit2 live with no demand
  uint8_t faultMask() const {
    return (fDead_.active ? 1 : 0) | (fCut_.active ? 2 : 0) | (fNoDemand_.active ? 4 : 0);
  }

private:
  static const uint8_t QUEUE_SIZE = 8;

  struct Debounce {
    bool stable, candidate;
    uint32_t since;
    void init(bool v, uint32_t now) { stable = candidate = v; since = now; }
    bool update(bool raw, uint32_t now, uint32_t holdMs) {
      if (raw != candidate) {
        candidate = raw;
        since = now;
      } else if (candidate != stable && (uint32_t)(now - since) >= holdMs) {
        stable = candidate;
      }
      return stable;
    }
  };

  struct FaultTrack {
    bool pending, active;
    uint32_t since, lastNotify;
    void init() { pending = active = false; since = lastNotify = 0; }
  };

  void trip(uint32_t now, uint32_t runMs, float battV) {
    if (tripCount_ < 255) tripCount_++;
    uint8_t last = cfg_.lockoutCount - 1;
    uint8_t idx = tripCount_ - 1;
    if (idx > last) idx = last;
    if (battV > 0.0f && battV < cfg_.lowBattV) idx = last;  // protect the battery
    lockoutDur_ = cfg_.lockoutMs[idx];
    lockoutStart_ = now;
    cut_ = true;
    state_ = LOCKOUT;
    push(EV_TRIP, runMs, lockoutDur_, tripCount_);
  }

  void faultCheck(FaultTrack& f, bool cond, EventType ev, uint32_t now) {
    if (cond) {
      if (!f.pending) {
        f.pending = true;
        f.since = now;
      }
      if (!f.active) {
        if ((uint32_t)(now - f.since) >= cfg_.faultConfirmMs) {
          f.active = true;
          f.lastNotify = now;
          push(ev, 0, 0, 0);
        }
      } else if ((uint32_t)(now - f.lastNotify) >= cfg_.faultRemindMs) {
        f.lastNotify = now;
        push(ev, 1, 0, 0);
      }
    } else {
      if (f.active) push(EV_FAULT_RECOVERED, 0, 0, (uint8_t)ev);
      f.pending = false;
      f.active = false;
    }
  }

  void push(EventType t, uint32_t a, uint32_t b, uint8_t n) {
    if (qCount_ >= QUEUE_SIZE) return;  // drop if the sketch is not draining
    uint8_t tail = (uint8_t)((qHead_ + qCount_) % QUEUE_SIZE);
    q_[tail].type = t;
    q_[tail].a = a;
    q_[tail].b = b;
    q_[tail].n = n;
    qCount_++;
  }

  Config cfg_;
  Debounce dDemand_, dFeed_;
  FaultTrack fDead_, fCut_, fNoDemand_;
  State state_;
  bool demand_, feed_, cut_, wouldTrip_;
  uint32_t runStart_, lockoutStart_, lockoutDur_;
  uint32_t lastRunMs_, maxRunMs_;
  uint8_t tripCount_;
  Event q_[QUEUE_SIZE];
  uint8_t qHead_, qCount_;
};

#endif
