#pragma once

#include <optional>
#include <string>
#include <utility>

namespace quest_newton {

enum class StatusCode {
  ok,
  invalid_argument,
  artifact_error,
  version_mismatch,
  symbol_missing,
  graph_error,
  non_finite_state,
};

struct Status {
  StatusCode code = StatusCode::ok;
  std::string message;

  [[nodiscard]] bool IsOk() const noexcept { return code == StatusCode::ok; }

  static Status Ok() { return {}; }
};

template <typename Value>
class StatusOr {
 public:
  StatusOr(Status status) : status_(std::move(status)) {}
  StatusOr(Value value) : value_(std::move(value)) {}

  [[nodiscard]] bool IsOk() const noexcept { return status_.IsOk(); }
  [[nodiscard]] const Status& status() const noexcept { return status_; }
  [[nodiscard]] Value& value() & { return *value_; }
  [[nodiscard]] const Value& value() const& { return *value_; }
  [[nodiscard]] Value&& value() && { return std::move(*value_); }

 private:
  Status status_;
  std::optional<Value> value_;
};

}  // namespace quest_newton
