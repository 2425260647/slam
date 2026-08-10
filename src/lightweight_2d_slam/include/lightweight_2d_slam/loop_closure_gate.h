#pragma once

#include <cstddef>
#include <map>
#include <set>
#include <utility>

#include "lightweight_2d_slam/types.h"

namespace lightweight_2d_slam {

struct LoopClosureGateOptions {
  int min_confirmations = 2;
  double confirmation_translation_tolerance = 0.15;
  double confirmation_rotation_tolerance = 0.12;
  int min_keyframes_between_accepts = 30;
};

struct LoopClosureProposal {
  std::size_t candidate_id = 0;
  std::size_t current_id = 0;
  std::size_t candidate_submap = 0;
  std::size_t current_submap = 0;
  Pose2D correction;
};

class LoopClosureGate {
 public:
  explicit LoopClosureGate(const LoopClosureGateOptions& options)
      : options_(options) {}

  bool IsPairAccepted(std::size_t candidate_submap,
                      std::size_t current_submap) const {
    return accepted_pairs_.count({candidate_submap, current_submap}) != 0;
  }

  bool Consider(const LoopClosureProposal& proposal) {
    const auto pair =
        std::make_pair(proposal.candidate_submap, proposal.current_submap);
    if (accepted_pairs_.count(pair) != 0) return false;
    if (has_accepted_loop_ &&
        proposal.current_id <
            last_accepted_current_id_ + static_cast<std::size_t>(
                                            options_.min_keyframes_between_accepts)) {
      ResetPending(proposal);
      return false;
    }

    PendingProposal& pending = pending_[pair];
    const bool confirms_pending =
        pending.valid && proposal.current_id > pending.current_id &&
        TranslationDistance(pending.correction, proposal.correction) <=
            options_.confirmation_translation_tolerance &&
        std::abs(NormalizeAngle(pending.correction.yaw -
                                proposal.correction.yaw)) <=
            options_.confirmation_rotation_tolerance;
    if (!confirms_pending) {
      ResetPending(proposal);
      return options_.min_confirmations <= 1 && AcceptPending(pair);
    }

    pending.current_id = proposal.current_id;
    pending.candidate_id = proposal.candidate_id;
    pending.correction = proposal.correction;
    ++pending.confirmations;
    if (pending.confirmations < options_.min_confirmations) return false;
    return AcceptPending(pair);
  }

 private:
  struct PendingProposal {
    bool valid = false;
    std::size_t candidate_id = 0;
    std::size_t current_id = 0;
    std::size_t candidate_submap = 0;
    std::size_t current_submap = 0;
    Pose2D correction;
    int confirmations = 0;
  };

  void ResetPending(const LoopClosureProposal& proposal) {
    PendingProposal& pending = pending_[
        {proposal.candidate_submap, proposal.current_submap}];
    pending.valid = true;
    pending.candidate_id = proposal.candidate_id;
    pending.current_id = proposal.current_id;
    pending.candidate_submap = proposal.candidate_submap;
    pending.current_submap = proposal.current_submap;
    pending.correction = proposal.correction;
    pending.confirmations = 1;
  }

  bool AcceptPending(
      const std::pair<std::size_t, std::size_t>& submap_pair) {
    accepted_pairs_.insert(submap_pair);
    has_accepted_loop_ = true;
    last_accepted_current_id_ = pending_.at(submap_pair).current_id;
    pending_.erase(submap_pair);
    return true;
  }

  LoopClosureGateOptions options_;
  std::map<std::pair<std::size_t, std::size_t>, PendingProposal> pending_;
  std::set<std::pair<std::size_t, std::size_t>> accepted_pairs_;
  bool has_accepted_loop_ = false;
  std::size_t last_accepted_current_id_ = 0;
};

}  // namespace lightweight_2d_slam
