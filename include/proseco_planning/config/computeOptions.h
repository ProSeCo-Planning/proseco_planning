/**
 * @file computeOptions.h
 * @brief This file defines the configuration related to the compute options.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <sys/types.h>
#include <string>

#include "nlohmann/json.hpp"
using json = nlohmann::json;

namespace proseco_planning::config {

/**
 * @brief The struct that contains the parameters for the similarity update, used for evaluating
 * similar actions of different action spaces.
 *
 */
struct SimilarityUpdate {
  /// The flag that indicates whether similarity update is enabled.
  const bool active;
  /// The value to control the size of the RBF kernel (larger values incorporate less actions,
  /// smaller values incorporate more actions).
  const float gamma;

  /**
   * @brief Constructs a new Similarity Update object.
   *
   * @param active The flag that indicates whether similarity update is enabled.
   * @param gamma The gamma to control the size of the RBF kernel.
   */
  SimilarityUpdate(bool active, float gamma) : active(active), gamma(gamma) {}

  json toJSON() const;

  static SimilarityUpdate fromJSON(const json& jSimilarityUpdate);
};

/**
 * @brief The struct that contains the parameters for the search guide, used for exploration.
 *
 */
struct SearchGuide {
  /// The number of samples to be drawn for the search guide calculation.
  const unsigned int n_samples;
  /// The type of the search guide.
  const std::string type;

  /**
   * @brief Constructs a new Search Guide object.
   *
   * @param n_samples The number of samples to be drawn for the search guide calculation.
   * @param type The type of the search guide.
   */
  SearchGuide(unsigned int n_samples, std::string type) : n_samples(n_samples), type(type) {}

  json toJSON() const;

  static SearchGuide fromJSON(const json& jSearchGuide);
};

/**
 * @brief The struct that contains the parameters responsible for move grouping in progressive
 * widening.
 */
struct MoveGroupingCriteriaPW {
  /// The flag that indicates whether progressive widening is based on move groups (i.e. true:
  /// action class specific values are used for criteria, false: node specific values are used for
  /// criteria).
  const bool active;
  /// The progressive widening coefficient.
  const float coefficient_pw;
  /// The progressive widening exponent.
  const float exponent_pw;
  /**
   * @brief Constructs a new Move Grouping Criteria PW object.
   *
   * @param active The flag that indicates whether progressive widening is based on move groups.
   * @param coefficient_pw The progressive widening coefficient.
   * @param exponent_pw The progressive widening exponent.
   */
  MoveGroupingCriteriaPW(bool active, float coefficient_pw, float exponent_pw)
      : active(active), coefficient_pw(coefficient_pw), exponent_pw(exponent_pw) {}

  json toJSON() const;

  static MoveGroupingCriteriaPW fromJSON(const json& jMoveGroupingCriteriaPW);
};

/**
 * @brief The struct that contains the the parameters for move grouping.
 *
 */
struct MoveGrouping {
  /// The flag that indicates whether move grouping is enabled.
  const bool active;
  /// The coefficient for the exploration term of UCT within move grouping.
  const float cp;
  /// The move grouping criteria used in progressive widening.
  const MoveGroupingCriteriaPW move_grouping_criteria_pw;
  /// The flag that indicates whether move grouping is used to bias sampling.
  const bool move_grouping_bias_pw;
  /// The flag that indicates whether move groups are used for the final selection policy.
  const bool final_decision;

  /**
   * @brief Constructs a new Move Grouping object.
   *
   * @param active The flag that indicates whether move grouping is enabled.
   * @param cp The coefficient for the exploration term of UCT within move grouping.
   * @param move_grouping_criteria_pw The move grouping criteria that determines the progressive
   * widening process.
   * @param move_grouping_bias_pw The flag that indicates whether move grouping is used to bias
   * sampling.
   * @param final_decision The flag that indicates whether move groups are used for the final
   * selection policy.
   */
  MoveGrouping(bool active, float cp, MoveGroupingCriteriaPW move_grouping_criteria_pw,
               bool move_grouping_bias_pw, bool final_decision)
      : active(active),
        cp(cp),
        move_grouping_criteria_pw(move_grouping_criteria_pw),
        move_grouping_bias_pw(move_grouping_bias_pw),
        final_decision(final_decision) {}

  json toJSON() const;

  static MoveGrouping fromJSON(const json& jMoveGrouping);
};

/**
 * @brief The struct that contains the the parameters for progressive widening in the MCTS.
 *
 */
struct ProgressiveWidening {
  /// The search depth up to which PW is being applied.
  const unsigned int max_depth_pw;
  /// The progressive widening exponent.
  const float exponent;
  /// The progressive widening coefficient.
  const float coefficient;

  /**
   * @brief Constructs a new Progressive Widening object.
   *
   * @param max_depth_pw The search depth up to which PW is being applied.
   * @param exponent The progressive widening exponent.
   * @param coefficient The progressive widening coefficient.
   */
  ProgressiveWidening(unsigned int max_depth_pw, float exponent, float coefficient)
      : max_depth_pw(max_depth_pw), exponent(exponent), coefficient(coefficient) {}

  json toJSON() const;

  static ProgressiveWidening fromJSON(const json& jProgressiveWidening);
};

/**
 * @brief The struct that contains the parameters for the parallelization of the MCTS.
 *
 */
struct ParallelizationOptions {
  /// The number of threads the root parallelization is run with.
  const unsigned int n_threads;
  /// The number of threads the leaf parallelization is run with.
  const unsigned int n_simulationThreads;
  /// The flag that indicates whether similarityVoting is active.
  const bool similarity_voting;
  /// The gamma for the similarity function within the root parallelization.
  const float similarity_gamma;
  /// The method for aggregating multiple simulation threads.
  const std::string simulation_aggregation;

  ParallelizationOptions(unsigned int n_threads, unsigned int n_simulationThreads,
                         bool similarity_voting, float similarity_gamma,
                         std::string simulation_aggregation)
      : n_threads(n_threads),
        n_simulationThreads(n_simulationThreads),
        similarity_voting(similarity_voting),
        similarity_gamma(similarity_gamma),
        simulation_aggregation(simulation_aggregation) {}

  json toJSON() const;

  static ParallelizationOptions fromJSON(const json& ParallelizationOptions);
};

/**
 * @brief The struct that contains the bundled information about the search in continous action
 * spaces necessary for the policy enhancement.
 *
 */
struct PolicyEnhancements {
  /// The struct storing the parameters for the similarity update.
  const SimilarityUpdate similarity_update;
  /// The struct storing the parameters for the search guide.
  const SearchGuide search_guide;
  /// The struct storing the parameters for the move grouping.
  const MoveGrouping move_grouping;
  /// The struct storing the parameters for the progressive widening.
  const ProgressiveWidening progressive_widening;
  /// The parameter that controls the the length of a trajectory in the execution phase.
  const float action_execution_fraction;
  /// The parameter that determines the sharpness of the sample-exp-q policy.
  const float q_scale;

  /**
   * @brief Constructs a new Policy Enhancements object.
   *
   * @param similarity_update
   * @param search_guide
   * @param move_grouping
   * @param progressive_widening
   * @param action_execution_fraction
   * @param q_scale
   */
  PolicyEnhancements(SimilarityUpdate similarity_update, SearchGuide search_guide,
                     MoveGrouping move_grouping, ProgressiveWidening progressive_widening,
                     float action_execution_fraction, float q_scale)
      : similarity_update(similarity_update),
        search_guide(search_guide),
        move_grouping(move_grouping),
        progressive_widening(progressive_widening),
        action_execution_fraction(action_execution_fraction),
        q_scale(q_scale) {}

  json toJSON() const;

  static PolicyEnhancements fromJSON(const json& jPolicyEnhancements);
};

/**
 * @brief The struct that contains the parameters for KernelRegressionLCB.
 *
 */
struct KernelRegressionLCB {
  /// The flag that indicates whether move grouping is enabled.
  bool move_grouping;

  /// The parameters for regression of actions.
  struct Action {
    /// The kernel variant.
    std::string kernel_variant;
    /// The gamma for the similarity/kernel function.
    float gamma;
    /// The coefficient for the exploration term.
    float cp;
  } action;

  /// The parameters for regression of action classes.
  struct ActionClass {
    /// The kernel variant.
    std::string kernel_variant;
    /// The gamma for the similarity/kernel function.
    float gamma;
    /// The coefficient for the exploration term.
    float cp;
  } action_class;

  json toJSON() const;

  static KernelRegressionLCB fromJSON(const json& jKernelRegressionLCB);
};

/**
 * @brief The struct that contains the parameters for the PolicyOptions.
 *
 */
struct PolicyOptions {
  /// The selection policy used by the MCTS.
  const std::string selection_policy;
  /// The expansion policy used by the MCTS.
  const std::string expansion_policy;
  /// The simulation policy used by the MCTS.
  const std::string simulation_Policy;
  /// The update policy used by the MCTS.
  const std::string update_policy;
  /// The final selection policy used by the MCTS (determines how the actual action that gets
  /// executed is chosen).
  const std::string final_selection_policy;
  /// The policy enhancement object.
  const PolicyEnhancements policy_enhancements;
  /// The parameters for FinalSelectionKernelRegressionLCB
  KernelRegressionLCB kernel_regression_lcb;

  /**
   * @brief Constructs a new Policy Options object.
   *
   * @param selection_policy
   * @param expansion_policy
   * @param simulation_Policy
   * @param update_policy
   * @param final_selection_policy
   * @param policy_enhancements
   */
  PolicyOptions(const std::string& selection_policy, const std::string& expansion_policy,
                const std::string& simulation_Policy, const std::string& update_policy,
                const std::string& final_selection_policy, PolicyEnhancements policy_enhancements)
      : selection_policy(selection_policy),
        expansion_policy(expansion_policy),
        simulation_Policy(simulation_Policy),
        update_policy(update_policy),
        final_selection_policy(final_selection_policy),
        policy_enhancements(policy_enhancements) {}

  json toJSON() const;

  static PolicyOptions fromJSON(const json& jPolicyOptions);
};

/**
 * @brief The struct that contains the parameters for generating noise in general.
 *
 */
struct Noise {
  /// The flag that indicates whether noise is enabled.
  const bool active;
  /// The mean of the noise.
  const float mean;
  /// The standard deviation of the noise.
  const float sigma;
  /**
   * @brief Constructs a new Noise object.
   *
   * @param active
   * @param mean
   * @param sigma
   */
  Noise(bool active, float mean, float sigma) : active(active), mean(mean), sigma(sigma) {}

  json toJSON() const;

  static Noise fromJSON(const json& jNoise);
};
/**
 * @brief The struct that contains the parameters for generating noise particularly for actions.
 *
 */
struct ActionNoise {
  /// The flag that indicates whether noise is enabled.
  const bool active;
  /// The mean of the noise for the y-coordinate.
  const float meanY;
  /// The standard deviation of the noise for the y-coordinate.
  const float sigmaY;
  /// The mean of the noise for the x-velocity.
  const float meanVx;
  /// The standard deviation of the noise for the x-velocity.
  const float sigmaVx;
  /**
   * @brief Constructs a new Action Noise object.
   *
   * @param active
   * @param meanY
   * @param sigmaY
   * @param meanVx
   * @param sigmaVx
   */
  ActionNoise(bool active, float meanY, float sigmaY, float meanVx, float sigmaVx)
      : active(active), meanY(meanY), sigmaY(sigmaY), meanVx(meanVx), sigmaVx(sigmaVx) {}

  json toJSON() const;

  static ActionNoise fromJSON(const json& jNoise);
};
/**
 * @brief The struct that contains the hyperparameters necessary for the computation of the MCTS.
 *
 */
struct ComputeOptions {
  /// The random seed used to generate identical results.
  const unsigned int random_seed;
  /// The number of iterations the MCTS is run for.
  const unsigned int n_iterations;
  /// The [s] maximum duration a scenario is allowed to take (i.e. ensuring that a scenario does not
  /// run forever independent of whether a desired state is reached); 0 means no maximum duration
  /// limit.
  const float max_scenario_duration;
  /// The maximum number of steps a scenario is allowed to take (aka episode length; 0 means no
  /// maximum step limit).
  const unsigned int max_scenario_steps;
  /// The [s] maximum duration a step is allowed to take (i.e. ensuring a certain planning frequency
  /// 0.1s => 10Hz); 0 means no maximum duration limit
  const float max_step_duration;
  /// The maximum depth the MCTS looks into the future (i.e. the planning horizon).
  const unsigned int max_search_depth;
  /// The maximum number of invalid actions (of a single agent) to be sampled when
  /// expanding/simulating a node.
  const unsigned int max_invalid_action_samples;
  /// The influence of future rewards on the current action/state-value.
  const float discount_factor;
  /// The time delta between collision checks.
  const float delta_t;
  /// The gravity of earth.
  static constexpr float gravity{9.807f};
  /// The threshold for comparison of floats.
  static constexpr float error_tolerance{1.0e-04};
  /** The initial UCT value for unexplored actions.
   * @note: This cannot be set to infinity or numeric_max() since the values are used for
   * calculations in the blindValue searchGuide. Setting max values will lead to overflow and
   * undefined behavior when summing UCT values there.
   */
  static constexpr float initial_uct{1.0e05};
  /// The duration of an action.
  const float action_duration;
  /// The type of collision checker.
  const std::string collision_checker;
  /// The safety distance used for collision checking.
  const float safety_distance;
  /// The type of end condition.
  const std::string end_condition;
  /// The policy options.
  const PolicyOptions policy_options;
  /// The parallel MCTS options.
  const ParallelizationOptions parallelization_options;
  /// The trajectory type.
  const std::string trajectory_type;
  /// The UCT cp.
  const float uct_cp;
  /// The noise added to agent position.
  const Noise noise;
  /// The noise applied to selected actions.
  const ActionNoise action_noise;
  /// if > 0, 1) this value acts as the maximum viewing distance & 2) the mcts is performed for
  /// every agent separately instead of in a single (centralized) node tree.
  const float region_of_interest;
  /**
   * @brief Constructs a new Compute Options object.
   *
   * @param random_seed
   * @param n_iterations
   * @param max_scenario_duration
   * @param max_scenario_steps
   * @param max_step_duration
   * @param max_search_depth
   * @param max_invalid_action_samples
   * @param discount_factor
   * @param delta_t
   * @param action_duration
   * @param collision_checker
   * @param safety_distance
   * @param end_condition
   * @param policy_options
   * @param parallelization_options
   * @param trajectory_generation
   * @param uct_cp
   * @param noise
   * @param action_noise
   * @param region_of_interest
   */
  ComputeOptions(unsigned int random_seed, unsigned int n_iterations, float max_scenario_duration,
                 unsigned int max_scenario_steps, float max_step_duration,
                 unsigned int max_search_depth, unsigned int max_invalid_action_samples,
                 float discount_factor, float delta_t, float action_duration,
                 std::string collision_checker, float safety_distance, std::string end_condition,
                 PolicyOptions policy_options, ParallelizationOptions parallelization_options,
                 std::string trajectory_type, float uct_cp, Noise noise, ActionNoise action_noise,
                 const float region_of_interest)
      : random_seed(random_seed),
        n_iterations(n_iterations),
        max_scenario_duration(max_scenario_duration),
        max_scenario_steps(max_scenario_steps),
        max_step_duration(max_step_duration),
        max_search_depth(max_search_depth),
        max_invalid_action_samples(max_invalid_action_samples),
        discount_factor(discount_factor),
        delta_t(delta_t),
        action_duration(action_duration),
        collision_checker(collision_checker),
        safety_distance(safety_distance),
        end_condition(end_condition),
        policy_options(policy_options),
        parallelization_options(parallelization_options),
        trajectory_type(trajectory_type),
        uct_cp(uct_cp),
        noise(noise),
        action_noise(action_noise),
        region_of_interest(region_of_interest) {}

  json toJSON() const;

  static ComputeOptions fromJSON(const json& jComputeOptions);
};
}  // namespace proseco_planning::config
