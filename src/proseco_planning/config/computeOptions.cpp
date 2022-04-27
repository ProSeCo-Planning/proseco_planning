#include "proseco_planning/config/computeOptions.h"

#include <chrono>
#include <map>

#include "nlohmann/json.hpp"

namespace proseco_planning::config {

/**
 * @brief Exports the parameters of the SimilarityUpdate object to JSON.
 *
 * @return json The parameters.
 */
json SimilarityUpdate::toJSON() const {
  json jSimilarityUpdate;
  jSimilarityUpdate["active"] = active;
  jSimilarityUpdate["gamma"]  = gamma;
  return jSimilarityUpdate;
}

/**
 * @brief Returns a new SimilarityUpdate object created from the parameters of the JSON file.
 *
 * @param jSimilarityUpdate The JSON file.
 * @return SimilarityUpdate
 */
SimilarityUpdate SimilarityUpdate::fromJSON(const json& jSimilarityUpdate) {
  SimilarityUpdate similarityUpdate = SimilarityUpdate(jSimilarityUpdate["active"].get<bool>(),
                                                       jSimilarityUpdate["gamma"].get<float>());
  return similarityUpdate;
}
/**
 * @brief Exports the parameters of the SearchGuide object to JSON.
 *
 * @return json The parameters.
 */
json SearchGuide::toJSON() const {
  json jSearchGuide;
  jSearchGuide["n_samples"] = n_samples;
  jSearchGuide["type"]      = type;
  return jSearchGuide;
}

/**
 * @brief Returns a new searchGuide object created from the parameters of the JSON file.
 *
 * @param jSearchGuide The JSON file.
 * @return SearchGuide
 */
SearchGuide SearchGuide::fromJSON(const json& jSearchGuide) {
  SearchGuide searchGuide = SearchGuide(jSearchGuide["n_samples"].get<unsigned int>(),
                                        jSearchGuide["type"].get<std::string>());
  return searchGuide;
}

/**
 * @brief Exports the parameters of the MoveGroupingCriteriaPW object to JSON.
 *
 * @return json The parameters.
 */
json MoveGroupingCriteriaPW::toJSON() const {
  json jMoveGroupingCriteriaPW;
  jMoveGroupingCriteriaPW["active"]         = active;
  jMoveGroupingCriteriaPW["coefficient_pw"] = coefficient_pw;
  jMoveGroupingCriteriaPW["exponent_pw"]    = exponent_pw;
  return jMoveGroupingCriteriaPW;
}

/**
 * @brief Returns a new MoveGroupingCriteriaPW object created from the parameters of the JSON file.
 *
 * @param jMoveGroupingCriteriaPW The JSON file.
 * @return MoveGroupingCriteriaPW
 */
MoveGroupingCriteriaPW MoveGroupingCriteriaPW::fromJSON(const json& jMoveGroupingCriteriaPW) {
  MoveGroupingCriteriaPW moveGroupingCriteriaPW =
      MoveGroupingCriteriaPW(jMoveGroupingCriteriaPW["active"].get<bool>(),
                             jMoveGroupingCriteriaPW["coefficient_pw"].get<float>(),
                             jMoveGroupingCriteriaPW["exponent_pw"].get<float>());
  return moveGroupingCriteriaPW;
}
/**
 * @brief Exports the parameters of the MoveGrouping object to JSON.
 *
 * @return json The parameters.
 */
json MoveGrouping::toJSON() const {
  json jMoveGrouping;
  jMoveGrouping["move_grouping_criteria_pw"] = move_grouping_criteria_pw.toJSON();
  jMoveGrouping["active"]                    = active;
  jMoveGrouping["cp"]                        = cp;
  jMoveGrouping["move_grouping_bias_pw"]     = move_grouping_bias_pw;
  jMoveGrouping["final_decision"]            = final_decision;
  return jMoveGrouping;
}

/**
 * @brief Returns a new MoveGrouping object created from the parameters of the JSON file.
 *
 * @param jMoveGrouping The JSON file.
 * @return MoveGrouping
 */
MoveGrouping MoveGrouping::fromJSON(const json& jMoveGrouping) {
  MoveGrouping moveGrouping =
      MoveGrouping(jMoveGrouping["active"].get<bool>(), jMoveGrouping["cp"].get<float>(),
                   MoveGroupingCriteriaPW::fromJSON(jMoveGrouping["move_grouping_criteria_pw"]),
                   jMoveGrouping["move_grouping_bias_pw"].get<bool>(),
                   jMoveGrouping["final_decision"].get<bool>());
  return moveGrouping;
}

/**
 * @brief Exports the parameters of the ProgressiveWidening object to JSON.
 *
 * @return json The parameters.
 */
json ProgressiveWidening::toJSON() const {
  json jProgressiveWidening;
  jProgressiveWidening["max_depth_pw"] = max_depth_pw;
  jProgressiveWidening["exponent"]     = exponent;
  jProgressiveWidening["coefficient"]  = coefficient;
  return jProgressiveWidening;
}

/**
 * @brief Returns a new ProgressiveWidening object created from the parameters of the JSON file.
 *
 * @param jProgressiveWidening The JSON file.
 * @return ProgressiveWidening
 */
ProgressiveWidening ProgressiveWidening::fromJSON(const json& jProgressiveWidening) {
  ProgressiveWidening progressiveWidening =
      ProgressiveWidening(jProgressiveWidening["max_depth_pw"].get<unsigned int>(),
                          jProgressiveWidening["exponent"].get<float>(),
                          jProgressiveWidening["coefficient"].get<float>());
  return progressiveWidening;
}

/**
 * @brief Exports the parameters of the PolicyEnhancement object to JSON.
 *
 * @return json The parameters.
 */
json PolicyEnhancements::toJSON() const {
  json jPolicyEnhancements;
  jPolicyEnhancements["similarity_update"]         = similarity_update.toJSON();
  jPolicyEnhancements["search_guide"]              = search_guide.toJSON();
  jPolicyEnhancements["move_grouping"]             = move_grouping.toJSON();
  jPolicyEnhancements["progressive_widening"]      = progressive_widening.toJSON();
  jPolicyEnhancements["action_execution_fraction"] = action_execution_fraction;
  jPolicyEnhancements["q_scale"]                   = q_scale;
  return jPolicyEnhancements;
}

/**
 * @brief Returns a new ProgressiveWidening object created from the parameters of the JSON file.
 *
 * @param jPolicyEnhancements The JSON file.
 * @return PolicyEnhancements
 */
PolicyEnhancements PolicyEnhancements::fromJSON(const json& jPolicyEnhancements) {
  PolicyEnhancements policyEnhancements =
      PolicyEnhancements(SimilarityUpdate::fromJSON(jPolicyEnhancements["similarity_update"]),
                         SearchGuide::fromJSON(jPolicyEnhancements["search_guide"]),
                         MoveGrouping::fromJSON(jPolicyEnhancements["move_grouping"]),
                         ProgressiveWidening::fromJSON(jPolicyEnhancements["progressive_widening"]),
                         jPolicyEnhancements["action_execution_fraction"].get<float>(),
                         jPolicyEnhancements["q_scale"].get<float>());
  return policyEnhancements;
}

/**
 * @brief Exports the parameters of the KernelRegressionLCB object to JSON.
 *
 * @return json The parameters.
 */
json KernelRegressionLCB::toJSON() const {
  /// @todo
  json jKernelRegressionLCB;
  jKernelRegressionLCB["move_grouping"]                  = move_grouping;
  jKernelRegressionLCB["action"]["kernel_variant"]       = action.kernel_variant;
  jKernelRegressionLCB["action"]["gamma"]                = action.gamma;
  jKernelRegressionLCB["action"]["cp"]                   = action.cp;
  jKernelRegressionLCB["action_class"]["kernel_variant"] = action_class.kernel_variant;
  jKernelRegressionLCB["action_class"]["gamma"]          = action_class.gamma;
  jKernelRegressionLCB["action_class"]["cp"]             = action_class.cp;
  return jKernelRegressionLCB;
}

/**
 * @brief Returns a new KernelRegressionLCB object created from the parameters of the JSON file.
 *
 * @param jKernelRegressionLCB The JSON file.
 * @return KernelRegressionLCB
 */
KernelRegressionLCB KernelRegressionLCB::fromJSON(const json& jKernelRegressionLCB) {
  /// @todo
  return {jKernelRegressionLCB["move_grouping"].get<bool>(),
          {jKernelRegressionLCB["action"]["kernel_variant"].get<std::string>(),
           jKernelRegressionLCB["action"]["gamma"].get<float>(),
           jKernelRegressionLCB["action"]["cp"].get<float>()},
          {jKernelRegressionLCB["action_class"]["kernel_variant"].get<std::string>(),
           jKernelRegressionLCB["action_class"]["gamma"].get<float>(),
           jKernelRegressionLCB["action_class"]["cp"].get<float>()}};
}

/**
 * @brief Exports the parameters of the PolicyOptions object to JSON.
 *
 * @return json The parameters.
 */
json PolicyOptions::toJSON() const {
  json jPolicyOptions;
  jPolicyOptions["selection_policy"]       = selection_policy;
  jPolicyOptions["expansion_policy"]       = expansion_policy;
  jPolicyOptions["simulation_Policy"]      = simulation_Policy;
  jPolicyOptions["update_policy"]          = update_policy;
  jPolicyOptions["final_selection_policy"] = final_selection_policy;
  jPolicyOptions["policy_enhancements"]    = policy_enhancements.toJSON();
  /// @todo
  if (final_selection_policy == "kernelRegressionLCB")
    jPolicyOptions["kernel_regression_lcb"] = kernel_regression_lcb.toJSON();
  return jPolicyOptions;
}

/**
 * @brief Returns a new PolicyOptions object created from the parameters of the JSON file.
 *
 * @param jPolicyOptions The JSON file.
 * @return PolicyOptions
 */
PolicyOptions PolicyOptions::fromJSON(const json& jPolicyOptions) {
  PolicyOptions policyOptions =
      PolicyOptions(jPolicyOptions["selection_policy"].get<std::string>(),
                    jPolicyOptions["expansion_policy"].get<std::string>(),
                    jPolicyOptions["simulation_Policy"].get<std::string>(),
                    jPolicyOptions["update_policy"].get<std::string>(),
                    jPolicyOptions["final_selection_policy"].get<std::string>(),
                    PolicyEnhancements::fromJSON(jPolicyOptions["policy_enhancements"]));
  /// @todo
  if (policyOptions.final_selection_policy == "kernelRegressionLCB")
    policyOptions.kernel_regression_lcb =
        KernelRegressionLCB::fromJSON(jPolicyOptions["kernel_regression_lcb"]);
  return policyOptions;
}
/**
 * @brief Exports the parameters of the ParallelizationOptions object to JSON.
 *
 * @return json The parameters.
 */
json ParallelizationOptions::toJSON() const {
  json jParallelizationOptions;
  jParallelizationOptions["n_threads"]              = n_threads;
  jParallelizationOptions["n_simulationThreads"]    = n_simulationThreads;
  jParallelizationOptions["similarity_voting"]      = similarity_voting;
  jParallelizationOptions["similarity_gamma"]       = similarity_gamma;
  jParallelizationOptions["simulation_aggregation"] = simulation_aggregation;
  return jParallelizationOptions;
}

/**
 * @brief Returns a new ParallelizationOptions object created from the parameters of the JSON file.
 *
 * @param jParallelizationOptions The JSON file.
 * @return ParallelizationOptions
 */
ParallelizationOptions ParallelizationOptions::fromJSON(const json& jParallelizationOptions) {
  return {jParallelizationOptions["n_threads"].get<unsigned int>(),
          jParallelizationOptions["n_simulationThreads"].get<unsigned int>(),
          jParallelizationOptions["similarity_voting"].get<bool>(),
          jParallelizationOptions["similarity_gamma"].get<float>(),
          jParallelizationOptions["simulation_aggregation"].get<std::string>()};
}

/**
 * @brief Exports the parameters of the Noise object to JSON.
 *
 * @return json The parameters.
 */
json Noise::toJSON() const {
  json jNoise;
  jNoise["active"] = active;
  jNoise["mean"]   = mean;
  jNoise["sigma"]  = sigma;
  return jNoise;
}

/**
 * @brief Returns a new Noise object created from the parameters of the JSON file.
 *
 * @param jNoise The JSON file.
 * @return Noise
 */
Noise Noise::fromJSON(const json& jNoise) {
  Noise noise = Noise(jNoise["active"].get<bool>(), jNoise["mean"].get<float>(),
                      jNoise["sigma"].get<float>());
  return noise;
}

/**
 * @brief Exports the parameters of the ActionNoise object to JSON.
 *
 * @return json The parameters.
 */
json ActionNoise::toJSON() const {
  json jNoise;
  jNoise["active"]   = active;
  jNoise["mean_y"]   = meanY;
  jNoise["sigma_y"]  = sigmaY;
  jNoise["mean_vx"]  = meanVx;
  jNoise["sigma_vx"] = sigmaVx;
  return jNoise;
}

/**
 * @brief Returns a new ActionNoise object created from the parameters of the JSON file.
 *
 * @param jNoise The JSON file.
 * @return ActionNoise
 */
ActionNoise ActionNoise::fromJSON(const json& jNoise) {
  ActionNoise noise = ActionNoise(jNoise["active"].get<bool>(), jNoise["mean_y"].get<float>(),
                                  jNoise["sigma_y"].get<float>(), jNoise["mean_vx"].get<float>(),
                                  jNoise["sigma_vx"].get<float>());
  return noise;
}

/**
 * @brief Exports the parameters of the ComputeOptions object to JSON.
 *
 * @return json The parameters.
 */
json ComputeOptions::toJSON() const {
  json jComputeOptions;
  jComputeOptions["random_seed"]                = random_seed;
  jComputeOptions["n_iterations"]               = n_iterations;
  jComputeOptions["max_scenario_duration"]      = max_scenario_duration;
  jComputeOptions["max_scenario_steps"]         = max_scenario_steps;
  jComputeOptions["max_step_duration"]          = max_step_duration;
  jComputeOptions["max_search_depth"]           = max_search_depth;
  jComputeOptions["max_invalid_action_samples"] = max_invalid_action_samples;
  jComputeOptions["discount_factor"]            = discount_factor;
  jComputeOptions["delta_t"]                    = delta_t;
  jComputeOptions["action_duration"]            = action_duration;
  jComputeOptions["collision_checker"]          = collision_checker;
  jComputeOptions["safety_distance"]            = safety_distance;
  jComputeOptions["end_condition"]              = end_condition;
  jComputeOptions["policy_options"]             = policy_options.toJSON();
  jComputeOptions["parallelization_options"]    = parallelization_options.toJSON();
  jComputeOptions["trajectory_type"]            = trajectory_type;
  jComputeOptions["uct_cp"]                     = uct_cp;
  jComputeOptions["noise"]                      = noise.toJSON();
  jComputeOptions["action_noise"]               = action_noise.toJSON();
  jComputeOptions["region_of_interest"]         = region_of_interest;
  return jComputeOptions;
}

/**
 * @brief Returns a new ComputeOptions object created from the parameters of the JSON file.
 *
 * @param jComputeOptions The JSON file.
 * @return ComputeOptions
 */
ComputeOptions ComputeOptions::fromJSON(const json& jComputeOptions) {
  unsigned int randomSeed = jComputeOptions["random_seed"].get<unsigned int>();
  // Zero, means take a random number as seed
  if (randomSeed == 0) {
    randomSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
  }

  ComputeOptions computeOptions = ComputeOptions(
      randomSeed, jComputeOptions["n_iterations"].get<unsigned int>(),
      jComputeOptions["max_scenario_duration"].get<float>(),
      jComputeOptions["max_scenario_steps"].get<unsigned int>(),
      jComputeOptions["max_step_duration"].get<float>(),
      jComputeOptions["max_search_depth"].get<unsigned int>(),
      jComputeOptions["max_invalid_action_samples"].get<unsigned int>(),
      jComputeOptions["discount_factor"].get<float>(), jComputeOptions["delta_t"].get<float>(),
      jComputeOptions["action_duration"].get<float>(),
      jComputeOptions["collision_checker"].get<std::string>(),
      jComputeOptions["safety_distance"].get<float>(),
      jComputeOptions["end_condition"].get<std::string>(),
      PolicyOptions::fromJSON(jComputeOptions["policy_options"]),
      ParallelizationOptions::fromJSON(jComputeOptions["parallelization_options"]),
      jComputeOptions["trajectory_type"].get<std::string>(), jComputeOptions["uct_cp"].get<float>(),
      Noise::fromJSON(jComputeOptions["noise"]),
      ActionNoise::fromJSON(jComputeOptions["action_noise"]),
      jComputeOptions["region_of_interest"].get<float>());
  return computeOptions;
}
}  // namespace proseco_planning::config