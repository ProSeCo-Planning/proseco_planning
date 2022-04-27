#include "proseco_planning/monteCarloTreeSearch.h"

#include <sys/types.h>
#include <chrono>
#include <climits>
#include <cstddef>
#include <fstream>
#include <future>
#include <map>
#include <string>
#include <utility>

#include "proseco_planning/action/action.h"
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/outputOptions.h"
#include "proseco_planning/math/mathlib.h"
#include "proseco_planning/node.h"
#include "proseco_planning/policies/expansionPolicy.h"
#include "proseco_planning/policies/finalSelectionPolicy.h"
#include "proseco_planning/policies/selectionPolicy.h"
#include "proseco_planning/policies/simulationPolicy.h"
#include "proseco_planning/policies/updatePolicy.h"

namespace proseco_planning {

//#########################################################
//### BUILD THE SEARCH TREE
//#########################################################

/**
 * @brief Builds the search tree according to the MCTS approach.
 *
 * @param root Pointer to the root node.
 * @return std::unique_ptr<Node> Pointer to the root node of the final search tree.
 */
std::unique_ptr<Node> computeTree(std::unique_ptr<Node> root) {
  //### create policies according to compute optinos
  auto selectionPolicy  = SelectionPolicy::createPolicy(cOpt().policy_options.selection_policy);
  auto simulationPolicy = SimulationPolicy::createPolicy(cOpt().policy_options.simulation_Policy,
                                                         root->m_agents.size());
  auto expansionPolicy  = ExpansionPolicy::createPolicy(cOpt().policy_options.expansion_policy);
  auto updatePolicy     = UpdatePolicy::createPolicy(cOpt().policy_options.update_policy);

  // initialize the available actions of the rootNode
  for (auto& agent : root->m_agents) {
    agent.setAvailableActions(root->m_depth);
  }

  // maximum duration of one planning step
  // Note: cOpt().max_step_duration must not exceed 4294 seconds (~= 72 mins) due to integer
  // overflow
  unsigned int maxStepDuration{cOpt().max_step_duration == 0
                                   ? UINT_MAX
                                   : static_cast<unsigned int>(cOpt().max_step_duration * 1000000)};
  // measure the elapsed time for this planning step
  unsigned int elapsedTime{0};

  for (unsigned int iteration = 0;
       (iteration < cOpt().n_iterations && elapsedTime < maxStepDuration); ++iteration) {
    // Start timer to measure the duration of this iteration
    auto startTime = std::chrono::steady_clock::now();

    // initialize the reward vector with full length (reserve storage).
    // The `stepReward` vector comprises the reward at a specific step of the tree path for each
    // agent. The tree path step is NOT related to the planning step.
    std::vector<float> stepReward(root->m_agents.size(), 0);
    // the nested `agentsRewards` vector contains the `stepReward` for each step along the explored
    // tree path starting from the root node
    std::vector<std::vector<float>> agentsRewards(cOpt().max_search_depth, stepReward);

    //#########################################################
    //### PHASE 1 SELECTION
    //#########################################################
    // Starting at the root node, a child selection policy is
    // recursively applied to descend through the tree until
    // the most urgent expandable node is reached. A node is
    // expandable if it represents a nonterminal state and has
    // unvisited (i.e., unexpanded) children.
    ActionSet actionSet;

    auto node = root.get();
    node      = selectionPolicy->selectNodeForExpansion(node, actionSet, agentsRewards);
    // `node` is now the selected node that shall be expanded.

    // Add noise to the position of the agents
    if (cOpt().noise.active) {
      for (auto& agent : node->m_agents) {
        agent.m_vehicle.m_positionX =
            agent.m_vehicle.m_positionX + math::getNoise(cOpt().noise.mean, cOpt().noise.sigma);
        agent.m_vehicle.m_positionY =
            agent.m_vehicle.m_positionY + math::getNoise(cOpt().noise.mean, cOpt().noise.sigma);
      }
    }

    //#########################################################
    //### PHASE 2 EXPANSION
    //#########################################################
    // One child node is added according to the actionSet to expand the tree.
    // certain limits can be set, such as min visits...

    node = expansionPolicy->expandTree(node, actionSet, agentsRewards, cOpt().max_search_depth);
    // `node` is now the new node that has been appended to the search tree.

    //#########################################################
    //### PHASE 3 SIMULATION
    //#########################################################
    // A simulation is run from the new node(s) according to
    // the simulation policy to produce an outcome.

    auto simDepth = simulationPolicy->runSimulation(node, agentsRewards, cOpt().max_search_depth);
    // `simDepth` specifies the depth of the executed simulation

    //#########################################################
    //### PHASE 4 BACKPROPAGATION
    //#########################################################
    // The evaluation of the added node is backed up to its parent step by step
    // (i.e., backpropagated) through the selected nodes
    // to update their statistics.

    updatePolicy->updateTree(node, agentsRewards, simDepth);

    // Measure the time to determine the duration of this iteration
    auto endTime = std::chrono::steady_clock::now();
    // Update the elapsed time for the planning step
    elapsedTime +=
        std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
  }
  // return the entire search tree
  return root;
}

//##########################################################################
//### MAIN ENTRY POINT: applies the MCTS and returns the best actionSetSequence
//##########################################################################

/**
 * @brief Applies the MCTS and returns the best action set sequence i.e. the sequence of action sets
 * that describes the actions for all agents (trajectories) over the planning horizon.
 * @note This function is the main entry point of this library.
 *
 * @param rootNode The pointer to the root node.
 * @param step The planning current planning step.
 * @return actionSetSequence The action set sequence containing the finally selected actions.
 */
ActionSetSequence computeActionSetSequence(std::unique_ptr<Node> rootNode, int step) {
  // the actionSetSequence that is returned by this function. It contains best action sets that
  // shall be executed successively.
  ActionSetSequence actionSetSequence;
  // root node of the final search tree
  std::unique_ptr<Node> rootFinal;

  // Set the seed for the thread-safe random engine
  // Multiply seed by step to be pseudo-random between planning steps
  /// @todo consider removal, this does currently not add any benefit
  math::Random::g_seed = cOpt().random_seed + step * 1151;

  unsigned int nThreads{cOpt().parallelization_options.n_threads};
  if (nThreads > 1) {
    //### FUTURES FOR ROOT PARALLELIZATION
    // create futures
    std::vector<std::future<std::unique_ptr<Node>>> rootFutures;
    std::vector<std::unique_ptr<Node>> roots(nThreads);
    for (unsigned int t = 0; t < nThreads; ++t) {
      roots[t] = std::make_unique<Node>(rootNode.get());
    }
    for (unsigned int t = 1; t <= nThreads; ++t) {
      // create a lambda function for the root parallelization
      auto func = [t, rootInLambda = &roots[t - 1], step]() -> std::unique_ptr<Node> {
        // multiply thread id with random number and bit shift the step to create a random salt
        math::Random::setSalt((t * 11779) + (step << 13));
        return computeTree(std::move(*rootInLambda));
      };
      // push back the jobs and start the processing
      rootFutures.push_back(std::async(std::launch::async, func));
    }

    // collect the results of the futures
    for (unsigned int t = 0; t < nThreads; ++t) {
      roots[t] = rootFutures[t].get();
    }
    if (cOpt().parallelization_options.similarity_voting) {
      actionSetSequence = similarityVoting(roots);
    } else {
      actionSetSequence = similarityMerge(roots);
    }
    //### FOR EVALUATION PURPOSES SET ROOTFINAL TO ONE OF THE ROOTS
    rootFinal = std::move(roots[0]);
  } else {
    rootFinal = computeTree(std::move(rootNode));

    // node that is used for the final selection, corresponds to the root node of the final search
    // tree
    auto nodeFinalSelection = rootFinal.get();
    // create the finalSelectionPolicy
    auto finalSelectionPolicy =
        FinalSelectionPolicy::createPolicy(cOpt().policy_options.final_selection_policy);

    actionSetSequence = finalSelectionPolicy->getBestPlan(nodeFinalSelection);
  }

  if (oOpt().hasExportType("tree")) {
    rootFinal->exportTree(step);
  }

  //### EXPORT THE DISTRIBUTION
  if (!actionSetSequence.empty()) {
    if (oOpt().hasExportType("childMap")) {
      rootFinal->exportChildMap(step, actionSetSequence[0]);
    }
    if (oOpt().hasExportType("permutationMap")) {
      rootFinal->exportPermutationMap(step, actionSetSequence[0]);
    }
    if (oOpt().hasExportType("moveGroups")) {
      rootFinal->exportMoveGroups(step);
    }
  }
  return actionSetSequence;
}

/**
 * @brief Updates the actions from `master` with the actions of `node`.
 *
 * @param master The master node.
 * @param node The node for comparison.
 */
void similarityUpdate(Node* const master, const Node* const node) {
  // update each agent
  for (unsigned int agentIndex = 0; agentIndex < master->m_agents.size(); ++agentIndex) {
    float similarity{0.0f};
    float n_new{0.0f};
    float n_old{0.0f};
    float q_old{0.0f};
    float n_other{0.0f};
    float q_other{0.0f};
    // masterActionPtr, masterActionValue
    for (auto& [masterActionPtr, masterActionValue] :
         master->m_agents[agentIndex].m_actionValues) {  // nodeActionPtr, nodeActionValue
      for (const auto& [nodeActionPtr, nodeActionValue] :
           node->m_agents[agentIndex].m_actionValues) {
        similarity = Action::getSimilarity(masterActionPtr, nodeActionPtr);
        n_old      = master->m_agents[agentIndex].m_actionVisits.at(masterActionPtr);
        q_old      = masterActionValue;
        n_other    = node->m_agents[agentIndex].m_actionVisits.at(nodeActionPtr);
        q_other    = nodeActionValue;
        // skip calculation for low similarities
        if (similarity > 0.1) {
          n_new = n_old + similarity * n_other;
          // calculation of q_new
          masterActionValue = 1 / n_new * (q_old * n_old + q_other * similarity * n_other);
          // set n_new
          master->m_agents[agentIndex].m_actionVisits.at(masterActionPtr) = n_new;
        }
      }
    }
  }
}

/**
 * @brief Inserts the actions of `node` into the master node.
 *
 * @param master The master node
 * @param node The node to be merged.
 */
void mergeTrees(Node* const master, const Node* const node) {
  master->m_visits += node->m_visits;
  for (unsigned int agentIndex = 0; agentIndex < master->m_agents.size(); ++agentIndex) {
    master->m_agents[agentIndex].m_actionValues.insert(
        node->m_agents[agentIndex].m_actionValues.begin(),
        node->m_agents[agentIndex].m_actionValues.end());
    master->m_agents[agentIndex].m_actionVisits.insert(
        node->m_agents[agentIndex].m_actionVisits.begin(),
        node->m_agents[agentIndex].m_actionVisits.end());
  }
}

/**
 * @brief Merges all child actions from `resultRoots` into a `rootFinal` node,
 * updates all actions of this new root node and returns the best final actions.
 *
 * @param resultRoots The root nodes from the generated search trees.
 * @return actionSetSequence Contains the finally selected actions.
 */
ActionSetSequence similarityMerge(const std::vector<std::unique_ptr<Node>>& resultRoots) {
  ActionSetSequence actionSetSequence;

  // create the finalSelectionPolicy
  auto finalSelectionPolicy =
      FinalSelectionPolicy::createPolicy(cOpt().policy_options.final_selection_policy);

  // Merge all threadTrees
  std::unique_ptr<Node> rootFinal = std::make_unique<Node>(resultRoots[0].get());
  for (size_t t = 1; t < resultRoots.size(); ++t) {
    mergeTrees(rootFinal.get(), resultRoots[t].get());
  }

  // update actions of root final
  for (auto& root : resultRoots) {
    similarityUpdate(rootFinal.get(), root.get());
  }

  // extract final actions
  Node* nodeFinalSelection = rootFinal.get();
  actionSetSequence        = finalSelectionPolicy->getBestPlan(nodeFinalSelection);

  return actionSetSequence;
}

/**
 * @brief Generates a voting to choose the best final actions out of all `resultRoots`.
 *
 * @param resultRoots The root nodes from the generated search trees.
 * @return ActionSetSequence Contains the finally selected actions.
 */
ActionSetSequence similarityVoting(const std::vector<std::unique_ptr<Node>>& resultRoots) {
  ActionSetSequence actionSetSequence;

  // create the finalSelectionPolicy
  auto finalSelectionPolicy =
      FinalSelectionPolicy::createPolicy(cOpt().policy_options.final_selection_policy);

  std::size_t size{resultRoots.size()};
  // assuming all nodes have the same amount of Agents!
  std::size_t agentsSize{resultRoots[0]->m_agents.size()};

  ActionSetSequence bestActions;
  for (size_t t = 0; t < size; ++t) {
    // determine best actions
    Node* nodeFinalSelection = resultRoots[t].get();
    bestActions.push_back(finalSelectionPolicy->getBestActionSet(nodeFinalSelection));
  }

  // create a matrix that contains the similarities between all best actions multiplied with the
  // action value
  std::vector<std::vector<std::vector<float>>> similarities;
  for (size_t t = 0; t < size; ++t) {
    std::vector<std::vector<float>> inner;
    for (size_t s = 0; s < size; ++s) {
      std::vector<float> innerInner(agentsSize, 0);
      inner.push_back(innerInner);
    }
    similarities.push_back(inner);
  }
  for (size_t t = 0; t < size; ++t) {
    for (size_t s = 0; s < size; ++s) {
      for (size_t a = 0; a < agentsSize; ++a) {
        similarities[t][s][a] = Action::getSimilarity(bestActions[t][a], bestActions[s][a]) *
                                resultRoots[s]->m_agents[a].m_actionValues.at(bestActions[s][a]);
      }
    }
  }

  // sum over each row to get a vote for each thread
  std::vector<std::vector<float>> sumSimilarities;
  for (size_t s = 0; s < size; ++s) {
    std::vector<float> innerInner(agentsSize, 0);
    sumSimilarities.push_back(innerInner);
  }
  for (size_t t = 0; t < size; ++t) {
    for (size_t s = 0; s < size; ++s) {
      for (size_t a = 0; a < agentsSize; ++a) {
        sumSimilarities[t][a] += similarities[t][s][a];
      }
    }
  }

  // determine index of the highest vote for each agent
  std::vector<int> max_index(agentsSize, 0);
  for (size_t a = 0; a < agentsSize; ++a) {
    max_index[a]  = 0;
    float current = sumSimilarities[0][a];
    for (size_t t = 0; t < size; ++t) {
      if (sumSimilarities[t][a] > current) {
        current      = sumSimilarities[t][a];
        max_index[a] = t;
      }
    }
  }

  // extract final actions
  ActionSet finalActionSet;
  for (size_t a = 0; a < agentsSize; ++a) {
    finalActionSet.push_back(bestActions[max_index[a]][a]);
  }

  actionSetSequence.push_back(finalActionSet);

  return actionSetSequence;
}

}  // namespace proseco_planning
