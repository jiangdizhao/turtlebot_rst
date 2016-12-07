#ifndef TYPES_H_
#define TYPES_H_

#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/deferral.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/event.hpp>


namespace sc = boost::statechart;
namespace mpl = boost::mpl;

#include <turtle_commander_messages/EvaluationData.h>
#include <turtle_commander_messages/CommandService.h>

namespace tcm = turtle_commander_messages;


struct EvaluationSettings
{
  std::string configuration_file_routes = "/home/stephan/Documents/config_routes.txt";
  std::string configuration_file_planner = "/home/stephan/Documents/config_planner.txt";
  std::string evaluation_file = "/home/stephan/Documents/evaluation.mat";
  int number_v_bins = 10;
  double v_max = 0.3;
  int number_omega_bins = 10;
  double omega_max = 0.5;      
};

#endif // TYPES_H_