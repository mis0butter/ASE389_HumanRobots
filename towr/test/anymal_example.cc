/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <cmath>
#include <iostream>
#include <fstream>

#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>


using namespace towr;

// A minimal example how to build a trajectory optimization problem using TOWR.
//
// The more advanced example that includes ROS integration, GUI, rviz
// visualization and plotting can be found here:
// towr_ros/src/towr_ros_app.cc
int main()
{
  NlpFormulation formulation;

  // terrain
  formulation.terrain_ = std::make_shared<FlatGround>(0.0);
  // formulation.terrain_ = std::make_shared<Slope>();

  // Kinematic limits and dynamic parameters of Anymal
  formulation.model_ = RobotModel(RobotModel::Anymal);

  // set the initial position of Anymal
  formulation.initial_base_.lin.at(kPos).z() = 0.42; // OG 0.5 
    const double x_nominal_b = 0.34;
    const double y_nominal_b = 0.19;
    const double z_nominal_b = 0; // OG -0.42
  formulation.initial_ee_W_.push_back({x_nominal_b, y_nominal_b, z_nominal_b});
  formulation.initial_ee_W_.push_back({x_nominal_b, -y_nominal_b, z_nominal_b});
  formulation.initial_ee_W_.push_back({-x_nominal_b, y_nominal_b, z_nominal_b});
  formulation.initial_ee_W_.push_back({-x_nominal_b, -y_nominal_b, z_nominal_b});

  // define the desired goal state of Anymal
  formulation.final_base_.lin.at(towr::kPos) << 10.0, 0.0, 0.5;

  // Parameters that define the motion. See c'tor for default values or
  // other values that can be modified.
  // First we define the initial phase durations, that can however be changed
  // by the optimizer. The number of swing and stance phases however is fixed.
  // alternating stance and swing:     ____-----_____-----_____-----
  
  // TO-DO! Add 3 more legs 
  // formulation.params_.ee_phase_durations_.push_back({0.4, 0.4, 0.2, 0.4, 0.2, 0.4, 0}); 
  // formulation.params_.ee_phase_durations_.push_back({0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2});
  // formulation.params_.ee_phase_durations_.push_back({0.4, 0.4, 0.2, 0.4, 0.2, 0.4, 0}); 
  // formulation.params_.ee_phase_durations_.push_back({0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2});

  formulation.params_.ee_phase_durations_.push_back({0.4, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0}); 
  formulation.params_.ee_phase_durations_.push_back({0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2});
  formulation.params_.ee_phase_durations_.push_back({0.4, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0});
  formulation.params_.ee_phase_durations_.push_back({0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2});

  // formulation.params_.ee_phase_durations_.push_back({0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2}); 
  // formulation.params_.ee_phase_durations_.push_back({0.2, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0});
  // formulation.params_.ee_phase_durations_.push_back({0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2});
  // formulation.params_.ee_phase_durations_.push_back({0.2, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0});
  formulation.params_.ee_in_contact_at_start_.push_back(true);
  formulation.params_.ee_in_contact_at_start_.push_back(true);
  formulation.params_.ee_in_contact_at_start_.push_back(true);
  formulation.params_.ee_in_contact_at_start_.push_back(true);

  // Initialize the nonlinear-programming problem with the variables,
  // constraints and costs.
  ifopt::Problem nlp;
  SplineHolder solution;
  for (auto c : formulation.GetVariableSets(solution))
    nlp.AddVariableSet(c);
  for (auto c : formulation.GetConstraints(solution))
    nlp.AddConstraintSet(c);
  for (auto c : formulation.GetCosts())
    nlp.AddCostSet(c);

  // You can add your own elements to the nlp as well, simply by calling:
  // nlp.AddVariablesSet(your_custom_variables);
  // nlp.AddConstraintSet(your_custom_constraints);

  // Choose ifopt solver (IPOPT or SNOPT), set some parameters and solve.
  // solver->SetOption("derivative_test", "first-order");
  auto solver = std::make_shared<ifopt::IpoptSolver>();
  solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
  solver->SetOption("max_cpu_time", 20.0);
  solver->Solve(nlp);

  // Can directly view the optimization variables through:
  // Eigen::VectorXd x = nlp.GetVariableValues()
  // However, it's more convenient to access the splines constructed from these
  // variables and query their values at specific times:
  using namespace std;
  cout.precision(2);
  nlp.PrintCurrent(); // view variable-set, constraint violations, indices,...
  cout << fixed;
  cout << "\n====================\nMonoped trajectory:\n====================\n";

  ofstream outfile;
  outfile.open ("example.txt");
  outfile << "time , lin_pos[m] , euler_rpy[deg] , foot1_pos[m] , foot2_pos[m] , foot3_pos[m] , foot4_pos[m] , foot1_F[N] , foot2_F[N] , foot3_F[N] , foot4_F[N] , foot1_contact , foot2_contact , foot3_contact , foot4_contact \n"; 
  double t = 0.0;
  while (t<=solution.base_linear_->GetTotalTime() + 1e-5) {
  // while (t<=solution.base_linear_->GetTotalTime() + 10) {
    cout << "t=" << t << "\n";
    outfile << t << ",";
    cout << "Base linear position x,y,z:   ,";
    cout << solution.base_linear_->GetPoint(t).p().transpose() << ",[m]" << endl;
    outfile << solution.base_linear_->GetPoint(t).p().transpose() << ",";

    cout << "Base Euler roll, pitch, yaw:  ,";
    Eigen::Vector3d rad = solution.base_angular_->GetPoint(t).p();
    cout << (rad/M_PI*180).transpose() << ",[deg]" << endl;
    outfile << (rad/M_PI*180).transpose() << "," ;

    cout << "Foot position x,y,z:          ,";
    cout << solution.ee_motion_.at(0)->GetPoint(t).p().transpose() << ",[m]" << endl;
    outfile <<solution.ee_motion_.at(0)->GetPoint(t).p().transpose() << "," ; 
    cout << solution.ee_motion_.at(1)->GetPoint(t).p().transpose() << ",[m]" << endl;
    outfile <<solution.ee_motion_.at(1)->GetPoint(t).p().transpose() << "," ; 
    cout << solution.ee_motion_.at(2)->GetPoint(t).p().transpose() << ",[m]" << endl;
    outfile <<solution.ee_motion_.at(2)->GetPoint(t).p().transpose() << "," ; 
    cout << solution.ee_motion_.at(3)->GetPoint(t).p().transpose() << ",[m]" << endl;
    outfile <<solution.ee_motion_.at(3)->GetPoint(t).p().transpose() << "," ; 

    cout << "Contact force x,y,z:          ,";
    cout << solution.ee_force_.at(0)->GetPoint(t).p().transpose() << ",[N]" << endl;
    outfile << solution.ee_force_.at(0)->GetPoint(t).p().transpose() << "," ; 
    cout << solution.ee_force_.at(1)->GetPoint(t).p().transpose() << ",[N]" << endl;
    outfile << solution.ee_force_.at(1)->GetPoint(t).p().transpose() << "," ; 
    cout << solution.ee_force_.at(2)->GetPoint(t).p().transpose() << ",[N]" << endl;
    outfile << solution.ee_force_.at(2)->GetPoint(t).p().transpose() << "," ; 
    cout << solution.ee_force_.at(3)->GetPoint(t).p().transpose() << ",[N]" << endl;
    outfile << solution.ee_force_.at(3)->GetPoint(t).p().transpose() << endl; 

    // bool contact = solution.phase_durations_.at(0)->IsContactPhase(t);
    // std::string foot_in_contact = contact? "yes" : "no";
    // cout << "Foot in contact:              ," + foot_in_contact << endl;
    // outfile << foot_in_contact << endl; 
    // bool contact = solution.phase_durations_.at(1)->IsContactPhase(t);
    // std::string foot_in_contact = contact? "yes" : "no";
    // cout << "Foot in contact:              ," + foot_in_contact << endl;
    // outfile << foot_in_contact << endl; 
    // bool contact = solution.phase_durations_.at(2)->IsContactPhase(t);
    // std::string foot_in_contact = contact? "yes" : "no";
    // cout << "Foot in contact:              ," + foot_in_contact << endl;
    // outfile << foot_in_contact << endl; 
    // bool contact = solution.phase_durations_.at(3)->IsContactPhase(t);
    // std::string foot_in_contact = contact? "yes" : "no";
    // cout << "Foot in contact:              ," + foot_in_contact << endl;
    // outfile << foot_in_contact << endl; 

    cout << endl;

    // outfile << "Writing this to a file.\n";

    t += 0.01;
  }
  outfile.close();
}
