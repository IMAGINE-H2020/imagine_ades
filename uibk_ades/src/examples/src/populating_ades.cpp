
/* @description : This example instantiate and populate an ADES (with fake data)
 *
 * */

#include <iostream>
#include <type_traits>

#include <ades/libades.h>
#include <boost/filesystem.hpp>

using namespace ades;

std::string const basic_color = "\033[97m";
std::string const info_color = "\033[92m";
std::string const error_color = "\033[31m";
std::string const default_color = "\033[39m";

std::ostream& operator << (std::ostream& os, const MotionType& obj)
{
    os << static_cast<std::underlying_type<MotionType>::type>(obj);
    return os;
}

std::string displayADESInfo(Ades & a)
{
    std::stringstream output;
    output << basic_color << "ADES name : " << info_color << a.getName() << std::endl;

    output << basic_color << "ADES PC : " << info_color << std::endl;
    auto ades_pc = a.getPreconditions();
    if(ades_pc.empty()){ output << error_color << "No PC defined yet"<< std::endl; }
    else
    {
        for(auto apc_it = ades_pc.begin() ; apc_it != ades_pc.end() ; apc_it++)
        {
           output << "\t" << apc_it->first << ":-" << apc_it->second << "." << std::endl;
        }
    }
    auto ades_ef = a.getEffects();
    output << basic_color << "ADES effects : " << info_color << std::endl;
    if(ades_ef.empty()){ output << error_color << "No Effects defined yet"<< std::endl; }
    else
    {
        for(auto aef_it = ades_ef.begin() ; aef_it != ades_ef.end() ; aef_it++)
        {
            output << "\t" << aef_it->first << ":-" << aef_it->second << std::endl;
        }
    }
    output << basic_color << "ADES motion sequences : " << info_color << std::endl;
    auto ades_ms = a.getMotionSequences();
    if(ades_ms.empty()){ output << error_color <<  "No Motion Sequences defined yet" << std::endl; }
    for(auto ams_it = ades_ms.begin() ; ams_it != ades_ms.end() ; ams_it++)
    {
          output << "\t" << ams_it->first << ", " << (ams_it->second).getID() << std::endl << "\tInput types : ";
          auto inputtypes = (ams_it->second).getInputTypes();
          for(auto it = inputtypes.begin() ; it != inputtypes.end() ; it++)
          {
              output << *it << ", " << std::flush;
          }
          auto motions = (ams_it->second).getMotions();
          output << std::endl << "\t" << "Motions : " << std::endl;
          for(auto it = motions.begin() ; it != motions.end() ; it++)
          {
              // Let's ignore type display for now
              output << "\t\t" << (*it)->getName() << ", " << (*it)->getMotionType() << std::endl;
              std::map<std::string, std::vector<double>> motionparams = (*it)->getMotionParameters();
              for(auto mit=motionparams.begin() ; mit !=motionparams.end() ; mit++)
              {
                  output << "\t\t\t" << mit->first << ": ";
                  for(auto mpit = mit->second.begin() ; mpit != mit->second.end() ; mpit++)
                  {
                      output << *mpit << ", ";
                  }
                  output << std::endl;
              }
          }
          output << "\t" << "Effect probability models : " << std::endl;
          auto effectprob = (ams_it->second ).getGMMEffectModels();

          for(auto emit = effectprob.begin() ; emit != effectprob.end() ; emit++)
          {
              output << "\t\t\t" << emit->first << ", dim: " << emit->second.Dimensionality() <<  ", nb gauss: " << emit->second.Gaussians() << std::endl;
          }

          output << "\t" << "Effect value models : " << std::endl;
          auto effectmean = (ams_it->second ).getGPEffectModels();

          for(auto emit = effectmean.begin() ; emit != effectmean.end() ; emit++)
          {
              output << "\t\t\t" << emit->first << ", dim: " << emit->second.get_input_dim() <<  ", nb samples in dataset: " << emit->second.get_sampleset_size() << std::endl;
          }
    }
    output << default_color << "----------------------";
    return output.str();
}

int main(int argc, char **argv)
{
    Ades fakeADES = Ades("myFakeAction");
    std::cout << displayADESInfo(fakeADES) << std::endl;
    fakeADES.setName("fakeLeverAction");
    std::cout << displayADESInfo(fakeADES) << std::endl;

    // Let's insert a set of planner/categorical preconditions
    // here, the assumption is that preconditions are prolog-like statements, the :- is not explicit but embedded in the key-value separation
    std::map<std::string, std::string> fakePreConds;
    fakePreConds.insert(std::pair<std::string, std::string>("leverable(X)","gap(X), reachable(X)"));
    fakeADES.insertPreconditions(fakePreConds);
    std::cout << displayADESInfo(fakeADES) << std::endl;

    // Let's insert a set of planner/categorical effects
    std::map<std::string, std::string> fakeEffects;
    fakeEffects.insert(std::pair<std::string, std::string>("levered(X)","\\+ gap(X)"));
    fakeADES.insertEffects(fakeEffects);
    std::cout << displayADESInfo(fakeADES) << std::endl;

    // Let's insert a motion sequence describing how to execute the action
    // 1st let's populate a simple motion_sequence
    MotionSequence firstMotionSequence;

    // Input dimensions to consider :
    std::string inputDimensions[] = {"target_position", "max_tool_force"};
    std::vector<std::string> fakeInputTypes(inputDimensions, inputDimensions+sizeof(inputDimensions)/sizeof(inputDimensions[0]));


    // List of motions :
    std::vector<double> gaussianCenters;
    std::vector<double> gaussianVariances;
    std::vector<double> weights;
    for(int i = 0 ; i < 10 ; i++)
    {
        gaussianCenters.push_back((double)i);
        gaussianVariances.push_back(1.0+(rand()%100)/100.0);
        weights.push_back((rand() % 100) / 100.0);
    }
    // tau, alpha_z, beta_z, alpha_g
    const double arr[] = {1.0, 1.5, 0.5, 0.1};
    std::vector<double> dmp_params(arr, arr + sizeof(arr) / sizeof(arr[0]) );

    DMPContainer fakeDMP(gaussianCenters, gaussianVariances, weights, dmp_params);
    std::cout << "Is this motion temporally scalable ? " << fakeDMP.isTemporallyScalable() << std::endl;
    std::cout << "What is this motion temporal scale ? " << fakeDMP.getTemporalScale() << std::endl;

    DMPContainer fakeDMP2(fakeDMP);
    fakeDMP.setName("fakeDMP1");
    fakeDMP2.setName("fakeDMP2");

    std::vector<std::vector<double>> traj_points;
    std::vector<double> point_x;
    std::vector<double> point_y;
    std::vector<double> point_z;
    for(int i = 0 ; i < 10 ; i++)
    {
        double x=(double)((rand()%1000 - 500)/500.0);
        double y=(double)((rand()%1000 - 500)/500.0);
        double z=(double)((rand()%1000 - 500)/500.0);
        point_x.push_back(x);
        point_y.push_back(y);
        point_z.push_back(z);
    }
    traj_points.push_back(point_x);
    traj_points.push_back(point_y);
    traj_points.push_back(point_z);
    TrajectoryContainer fakeTraj(traj_points);
    fakeTraj.setName("fakeTraj1");

    // Chaining twice the same gesture
    firstMotionSequence.insertInputTypes(fakeInputTypes);


    firstMotionSequence.insertMotion(0, &fakeDMP);
    firstMotionSequence.insertMotion(1, &fakeDMP2);
    firstMotionSequence.insertMotion(2, &fakeTraj);
    firstMotionSequence.insertMotion(3, &fakeDMP);


    // Model of the probability of gap surfance variation:
    firstMotionSequence.insertGMMEffectModel("gap_surface", 3, 1);
    // Model of the gap surfance variation given input type ; provide GP dimensionality and covariance function
    firstMotionSequence.insertGPEffectModel("gap_surface_given_lever_pose", 6, "CovSum ( CovSEiso, CovNoise)");

    // then add it the same way we add precondtions or effects :
    fakeADES.insertMotionSequence("motionSequence0", firstMotionSequence);

    std::cout << displayADESInfo(fakeADES) << std::endl;

    // Serializing ADES
    std::string home =  std::string(getenv("HOME"))+"/ADESDB";
    std::cout << "Opening database at: " << home << std::endl;
    auto db = AdesDB(home, 0);
    std::cout << "Storing ADES" << std::endl;
    db.addAdes(fakeADES);

    std::cout << "Done." << std::endl;
}
