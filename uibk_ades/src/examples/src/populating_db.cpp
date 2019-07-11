
/* @description : This example shows how to load an ADES (with fake data) and fill the DB
 *
 * */

#include <iostream>
#include <type_traits>

#include <ades/libades.h>


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
              //output << static_cast<std::underlying_type<DMPContainer>::type>(it) << ", " ;
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
    srand(time(NULL));
    Ades fakeADES0 = Ades("loadedAdes0");
//    Ades fakeADES1 = Ades("loadedAdes1");
//    Ades fakeADES2 = Ades("loadedAdes2");
//    Ades fakeADES3 = Ades("loadedAdes3");

    AdesDB database = AdesDB("./databaseADES", 0);

    // Serializing ADES :
    std::string ades_fn = "./serializedADES/fakeADES0.xml";
    std::ifstream ifs0(ades_fn);
//    std::ifstream ifs1(ades_fn);
//    std::ifstream ifs2(ades_fn);
//    std::ifstream ifs3(ades_fn);
//    boost::archive::xml_iarchive ia1(ifs1);
//    boost::archive::xml_iarchive ia2(ifs2);
//    boost::archive::xml_iarchive ia3(ifs3);

//    try
//    {
        {
        boost::archive::xml_iarchive ia0(ifs0);
        std::cout << "Deserializing" << std::endl;
        ia0 >> BOOST_SERIALIZATION_NVP(fakeADES0);
        }
//    }
 //   catch( const std::exception & e)
  //  {
   //     std::cout << e.what() << std::endl;
    //}
  /*  try
    {
        std::cout << "Deserializing 1" << std::endl;
        ia1 >> BOOST_SERIALIZATION_NVP(fakeADES1);
    }
    catch( const std::exception & e)
    {
        std::cout << e.what() << std::endl;
    }
    try
    {
        std::cout << "Deserializing 2" << std::endl;
        ia2 >> BOOST_SERIALIZATION_NVP(fakeADES2);
    }
    catch( const std::exception & e)
    {
        std::cout << e.what() << std::endl;
    }
    try
    {
        std::cout << "Deserializing 3" << std::endl;
        ia3 >> BOOST_SERIALIZATION_NVP(fakeADES3);
    }
    catch( const std::exception & e)
    {
        std::cout << "Banana happened !" << std::endl;
        std::cout << e.what() << std::endl;
    }*/
    ifs0.close();
/*    ifs1.close();
    ifs2.close();
    ifs3.close();
*/
    fakeADES0.setName("loadedAdes0");
  /*  fakeADES1.setName("loadedAdes1");
    fakeADES2.setName("loadedAdes2");
    fakeADES3.setName("loadedAdes3");
*/
    std::cout << "Banana" << std::endl;

//    std::cout << displayADESInfo(fakeADES0) << std::endl;
//    std::cout << displayADESInfo(fakeADES1) << std::endl;
//    std::cout << displayADESInfo(fakeADES2) << std::endl;
//    std::cout << displayADESInfo(fakeADES3) << std::endl;

    std::vector<Ades> known_ADES;
    known_ADES.push_back(fakeADES0);
  /*  known_ADES.push_back(fakeADES1);
    known_ADES.push_back(fakeADES2);
    known_ADES.push_back(fakeADES3);
   */
    database.addAdes(known_ADES);

    std::cout << "Nb of ades in DB: " << database.getAdesNb() << std::endl;
}
