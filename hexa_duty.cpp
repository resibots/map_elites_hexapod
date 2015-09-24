
//#define EVAL_ALL
#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/StdVector>
#include <Eigen/Core>
//#include <Eigen/Array>
#include <time.h>
#include <netinet/in.h>
//#define NO_WHEEL
#define Z_OBSTACLE
//#define GRAPHIC
#include <unistd.h>
#include <iostream>
#include <numeric>
#include <tbb/parallel_reduce.h>
#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <sferes/phen/parameters.hpp>
#include <sferes/gen/evo_float.hpp>
#include <sferes/gen/sampled.hpp>
//#include <sferes/ea/nsga2.hpp>

#include "nsga2_genocrowd.hpp"
#include <sferes/stat/pareto_front.hpp>
#include <sferes/modif/dummy.hpp>
#include <sferes/run.hpp>
#include <sferes/stc.hpp>


#ifndef RANDOMGEN
#include "mole_grid.hpp"
#else
#include "mole_random_gen.hpp"
#endif

#include "stat_progress_archive.hpp"
#include "behavior.hpp"


#define NO_MPI


//#define NO_PARALLEL
//#define NODIV

#include "ode/box.hh"

//#define GRAPHIC


//#undef GRAPHIC
#ifdef GRAPHIC
#define NO_PARALLEL
#include "renderer/osg_visitor.hh"
#endif


#ifndef NO_PARALLEL
#include <sferes/eval/parallel.hpp>
#ifndef NO_MPI
#include <sferes/eval/mpi.hpp>
#endif
#else
#include <sferes/eval/eval.hpp>
#endif



#include "hexapod.hh"
#include "simu.hpp"
#ifdef ROBOT
#include "robotHexa.hpp"
#endif
#include "diversity_modifier.hpp"
//#include "svm_modifier.hpp"




#include <boost/fusion/sequence/intrinsic/at.hpp>



using namespace sferes;
using namespace boost::assign;
using namespace sferes::gen::evo_float;




struct Params
{
  struct surrogate
  {
    SFERES_CONST int nb_transf_max = 10;
    SFERES_CONST float tau_div = 0.05f;
    //SFERES_ARRAY(float, features_norm, 1.0f, 0.5f, 0.5f);
  };

  struct sampled
  {
    //SFERES_ARRAY(float, values,0,0.25,0.5,0.75,1);
    SFERES_ARRAY(float, values,0.00,0.05,
		 0.10,0.15,
		 0.20,0.25,
		 0.30,0.35,
		 0.40,0.45,
		 0.50,0.55,
		 0.60,0.65,
		 0.70,0.75,
		 0.80,0.85,
		 0.90,0.95,
		 1);
    SFERES_CONST float mutation_rate=0.05f;
    SFERES_CONST float cross_rate = 0.00f;
    SFERES_CONST bool ordered = false;
  };
  struct evo_float
  {
    SFERES_CONST float cross_rate = 0.0f;
    SFERES_CONST float mutation_rate = 1.0f/36.0f;
    SFERES_CONST float eta_m = 10.0f;
    SFERES_CONST float eta_c = 10.0f;
    SFERES_CONST mutation_t mutation_type = polynomial;
    SFERES_CONST cross_over_t cross_over_type = sbx;
  };
  struct pop
  {
    SFERES_CONST unsigned size = 200;
    SFERES_CONST unsigned nb_gen = 100001;
    SFERES_CONST int dump_period = 50;
    SFERES_CONST int initial_aleat = 1;

  };
  struct parameters
  {
    SFERES_CONST float min = 0.0f;
    SFERES_CONST float max = 1.0f;
  };
};



// typedef gen::EvoFloat<36, Params> genom_t;
typedef gen::Sampled<36,Params> genom_t;



struct ordering
{
  bool operator ()(std::pair<int, float> const& a, std::pair<int, float> const& b)
  {
    return a.second < b.second;
  }
};




///variables globales------------


namespace global
{
  boost::shared_ptr<ode::Environment_hexa> env;
#ifndef ROBOT
  boost::shared_ptr<robot::Hexapod> robot;
#else
  boost::shared_ptr<RobotHexa> robot;
#endif

  std::vector<int> brokenLegs;
     
};
///---------------------------



void init_simu(  int argc ,char** argv,bool master)
{
  global::env = boost::shared_ptr<ode::Environment_hexa>(new ode::Environment_hexa());
#ifndef ROBOT
    //global::brokenLegs.push_back(0);
    //global::brokenLegs.push_back(1);
    //global::brokenLegs.push_back(2);
    //global::brokenLegs.push_back(3);
    //global::brokenLegs.push_back(4);
    //global::brokenLegs.push_back(5);


  global::robot = boost::shared_ptr<robot::Hexapod>(new robot::Hexapod(*global::env, Eigen::Vector3d(0, 0, 0.1),global::brokenLegs));//a robot with the leg number 0 broken
#else
    if(master)
    global::robot->initRosNode(argc , argv);
#endif

  float step = 0.001;

  // low gravity to slow things down (eq. smaller timestep?)
  global::env->set_gravity(0, 0, -9.81);
  bool stabilized = false;
  int stab = 0;
  for (size_t s = 0; s < 1000 && !stabilized; ++s)
    {
      Eigen::Vector3d prev_pos = global::robot->pos();
      global::robot->next_step(step);
      global::env->next_step(step);

      if ((global::robot->pos() - prev_pos).norm() < 1e-5)
	stab++;
      else
	stab = 0;
      if (stab > 100)
	stabilized = true;
    }
  assert(stabilized);
  global::env->set_gravity(0, 0, -9.81);
}






SFERES_FITNESS(FitAdapt, sferes::fit::Fitness)
{
 public:
  template<typename Indiv>

    void eval(Indiv& indiv, bool write_objs = false)
  {

    this->_objs.resize(2);
    std::fill(this->_objs.begin(), this->_objs.end(), 0);
    _dead=false;
    _eval(indiv, write_objs);
  }

    template<class Archive>
      void serialize(Archive & ar, const unsigned int version)
    {
      dbg::trace trace("fit", DBG_HERE);

      ar & boost::serialization::make_nvp("_value", this->_value);
      ar & boost::serialization::make_nvp("_objs", this->_objs);

      ar & BOOST_SERIALIZATION_NVP(_behavior);
      ar & BOOST_SERIALIZATION_NVP(_covered_distance);
    ar & BOOST_SERIALIZATION_NVP(_dead);
    }

  Behavior& behavior(){return this->_behavior;}
  const Behavior& behavior() const{return this->_behavior;}
  float covered_distance(){return this-> _covered_distance;}
  bool dead(){return this->_dead;}

 protected:
  Behavior _behavior;
  float _covered_distance;
  bool _dead;


  template<typename Indiv>
    void _eval(Indiv& indiv, bool write_objs)
  {


    // std::cout <<"debut eval "<<indiv.size()<<std::endl;

    // copy of controler's parameters

    _behavior.controller.clear();

    for (int i=0;i < indiv.size();++i)
      {
	//std::cout<<indiv.data(i)<<" ";
	_behavior.controller.push_back(indiv.data(i));
      }
    //std::cout<<std::endl;

    if (this->mode() == sferes::fit::mode::view)
      {
	//            return;
      }



    //launching the simulation
    //    std::cout <<"debut simu eval "<<std::endl;
    Simu simu = Simu(_behavior.controller, global::robot,global::brokenLegs);
    /*    Simu simu2 = Simu(_behavior.controller, global::robot,global::brokenLegs);

    if(fabs(simu.covered_distance()-simu2.covered_distance())>=0.01)
      std::cout<<"DIFFERENCE"<<std::endl;
    */
    //std::cout <<"fin simu"<<std::endl;
    _covered_distance=simu.covered_distance();
    
    _behavior.covered_distance=_covered_distance;

    //        std::cout<<"fin recup donnés"<<std::endl;



	
	   
	if(this->_covered_distance<-1000)
      {
	_dead=true;
	//mort subite
	_behavior.duty_cycle.resize(6);
	_behavior.duty_cycle[0]=0;
	_behavior.duty_cycle[1]=0;
	_behavior.duty_cycle[2]=0;
	_behavior.duty_cycle[3]=0;
	_behavior.duty_cycle[4]=0;
	_behavior.duty_cycle[5]=0;
	_behavior.covered_distance=-10000;

      }
    else
      _behavior.duty_cycle= simu.get_duty_cycle();

    //	this->_objs[0]=this->_covered_distance;
    //writting objectives if needed
    if (write_objs)
      {
	std::cout<<std::endl<<"fitness" << this->_objs[0] << std::endl;

      }
    //        std::cout<<"fin eval"<<std::endl;
  }

};

typedef FitAdapt<Params> fit_t;
typedef phen::Parameters<genom_t, fit_t, Params> phen_t;

struct elem_archive
{
  std::vector<float> duty_cycle;
  float fit;
  std::vector<float> controller;

};

SFERES_STAT(SanityCheck,sferes::stat::Stat)
{
 public:
  template<typename Ea>
    void refresh(Ea& ea)
  {
    std::cout<<"sanity check"<<std::endl;
    int error;
    for (size_t i = 0; i < ea.array().size(); ++i)
      for (size_t j = 0; j < ea.array().size(); ++j)
	for (size_t k = 0; k < ea.array().size(); ++k)
	  for (size_t l = 0; l < ea.array().size(); ++l)
	    for (size_t m = 0; m < ea.array().size(); ++m)
	      for (size_t n = 0; n < ea.array().size(); ++n)
		if (ea.array()[i][j][k][l][m][n])
		  {
		    Simu simu = Simu(ea.array()[i][j][k][l][m][n]->fit().behavior().controller, global::robot,global::brokenLegs);
		    if(fabs(ea.array()[i][j][k][l][m][n]->fit().behavior().covered_distance-simu.covered_distance())>0.01)
		      {
			error++;
			std::cout<<error<< " archive : "<<fabs(ea.array()[i][j][k][l][m][n]->fit().behavior().covered_distance-simu.covered_distance())<<std::endl;
		      }

		  }
    if (ea.gen() % Params::pop::dump_period == 0)
      {
	std::cout<<"check archive file"<<std::endl;
	std::ostringstream oss;
	// écrire un nombre dans le flux                                                                                                
	oss << ea.gen();
	lecture((ea.res_dir() + "/archive"+oss.str()+".dat"));
      }
  }

};

void lecture(std::string name)
{
#ifdef ROBOT
  global::robot = boost::shared_ptr<RobotHexa>(new RobotHexa);
#endif
  //initilisation of the simulation and the simulated robot

  
  
  
  std::vector<elem_archive> archive;
  std::ifstream monFlux(name.c_str());  //Ouverture d'un fichier en lecture 
  if(monFlux)
    {
      while(!monFlux.eof())
        {
	  elem_archive elem;
          for(int i =0;i<43;i++)
            {
              if(monFlux.eof())
                break;
              float data;
              monFlux>>data;
	      if(i<=5)
		elem.duty_cycle.push_back(data);
	      if(i==6)
		elem.fit=data;
	      if(i>=7)
		elem.controller.push_back(data);
          
            }
          if(elem.controller.size()==36)
            archive.push_back(elem);
          
        }
    }
  else
    {
      std::cout << "ERREUR: Impossible d'ouvrir le fichier en lecture." << std::endl;
      return ;
    }

  std::cout<<archive.size()<<" controllers loaded"<<std::endl;
  int i=0;
  int error=0;
  for(;i<archive.size();i++)
    {
      if(i%100==0)
	std::cout<<"progression: "<<(float)i/archive.size()<<std::endl;
      //      ofile<<i<<": ";                                                                                                               
      /*phen_t indiv;
            for(int k=0;k<indiv.gen().size();k++)
        indiv.gen().data(k,archive[i].controller[k]);
      indiv.develop();
      indiv.fit().eval(indiv);*/
      Simu simu = Simu(archive[i].controller, global::robot,global::brokenLegs);


      //      if(fabs(indiv.fit().behavior().covered_distance-archive[i].fit)>0.01)
      if(fabs(simu.covered_distance()-archive[i].fit)>0.01)
        {
	  error++;
	  std::cout<<error<< " : "<<fabs(simu.covered_distance()-archive[i].fit)<<std::endl;//<<fabs(indiv.fit().behavior().covered_distance-archive[i].fit)<<std::endl;
        }




    }
  

}

int main(int argc, char **argv)
{


   //initialisation of the simulator
  dInitODE();






#ifndef NO_PARALLEL
#ifndef NO_MPI
    typedef eval::Mpi<Params> eval_t;    
#else
    typedef eval::Parallel<Params> eval_t;
#endif
#else

    typedef eval::Eval<Params> eval_t;
#endif

    typedef boost::fusion::vector</*SanityCheck<phen_t,Params>,*/sferes::stat::ProgressArchive<phen_t,Params> >  stat_t;
    typedef ea::MoleGrid<phen_t, eval_t, stat_t,modif::Dummy<Params>, Params> ea_t;

  if(argc ==37)
    { 

#ifdef ROBOT
	 std::cout<<"init ROBOT"<<std::endl;
	 global::robot = boost::shared_ptr<RobotHexa>(new RobotHexa);
#endif
      
	init_simu(argc, argv,true);
	//init_simu(argc, argv,false);                                                   
	//      genom_t indiv;
      std::cout << "LOADING..." << std::endl;
      std::vector<float> ctrl;
      for(int i=0;i < 36;++i)
	   {
	     //   indiv.data(i, atof(argv[i+1]));
	     //indiv.set_data(i, atof(argv[i+1])*4);
	     ctrl.push_back(atof(argv[i+1]));
	   }
      //      fit_t fitness;
      std::cout << "fin loading..." << std::endl;
      //fitness.eval(indiv);
      Simu simu = Simu(ctrl, global::robot,std::vector<int>(),true,5);
  
      //      std::cout<<"covered_distance simu: "<<fitness.covered_distance()<<std::endl;
      std::cout<<"covered_distance real: "<<simu.covered_distance()<<std::endl;
      std::cout<<"duty cycles:"<<std::endl;
      
      simu.write_traj("traj.dat");
      
      //      for(int i=0;i< fitness.behavior().duty_cycle.size();i++)
      //	std::cout<<fitness.behavior().duty_cycle[i]<<" ";
      std::cout<<std::endl;
      
      global::robot.reset();

      global::env.reset();
      dCloseODE();

      return 0;
    }

  // lecture fichier results.dat
  if(argc == 2)
      {
	init_simu(argc, argv,true);
	lecture(argv[1]);
      }


    
  ea_t ea;

#ifndef NO_MPI
    if (ea.eval().rank() == 0)
      {
#ifdef ROBOT
	 std::cout<<"init ROBOT"<<std::endl;
	 global::robot = boost::shared_ptr<RobotHexa>(new RobotHexa);
#endif
	//initilisation of the simulation and the simulated robot
	 std::cout<<"init SIMU"<<std::endl;
	init_simu(argc, argv,true);
      }
    else
      init_simu(argc, argv,false);
#else
#ifdef ROBOT
	global::robot = boost::shared_ptr<RobotHexa>(new RobotHexa);
#endif
	//initilisation of the simulation and the simulated robot
	init_simu(argc, argv,true);

#endif
 std::cout<<"debut run"<<std::endl;

  run_ea(argc, argv, ea);
  std::cout <<"fin run"<<std::endl;

  global::robot.reset();

  global::env.reset();
  dCloseODE();
  std::cout <<"fin"<<std::endl;
  return 0;
}
