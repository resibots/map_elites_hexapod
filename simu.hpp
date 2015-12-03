//| This file is a part of the ERC ResiBots project.
//| Copyright 2015, ISIR / Universite Pierre et Marie Curie (UPMC)
//| Main contributor(s): Jean-Baptiste Mouret, mouret@isir.fr
//|                      Antoine Cully, cully@isir.upmc.fr
//|
//| This software is governed by the CeCILL license under French law
//| and abiding by the rules of distribution of free software.  You
//| can use, modify and/ or redistribute the software under the terms
//| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
//| following URL "http://www.cecill.info".
//|
//| As a counterpart to the access to the source code and rights to
//| copy, modify and redistribute granted by the license, users are
//| provided only with a limited warranty and the software's author,
//| the holder of the economic rights, and the successive licensors
//| have only limited liability.
//|
//| In this respect, the user's attention is drawn to the risks
//| associated with loading, using, modifying and/or developing or
//| reproducing the software by the user in light of its specific
//| status of free software, that may mean that it is complicated to
//| manipulate, and that also therefore means that it is reserved for
//| developers and experienced professionals having in-depth computer
//| knowledge. Users are therefore encouraged to load and test the
//| software's suitability as regards their requirements in conditions
//| enabling the security of their systems and/or data to be ensured
//| and, more generally, to use and operate it in the same conditions
//| as regards security.
//|
//| The fact that you are presently reading this means that you have
//| had knowledge of the CeCILL license and that you accept its terms.

#ifndef SIMU_HPP
#define SIMU_HPP

#include <boost/tuple/tuple.hpp>
#include <boost/shared_ptr.hpp>
//#include <robot/quadruped.hh>
#include <ode/box.hh>

//#include "actuator.hpp"
#include "hexapod.hh"
#include "controllerDuty.hpp"
#define MAX_ANG_VEL 11.9380521
#define DYN2RAD 0.00511326929
#define SAMPLING_FREQUENCY 20


#include <fstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/filesystem.hpp>


//#include "robotHexa.hpp"


class Simu
{
public:
  static constexpr float step = 0.015;
  typedef boost::shared_ptr<robot::Hexapod> robot_t;
  typedef std::vector<float> ctrl_t;

  Simu(const ctrl_t& ctrl, const robot_t& robot,std::vector<int> brokenLegs,bool transf=false,float duration=5,int transfer_number=1) : //-1 in simulation, other in reality
    _brokenLegs(brokenLegs),
    _controller(ctrl,brokenLegs),
    _covered_distance(-10002),
    _slam_duration(0.0f),
    _energy(0.0f),
    _env(new ode::Environment_hexa()),
    _transf(transf)
  {

    assert(ctrl.size() == 36);
    _robot = robot->clone(*_env);//robot_t(new robot::MyRobot(*_env, Eigen::Vector3d(0, 0, 0.175)));
#ifdef GRAPHIC
    _robot->accept(_visitor);
#endif
    _env->set_gravity(0, 0, -9.81);

    //if (simReal==false)
    if(_transf)
      {
	_writing_path=create_exp_folder();
	//write_data(ctrl,"ctrl");
      }
    try
      {
	_make_robot_init(duration);
      }
    catch (int e)
      {
	std::cout << "An exception occurred. Exception Nr. " << e << std::endl;
	_covered_distance=-10002;
      }	


    // else
    // _make_robot_init_real();
  }


  /*        Simu(const ctrl_t& ctrl, boost::shared_ptr<RobotHexa> robot, std::vector<int> brokenLegs,bool transf=false,float duration=5,int transfer_number=1) :
      _brokenLegs(brokenLegs),
      _controller(ctrl,brokenLegs),
      _covered_distance(10.0f),
      _slam_duration(0.0f),
      _energy(0.0f)
      //,
      //_env(new ode::Environment(true))
      {
      _writing_path=create_exp_folder();
      _real_robot( *robot,ctrl,duration,transfer_number);
      }
  */
  ~Simu()
  {
   
    // we have to clean in the good order
    _robot.reset();
    _env.reset();
   
  }
  void next_step()
  {
    _robot->next_step(step);
    _env->next_step(step);
#ifdef GRAPHIC
    _visitor.update();
    usleep(1e4);

#endif
  }
  robot_t robot()
  {
    return _robot;
  }

  float covered_distance()
  {
    return _covered_distance;
  }
  float slam_duration()
  {
    return _slam_duration;
  }

  std::vector<float> get_duty_cycle()
  {
    std::vector<float> results;
    float sum=0;
    for(int i=0;i<_behavior_contact_0.size();i++)
      sum+=_behavior_contact_0[i];
    sum/=_behavior_contact_0.size();
    results.push_back(round(sum*100)/100.0);

    sum=0;
    for(int i=0;i<_behavior_contact_1.size();i++)
      sum+=_behavior_contact_1[i];
    sum/=_behavior_contact_1.size();
    results.push_back(round(sum*100)/100.0);



    sum=0;
    for(int i=0;i<_behavior_contact_2.size();i++)
      sum+=_behavior_contact_2[i];
    sum/=_behavior_contact_2.size();
    results.push_back(round(sum*100)/100.0);




    sum=0;
    for(int i=0;i<_behavior_contact_3.size();i++)
      sum+=_behavior_contact_3[i];
    sum/=_behavior_contact_3.size();
    results.push_back(round(sum*100)/100.0);




    sum=0;
    for(int i=0;i<_behavior_contact_4.size();i++)
      sum+=_behavior_contact_4[i];
    sum/=_behavior_contact_4.size();
    results.push_back(round(sum*100)/100.0);




    sum=0;
    for(int i=0;i<_behavior_contact_5.size();i++)
      sum+=_behavior_contact_5[i];
    sum/=_behavior_contact_5.size();
    results.push_back(round(sum*100)/100.0);




    // TODOOOO
    return results;
      
  }



  float energy()
  {
    return _energy;
  }
  float direction()  {return _direction;}
  float arrival_angle() {return _arrival_angle;}
  std::vector<float> final_pos(){return _final_pos;}
  void write_contact(std::string const name)
  {


    std::ofstream workingFile(name.c_str());

    if (workingFile)
      {
	for (int i =0;i<_behavior_contact_0.size();i++)
	  {
	    workingFile<<_behavior_contact_0[i]<<" "<<_behavior_contact_1[i]<<" "<<_behavior_contact_2[i]<<" "<<_behavior_contact_3[i]<<" "<<_behavior_contact_4[i]<<" "<<_behavior_contact_5[i]<<std::endl;
	  }
      }
    else
      {
	std::cout << "ERROR: Impossible to open the file." << std::endl;
      }


  }

  void write_traj(std::string const name)
  {

    std::ofstream workingFile(name.c_str());

    if (workingFile)
      {
	for (int i =0;i<_behavior_traj.size();i++)
	  {
	    workingFile<<_behavior_traj[i][0]<<" "<<_behavior_traj[i][1]<<" "<<_behavior_traj[i][2]<<std::endl;
	  }
      }
    else
      {
	std::cout << "ERROR: Impossible to open the file." << std::endl;
      }


  }



  const std::vector<Eigen::Vector3d>& get_traj()
  {
    return _behavior_traj;
  }
  const std::vector<float>& get_contact(int i)
  {
    switch (i)
      {
      case 0:
	return _behavior_contact_0;
	break;
      case 1:
	return _behavior_contact_1;
	break;
      case 2:
	return _behavior_contact_2;
	break;
      case 3:
	return _behavior_contact_3;
	break;
      case 4:
	return _behavior_contact_4;
	break;
      case 5:
	return _behavior_contact_5;
	break;

      }
    assert(false);
    return _behavior_contact_0;

  }
protected:

  bool stabilize_robot()
  {
    robot_t rob = this->robot();

    //_controller.moveRobot(rob,0);
    // low gravity to slow things down (eq. smaller timestep?)
    _env->set_gravity(0, 0, -9.81/2);
    bool stabilized = false;
    int stab = 0;

    for (size_t s = 0; s < 1000 && !stabilized; ++s)
      {
	Eigen::Vector3d prev_pos = rob->pos();
	//rob->next_step(step);
	//_env->next_step(step);
	next_step();
	//	std::cout<<rob->pos()[2]<<std::endl;
	if ((rob->pos() - prev_pos).norm() < 1e-4)
	  stab++;
	else
	  stab = 0;
	if (stab > 30)
	  stabilized = true;
      }
    _env->set_gravity(0, 0, -9.81);
    return(stabilized);
  }




  //void _real_robot(RobotHexa& robot,const ctrl_t& ctrl,float duration,int transfer_number);
  void _make_robot_init(float duration);
  boost::filesystem::path  create_database_folder();
  boost::filesystem::path create_exp_folder();
  template<typename Data_t>
  void write_data(Data_t data,std::string name);



  template<typename R>
  Eigen::VectorXd _get_state(const R& rob)
  {
    Eigen::VectorXd act_state = Eigen::VectorXd::Zero(rob->servos().size() + rob->motors().size());
    size_t k = 0;
    for (size_t i = 0; i < rob->servos().size(); ++i, ++k)
      act_state[k] = rob->servos()[i]->get_angle(2);
    for (size_t i = 0; i < rob->motors().size(); ++i, ++k)
      act_state[k] = rob->motors()[i]->get_pos();

    return act_state;
  }


  std::vector<int> _brokenLegs;
  std::vector<Eigen::Vector3d> _behavior_traj;
  std::vector<float> _behavior_contact_0;
  std::vector<float> _behavior_contact_1;
  std::vector<float> _behavior_contact_2;
  std::vector<float> _behavior_contact_3;
  std::vector<float> _behavior_contact_4;
  std::vector<float> _behavior_contact_5;
  ControllerDuty _controller;
  robot_t _robot;
  std::vector<float> _final_pos;
  float _direction;
  float _arrival_angle;
  float _covered_distance;
  float _slam_duration;
  float _energy;
  boost::shared_ptr<ode::Environment_hexa> _env;
  bool _transf;
#ifdef GRAPHIC
  renderer::OsgVisitor _visitor;
#endif
  boost::filesystem::path _writing_path;
};

#endif
