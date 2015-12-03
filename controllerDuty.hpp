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

#ifndef CONTROLLERPHASE_HPP
#define CONTROLLERPHASE_HPP

#include <vector>
#include <boost/shared_ptr.hpp>
#include "robot/hexapod.hh"
#include <boost/array.hpp>
#define ARRAY_DIM 100



class ControllerDuty
{
public:
  typedef boost::shared_ptr<robot::Hexapod> robot_t;
  typedef boost::array<float,ARRAY_DIM> array_t;
protected :
  std::vector<array_t >& selector(int leg);
  std::vector<array_t > _legs0commands;
  std::vector<array_t > _legs1commands;
  std::vector<array_t > _legs2commands;
  std::vector<array_t > _legs3commands;
  std::vector<array_t > _legs4commands;
  std::vector<array_t > _legs5commands;


  std::vector<int> _brokenLegs;



public :



  ControllerDuty(const std::vector<float>& ctrl,std::vector<int> brokenLegs):_brokenLegs(brokenLegs)
  {
    if(ctrl.size()!=36)
      std::cout<<ctrl.size()<<std::endl;
    assert(ctrl.size()==36);
  
    _legs0commands.push_back(control_signal(ctrl[0],ctrl[1],ctrl[2]));
    _legs0commands.push_back(control_signal(ctrl[3],ctrl[4],ctrl[5]));
    _legs0commands.push_back(control_signal(ctrl[3],ctrl[4],ctrl[5]));

    _legs1commands.push_back(control_signal(ctrl[6],ctrl[7],ctrl[8]));
    _legs1commands.push_back(control_signal(ctrl[9],ctrl[10],ctrl[11]));
    _legs1commands.push_back(control_signal(ctrl[9],ctrl[10],ctrl[11]));

    _legs2commands.push_back(control_signal(ctrl[12],ctrl[13],ctrl[14]));
    _legs2commands.push_back(control_signal(ctrl[15],ctrl[16],ctrl[17]));
    _legs2commands.push_back(control_signal(ctrl[15],ctrl[16],ctrl[17]));

    _legs3commands.push_back(control_signal(ctrl[18],ctrl[19],ctrl[20]));
    _legs3commands.push_back(control_signal(ctrl[21],ctrl[22],ctrl[23]));
    _legs3commands.push_back(control_signal(ctrl[21],ctrl[22],ctrl[23]));

    _legs4commands.push_back(control_signal(ctrl[24],ctrl[25],ctrl[26]));
    _legs4commands.push_back(control_signal(ctrl[27],ctrl[28],ctrl[29]));
    _legs4commands.push_back(control_signal(ctrl[27],ctrl[28],ctrl[29]));

    _legs5commands.push_back(control_signal(ctrl[30],ctrl[31],ctrl[32]));
    _legs5commands.push_back(control_signal(ctrl[33],ctrl[34],ctrl[35]));
    _legs5commands.push_back(control_signal(ctrl[33],ctrl[34],ctrl[35]));
    
    
  }



  array_t control_signal(float amplitude, float phase, float duty_cycle);
  
  
  bool isBroken(int leg)
  {
    for (int j=0;j<_brokenLegs.size();j++)
        {
            if (leg==_brokenLegs[j])
            {
                return true;
            }
        }
        return false;
  }


    void moveRobot(robot_t& robot, float t);
      std::vector<int> get_pos_dyna(float t);
  // std::vector<int> get_speeds_dyna( );
  //  std::vector<bool> get_directions_dyna( );


};

#endif
