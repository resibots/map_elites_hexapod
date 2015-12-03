#! /usr/bin/env python
#| This file is a part of the ERC ResiBots project.
#| Copyright 2015, ISIR / Universite Pierre et Marie Curie (UPMC)
#| Main contributor(s): Jean-Baptiste Mouret, mouret@isir.fr
#|                      Antoine Cully, cully@isir.upmc.fr
#|
#| This software is governed by the CeCILL license under French law
#| and abiding by the rules of distribution of free software.  You
#| can use, modify and/ or redistribute the software under the terms
#| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
#| following URL "http://www.cecill.info".
#|
#| As a counterpart to the access to the source code and rights to
#| copy, modify and redistribute granted by the license, users are
#| provided only with a limited warranty and the software's author,
#| the holder of the economic rights, and the successive licensors
#| have only limited liability.
#|
#| In this respect, the user's attention is drawn to the risks
#| associated with loading, using, modifying and/or developing or
#| reproducing the software by the user in light of its specific
#| status of free software, that may mean that it is complicated to
#| manipulate, and that also therefore means that it is reserved for
#| developers and experienced professionals having in-depth computer
#| knowledge. Users are therefore encouraged to load and test the
#| software's suitability as regards their requirements in conditions
#| enabling the security of their systems and/or data to be ensured
#| and, more generally, to use and operate it in the same conditions
#| as regards security.
#|
#| The fact that you are presently reading this means that you have
#| had knowledge of the CeCILL license and that you accept its terms.

import sferes

def build(bld):
    osg = not bld.all_envs['default']['NO_OSG']
    libs = 'ODE ROBDYN EIGEN3 BOOST  ROS BOOST_SYSTEM BOOST_THREAD BOOST_SERIALIZATION BOOST_FILESYSTEM DYNAMIXEL IMU_RAZOR '
    cxxflags = bld.get_env()['CXXFLAGS']
#    if osg:
#        libs += ' OSG'

    model = bld.new_task_gen('cxx', 'staticlib')
    model.source = 'hexapod.cc simu.cpp controllerDuty.cpp  '
    model.includes = '. ../../'
    model.target = 'hexapod'
    model.uselib = libs

    sferes.create_variants(bld,
                           source = 'hexa_duty.cpp',
                           uselib_local = 'sferes2 hexapod',# robot',
                           uselib = libs,
                           target = 'hexa_duty',
                           json = '',
                           variants = ['TEXT','TEXT RANDOMGEN'])





    if osg:
        print 'osg activated'
        modelg = bld.new_task_gen('cxx', 'staticlib')
        modelg.source = 'hexapod.cc simu.cpp controllerDuty.cpp'
        modelg.includes = '. ../../'
        modelg.target = 'hexapodg'
        modelg.uselib = libs
        modelg.cxxflags = cxxflags = ['-DGRAPHIC' ]

        sferes.create_variants(bld,
                           source = 'hexa_duty.cpp',
                           uselib_local = 'sferes2 hexapodg ',
                           uselib = libs,
                           target = 'hexa_duty',
                           json = '',
                           variants = ['GRAPHIC'])
