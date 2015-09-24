#! /usr/bin/env python
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


