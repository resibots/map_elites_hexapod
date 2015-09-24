//#define GRAPHIC


#ifdef GRAPHIC
#include "renderer/osg_visitor.hh"
#endif

#include <sferes/misc.hpp>
#include <numeric>
#include "simu.hpp"


void Simu ::_make_robot_init(float duration)
{
    robot_t rob = this->robot();
    Eigen::Vector3d rot=rob->rot();
    _arrival_angle= atan2( cos(rot[2])* sin(rot[1])* sin(rot[0]) + sin(rot[2])* cos(rot[0]), cos(rot[2])* cos(rot[1]))*180/M_PI;
    //std::cout<<rot[0]<< " " << rot[1] <<" "<<rot[2]<<std::endl;
    // std::cout<<"initial angle "<<_arrival_angle<<std::endl;
    Eigen::Vector3d target_pos(0,2,0.2);

    _controller.moveRobot(rob,0);
    /* float t = 0.0f;
    while (t < 1)
    {
        t += step;
        next_step();
	}*/
    /*if(!stabilize_robot())
    {
       _energy=1e20;
       _covered_distance=1e10;
       std::cout<<"non stab"<<std::endl;
       return;
    }*/

    Eigen::Vector3d prev_pos = rob->pos();

    float t=0;
    int index = 0;
#ifdef GRAPHIC
    while (t < duration && !_visitor.done())
#else
    while (t < duration)
#endif
    {

        _controller.moveRobot(rob,t);
	if(_robot->bodies()[0]->get_in_contact() || _env->get_colision_between_legs() || _robot->pos()[2]>0.5)
	  {
	    #ifdef GRAPHIC
	    std::cout<<"mort subite"<<std::endl;
	    #endif
	    _covered_distance=-10002;
	    return;
	  }
        int nbCassee=0;
        if (index%2==0)
            for (unsigned i = 0; i < 6; ++i)
            {
                switch (i)
                {
                case 0:
                    if (_controller.isBroken(i))
                    {
                        _behavior_contact_0.push_back(0);
                        nbCassee++;
                    }
                    else
                    {
                        _behavior_contact_0.push_back( _robot->bodies()[(i-nbCassee) * 3 + 3]->get_in_contact() );
                    }
                    break;
                case 1:
                    if (_controller.isBroken(i))
                    {
                        _behavior_contact_1.push_back(0);
                        nbCassee++;
                    }
                    else
                    {
                        _behavior_contact_1.push_back( _robot->bodies()[(i-nbCassee) * 3 + 3]->get_in_contact() );
                    }
                    break;
                case 2:
                    if (_controller.isBroken(i))
                    {
                        _behavior_contact_2.push_back(0);
                        nbCassee++;
                    }
                    else
                    {
                        _behavior_contact_2.push_back( _robot->bodies()[(i-nbCassee) * 3 + 3]->get_in_contact() );
                    }
                    break;
                case 3:
                    if (_controller.isBroken(i))
                    {
                        _behavior_contact_3.push_back(0);
                        nbCassee++;
                    }
                    else
                    {
                        _behavior_contact_3.push_back( _robot->bodies()[(i-nbCassee) * 3 + 3]->get_in_contact() );
                    }
                    break;
                case 4:
                    if (_controller.isBroken(i))
                    {
                        _behavior_contact_4.push_back(0);
                        nbCassee++;
                    }
                    else
                    {
                        _behavior_contact_4.push_back( _robot->bodies()[(i-nbCassee) * 3 + 3]->get_in_contact() );
                    }
                    break;
                case 5:
                    if (_controller.isBroken(i))
                    {
                        _behavior_contact_5.push_back(0);
                        nbCassee++;
                    }
                    else
                    {
                        _behavior_contact_5.push_back( _robot->bodies()[(i-nbCassee) * 3 + 3]->get_in_contact() );
                    }
                    break;
                }
            }
        _behavior_traj.push_back(rob->pos());

        t += step;
        next_step();

        ++index;

    }

    Eigen::Vector3d end_mov_pos = rob->pos();
    //std::cout<<"fin mov:"<<end_mov_pos<<std::endl;
    stabilize_robot();
    Eigen::Vector3d end_stab_pos = rob->pos();

    //std::cout<<"fin stab:"<<end_stab_pos<<std::endl;
    if((end_stab_pos-end_mov_pos).norm()>0.2)
      {
	    _covered_distance=-10002;
	    return;
      }
    // _covered_distance=sqrt((next_pos[0] - target_pos[0])*(next_pos[0] - target_pos[0])+(next_pos[1] - target_pos[1])*(next_pos[1] - target_pos[1])+(next_pos[2] - target_pos[2])*(next_pos[2] - target_pos[2]));

    // _covered_distance=2-_covered_distance;
  
    //    _covered_distance = round(next_pos[1]*100) / 100.0f;

    _final_pos.resize(2);
    _final_pos[0]=end_stab_pos[0]-prev_pos[0];
    _final_pos[1]=end_stab_pos[1]-prev_pos[1];
    //    std::cout<<"final pos:"<<_final_pos[0]<<" "<< _final_pos[1]<<std::endl;
    //    _covered_distance = round(sqrt(next_pos[0]*next_pos[0]+next_pos[1]*next_pos[1]+next_pos[2]*next_pos[2])*100) / 100.0f;
   _covered_distance = round(_final_pos[1]*100) / 100.0f;
    
}


/*void Simu ::_real_robot(RobotHexa& robot,const ctrl_t& ctrl,float duration,int transfer_number)
{

    write_data(ctrl,"ctrl");

    bool ok=false;
    do
    {
        try
        {
            std::cin.clear();
            std::cout<<"Reset Robot"<<std::endl;
            robot.reset();
            std::cout<<"Transfert on the Robot"<<std::endl;
            robot.transfer(_controller, duration,transfer_number);
            _behavior_contact_0=robot.get_contact(0);
            _behavior_contact_1=robot.get_contact(1);
            _behavior_contact_2=robot.get_contact(2);
            _behavior_contact_3=robot.get_contact(3);
            _behavior_contact_4=robot.get_contact(4);
            _behavior_contact_5=robot.get_contact(5);
            std::cout<<"relax robot"<<std::endl;
            robot.relax();

        }
        catch (dynamixel::Error e)
        {
            std::cerr << "error (dynamixel): " << e.msg() << std::endl;
            std::cout<<"closing serials"<<std::endl;
            robot.close_usb_controllers();
            robot.send_ros_stop(1,transfer_number);
        }
        std::cin.clear();
        std::cout<<"transfert ok? :"<<std::endl;
        std::cin>> ok;
        std::cin.clear();
        std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
    }
    while (!ok);
    //std::cout<<"entrez la distance parcourue :"<<std::endl;
    //std::cin>> _covered_distance;
    //std::cin.clear();
    //std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
    //_covered_distance=10 - _covered_distance;
    _covered_distance=robot.covered_distance();
    _slam_duration = robot.slam_duration();
    
    _final_pos=robot.final_pos();
        
    _arrival_angle=robot.final_angle();

    
    std::vector<std::vector<float> > contacts;
    contacts.push_back(robot.get_contact(0));
    contacts.push_back(robot.get_contact(1));
    contacts.push_back(robot.get_contact(2));
    contacts.push_back(robot.get_contact(3));
    contacts.push_back(robot.get_contact(4));
    contacts.push_back(robot.get_contact(5));
    write_data(contacts,"contacts");
    write_data(robot.get_angles(),"angles");
    write_data(-_covered_distance,"fit");


    }
*/

template<typename Data_t>
void Simu::write_data(Data_t data,std::string name)
{
    //boost::filesystem::path expDir = create_exp_folder();

    std::ofstream ofs((_writing_path.string()+std::string("/")+name).c_str());
    // save data to archive
    {
        boost::archive::text_oarchive  oa(ofs);
        // write class instance to archive
        oa << data;
        // archive and stream closed when destructors are called
    }
}



boost::filesystem::path Simu::create_database_folder()
{
    boost::filesystem::path thePath = boost::filesystem::current_path();
    boost::filesystem::path newDir = thePath / "database";

    if (!boost::filesystem::exists(newDir) || !boost::filesystem::is_directory(newDir))    // does p actually exist?
    {


        bool bDidCreate = boost::filesystem::create_directory(newDir);

        if (!bDidCreate)
            std::cout << "Databse's directory creation failed!" << std::endl;
    }

    return newDir;
}

boost::filesystem::path Simu::create_exp_folder()
{
    struct tm today;
    time_t maintenant;

    time(&maintenant);
    today = *localtime(&maintenant);
    std::ostringstream oss;
    oss<<today.tm_year + 1900<<"-"<< today.tm_mon + 1<<"-"<< today.tm_mday<<"_"<<today.tm_hour<<"_"<<today.tm_min<<"_"<<today.tm_sec;


    // create and open a character archive for output


    boost::filesystem::path newDir = create_database_folder();

    newDir = newDir / oss.str().c_str();

    if (!boost::filesystem::exists(newDir) || !boost::filesystem::is_directory(newDir))    // does p actually exist?
    {


        bool bDidCreate = boost::filesystem::create_directory(newDir);

        if (!bDidCreate)
            std::cout << "Exp's directory creation failed!" << std::endl;
    }
    return newDir;
}
