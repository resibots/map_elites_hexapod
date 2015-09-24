#ifndef BEHAVIOR_HPP
#define BEHAVIOR_HPP


struct Behavior
{
  std::vector<float> duty_cycle;
    float covered_distance;
  //float transferability;
  
  std::vector<float> controller;
  // std::vector<float> features;
 template<class Archive>
      void serialize(Archive & ar, const unsigned int version)
    {
      dbg::trace trace("behavior", DBG_HERE);
      //ar & BOOST_SERIALIZATION_NVP(this->_value);
      //ar & BOOST_SERIALIZATION_NVP(this->_objs);

     
      ar & BOOST_SERIALIZATION_NVP(duty_cycle);
      ar & BOOST_SERIALIZATION_NVP(covered_distance);

      ar & BOOST_SERIALIZATION_NVP(controller);


    }
};
#endif
