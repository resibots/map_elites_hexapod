//| This file is a part of the sferes2 framework.
//| Copyright 2009, ISIR / Universite Pierre et Marie Curie (UPMC)
//| Main contributor(s): Jean-Baptiste Mouret, mouret@isir.fr
//|
//| This software is a computer program whose purpose is to facilitate
//| experiments in evolutionary computation and evolutionary robotics.
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




#ifndef MOLE_NEIGH_HPP_
#define MOLE_NEIGH_HPP_


#include <algorithm>
#include <limits>

#include <boost/foreach.hpp>
#include <boost/multi_array.hpp>
#include <boost/array.hpp>

#include <sferes/stc.hpp>
#include <sferes/parallel.hpp>
#include <sferes/ea/ea.hpp>
#include <sferes/fit/fitness.hpp>
#include <sferes/ea/dom_sort.hpp>
//#include <ssrc/spatial/kd_tree.h>


namespace sferes
{
  namespace ea
  {

    // Main class
    SFERES_EA(MoleNeigh, Ea)
    {
    public:
      typedef boost::shared_ptr<Phen> indiv_t;
      typedef typename std::vector<indiv_t> pop_t;
      typedef typename pop_t::iterator it_t;
      typedef typename std::vector<std::vector<indiv_t> > front_t;
      typedef boost::array<float, 2> point_t;
      typedef boost::shared_ptr<Phen> phen_t;
      typedef boost::multi_array<phen_t, 2> array_t;
      typedef boost::multi_array<int, 2> hits_t;



      MoleNeigh()
        {

        }

      void random_pop()
      {

	parallel::init();
	this->_pop.resize(Params::pop::size);
	BOOST_FOREACH(boost::shared_ptr<Phen>&indiv, this->_pop)
          {
            indiv = boost::shared_ptr<Phen>(new Phen());
            indiv->random();
	    indiv->develop();
          }
	this->_eval.eval(this->_pop, 0, this->_pop.size());
	BOOST_FOREACH(boost::shared_ptr<Phen>&indiv, this->_pop)
          _add_to_archive(indiv);

	//  _compute_scores(this->_pop);
      }

      void epoch()
      {

	this->_pop.clear();

	_added=0;
	_swapped=0;
	pop_t ptmp;
	for (size_t i = 0; i < Params::pop::size; ++i)
          {
            indiv_t p1 = _selection(this->_archive); //fronts[0]); //this->_pop);                                                                                                                               
            indiv_t p2 = _selection(this->_archive); //fronts[0]); //this->_pop);                                                                                                                               
	    boost::shared_ptr<Phen> i1, i2;
            p1->cross(p2, i1, i2);
            i1->mutate();
            i2->mutate();
            i1->develop();
            i2->develop();
            ptmp.push_back(i1);
            ptmp.push_back(i2);
          }


	this->_eval.eval(ptmp, 0, ptmp.size());
	for (size_t i = 0; i < ptmp.size(); ++i)
	  _add_to_archive(ptmp[i]);

	//          if (this->gen() % 200 == 0)
	/*          {
		    std::cout << "writing..." << this->gen() << std::endl;
		    std::string fname = boost::lexical_cast<std::string>(this->gen()) + std::string(".dat");
		    std::ofstream ofs(fname.c_str());
		    for (size_t i = 0; i < res_x; ++i)
		    for (size_t j = 0; j < res_y; ++j)
		    if (_array[i][j])
		    ofs << i / (float) res_x
		    << " " << j / (float) res_y
		    << " " << _array[i][j]->fit().value() << std::endl;
		    }*/
	std::ofstream ofsvar("variations.dat",std::ios_base::app);
	std::cout<<"archive size:"<<_archive.size()<<" add=" << _added<<" swapped="<< _swapped<<std::endl;
	ofsvar<<"archive size:"<<_archive.size()<<" add=" << _added<<" swapped="<< _swapped<<std::endl;

	if (this->gen() % Params::pop::dump_period == 0)
	  {
	    // créer un flux de sortie
	    std::ostringstream oss;
	    // écrire un nombre dans le flux
	    oss << this->gen();
	    // récupérer une chaîne de caractères
	    std::ofstream ofs((this->res_dir() + "/archive"+oss.str()+".dat").c_str());
	    for (size_t i = 0; i < _archive.size(); ++i)
	      {
		//	    assert(_archive[i].size() == 2);
	    
		ofs<<_archive[i]->fit().behavior().duty_cycle[0]<<" "<<_archive[i]->fit().behavior().duty_cycle[1]<<" "<<_archive[i]->fit().behavior().duty_cycle[2]<<" "<<_archive[i]->fit().behavior().duty_cycle[3]<<" "<<_archive[i]->fit().behavior().duty_cycle[4]<<" "<<_archive[i]->fit().behavior().duty_cycle[5]<<"    "<<_archive[i]->fit().behavior().covered_distance<<"    ";
		for(int j=0;j<_archive[i]->fit().behavior().controller.size();j++)
		  ofs<<_archive[i]->fit().behavior().controller[j]<<" ";
		ofs<<std::endl;
	      }
	  }

      }

      std::vector<indiv_t > & archive() { return _archive; }
    protected:
      std::vector<indiv_t > _archive;

      int _added;
      int _swapped;

      bool _add_to_archive(indiv_t i1)
      {


	if(i1->fit().dead())
	  return false;

	static const    size_t  k = 15; //voisinage

	static const    float  rho_min = 0.60; //voisinage

	tbb::parallel_sort(_archive.begin(),
                           _archive.end(),
                           compare_dist_f<phen_t>(i1));

	float n=0;
	if (_archive.size() >= k)
	  for (size_t j = 0; j < k; ++j)
	    {
	      n += dist(_archive[j]->fit().behavior().duty_cycle, i1->fit().behavior().duty_cycle);
	    }

	else
	  n += 1;
	n /= k;

	bool isAlreadyIn=true;
	if(_archive.size()==0)
	  isAlreadyIn=false;
	else
	  for(int i=0;i<i1->data().size();i++)
	    if(_archive[0]->data(i)!=i1->data(i))
	      {
		isAlreadyIn=false;
		break;
	      }
	
	
	
	if (!isAlreadyIn && (_archive.size() < k || n > rho_min))
	  {
	    _added++;
	    _archive.push_back(i1);
	    return true;
	  }
	else
	  {
	    if(i1->fit().behavior().covered_distance>_archive[0]->fit().behavior().covered_distance && dist(i1->fit().behavior().duty_cycle,_archive[0]->fit().behavior().duty_cycle)<rho_min/1.0f)
	      {
		_swapped++;
		_archive[0]=i1;
		return true;
	      }


	    return false;
	  }

      }

      // --- Pareto tournament selection --
      indiv_t _selection(const pop_t& pop)
      {
	int x1 = misc::rand< int > (0, _archive.size());
	return _archive[x1];

      }


      template<typename V1, typename V2>
	static float dist(const V1& v1, const V2& v2)
      {
	assert(v1.size() == v2.size());
	typename V1::const_iterator it1 = v1.begin(), it2 = v2.begin();
	float res = 0.0f;
	while (it1 != v1.end())
	  {
	    float v = (float)*it1 - (float)*it2;
	    res += v * v;
	    ++it1;
	    ++it2;
	  }
	return sqrtf(res);
      }




      template<typename T>
	struct compare_dist_f
      {
	compare_dist_f(const T& v) : _v(v) {}
	const T _v;
	bool operator()(const T& v1, const T& v2) const
	{
	  assert(v1->fit().behavior().duty_cycle.size() == _v->fit().behavior().duty_cycle.size());
	  assert(v2->fit().behavior().duty_cycle.size() == _v->fit().behavior().duty_cycle.size());
	  return dist(v1->fit().behavior().duty_cycle, _v->fit().behavior().duty_cycle) < dist(v2->fit().behavior().duty_cycle, _v->fit().behavior().duty_cycle);
	}
      };


    };
  }
}
#endif


