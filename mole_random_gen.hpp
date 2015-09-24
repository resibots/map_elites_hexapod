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




#ifndef MOLE_HPP_
#define MOLE_HPP_


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
    SFERES_EA(MoleGrid, Ea)
    {
    public:
      typedef boost::shared_ptr<Phen> indiv_t;
      typedef typename std::vector<indiv_t> pop_t;
      typedef typename pop_t::iterator it_t;
      typedef typename std::vector<std::vector<indiv_t> > front_t;
      typedef boost::array<float, 6> point_t;
      typedef boost::shared_ptr<Phen> phen_ptr_t;
      typedef boost::multi_array<phen_ptr_t, 6> array_t;
      typedef boost::multi_array<int, 6> hits_t;

      SFERES_CONST size_t res =5;
      SFERES_CONST float epsilon =0.05;

      MoleGrid() :
	_array(boost::extents[res][res][res][res][res][res]),
	_hits(boost::extents[res][res][res][res][res][res])
	  {
	    for (size_t i = 0; i < res; ++i)
	      for (size_t j = 0; j < res; ++j)
		for (size_t k = 0; k < res; ++k)
		  for (size_t l = 0; l < res; ++l)
		    for (size_t m = 0; m < res; ++m)
		      for (size_t n = 0; n < res; ++n)
			_hits[i][j][k][l][m][n] = 0;
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
	this->_eval_pop(this->_pop, 0, this->_pop.size());
	BOOST_FOREACH(boost::shared_ptr<Phen>&indiv, this->_pop)
          _add_to_archive(indiv);
	//  _compute_scores(this->_pop);
      }

      void epoch()
      {
	this->_pop.clear();

	for (size_t i = 0; i < res; ++i)
	  for (size_t j = 0; j < res; ++j)
	    for (size_t k = 0; k < res; ++k)
	      for (size_t l = 0; l < res; ++l)
		for (size_t m = 0; m < res; ++m)
		  for (size_t n = 0; n < res; ++n)
		    if (_array[i][j][k][l][m][n])
		      this->_pop.push_back(_array[i][j][k][l][m][n]);

	//  _compute_scores(this->_pop);
	//          _compute_score_region();
	front_t fronts;
	std::vector<size_t> ranks;
	//          _fast_domsort(this->_pop, fronts);
	pop_t ptmp;
	for (size_t i = 0; i < Params::pop::size; ++i)
          {
            indiv_t indiv = boost::shared_ptr<Phen>(new Phen());
            indiv->random();
	    indiv->develop();

            ptmp.push_back(indiv);
          }
	this->_eval_pop(ptmp, 0, ptmp.size());
	for (size_t i = 0; i < ptmp.size(); ++i)
	  _add_to_archive(ptmp[i]);

	//          if (this->gen() % 200 == 0)
	/*{
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
	/*	std::ofstream ofsvar("variations.dat",std::ios_base::app);
		std::cout<<"archive size:"<<_archive.size()<<" add=" << _added<<" swapped="<< _swapped<<std::endl;
		ofsvar<<"archive size:"<<_archive.size()<<" add=" << _added<<" swapped="<< _swapped<<std::endl;
	*/
	std::cout<<"archive size:"<<this->_pop.size()<<std::endl;
	if (this->gen() % Params::pop::dump_period == 0)
	  {
	    // créer un flux de sortie
	    std::ostringstream oss;
	    // écrire un nombre dans le flux
	    oss << this->gen();
	    // récupérer une chaîne de caractères
	    std::ofstream ofs((this->res_dir() + "/archive"+oss.str()+".dat").c_str());
	    for (size_t i = 0; i < res; ++i)
	      for (size_t j = 0; j < res; ++j)
		for (size_t k = 0; k < res; ++k)
		  for (size_t l = 0; l < res; ++l)
		    for (size_t m = 0; m < res; ++m)
		      for (size_t n = 0; n < res; ++n)
			if (_array[i][j][k][l][m][n])
			  {
			    phen_ptr_t indiv=_array[i][j][k][l][m][n];
			    ofs<<indiv->fit().behavior().duty_cycle[0]<<" "<<indiv->fit().behavior().duty_cycle[1]<<" "<<indiv->fit().behavior().duty_cycle[2]<<" "<<indiv->fit().behavior().duty_cycle[3]<<" "<<indiv->fit().behavior().duty_cycle[4]<<" "<<indiv->fit().behavior().duty_cycle[5]<<"    "<<indiv->fit().behavior().covered_distance<<"    ";
			    for(int j=0;j<indiv->fit().behavior().controller.size();j++)
			      ofs<<indiv->fit().behavior().controller[j]<<" ";
			    ofs<<std::endl;
			  }
	    
	  }
	
	
      }
      const array_t& array() const { return _array; }
      const array_t& archive() const { return _array; }

    protected:
      array_t _array;
      hits_t _hits;


      bool _add_to_archive(indiv_t i1)
      {

	if(i1->fit().dead())
	  return false;
	point_t p = _get_point(i1);
	size_t i = round(p[0] * (res-1));
	size_t j = round(p[1] * (res-1));
	size_t k = round(p[2] * (res-1));
	size_t l = round(p[3] * (res-1));
	size_t m = round(p[4] * (res-1));
	size_t n = round(p[5] * (res-1));
	i = std::min(i, res-1 );
	j = std::min(j, res-1 );
	k = std::min(k, res-1 );
	l = std::min(l, res-1 );
	m = std::min(m, res-1 );
	n = std::min(n, res-1 );

	assert(i < res);
	assert(j < res);
	assert(k < res);
	assert(l < res);
	assert(m < res);
	assert(n < res);


	_hits[i][j][k][l][m][n]++;

	_dist_center(i1);

	if (!_array[i][j][k][l][m][n]
	    || i1->fit().behavior().covered_distance - _array[i][j][k][l][m][n]->fit().behavior().covered_distance > epsilon
	    || (fabs(i1->fit().behavior().covered_distance - _array[i][j][k][l][m][n]->fit().behavior().covered_distance) <= epsilon && _dist_center(i1)<_dist_center(_array[i][j][k][l][m][n]) ))
	  {
	    _array[i][j][k][l][m][n] = i1;
	    //   _archive.insert(p, i1);
	    return true;
	  }
	return false;
      }
      template<typename I>
      	float _dist_center(const I& indiv)
      {
	float dist=0;
	point_t p =_get_point(indiv);
	for(int i=0;i<6;i++)
	  dist+=pow(p[i]-(float)round(p[i]*(res-1))/(float)(res-1),2);

	dist=sqrt(dist);
	return dist;
      }

      template<typename I>
	point_t _get_point(const I& indiv)
      {
	point_t p;
	p[0] = std::min(1.0f, indiv->fit().behavior().duty_cycle[0]);
	p[1] = std::min(1.0f, indiv->fit().behavior().duty_cycle[1]);
	p[2] = std::min(1.0f, indiv->fit().behavior().duty_cycle[2]);
	p[3] = std::min(1.0f, indiv->fit().behavior().duty_cycle[3]);
	p[4] = std::min(1.0f, indiv->fit().behavior().duty_cycle[4]);
	p[5] = std::min(1.0f, indiv->fit().behavior().duty_cycle[5]);


	return p;
      }

      // --- Pareto tournament selection --
      indiv_t _selection(const pop_t& pop)
      {
	int x1 = misc::rand< int > (0, pop.size());
	//          int x2 = misc::rand< int > (0, pop.size());
	//if (pop[x1]->fit().value() > pop[x2]->fit().value())
	return pop[x1];
	//else
	//return pop[x2];
      }


    };
  }
}
#endif


