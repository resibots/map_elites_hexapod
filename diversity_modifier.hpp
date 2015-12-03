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

#ifndef MODIFIER_BEHAVIOR_DIV_HPP
#define MODIFIER_BEHAVIOR_DIV_HPP

#include <sferes/stc.hpp>

namespace sferes
{
    float euclidean_dist(const ::std::vector<float> &d1,
                         const ::std::vector<float> &d2)
    {
        float dist = 0;
        assert(d1.size() == d2.size());
        for (size_t i = 0; i < d1.size(); i++)
            dist += (float) fabs(d1[i] - d2[i]);
        return dist / (float)d1.size();
    }

    namespace modif
    {
        namespace modifier_div
        {
            template<typename Phen>
            struct _parallel_div
            {
                typedef std::vector<boost::shared_ptr<Phen> > pop_t;
                pop_t _pop;

                _parallel_div(pop_t& pop) : _pop(pop) {}
                _parallel_div(const _parallel_div& ev) : _pop(ev._pop) {}
                void operator() (const parallel::range_t& r) const
                {
                    for (size_t i = r.begin(); i != r.end(); ++i)
                    {
                        float d = 0.0f;
                        if (_pop[i]->fit().objs()[0] <= -10000)
                            d=-1000;
                        else
                        {

                            for (size_t j = 0; j < _pop.size(); ++j)
                                d += euclidean_dist(_pop[i]->data(),_pop[j]->data());
                            d /= (float)_pop.size();
                        }
                        int l =  _pop[i]->fit().objs().size()-1;
                        _pop[i]->fit().set_obj(l, d);
                    }
                }
            };
        }

        // ADD the mean distance to the population to the last objective (it
        // DOESN'T add the objective automatically)
        // you HAVE to initialize this value to a "good" one (depending on
        // your constraints scheme; for instance, 0)
        // you phenotype/individual class must have a float dist(const
        // Phen& o) method (the dist method must be thread-safe)
        SFERES_CLASS(DiversityModifier)
        {
public:
            template<typename Ea>
            void apply(Ea& ea)
            {
                // parallel compute
                parallel::init();
                parallel::p_for(parallel::range_t(0, ea.pop().size()),
                                modifier_div::_parallel_div<typename Ea::phen_t>(ea.pop()));
            }
        };
    }
}

#endif
