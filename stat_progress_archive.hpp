#ifndef STAT_BEST_HPP
#define STAT_BEST_HPP
#include <sferes/stat/stat.hpp>
namespace sferes
{
  namespace stat
  {

    SFERES_STAT(ProgressArchive, Stat)
    {
    public:
      template<typename E>
	void refresh(const E& ea)
      {
	if(ea.gen()%50!=0)
	  return;

	this->_create_log_file(ea, "progress_archive.dat");	

	int archive_size=0;
	float archive_mean=0;
	float archive_max=0;
	float mean_dist_center=0;

	for (size_t i = 0; i < ea.array().size(); ++i)
	  for (size_t j = 0; j < ea.array().size(); ++j)
	    for (size_t k = 0; k < ea.array().size(); ++k)
	      for (size_t l = 0; l < ea.array().size(); ++l)
		for (size_t m = 0; m < ea.array().size(); ++m)
		  for (size_t n = 0; n < ea.array().size(); ++n)
		    if (ea.array()[i][j][k][l][m][n])
		      {
			typename E::indiv_t indiv=ea.array()[i][j][k][l][m][n];
			archive_size++;
			archive_mean+=indiv->fit().behavior().covered_distance;
			if(archive_max<indiv->fit().behavior().covered_distance)
			  archive_max=indiv->fit().behavior().covered_distance;
			float dist=0;
			for(int dc=0;dc<indiv->fit().behavior().duty_cycle.size();dc++)
			  {
			    float diff=(indiv->fit().behavior().duty_cycle[dc]-round(indiv->fit().behavior().duty_cycle[dc]*4)/4);
			    dist+=diff*diff;
			  }
			mean_dist_center+=sqrtf(dist);
		      }
	archive_mean/=archive_size;
	mean_dist_center/=archive_size;


	(*this->_log_file)<<ea.gen()<< " "<<archive_size<<" "<<archive_mean<<" "<<archive_max<<" "<<mean_dist_center<<std::endl;
	  
      } 
    };
  }
}
#endif
