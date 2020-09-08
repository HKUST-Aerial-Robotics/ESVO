#include <esvo_core/core/DepthRegularization.h>

namespace esvo_core
{
namespace core
{
DepthRegularization::DepthRegularization(
  std::shared_ptr<DepthProblemConfig> & dpConfigPtr)
{
  dpConfigPtr_ = dpConfigPtr;
  //parameters
  _regularizationRadius = 5;//10
  _regularizationMinNeighbours = 8;//16
  _regularizationMinCloseNeighbours = 8;//16
}

DepthRegularization::~DepthRegularization() {}

void DepthRegularization::apply( DepthMap::Ptr& depthMapPtr )
{
  DepthMap &dm = *depthMapPtr.get();

  DepthMap dmTmp(dm.rows(), dm.cols());

  DepthMap::iterator it = dm.begin();
  while (it != dm.end())
  {
    dmTmp.set(it->row(), it->col(), *it);
    DepthPoint &newDp = dmTmp.get(it->row(), it->col());

    if (it->valid())
    {
      //get the valid neighbourhood pixels
      std::vector<DepthPoint *> neighbours;
      dm.getNeighbourhood(it->row(), it->col(), _regularizationRadius, neighbours);

      bool isSet = false;
      if (neighbours.size() > _regularizationMinNeighbours)
      {
        //find close neighbours (will include this point)
        std::vector<DepthPoint *> closeNeighbours;
        for (size_t i = 0; i < neighbours.size(); i++)
        {
          if (neighbours[i]->valid())
          {
            double diff = fabs(it->invDepth() - neighbours[i]->invDepth());
            if (diff < 2.0 * sqrt(it->variance()) ||
                diff < 2.0 * sqrt(neighbours[i]->variance()))
              closeNeighbours.push_back(neighbours[i]);
          }
        }
        // regularizationMinCloseNeighbours is larger than the fusion's applied region (namely 4 pixels in my implementation)
        if (closeNeighbours.size() > _regularizationMinCloseNeighbours)
        {
          double statisticalMean = 0.0;
          if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "l2") == 0)
          {
            //compute statistical average
            double totalInvVariances = 0.0;
            for (size_t i = 0; i < closeNeighbours.size(); i++)
              totalInvVariances += 1.0 / closeNeighbours[i]->variance();
            for (size_t i = 0; i < closeNeighbours.size(); i++)
              statisticalMean += closeNeighbours[i]->invDepth() *
                                 (1.0 / closeNeighbours[i]->variance()) / totalInvVariances;
          }
          else if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "Tdist") == 0)
          {
            double nu_post = closeNeighbours[0]->nu();
            double invDepth_post = closeNeighbours[0]->invDepth();
            double scale2_post = closeNeighbours[0]->scaleSquared();

            for (size_t i = 1; i < closeNeighbours.size(); i++)
            {
              double nu_prior = nu_post;
              double invDepth_prior = invDepth_post;
              double scale2_prior = scale2_post;

              double nu_obs = closeNeighbours[i]->nu();
              double invDepth_obs = closeNeighbours[i]->invDepth();
              double scale2_obs = closeNeighbours[i]->scaleSquared();

              nu_post = std::min(nu_prior, nu_obs);
              invDepth_post = (scale2_obs * invDepth_prior + scale2_prior * invDepth_obs) / (scale2_obs + scale2_prior);
              scale2_post = (nu_post + pow(invDepth_prior - invDepth_obs,2) / (scale2_prior + scale2_obs)) /
                (nu_post + 1) * (scale2_prior * scale2_obs) / (scale2_prior + scale2_obs);
            }
            statisticalMean = invDepth_post;
          }
          else
          {
            LOG(INFO) << "(Regularization) Wrong dpConfiguration is provided.";
            exit(-1);
          }

          //set the statistical average (everything else is simply copied)
          newDp.invDepth() = statisticalMean;
          isSet = true;
        }
      }

      if (!isSet)
        newDp.invDepth() = -1.0;
    }

    it++;
  }

  //transfer the result
  dm = dmTmp;
}

}// core
}// esvo_core