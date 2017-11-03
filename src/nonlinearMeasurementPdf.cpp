#include "ras_group8_localization/nonlinearMeasurementPdf.h"
#include <wrappers/rng/rng.h> // Wrapper around several rng libraries
#include "math.h"

#define MEASMODEL_NUMCONDARGUMENTS_MOBILE 1
#define MEASMODEL_DIMENSION_MOBILE        3

namespace BFL
{
 using namespace MatrixWrapper;

 NonlinearMeasurementPdf::NonlinearMeasurementPdf(const Gaussian& measNoise, map_t *map)
   : ConditionalPdf<ColumnVector,ColumnVector>(MEASMODEL_DIMENSION_MOBILE,MEASMODEL_NUMCONDARGUMENTS_MOBILE)
 {
   _measNoise = measNoise;
   _map =  map;
 }


 NonlinearMeasurementPdf::~NonlinearMeasurementPdf(){}

 Probability
 NonlinearMeasurementPdf::ProbabilityGet(const ColumnVector& measurement) const
 {
   ColumnVector state = ConditionalArgumentGet(0);

   ColumnVector expected_measurement(3);

   // Compute the range according to the map
   expected_measurement(1) = map_calc_range(_map, state(1), state(2), state(3) + 0, 5.0); //front
   expected_measurement(2) = map_calc_range(_map, state(1), state(2), state(3) + M_PI_2, 5.0); //left
   expected_measurement(2) = map_calc_range(_map, state(1), state(2), state(3) - M_PI_2, 5.0); //right

   Probability prb = _measNoise.ProbabilityGet(measurement-expected_measurement);

   return prb;
 }

 double map_calc_range(map_t *map, double x, double y, double theta, double max_range)
 {
   double step=map->scale;
   double nextx=x;
   double nexty=y
   while (1)
   {
      int posx=(int)nextx/map->scale;
      int posy=(int)nexty/map->scale;
      if (map->cells[nextx*map->sizey+nexty]==1)
      {
         return sqrt((nextx-x)^2+(nexty-y)^2);
      }
      nextx=nextx+step*std::cos(theta);
      nexty=nexty+step*std::sin(theta);
      if (sqrt((nextx-x)^2+(nexty-y)^2)>max_range)
      {
         return max_range
      }
   }


 }

}//namespace BFL