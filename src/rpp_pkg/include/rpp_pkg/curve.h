
#ifndef RPP_PKG_CURVE_H_
#define RPP_PKG_CURVE_H_

#include "rpp_pkg/point.h"
#include "rpp_pkg/rpp_math.h"

namespace rpp_pkg
{

class Curve
{
public:
  /**
   * @brief Construct a new Curve object
   * @param step  Simulation or interpolation size
   */
  Curve(double step);

  /**
   * @brief Destroy the curve generation object
   */
  virtual ~Curve() = default;

  /**
   * @brief Running trajectory generation
   * @param points path points <x, y>
   * @param path generated trajectory
   * @return true if generate successfully, else failed
   */
  virtual bool run(const Points2d& points, Points3d& path) = 0;

  /**
   * @brief Running trajectory generation
   * @param points path points <x, y, theta>
   * @param path generated trajectory
   * @return true if generate successfully, else failed
   */
  virtual bool run(const Points3d& points, Points3d& path) = 0;

  /**
   * @brief Generate the path.
   * @param start Initial pose (x, y, yaw)
   * @param goal  Target pose (x, y, yaw)
   * @param path  The smoothed trajectory points
   * @return true if generate successfully, else failed
   */
  virtual bool generation(const Point3d& start, const Point3d& goal, Points3d& path) = 0;

  /**
   * @brief Calculate the distance of given path.
   * @param path    the trajectory
   * @return length the length of path
   */
  double distance(const Points3d& path);

  /**
   * @brief Configure the simulation step.
   * @param step    Simulation or interpolation size
   */
   void setStep(double step);

protected:
   double step_;  // Simulation or interpolation size
};
}  

#endif