#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
 public:
     
  enum SensorType{
    LASER=0,
    RADAR=1
  } sensor_type_;

  MeasurementPackage(SensorType st, Eigen::VectorXd meas, long long ts) :
      sensor_type_(st), raw_measurements_(meas), timestamp_(ts) {};

  long long timestamp_;

  Eigen::VectorXd raw_measurements_;
};

#endif // MEASUREMENT_PACKAGE_H_
