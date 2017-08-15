#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* measurement noise covariance matrix (forRadar)
  MatrixXd RR_;

  ///* measurement noise covariance matrix (for Lidar)
  MatrixXd RL_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;
  double std_a_sq_; //square of std_a

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;
  double std_yawdd_sq_;//square of std_a

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* number of Sigma points
  int n_sig_;

  //radar measurement dimension, radar can measure r, phi, and r_dot
  int n_z_radar_;
  int n_z_lidar_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* step counter
  int step_;

  ///* store the future predicted location of the target
  VectorXd predictor_x_;

  ///* current NIS for laser
  double NIS_laser_;

  ///* current NIS for radar
  double NIS_radar_;

  ///* timestamp on previous step
  double previous_timestamp_;

  ///* time between steps for last step
  double dt_;


  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);
  /**
   * Estimates the position of the target at time in future without affecting
   * UKF model.
   * @param delta_t Time between k and k+1 in s
   */
  void Predictor(double delta_t);
};

#endif /* UKF_H */
