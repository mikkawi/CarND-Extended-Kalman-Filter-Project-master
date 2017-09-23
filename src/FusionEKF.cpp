#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225,      0,
                   0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09,      0,    0,
                 0, 0.0009,    0,
                 0,      0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  ekf_.epsilon = 0.001;
  ekf_.x_ = VectorXd(4);

  ekf_.P_ = MatrixXd(4, 4); // state covariance matrix
  ekf_.P_ << 1, 0,    0,    0,
             0, 1,    0,    0,
             0, 0, 1000,    0,
             0, 0,    0, 1000;

  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  H_laser_ << 1,0,0,0,
              0,1,0,0;  


}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    cout<<"Not Initialized"<< endl;
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        float px = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
        float py = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);
        float vx = measurement_pack.raw_measurements_[2] * cos(measurement_pack.raw_measurements_[1]);
        float vy = measurement_pack.raw_measurements_[2] * sin(measurement_pack.raw_measurements_[1]);
          if (px == 0 && py == 0) {
            px = ekf_.epsilon;
            py = ekf_.epsilon;
            }
        ekf_.x_ << px, py, vx, vy;
    }  
    
    previous_timestamp_  = measurement_pack.timestamp_;
    // first measurement
    cout << "EKF: " << endl;
    //ekf_.x_ = VectorXd(4);
    //ekf_.x_ << 1, 1, 1, 1;
  
    
    //if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
    //}
    //else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
    //}

    // done initializing, no need to predict or update
    is_initialized_ = true;
    cout << "x_ initialized:  " << is_initialized_ <<endl;
    //cout << "INITIALIZED" << endl;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  float dt_2 = pow(dt, 2);
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  //cout << "FusionEKF: time stamp calc: "<< dt << endl;
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  int noise_ay = 9;
  int noise_ax = 9;

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4 / 4.0 * noise_ax, 0                , dt_3/2.0*noise_ax, 0                ,
             0                    , dt_4/4.0*noise_ay, 0                , dt_3/2.0*noise_ay,
             dt_3/2.0*noise_ax    , 0                , dt_2*noise_ax    , 0                ,
             0                    , dt_3/2.0*noise_ay, 0                , dt_2*noise_ay    ;
  
  if(dt>ekf_.epsilon){
    //cout << "FusionEKF: Predict"<<endl;
    ekf_.Predict();
  
  }
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    //cout <<"FusionEKF: Radar Update"<< endl;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
   
  } else {
    // Laser updates
    //cout <<"FusionEKF: Laser Update"<<endl;
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
    
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
