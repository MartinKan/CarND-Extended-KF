#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <fstream>

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
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ << 1, 0, 0, 0,
        0, 1, 0, 0;
  
  //initialize the acceleration noise components
  noise_ax = 5;
  noise_ay = 5;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {


}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1,1,1,1;
    previous_timestamp_ = measurement_pack.timestamp_;

    //state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;

    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float dx = cos(measurement_pack.raw_measurements_[1]) * measurement_pack.raw_measurements_[0];
      float dy = sin(measurement_pack.raw_measurements_[1]) * measurement_pack.raw_measurements_[0];
      ekf_.x_ << dx, dy, 0, 0;
      Hj_ = tools.CalculateJacobian(ekf_.x_);

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

    }

    // FOR DEBUGGING PURPOSES
    // myfile.open ("log.txt");
    // if (myfile.is_open())
    // {

    //   myfile << "BEGIN DEBUGGING" "\n";
    //   myfile.close();
    // }
    // step = 1;

    // done initializing, no need to predict or update
    is_initialized_ = true;
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

  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // //Set the F matrix so that the time is integrated
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;

  //set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

         
  // FOR DEBUGGING PURPOSES
  //
  // step++;
  //
  // myfile.open ("log.txt", ios::app);
  // if (myfile.is_open())
  // {
  //   myfile << "STEP" << step << ":\n";
  //   myfile << "raw_measurements_: " << "\n" << measurement_pack.raw_measurements_ << "\n";
  //   myfile << "BEFORE PREDICTION" << "\n";
  //   myfile << "x_ = " << ekf_.x_ << "\n";
  //   myfile << "P_ = " << ekf_.P_ << "\n";
  //   myfile.close();
  // }


  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  // FOR DEBUGGING PURPOSES
  // myfile.open ("log.txt", ios::app);
  // if (myfile.is_open())
  // {
  //   myfile << "AFTER PREDICTION" << "\n";
  //   myfile << "x_ = " << ekf_.x_ << "\n";
  //   myfile << "P_ = " << ekf_.P_ << "\n";
  //   myfile.close();
  // }

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
      // Initialize the Jacobian matrix and measurement covariance matrix for radar
      Hj_ = tools.CalculateJacobian(ekf_.x_);
      ekf_.H_ = Hj_;
      ekf_.R_ = R_radar_;
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
      // Initialize the measurement and measurement covariance matrix for laser
      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_;
      ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "raw_measurements_: " << measurement_pack.raw_measurements_ << endl;
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;

  // FOR DEBUGGING PURPOSES
  // myfile.open ("log.txt", ios::app);
  // if (myfile.is_open())
  // {
  //   myfile << "raw_measurements_: " << measurement_pack.raw_measurements_ << "\n";
  //   myfile << "AFTER UPDATE" << "\n";
  //   myfile << "x_ = " << ekf_.x_ << "\n";
  //   myfile << "P_ = " << ekf_.P_ << "\n";
  //   myfile.close();
  // }


}
