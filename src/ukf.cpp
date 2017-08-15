#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

//#define DEBUG
//#define DEBUG1

#ifdef DEBUG
#define D(x) x
#ifdef DEBUG1
#define D1(x) x
#endif
#else
#define D(x)
#define D1(x)
#endif

inline double wrapAngletoPI( double in_angle );

inline double wrapAngletoPI( double in_angle )
{

	double out_angle = in_angle - 2. * M_PI * floor((in_angle + M_PI) / (2. * M_PI));
	if (out_angle != in_angle){
		cout << "wrapAngletoPI: Adjusted in_angle " << in_angle << " to out_angle " << out_angle << "." << endl;
	}
	return out_angle;
}

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF()
{
	// if this is false, laser measurements will be ignored (except during init)
	use_laser_ = true;

	// if this is false, radar measurements will be ignored (except during init)
	use_radar_ = true;

	//set state dimension
	n_x_ = 5;

	//set augmented dimension
	n_aug_ = 7;

	//set measurement dimension, radar can measure r, phi, and r_dot
	n_z_radar_ = 3;

  //set measurement dimension, lidar can measure x, y
	n_z_lidar_ = 2;

	// number of sigma points
	n_sig_ = 2 * n_aug_ + 1; //15

	// initial state vector
	x_ = VectorXd(n_x_);

	// initial covariance matrix
	P_ = MatrixXd(n_x_, n_x_);

	//measurement noise covariance matrix (for Radar)
	RR_ = MatrixXd(n_z_radar_,n_z_radar_);

	//measurement noise covariance matrix (for Radar)
	RL_ = MatrixXd(2,2);

	// Sigma point matrix
	Xsig_pred_ = MatrixXd(n_x_, n_sig_);

	//weights vector
	weights_ = VectorXd(n_sig_);

	// Process noise standard deviation longitudinal acceleration in m/s^2
	std_a_ = 1.0; /* first guess was 1.7, then 1.5 */
	std_a_sq_ = std_a_ * std_a_;

	// Process noise standard deviation yaw acceleration in rad/s^2
	std_yawdd_ = M_PI / 4; /* first guess was 16, then 14, then 10 - met reqs, then 6 (Tony) */
	std_yawdd_sq_ = std_yawdd_ * std_yawdd_;

	// Laser measurement noise standard deviation position1 in m
	std_laspx_ = 0.15;

	// Laser measurement noise standard deviation position2 in m
	std_laspy_ = 0.15;

	// Radar measurement noise standard deviation radius in m
	std_radr_ = 0.3;

	// Radar measurement noise standard deviation angle in rad
	std_radphi_ = 0.03; //0.0175 in assignment

	// Radar measurement noise standard deviation radius change in m/s
	std_radrd_ = 0.3; //0.1 in assignment

	// spreading parameter
	lambda_ = 3 - n_aug_;

	//create vector for predicted state
	predictor_x_ = VectorXd(n_x_);

	// set to a reasonable value to prevent errors in main.c
	dt_ = 0.1;

	/**
	   TODO:

	   Complete the initialization. See ukf.h for other member properties.

	   Hint: one or more values initialized above might be wildly off...
	 */
	// Haven't received a measurement yet
	is_initialized_ = false;

	// Haven't taken a step yet.
	step_ = 0;
	// set time.
	time_us_ = 0.;
}

/**
 * Destructor
 */
UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
	/**
	   TODO:

	   Complete this function! Make sure you switch between lidar and radar
	   measurements.
	 */
	step_ += 1;
	cout << "UKF step # " << step_ << endl;
	if (!is_initialized_)
	{
		/**
		   TODO:
		 * Initialize the state x_ with the first measurement.
		 * Create the covariance matrix.
		 * Remember: you'll need to convert radar from polar to cartesian coordinates.
		 */
		// first measurement
		cout << "ProcessMeasurement: UKF Initialization " << endl;
		x_ = VectorXd(5);
		x_ << 1, 1, 1, 1, 0.05; //TK changed from 1,1,1,1,1
		D1(cout << "set x_" << endl;)

		/* try using diagonal 1s - changed first two diags to 0.1*/
		P_ << 0.1, 0, 0, 0, 0,
		        0, 0.1, 0, 0, 0,
		        0, 0, 1, 0, 0,
		        0, 0, 0, 1, 0,
		        0, 0, 0, 0, 1;

		if (meas_package.sensor_type_ == MeasurementPackage::RADAR and use_radar_ == true)
		{
			D(cout << "ProcessMeasurement: Init UKF Radar Measurement" << endl;)
			/**
			   Convert radar from polar to cartesian coordinates and initialize state.
			 */
			// just set x_(0) to ro*cos(theta)
			x_(0) = meas_package.raw_measurements_(0) *
                                  cos(meas_package.raw_measurements_(1));
			// just set x_(1) to ro*sin(theta)
			x_(1) = meas_package.raw_measurements_(0) *
                                  sin(meas_package.raw_measurements_(1));
			is_initialized_ = true;
		}
		else if (meas_package.sensor_type_ == MeasurementPackage::LASER and use_laser_ == true)
		{
			D(cout << "ProcessMeasurement: Init UKF Laser Measurement" << endl;)
			/**
			   Initialize state.
			 */
			// set x_(0)to x
			x_(0) = meas_package.raw_measurements_(0);
			// set x_(1)to y
			x_(1) = meas_package.raw_measurements_(1);
			if (fabs(x_(0)) < 0.001 and fabs(x_(1)) < 0.001)
			{
        x_(0) = 0.001;
        x_(1) = 0.001;
      }
			is_initialized_ = true;
		}

		// set up weights
		weights_.fill(double(0.5 / (n_aug_ + lambda_)));
		weights_(0) = double (lambda_ / (lambda_ + n_aug_));

		//covariance matrix for Radar
		RR_ <<  std_radr_ * std_radr_, 0, 0,
		        0, std_radphi_ * std_radphi_, 0,
		        0, 0, std_radrd_ * std_radrd_;

    //covariance matrix for Lidar
		RL_ << std_laspx_ * std_laspx_, 0,
		        0, std_laspx_ * std_laspx_;

		previous_timestamp_ = meas_package.timestamp_;

		// done initializing, no need to predict or update

		cout << "ProcessMeasurement: UKF Initialized - timestamp: " << previous_timestamp_ << endl;
		// print the output
		D(cout << "ProcessMeasurement: x_ = " << x_ << endl;)
		D(cout << "ProcessMeasurement: P_ = " << P_ << endl;)
		return;
	}

	/* Not initial timestep, so run prediction
	            - calculate elapsed time first */
	dt_ = (double(meas_package.timestamp_) - previous_timestamp_) /
	            1000000.0;
	previous_timestamp_ = double(meas_package.timestamp_);

  D1(cout << "ProcessMeasurement: Calling Prediction, _ = " << dt_ << endl);
	Prediction(dt_);

	/* Now run Update */
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR and use_radar_ == true)
	{
		cout << "ProcessMeasurement: UKF update with Radar Measurement" << endl;
		// Radar updates
		UpdateRadar(meas_package);
	}
	else if (meas_package.sensor_type_ == MeasurementPackage::LASER and use_laser_ == true)
	{
		// Laser updates
		cout << "ProcessMeasurement: UKF update with Laser Measurement" << endl;
		UpdateLidar(meas_package);
	}

	// print the output
	D(cout << "ProcessMeasurement: x_ = " << x_ << endl;)
	D(cout << "ProcessMeasurement: P_ = " << P_ << endl;)
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t)
{
	/**
	   TODO:

	   Complete this function! Estimate the object's location. Modify the state
	   vector, x_. Predict sigma points, the state, and the state covariance matrix.
	 */
	//set up augmented sigma points
	D1(cout << "In Prediction" << endl;)

	//create augmented state covariance
	MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

	P_aug.fill(0.0);
	P_aug.topLeftCorner(5,5) = P_;
	P_aug(5,5) = std_a_sq_;// * std_a_;
	P_aug(6,6) = std_yawdd_sq_; // * std_yawdd_;
  D1(cout <<"Prediction: P_aug set up " << P_aug <<endl;)

	//calculate square root of P_aug
	MatrixXd L = P_aug.llt().matrixL();
	D1(cout << "Prediction: Sqrt of P_aug = L = " << L << "." <<endl;)

	//create augmented mean vector
	VectorXd x_aug = VectorXd(n_aug_);
	//set up augmented x with first 5 elements the x vector,
	//and the remaining two zeros (zero means)
	x_aug.fill(0.0);
	x_aug.head(5) = x_;
  D1(cout << "Prediction: x_aug set up " << x_aug << endl;)

  //create augmented sigma points
	MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);
	//set first column of sigma point matrix to
	Xsig_aug.col(0)  = x_aug;
	//set remaining sigma points
	for (int i = 0; i < n_aug_; i++)
	{
		Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
		Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
	}
  D1(cout << "Prediction: Xsig_aug sigma points initialized " << Xsig_aug << endl;)

	//predict sigma points
	for (int i = 0; i < n_sig_; i++)
	{
		D1(cout << "Prediction: Xsig_pred column " << i << ". Xsig_aug = "<< Xsig_aug.col(i) << "." << endl;)
		//extract values for better readability
		double p_x = Xsig_aug(0,i);
		double p_y = Xsig_aug(1,i);
		double v = Xsig_aug(2,i);
		double yaw = Xsig_aug(3,i);
		double yawd = Xsig_aug(4,i);
		double nu_a = Xsig_aug(5,i);
		double nu_yawdd = Xsig_aug(6,i);

		//predicted state values
		double px_p, py_p;

		//avoid division by zero
		if (fabs(yawd) > 0.001)
		{
			D1(cout << "Prediction: fabs(yawd) > 0.001 = "<< fabs(yawd) <<endl;)
			px_p = p_x + v / yawd *
			       ( sin (yaw + yawd * delta_t) - sin(yaw));
			py_p = p_y + v / yawd *
			       ( cos(yaw) - cos(yaw + yawd * delta_t) );
		}
		else //fabs(yawd) <= 0.001
		{
			D(cout << "Prediction: fabs(yawd) <= 0.001 = "<< fabs(yawd) <<endl;)
			px_p = p_x + v * delta_t * cos(yaw);
			py_p = p_y + v * delta_t * sin(yaw);
		}

		double v_p = v;
		double yaw_p = yaw + yawd * delta_t;
		double yawd_p = yawd;

		//add noise
		px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
		py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
		v_p = v_p + nu_a * delta_t;

		yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
		yawd_p = yawd_p + nu_yawdd * delta_t;

		//write predicted sigma point into right column
		Xsig_pred_(0,i) = px_p;
		Xsig_pred_(1,i) = py_p;
		Xsig_pred_(2,i) = v_p;
		Xsig_pred_(3,i) = yaw_p;
		Xsig_pred_(4,i) = yawd_p;
	}
  D1(cout << "Prediction: Xsig_pred_ set up" << Xsig_pred_ <<endl;)

	//predicted state mean
	//create vector for predicted state
	x_.fill(0.0);
	for (int i = 0; i < n_sig_; i++) //iterate over sigma points
		x_ = x_ + weights_(i) * Xsig_pred_.col(i);
	D1(cout << "Prediction: x_ calculated \n"<< x_ <<endl;)

	//predicted state covariance matrix
	//create covariance matrix for prediction
	P_.fill(0.0);
	for (int i = 0; i < n_sig_; i++) //iterate over sigma points
	{// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		//angle normalization
		x_diff(3) = wrapAngletoPI(x_diff(3));
		P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
	}
	D1(cout << "Prediction: P_ calculated \n" << P_ << "." << endl;)

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package)
{
	/**
	   TODO:

	   Complete this function! Use lidar data to update the belief about the object's
	   position. Modify the state vector, x_, and covariance, P_.

	   You'll also need to calculate the lidar NIS.
	*/

	/* Using EKF logic since all that is needed here */
	VectorXd z = meas_package.raw_measurements_;

	MatrixXd Zsig = MatrixXd(n_z_lidar_ , n_sig_);

	//transform sigma points into measurement space
	for (int i = 0; i < n_sig_; i++) //2n+1 simga points

	{// extract values for better readibility
		double p_x = Xsig_pred_(0,i);
		double p_y = Xsig_pred_(1,i);

		// measurement model
		Zsig(0,i) = p_x;
    Zsig(1,i) = p_y;
	}

	VectorXd z_pred =	VectorXd(n_z_lidar_);
	z_pred.fill(0.0);
	for(int i = 0; i < n_sig_; i++) //2n+1 simga points
	{
		z_pred = z_pred + weights_(i) * Zsig.col(i);
	}
	//measurement covariance matrix S
	MatrixXd S = MatrixXd(n_z_lidar_, n_z_lidar_);
	S.fill(0.0);
	MatrixXd Tc = MatrixXd(n_x_, n_z_lidar_);
	Tc.fill(0.0);
	VectorXd z_diff = VectorXd(n_z_lidar_);
	VectorXd x_diff = VectorXd(n_x_);
	for(int i = 0; i < n_sig_; i++) //2n+1 simga points
	{
		z_diff = Zsig.col(i) - z_pred;
		x_diff = Xsig_pred_.col(i) - x_;

		MatrixXd z_diff_t = z_diff.transpose();

		D1(cout << "z_diff = " << z_diff << ". z_diff_t = "<< z_diff_t <<"."<<endl;)

		S = S + weights_(i) * z_diff * z_diff_t;
		Tc = Tc + weights_(i) * x_diff * z_diff_t;
	}

	S = S + RL_; //add noise covariance

	MatrixXd Si = S.inverse();

	//Kalman gain K;
	MatrixXd K = Tc * Si;

  z_diff = z - z_pred;

	x_ = x_ + (K * z_diff);

	P_ = P_ - K * S * K.transpose();

	D1(cout << "UpdateLidar: x_ \n" << x_ << "." << endl;)
	D1(cout << "UpdateLidar: P_ \n" << P_ << "." << endl;)

  //calculate the normalized innovation squared for laser
  NIS_laser_ =  z_diff.transpose() * Si * z_diff;

	D(cout << "UpdateLidar: NIS_laser_ = " << NIS_laser_ << "." << endl;)
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package)
{
	/**
	   TODO:

	   Complete this function! Use radar data to update the belief about the object's
	   position. Modify the state vector, x_, and covariance, P_.

	   You'll also need to calculate the radar NIS.
	 */
  VectorXd z = meas_package.raw_measurements_;
  D1(cout << "UpdateRadar: z set to " << z << "." <<endl;)

	//create matrix for sigma points in measurement space
	MatrixXd Zsig = MatrixXd(n_z_radar_, n_sig_);

	//transform sigma points into measurement space
	for (int i = 0; i < n_sig_; i++) //2n+1 simga points
	{// extract values for better readibility
		double p_x = Xsig_pred_(0,i);
		double p_y = Xsig_pred_(1,i);
		double v  = Xsig_pred_(2,i);
		double yaw = Xsig_pred_(3,i);

		double v1 = cos(yaw) * v;
		double v2 = sin(yaw) * v;

		// measurement model
		Zsig(0,i) = sqrt(p_x * p_x + p_y * p_y); //r
    if((fabs(p_x) > 0.001) and (fabs(p_x) + fabs(p_y) > 0.001)){
		  Zsig(1,i) = atan2(p_y,p_x);             //phi
		  Zsig(2,i) = (p_x * v1 + p_y * v2 ) / Zsig(0,i); //sqrt(p_x * p_x + p_y * p_y); //r_dot
    }else{
			D(cout << "UpdateRadar: !(fabs(p_x) > 0.001) && (fabs(p_x) + fabs(p_y) > 0.001) " << fabs(p_x) << " " << fabs(p_y) << "." << endl;)
			p_x = 0.001;
			p_y = 0.001;
			Zsig(1,i) = atan2(p_y,p_x);             //phi
		  Zsig(2,i) = (p_x * v1 + p_y * v2 ) / sqrt(p_x * p_x + p_y * p_y); //r_dot
    }
	}
	//mean predicted measurement
	VectorXd z_pred = VectorXd(n_z_radar_);
	z_pred.fill(0.0);
	for (int i = 0; i < n_sig_; i++)
		z_pred = z_pred + weights_(i) * Zsig.col(i);
  D1(cout << "UpdateRadar: z_pred set up " << z_pred << "." << endl;)

	//measurement covariance matrix S
	MatrixXd S = MatrixXd(n_z_radar_, n_z_radar_);
	S.fill(0.0);

	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(n_x_, n_z_radar_);
	Tc.fill(0.0);

  VectorXd z_diff = VectorXd(n_z_radar_);
  VectorXd x_diff	= VectorXd(n_x_);
	for (int i = 0; i < n_sig_; i++) //2n+1 simga points
	{//residual
		D1(cout << "UpdateRadar: Sigma point " << i << "." << endl;)
		z_diff = Zsig.col(i) - z_pred;
		//angle normalization
		z_diff(1) = wrapAngletoPI(z_diff(1));
    D1(cout << "UpdateRadar: z_diff " << z_diff << "." << endl;)

		// state difference
		x_diff = Xsig_pred_.col(i) - x_;
		//angle normalization
		x_diff(3) = wrapAngletoPI(x_diff(3));
    D1(cout << "UpdateRadar: x_diff " << x_diff << "." << endl;)

		MatrixXd z_diff_t = z_diff.transpose();
		S = S + weights_(i) * z_diff * z_diff_t;
		Tc = Tc + weights_(i) * x_diff * z_diff_t;
	}

	D1(cout << "UpdateRadar: Tc \n" << Tc << "." << endl;)

	S = S + RR_;  //add noise covariance
  D1(cout << "UpdateRadar: S \n" << S << "." << endl;)

  MatrixXd Si = S.inverse();

	//Kalman gain K;
	MatrixXd K = Tc * Si;
	D1(cout << "UpdateRadar: K " << K << "." << endl;)

	//residual
	z_diff = z - z_pred;
	z_diff(1) = wrapAngletoPI(z_diff(1)); //angle normalization
  D1(cout << "UpdateRadar: z_diff " << z_diff << "." << endl;)

	//update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = P_ - K * S * K.transpose();

	D1(cout << "UpdateRadar: x_ \n" << x_ << "." << endl;)
	D1(cout << "UpdateRadar: P_ \n" << P_ << "." << endl;)

  //calculate the normalized innovation squared for radar
  NIS_radar_ = z_diff.transpose() * Si * z_diff;
	D(cout << "UpdateRadar: NIS_radar = " << NIS_radar_ << "." << endl;)
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Predictor(double delta_t)
{
	/**
	   Function to estimate future location without modifying any matrices
	 */
	//set up augmented sigma points
	D1(cout << "In Predictor" << endl;)

	//create augmented state covariance
	MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

	P_aug.fill(0.0);
	P_aug.topLeftCorner(5,5) = P_;
	P_aug(5,5) = std_a_sq_;// * std_a_;
	P_aug(6,6) = std_yawdd_sq_; // * std_yawdd_;
  D1(cout <<"Predictor: P_aug set up " << P_aug <<endl;)

	//calculate square root of P_aug
	MatrixXd L = P_aug.llt().matrixL();
	D1(cout << "Predictor: Sqrt of P_aug = L = " << L << "." <<endl;)

	//create augmented mean vector
	VectorXd x_aug = VectorXd(n_aug_);
	//set up augmented x with first 5 elements the x vector,
	//and the remaining two zeros (zero means)
	x_aug.fill(0.0);
	x_aug.head(5) = x_;
  D1(cout << "Predictor: x_aug set up " << x_aug << endl;)

  //create augmented sigma points
	MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);
	//set first column of sigma point matrix to
	Xsig_aug.col(0)  = x_aug;
	//set remaining sigma points
	for (int i = 0; i < n_aug_; i++)
	{
		Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
		Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
	}
  D1(cout << "Predictor: Xsig_aug sigma points initialized " << Xsig_aug << endl;)

	//predict sigma points
	for (int i = 0; i < n_sig_; i++)
	{
		D1(cout << "Predictor: Xsig_pred column " << i << ". Xsig_aug = "<< Xsig_aug.col(i) << "." << endl;)
		//extract values for better readability
		double p_x = Xsig_aug(0,i);
		double p_y = Xsig_aug(1,i);
		double v = Xsig_aug(2,i);
		double yaw = Xsig_aug(3,i);
		double yawd = Xsig_aug(4,i);
		double nu_a = Xsig_aug(5,i);
		double nu_yawdd = Xsig_aug(6,i);

		//predicted state values
		double px_p, py_p;

		//avoid division by zero
		if (fabs(yawd) > 0.001)
		{
			D1(cout << "Predictor: fabs(yawd) > 0.001 = "<< fabs(yawd) <<endl;)
			px_p = p_x + v / yawd *
			       ( sin (yaw + yawd * delta_t) - sin(yaw));
			py_p = p_y + v / yawd *
			       ( cos(yaw) - cos(yaw + yawd * delta_t) );
		}
		else //fabs(yawd) <= 0.001
		{
			D(cout << "Prediction: fabs(yawd) <= 0.001 = "<< fabs(yawd) <<endl;)
			px_p = p_x + v * delta_t * cos(yaw);
			py_p = p_y + v * delta_t * sin(yaw);
		}

		double v_p = v;
		double yaw_p = yaw + yawd * delta_t;
		double yawd_p = yawd;

		//add noise
		px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
		py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
		v_p = v_p + nu_a * delta_t;

		yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
		yawd_p = yawd_p + nu_yawdd * delta_t;

		//write predicted sigma point into right column
		Xsig_pred_(0,i) = px_p;
		Xsig_pred_(1,i) = py_p;
		Xsig_pred_(2,i) = v_p;
		Xsig_pred_(3,i) = yaw_p;
		Xsig_pred_(4,i) = yawd_p;
	}
  D1(cout << "Predictir: Xsig_pred_ set up" << Xsig_pred_ <<endl;)

	//predicted state mean
	predictor_x_.fill(0.0);
	for (int i = 0; i < n_sig_; i++) //iterate over sigma points
		predictor_x_ = predictor_x_ + weights_(i) * Xsig_pred_.col(i);
	D1(cout << "Predictor: x calculated \n"<< predictor_x_ <<endl;)

}
