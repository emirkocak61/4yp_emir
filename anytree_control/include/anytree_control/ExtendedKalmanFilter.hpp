/*
    A kalman filter for the unitree arm. It defines the states as position and velocity and 
    inputs as acceleration. Models the dynamics as euler integration, hence assumes linear system
    and linear kalman filter. The mapping from input torque to acceleration is computed using aba 
    algorithm.
    We can measure both states so C is just Identity and hence not implemented
*/
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/aba-derivatives.hpp>
#include <pinocchio/multibody/model.hpp>
#include <ros/package.h>

class EKF {
public:
    EKF() : dt(0.01) {
        //Initialize pinocchio
        std::string package_path = ros::package::getPath("anytree_description");
        std::string urdf_path = package_path + "/urdf/z1.urdf";
        pinocchio::urdf::buildModel(urdf_path, model);
        data = pinocchio::Data(model);
        InitializeVariables();
        SetParameters();
    }

    Eigen::VectorXd TauToQdd(Eigen::VectorXd& tau) {
        //Compute forward dynamics using the aba algorithm
        Eigen::VectorXd q = state.row(0);
        Eigen::VectorXd v = state.row(1);
        pinocchio::aba(model,data,q,v,tau);
        Eigen::VectorXd a = data.ddq;
        return a;
    }


    void predict(Eigen::VectorXd& input) {
        //Compute the acceleration and derivatives of acceleration
        q = state.head(6);
        v = state.tail(6);
        //Forward dynamics
        da_dq = Eigen::MatrixXd::Zero(num_inputs,num_inputs); //6x6
        da_dv = Eigen::MatrixXd::Zero(num_inputs,num_inputs);
        da_dtau = Eigen::MatrixXd::Zero(num_inputs,num_inputs);
        computeABADerivatives(model, data, q, v, input, da_dq, da_dv, da_dtau);
        //Get accelerations
        Eigen::VectorXd a = data.ddq;
        //Forward kinematics
        state = A * state + B * a;
        //Compute the jacobian of the dynamics
        df_dx.block<6,6>(0,0) = da_dq;
        df_dx.block<6,6>(0,6) = da_dv;
        J = A + B * df_dx; //noalias() for optimizing operations
        //Error covariance prediction
        P = J * P * J.transpose() + Q;
    }

    Eigen::VectorXd correct(Eigen::VectorXd& measurement) {
        //Compute kalman gain
        K = P * C.transpose()*(C * P * C.transpose() + R).inverse() ;
        //State update
        state = state + K * (measurement - C * state);
        //Covariance update
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(num_states,num_states);
        P = (I - K*C) * P;
        return state;
    }

    double num_states;
    double num_inputs;

private:
    void InitializeVariables() {
        std::cout << "Initializing variables" << std::endl;
        num_states = 12;
        num_inputs = 6;
        //Initialize state
        state = Eigen::VectorXd::Zero(num_states); //12x1
        //Initialize state transition matrix
        A = Eigen::MatrixXd::Zero(num_states,num_states);//12x12
        //Initialize the input vector
        B = Eigen::MatrixXd::Zero(num_states,num_inputs); //12x6
        //Initialize C
        C = Eigen::MatrixXd::Identity(num_states,num_states);
        //Initialize the process noise covariance matrix
        Q = Eigen::MatrixXd::Zero(num_states,num_states);
        //Measurement Noise covariance matrix
        R = Eigen::MatrixXd::Zero(num_states,num_states);
        //Kalman gain
        K = Eigen::MatrixXd::Zero(num_states,num_states);
        P = Eigen::MatrixXd::Zero(num_states,num_states);
        //Initialize Jacobian
        J = Eigen::MatrixXd::Zero(num_states,num_states);
        df_dx = Eigen::MatrixXd::Zero(num_inputs,num_states); //6x12
        da_dq = Eigen::MatrixXd::Zero(num_inputs,num_inputs); //6x6
        da_dv = Eigen::MatrixXd::Zero(num_inputs,num_inputs);
        da_dtau = Eigen::MatrixXd::Zero(num_inputs,num_inputs);
    }

    void SetParameters() {
        std::cout << "Settin parameters" << std::endl;
        //Set up A
        A.topLeftCorner(6,6) = Eigen::MatrixXd::Identity(6,6);
        A.topRightCorner(6,6) = Eigen::MatrixXd::Identity(6,6) * dt;
        A.bottomRightCorner(6,6) = Eigen::MatrixXd::Identity(6,6);
        std::cout << "A: " << A << std::endl;
        //Set up B
        B.block<6,6>(6,0) = Eigen::MatrixXd::Identity(6,6) * dt;
        std::cout << "B: " << B << std::endl;
        //Set up Q and R, process and noise covariance matrices respectively
        double w = 1e-2; //Process noise covariance;
        double v = 1e-5; //Sensor noise covariance;
        Q = Eigen::MatrixXd::Identity(num_states,num_states) * w;
        R = Eigen::MatrixXd::Identity(num_states,num_states) * v;
        std::cout << "Q: " << Q << std::endl;
        std::cout << "R: " << R << std::endl;
        P = Eigen::MatrixXd::Identity(12,12) * 1e-3;
    }
    
    pinocchio::Model model;
    pinocchio::Data data;

    double dt;
    //Matrix to store the states
    Eigen::VectorXd state;
    Eigen::MatrixXd J; //Jacobian
    Eigen::MatrixXd df_dx;
    //Initiliaze the matrices for kalman filter steps
    Eigen::MatrixXd A; //State transition matrix
    Eigen::MatrixXd B; //Input vector
    Eigen::MatrixXd C; //Measurement matrix
    Eigen::MatrixXd Q; //Process noise covariance
    Eigen::MatrixXd R; //Measurement noise covariance
    Eigen::MatrixXd P; //P matrix
    Eigen::MatrixXd K; //The kalman gain

    //Related vectors and matrices
    Eigen::VectorXd q;
    Eigen::VectorXd v;
    Eigen::MatrixXd da_dq;
    Eigen::MatrixXd da_dv;
    Eigen::MatrixXd da_dtau;
};