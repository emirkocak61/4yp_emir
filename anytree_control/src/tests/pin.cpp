
/*
    A template class that implements the extended kalman filter for a robotic arm
*/
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/multibody/model.hpp>
#include <ros/ros.h>
#include <ros/package.h>


int main(int argc,char** argv) {
    //Initialize the model
    pinocchio::Model model;

    std::string package_path = ros::package::getPath("anytree_description");
    std::string urdf_filename = package_path + "/urdf/z1.urdf";

    //Load the urdf mode
    // Load the URDF model
    try {
        pinocchio::urdf::buildModel(urdf_filename, model);
        std::cout << "Successfully loaded URDF model." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error loading URDF model: " << e.what() << std::endl;
        return -1;
    }

    pinocchio::Data data(model);

    // Define some example joint positions and velocities
    Eigen::VectorXd q(6);
    q << 0.0, 0.0, -0.005, -0.074, 0.0, 0.0;
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);

    //q = pinocchio::neutral(model);
    //pinocchio::normalize(model, q);

    // Compute the forward dynamics using the ABA
    pinocchio::aba(model, data, q, v, tau);

    // Print out the computed accelerations
    std::cout << "Joint positions" << q.transpose() << std::endl;
    std::cout << "Joint velocities" << v.transpose() << std::endl;
    std::cout << "Joint torques: " << tau.transpose() << std::endl;
    std::cout << "Joint accelerations: " << data.ddq.transpose() << std::endl;

    return 0;
}