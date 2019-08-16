/*!
* @file 	main.cpp
* @author   Dominik Frey
* @date		16.08.19
* @version 	0.1
* @brief    An easy implementation to average multiple quaternions where each of them can be given a weight.
*/

#include <iostream>

// Eigen Library
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

// datastructures for storing values in rows
#include <vector>

Eigen::Vector4d getWeightedQuaternionAverage(std::vector<Eigen::Vector4d> quaternions,std::vector<float> weights){

  if(quaternions.size() != weights.size())
    throw std::invalid_argument("The amount of quaternions and the amount of weights was not the same. For each quaternion a weight is needed!");

  //! Form the symmetric accumulator matrix
  Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
  double weigthsSum = 0;

  for(int i = 0; i < quaternions.size() ; i++){
    A = weights.at(i)*(quaternions.at(i)*quaternions.at(i).transpose()) + A; // rank 1 update
    weigthsSum = weigthsSum + weights.at(i);
  }

  //! scale according to the weigths
  A = (1.0 / weigthsSum) * A;

  //! compute the eigenvalues and eigenvectors of the resulting matrix
  Eigen::EigenSolver<Eigen::Matrix4d> es;
  es.compute(A,true);

  Eigen::Vector4d eigenvalueAbsVector;
  for(int i = 0; i < 4; i++) {
    eigenvalueAbsVector(i) = abs(es.eigenvalues()(i));
  }

  //! find index of biggest eigenvalue
  int maxEigenvalueIndex = 0;
  eigenvalueAbsVector.maxCoeff(&maxEigenvalueIndex);

  //! return the eigenvector corresponding to largest eigen value
  return es.eigenvectors().col(maxEigenvalueIndex).cwiseAbs();;
}

int main() {
  //! Hardcode some quaternions
  Eigen::Vector4d quaternion1 (0, 0.0871557, 0, 0.9961947); //XYZW
  Eigen::Vector4d quaternion2 (0, 0.1736482, 0, 0.9848078); //XYZW
  Eigen::Vector4d quaternion3 (0, 0.1305262, 0, 0.9914449); //XYZW
  Eigen::Vector4d quaternion4 (0, 0.0436194, 0, 0.9990482); //XYZW

  std::vector<Eigen::Vector4d> quaternionVector = {quaternion1,quaternion2,quaternion3,quaternion4};

  //! Hardcode some weights
  float weight1 = 5.0;
  float weight2 = 1.0;
  float weight3 = 1.0;
  float weight4 = 1.0;
  std::vector<float> weightVector = {weight1,weight2,weight3,weight4};

  std::cout << "Input Quaternions: " << std::endl;
  for (int i = 0; i < quaternionVector.size(); ++i) {
    std::cout << "Quaternion " << i << std::endl;
    std::cout << quaternionVector.at(i) << std::endl;
    std::cout << "weight: " << weightVector.at(i) << std::endl << std::endl;
  }

  Eigen::Vector4d resultQuaternion = getWeightedQuaternionAverage(quaternionVector,weightVector);

  std::cout << "Result Quaternion: " << std::endl << resultQuaternion << std::endl;
  return 0;
}
