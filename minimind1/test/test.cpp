// ubuntu: g++ -I /usr/include/eigen3/ test.cpp -o test.exe

#include <Eigen/Dense>
#include <iostream>

int main() {

    Eigen::Matrix2f a; //2x2
    Eigen::Vector2f v; //2x1
    a << 1, 2,
         3, 4;
    a = 2*a;
    v << 5,
         6;
    std::cout << a << std::endl;


    // Define a 3x2 matrix
    Eigen::VectorXd matrix(3);

    // You can initialize the elements of the matrix like this
    matrix << 2,
              4,
              5;

    // Access elements
    std::cout << "Matrix(1, 1) = " << matrix(1) << std::endl;


    Eigen::MatrixXd kalman_gain(2,2);
    kalman_gain << 1,2,3,4;

    Eigen::MatrixXd something(2,2);
    something << kalman_gain;

}