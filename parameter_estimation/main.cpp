#include <rapidcsv.h>
#include <Eigen/Dense>

Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd &A)
{
    return A.completeOrthogonalDecomposition().pseudoInverse();
}

int main()
{
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
