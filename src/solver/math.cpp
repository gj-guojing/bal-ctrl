#include "math.hpp"


Eigen::MatrixXd computePeseudoInverse(const Eigen::MatrixXd& jacobian) {
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
	const double tolerance = 1e-7; // ���̶ȣ������ж�����ֵ�Ƿ�Ϊ��
	double toleranceSq = tolerance * tolerance;

	// ����α�����
	Eigen::MatrixXd singularValuesInv = svd.singularValues();
	for (int i = 0; i < singularValuesInv.size(); i++) {
		if (singularValuesInv(i) > toleranceSq)
			singularValuesInv(i) = 1.0 / singularValuesInv(i);
		else
			singularValuesInv(i) = 0.0;
	}
	return svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
}
