#ifndef CONTROL_H
#define CONTROL_H

#include <aris.hpp>
#include <iostream>
#include <fstream>
#include <cmath>
//#include <Eigen/Dense>
//#include <Eigen/Core>
//#include <Eigen/SVD>

#include "control.hpp"
//#include "math.hpp"
#include "model.hpp"
#include "export.h"


namespace triple {

	using namespace aris::dynamic;
	using namespace triple;

	class BAL_CTRL_API Controller
	{
	public:
		void init(triple::TripleModel* model);
		//auto setModel(std::shared_ptr<aris::dynamic::Model> m) -> std::shared_ptr<aris::dynamic::Model> { return this->imp_->m_ = m; };
		void getStateVar(std::vector<double>& data, std::vector<double>& acc);
		void estimateState();
		void cptModelCm();
		void cptModelAngularMoment();
		void calcAandBColumn(double* force);
		void calculateAandB();
		void checkrealAcc(int m, int n);
		void cptdesiredCoMAcc();
		void calculateTorque();
		auto getDesiredAcc() -> double* { return lastrealAcc.data(); };
		auto sendTorque() -> std::vector<double>;
		void verifyAccelerate(std::vector<double>& data);
		void test();
		void inverseMethodsolveTorque();
		void cptInverseA2toruqe();
		void calculateTest();

		Controller(const Controller& other);
		explicit Controller(int outputSize = 2, int intputSize = 3);
		explicit Controller(triple::TripleModel* model, int outputSize = 2, int intputSize = 3);
		~Controller();

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
		std::vector<double> lastTorque;
		std::vector<double> lastrealAcc{0.0, 0.0, 0.0 };

	};


}







#endif // !CONTROL_H
