// model.h: 标准系统包含文件的包含文件

#ifndef MODEL_H
#define MODEL_H

#include <aris.hpp>
#include <iostream>
#include <fstream>
#include "export.h"

namespace triple  {

	using namespace aris::dynamic;

	class BAL_CTRL_API TripleModel : public aris::dynamic::Model
	{
	public:
		void TripleModel::createModel();
		void TripleModel::calcuForwardKinematics(std::vector<double>& data);
		auto TripleModel::getmodel()->std::shared_ptr<aris::dynamic::Model> { return this->m_; };
		auto TripleModel::getmodel()const -> const std::shared_ptr<aris::dynamic::Model> { return const_cast<TripleModel*>(this)->m_; };
		  
		TripleModel::TripleModel();

		TripleModel::~TripleModel();

	private:
		std::shared_ptr<aris::dynamic::Model> m_{nullptr};
	};
}





#endif // !MODEL_H