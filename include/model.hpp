#ifndef MODEL_H
#define MODEL_H

#include <aris.hpp>
// #include <iostream>
// #include <fstream>
#include "export.h"

namespace triple  {

	using namespace aris::dynamic;
	
	auto BAL_CTRL_API createModel() -> std::unique_ptr<Model>;

	class BAL_CTRL_API TripleModel : public aris::dynamic::Model
	{
	public:
		void calcuForwardKinematics(std::vector<double>& data);
		// auto getmodel()->std::shared_ptr<aris::dynamic::Model> { return this->m_; };
		// auto getmodel()const -> const std::shared_ptr<aris::dynamic::Model> { return const_cast<TripleModel*>(this)->m_; };
		  
		TripleModel();
		~TripleModel();
	};
}

#endif // !MODEL_H