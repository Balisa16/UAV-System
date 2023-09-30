#ifndef PARAM_H
#define PARAM_H

#include <iostream>
#include <memory>
#include <copter.hpp>

namespace EMIRO{
	class Param
	{
	private:
		std::shared_ptr<EMIRO::Copter> copter;
	public:
		Param();
		void init(std::shared_ptr<EMIRO::Copter> copter);
		~Param();
		
	};
}

#endif // PARAM