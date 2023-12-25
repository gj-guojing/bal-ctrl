/// 走x方向的轨迹
//-------------------------------------------------------------------------------------------------------//
//三个杆件，最后一根连杆上有一个质量
//模型示意：
//                                    
//                                      ** (body)
//                                      /
//                                     /  link3 (c)
//                                    o
//                                   /
//                                  /  link2(b)
//                                 o                           y
//                                /                            ^
//                               / link1(a)                    | 
//                              o                              *---> x
//                            ground
//
//-------------------------------------------------------------------------------------------------------//

#include <filesystem>
#include <iostream>
#include <aris.hpp>

#include "plan/plan.hpp"
#include "controller/model.hpp"
#include "zmq/zmqmsg.hpp"
#include "controller/control.hpp"
#include "utils/utils.hpp"

const double PI = 3.141592653589793;


auto init() {

}

auto algorithm() {


}





int main(int argc, char* argv[])
{
	triple::TripleModel tripleModel;

	triple::Controller triplePendulumController(&tripleModel);

	Zmqmsg zmqmsg;
	zmqmsg.init();
	
	// Open the file which name is "data+time", and the address is out
	openFile();
	
	// Initialize plan to get desired accelerate;
	// (0.2 0.8) weightxy 5,10, kx = 2
	//std::vector<double> targetxy{ 0.16, 1.0 };
	//std::vector<double> weightxy{ 5, 10, 2, 8 };

	std::vector<double> targetxy{ 0, 0.725 };
	std::vector<double> weightxy{ 1, 10, 1, 10 };
	triple::fixedPlan fixedPointPlan(targetxy, weightxy);

	std::vector<double> data{0};
	std::vector<double> desiredValue{0};

	while (true) {
		data = zmqmsg.get_request();

		// data: joint1, joint2, joint3, w1, w2, w3, x, y, angle, vx, vy, wz, ax, ay, bz, ja1, ja2, ja3
		tripleModel.calcuForwardKinematics(data);

		// stateValue:  joint1, joint2, joint3, w1, w2, w3, ax, ay
		fixedPointPlan.setStateVal(data);

		// desiredValue: x desired acc, y desired acc
		desiredValue = fixedPointPlan.getStateVal();

		// calculate triple pendulum torque 
		triplePendulumController.getStateVar(data, desiredValue);
		std::vector<double> torque = triplePendulumController.sendTorque();

		zmqmsg.send_msg(torque);

		// 验证计算是否正确
		// triplePendulumController.verifyAccelerate(data);

	}

	return 0;
}




























