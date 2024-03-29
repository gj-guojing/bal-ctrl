#include <filesystem>
#include <iostream>
#include <aris.hpp>

#include "plan.hpp"
#include "model.hpp"
#include "zmqmsg.hpp"
#include "control.hpp"

using namespace std;
string inputpath = "D:/Code/TriplePendulum/bal_ctrl/demo/demo_triplependulum/data/mvbal_data_2024-03-24--23-32-54.txt";
string outputpath = "D:/Code/TriplePendulum/bal_ctrl/demo/demo_triplependulum/data/control-test.txt";

int main(int argc, char* argv[]) {

	std::ifstream file(inputpath);
    if (!file.is_open()) {
        std::cerr << "无法打开文件" << std::endl;
        return 1;
    }

    std::ofstream outputFile(outputpath);
    if (!outputFile.is_open()) {
        std::cerr << "无法创建输出文件" << std::endl;
        return 1;
    }


    // 读取第一行，获取列名
    std::string line;
    std::getline(file, line);
    std::istringstream headerStream(line);
    std::vector<std::string> columnNames;
    std::string columnName;

    while (headerStream >> columnName) {
        columnNames.push_back(columnName);
    }

    // 定义变量
    int count;
    double q1, q2, q3, qd1, qd2, qd3, x, y, theta, trq2, trq3, trq2_t, trq3_t, trq2_act, trq3_act;
    double qdd1, qdd2, qdd3;
    double qdd1_f, qdd2_f, qdd3_f, qdd1_dyn, qdd2_dyn, qdd3_dyn;
    double qd1_d, qd2_d, qd3_d;
    double qd1_df, qd2_df, qd3_df;
    double qd1_f, qd2_f, qd3_f;
    double q1_f, q2_f, q3_f;
    double trq2_act_f, trq3_act_f;

    // 打印列名
    for (const auto& name : columnNames) {
        std::cout << name << "\t";
    }
    std::cout << std::endl;
    outputFile
        << "q1" << "\t"
        << "q2" << "\t"
        << "q3" << "\t"
        << "qd1" << "\t"
        << "qd2" << "\t"
        << "qd3" << "\t"
        << "qdd1" << "\t"
        << "qdd2" << "\t"
        << "qdd3" << "\t"
        << "trq2_dyn" << "\t"
        << "trq3_dyn" << "\t"
        << std::endl;

    triple::TripleModel tripleModel;
    triple::Controller triplePendulumController(tripleModel.createModel().release());

    std::vector<double> targetxy{ 0.05, 0.7 };
    std::vector<double> weightxy{ 5, 10, 2, 8 };
    triple::fixedPlan fixedPointPlan(targetxy, weightxy);

    std::vector<double> data{0};
    std::vector<double> desiredValue{0};

    std::vector<double> desiredAcc{0};
    std::vector<double> lambda{ 1e-5, 1e-5, 1e-1, 1e-1, 1000 };
    triplePendulumController.setQPparameters(lambda);

    data.resize(18);
    //count	q1	q2	q3	qd1	qd2	qd3	x	y	theta	qdd1	qdd2	qdd3	trq2	trq3	trq2_act	trq3_act
    while (file >> count >> q1 >> q2 >> q3 >> qd1 >> qd2 >> qd3 >>
                x >> y >> theta >> qdd1 >> qdd2 >> qdd3 >>  trq2 >> trq3 >> trq2_act >> trq3_act) {

        //std::cout << count << "\t" << q1 << "\t" << q2 << "\t" << q3 << "\t"
        //    << qd1 << "\t" << qd2 << "\t" << qd3 << "\t"
        //    << x << "\t" << y << "\t" << theta << "\t" << qdd1 << "\t" << qdd2 << "\t"
        //    << qdd3 << "\t" << trq2 << "\t" << trq3 << std::endl;
        //std::cout << " " << std::endl;
        
        data = { q1, q2, q3, qd1, qd2, qd3, x, y, theta, 
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, qdd1, qdd2, qdd3 };

        triplePendulumController.calcuForwardKinematics(data);

        fixedPointPlan.setStateVal(data);
        
        desiredValue = fixedPointPlan.getStateVal();

        // calculate triple pendulum torque 
        triplePendulumController.getStateVar(data, desiredValue);

        std::vector<double> torque = triplePendulumController.sendTorque();



        outputFile
            << q1 << "\t"
            << q2 << "\t"
            << q3 << "\t"
            << qd1 << "\t"
            << qd2 << "\t"
            << qd3 << "\t"
            << qdd1 << "\t"
            << qdd2 << "\t"
            << qdd3 << "\t"
            << torque[0] << "\t"
            << torque[1] << "\t"
            << std::endl;

     //   std::cout 
     //       << q1 << "\t"
     //       << q2 << "\t"
     //       << q3 << "\t"
     //       << qd1 << "\t"
     //       << qd2 << "\t"
     //       << qd3 << "\t"
     //       << qdd1 << "\t"
     //       << qdd2 << "\t"
     //       << qdd3 << "\t"
     //       << torque[0] << "\t"
     //       << torque[1] << "\t"
     //       << std::endl;
     }


}