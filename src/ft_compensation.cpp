#include <barrett/standard_main_function.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/wam.h>
#include <barrett/products/product_manager.h>

#include <barrett/systems.h>
#include <barrett/systems/tuple_grouper.h>
#include <barrett/units.h>

#include <boost/tuple/tuple.hpp>

#include <iostream>
#include <string>

#include "atift_system.hpp"
#include "tb_transformation.hpp"
#include "jacobian.hpp"
#include "bj_transformation.hpp"
#include "ft_csv_logger.hpp"

template<size_t DOF>
int wam_main(int argc, char** argv,
             barrett::ProductManager& pm,
             barrett::systems::Wam<DOF>& wam)
{
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

    typedef typename barrett::math::Vector<6>::type ft_type;
    typedef boost::tuple<double, ft_type> ft_sample_type;

    const std::string calPath =
        "/home/hela/Desktop/ft_libbarrett/cal/FT9236/FT9236.cal";

    const std::string logFile =
        "/home/hela/Desktop/ft_libbarrett/data/ft_libbarrett_log.csv";

    ATIFTSystem ftSensor(
        pm.getExecutionManager(),
        calPath,
        "Dev2/ai16:21"
    );

    ToolOrientationOutput<DOF> toolOrientation(
        pm.getExecutionManager(),
        wam
    );

    ToolFTToBaseFT toolFTToBaseFT(
        pm.getExecutionManager()
    );

    ToolJacobianOutput<DOF> toolJacobian(
        wam,
        pm.getExecutionManager()
    );

    BaseFTToJointTorque<DOF> baseFTToJointTorque(
        pm.getExecutionManager()
    );

    barrett::systems::connect(
        ftSensor.ftOutput,
        toolFTToBaseFT.toolFTInput
    );

    barrett::systems::connect(
        toolOrientation.rotationOut,
        toolFTToBaseFT.rotationInput
    );

    barrett::systems::connect(
        toolJacobian.output,
        baseFTToJointTorque.jacobianInput
    );

    barrett::systems::connect(
        toolFTToBaseFT.baseFTOutput,
        baseFTToJointTorque.baseFTInput
    );


    barrett::systems::PrintToStream<ft_type> printSensorFT(
        pm.getExecutionManager(),
        "Sensor/Tool FT: "
    );

    barrett::systems::PrintToStream<ft_type> printBaseFT(
        pm.getExecutionManager(),
        "Base FT: "
    );

    barrett::systems::PrintToStream<jt_type> printJointTorque(
        pm.getExecutionManager(),
        "Joint Torque from FT: "
    );

    barrett::systems::PrintToStream<jt_type> printWamjtSum(
        pm.getExecutionManager(),
        "wam.jtSum: "
    );

    barrett::systems::PrintToStream<jt_type> printGravity(
        pm.getExecutionManager(),
        "Gravity: "
    );

    barrett::systems::connect(ftSensor.ftOutput, printSensorFT.input);
    barrett::systems::connect(toolFTToBaseFT.baseFTOutput, printBaseFT.input);
    barrett::systems::connect(baseFTToJointTorque.jointTorqueOutput, printJointTorque.input);
    barrett::systems::connect(wam.jtSum.output, printWamjtSum.input);
    barrett::systems::connect(wam.gravity.output, printGravity.input);


    wam.gravityCompensate();

    barrett::systems::Ramp time(
        pm.getExecutionManager(),
        1.0
    );

    time.stop();

    barrett::systems::TupleGrouper<double, ft_type> ftLogTg("FTLogGroup");
    pm.getExecutionManager()->startManaging(ftLogTg);

    barrett::systems::connect(time.output, ftLogTg.getInput<0>());
    barrett::systems::connect(ftSensor.ftOutput, ftLogTg.getInput<1>());

    const size_t periodMultiplier = 1;

    FTCsvLogger<ft_sample_type>* ftLogger = NULL;

    std::cout << "FT sensor ready.\n";
    std::cout << "Log file: " << logFile << "\n";
    std::cout << "Commands:\n";
    std::cout << "  f + Enter : toggle FT joint-torque feedback to WAM input\n";
    std::cout << "  q + Enter : quit\n";

    time.stop();
    time.reset();
    time.start();

    ftLogger = new FTCsvLogger<ft_sample_type>(
        pm.getExecutionManager(),
        logFile,
        periodMultiplier,
        "FTCsvLogger"
    );

    barrett::systems::forceConnect(ftLogTg.output, ftLogger->input);

    bool ftFeedbackEnabled = false;
    char cmd;

    while (std::cin >> cmd) {
        if (cmd == 'f') {
            if (!ftFeedbackEnabled) {
                // barrett::systems::forceConnect(
                //     baseFTToJointTorque.jointTorqueOutput,
                //     wam.input
                // );
                wam.trackReferenceSignal(baseFTToJointTorque.jointTorqueOutput);

                ftFeedbackEnabled = true;
                std::cout << "FT joint-torque feedback ENABLED.\n";
            } else {
                // barrett::systems::disconnect(wam.input);
                wam.idle();

                // wam.gravityCompensate();

                ftFeedbackEnabled = false;
                std::cout << "FT joint-torque feedback DISABLED. Back to gravity compensation.\n";
            }
        } else if (cmd == 'q') {
            break;
        }
    }

    if (ftFeedbackEnabled) {
        barrett::systems::disconnect(wam.input);
        wam.gravityCompensate();
    }

    barrett::systems::disconnect(ftLogger->input);
    ftLogger->closeFile();

    delete ftLogger;
    ftLogger = NULL;

    time.stop();

    std::cout << "Logging stopped.\n";
    std::cout << "CSV saved: " << logFile << std::endl;

    pm.getSafetyModule()->waitForMode(barrett::SafetyModule::IDLE);

    return 0;
}