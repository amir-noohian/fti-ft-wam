#include <barrett/standard_main_function.h>
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

    // Print raw sensor/tool-frame force/torque
    barrett::systems::PrintToStream<ft_type> printSensorFT(
        pm.getExecutionManager(),
        "Sensor/Tool FT: "
    );

    // Print base-frame force/torque
    barrett::systems::PrintToStream<ft_type> printBaseFT(
        pm.getExecutionManager(),
        "Base FT: "
    );

    // Print joint torques from Jacobian transpose
    barrett::systems::PrintToStream<jt_type> printJointTorque(
        pm.getExecutionManager(),
        "Joint Torque from FT: "
    );

    barrett::systems::connect(
        ftSensor.ftOutput,
        printSensorFT.input
    );

    barrett::systems::connect(
        toolFTToBaseFT.baseFTOutput,
        printBaseFT.input
    );

    barrett::systems::connect(
        baseFTToJointTorque.jointTorqueOutput,
        printJointTorque.input
    );

    wam.gravityCompensate();


    // Libbarrett time signal
    barrett::systems::Ramp time(
        pm.getExecutionManager(),
        1.0
    );

    time.stop();

    // Group time + raw sensor FT data
    barrett::systems::TupleGrouper<double, ft_type> ftLogTg("FTLogGroup");
    pm.getExecutionManager()->startManaging(ftLogTg);

    barrett::systems::connect(time.output, ftLogTg.getInput<0>());
    barrett::systems::connect(ftSensor.ftOutput, ftLogTg.getInput<1>());

    const size_t periodMultiplier = 1;

    FTCsvLogger<ft_sample_type>* ftLogger = NULL;

    std::cout << "FT sensor ready.\n";
    std::cout << "Log file: " << logFile << "\n";
    std::cout << "Press Enter to START logging.\n";

    std::cin.get();

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

    std::cout << "Logging started.\n";
    std::cout << "Press Enter to STOP logging.\n";

    std::cin.get();

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
