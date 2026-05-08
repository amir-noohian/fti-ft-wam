#include <barrett/standard_main_function.h>
#include <barrett/systems/wam.h>
#include <barrett/products/product_manager.h>

#include <barrett/systems.h>
#include <barrett/systems/tuple_grouper.h>

#include <boost/tuple/tuple.hpp>

#include <iostream>
#include <string>

#include "atift_system.hpp"
#include "ft_csv_logger.hpp"

template<size_t DOF>
int wam_main(int argc, char** argv,
             barrett::ProductManager& pm,
             barrett::systems::Wam<DOF>& wam)
{
    typedef typename barrett::math::Vector<6>::type cf_type;
    typedef boost::tuple<double, cf_type> ft_sample_type;

    const std::string calPath =
        "/home/hela/Desktop/ft_libbarrett/cal/FT9235/FT9235.cal";

    const std::string logFile =
        "/home/hela/Desktop/ft_libbarrett/data/ft_libbarrett_log.csv";

    ATIFTSystem ftSensor(
        pm.getExecutionManager(),
        calPath,
        "Dev2/ai0:5"
    );

    // Optional print
    barrett::systems::PrintToStream<cf_type> printFT(
        pm.getExecutionManager(),
        "FT: "
    );

    // barrett::systems::connect(ftSensor.ftOutput, printFT.input);

    // Libbarrett time signal
    barrett::systems::Ramp time(
        pm.getExecutionManager(),
        1.0
    );

    time.stop();

    // Group time + FT data
    barrett::systems::TupleGrouper<double, cf_type> ftLogTg("FTLogGroup");
    pm.getExecutionManager()->startManaging(ftLogTg);

    barrett::systems::connect(time.output, ftLogTg.getInput<0>());
    barrett::systems::connect(ftSensor.ftOutput, ftLogTg.getInput<1>());

    const size_t periodMultiplier = 1;

    FTCsvLogger<ft_sample_type>* ftLogger = NULL;

    std::cout << "FT sensor ready.\n";
    std::cout << "Log file: " << logFile << "\n";
    std::cout << "Press Enter to START logging.\n";

    std::cin.get();

    // Start timing exactly when logging starts
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

    // Stop logger first
    barrett::systems::disconnect(ftLogger->input);
    ftLogger->closeFile();

    delete ftLogger;
    ftLogger = NULL;

    // Stop time
    time.stop();

    std::cout << "Logging stopped.\n";
    std::cout << "CSV saved: " << logFile << std::endl;

    pm.getSafetyModule()->waitForMode(barrett::SafetyModule::IDLE);

    return 0;
}
