#ifndef ATI_FT_SYSTEM_HPP
#define ATI_FT_SYSTEM_HPP

#include <barrett/systems.h>
#include <barrett/detail/ca_macro.h>

#include <Eigen/Core>
#include <string>

#include "atift_reader.hpp"

class ATIFTSystem : public barrett::systems::System {
    
    typedef typename barrett::math::Vector<6>::type sf_type;

public:
    Output<sf_type> ftOutput;

protected:
    typename Output<sf_type>::Value* ftOutputValue;

public:
    explicit ATIFTSystem(
        barrett::systems::ExecutionManager* em,
        const std::string& calPath,
        const std::string& channelString = "Dev2/ai16:21",
        const std::string& sysName = "ATIFTSystem"
    )
        : barrett::systems::System(sysName)
        , ftOutput(this, &ftOutputValue)
        , reader_(calPath, channelString)
    {
        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~ATIFTSystem()
    {
        this->mandatoryCleanUp();
    }

protected:
    ATIFTReader::Vector6 ftArray;
    sf_type ft;

    virtual void operate()
    {
        ftArray = reader_.read();

        for (int i = 0; i < 6; ++i) {
            ft(i) = ftArray[i];
        }

        ftOutputValue->setData(&ft);
    }

private:
    ATIFTReader reader_;

    DISALLOW_COPY_AND_ASSIGN(ATIFTSystem);
};

#endif