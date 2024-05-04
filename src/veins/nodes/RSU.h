#ifndef RSU_H_
#define RSU_H_

#include <omnetpp.h>
#include "veins/base/utils/MiXiMDefs.h"

using namespace omnetpp;

class MIXIM_API RSU : public cModule {
  protected:
    virtual void initialize(int stage) override;
    virtual void finish() override;
};

#endif /* RSU_H_ */