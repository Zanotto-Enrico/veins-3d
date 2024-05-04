#ifndef RSUMOBILITY_H_
#define RSUMOBILITY_H_

#include "veins/base/modules/BaseMobility.h"
#include "veins/base/utils/MiXiMDefs.h"
#include "veins/base/utils/Coord.h"

class MIXIM_API RsuMobility : public BaseMobility {
  public:
    RsuMobility();
    virtual ~RsuMobility();

    void makeMove(const Coord& newPosition);
};

#endif /* RSUMOBILITY_H_ */
