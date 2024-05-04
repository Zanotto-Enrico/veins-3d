#include "RsuMobility.h"

Define_Module(RsuMobility);

RsuMobility::RsuMobility() : BaseMobility() {}

RsuMobility::~RsuMobility() {}

void RsuMobility::makeMove(const Coord& newPosition) {
    move.setStart(newPosition);
    updatePosition();
}
