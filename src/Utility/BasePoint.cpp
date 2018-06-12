#include "BasePoint.h"

BasePoint::BasePoint(){
    this->id = 0;
}
//Reconstruct: Operator overloading created, since it was problematic in terms of invariants
void BasePoint::operator = (const BasePoint& bp)
{
    this->id = bp.id;
    this->avgVal = bp.avgVal;
    this->varVal = bp.varVal;
    this->avgLas = bp.avgLas;
    this->varLas = bp.varLas;
    this->intensityInvariants = bp.intensityInvariants.clone();
    this->hueInvariants = bp.hueInvariants.clone();
    this->status = bp.status;
    this->location_x = bp.location_x;
    this->location_y = bp.location_y;
    this->pID = bp.pID;
}
