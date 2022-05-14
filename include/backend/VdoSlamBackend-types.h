#pragma once

#include "utils/macros.h"
#include "utils/types.h"
#include "factors/Point3DFactor.h"

#include <iostream>
#include <vector>

namespace VDO_SLAM {

typedef boost::shared_ptr<Point3DFactor> Point3DFactorPtr;
typedef std::vector<Point3DFactorPtr> Point3DFactors;

//quick reverse lookup for unique vertices
struct IJSymbol {
    int i;
    int j;
    unsigned char symbol;
    
    IJSymbol()
        :   i(-1),
            j(-1),
            symbol('\0') {}
};


std::ostream& operator<<(std::ostream& os, const IJSymbol& symbol);


} //VDO_SLAM