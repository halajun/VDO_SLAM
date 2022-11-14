#include "backend/VdoSlamBackend-types.h"

namespace VDO_SLAM
{
std::ostream& operator<<(std::ostream& os, const IJSymbol& symbol)
{
  os << "[i: " << symbol.i << " j: " << symbol.j << " sym: " << symbol.symbol << "]";
  return os;
}

}  // namespace VDO_SLAM
