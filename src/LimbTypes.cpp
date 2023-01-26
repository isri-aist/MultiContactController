#include <MultiContactController/LimbTypes.h>

using namespace MCC;

std::string std::to_string(const Limb & limb)
{
  return limb.name;
}

bool std::equal_to<Limb>::operator()(const Limb & lhs, const Limb & rhs) const
{
  return lhs.name == rhs.name;
}

size_t std::hash<Limb>::operator()(const Limb & limb) const
{
  return std::hash<std::string>()(limb.name);
}
