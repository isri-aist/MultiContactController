#include <MultiContactController/LimbTypes.h>

using namespace MCC;

Limb::Limb(const std::string & _name, const std::string & _group) : name(_name), group(_group)
{
  if(group.empty())
  {
    if(name.find("Hand") != std::string::npos)
    {
      group = Group::Hand;
    }
    else if(name.find("Foot") != std::string::npos)
    {
      group = Group::Foot;
    }
  }
}

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
