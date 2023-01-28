#include <locale>

#include <MultiContactController/LimbTypes.h>

using namespace MCC;

Limb::Limb(const std::string & _name, const std::string & _group) : name(_name), group(_group)
{
  if(group.empty())
  {
    std::string lowerName = std::tolower(name, std::locale::classic());
    if(lowerName.find("hand") != std::string::npos)
    {
      group = Group::Hand;
    }
    else if(lowerName.find("foot") != std::string::npos)
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
