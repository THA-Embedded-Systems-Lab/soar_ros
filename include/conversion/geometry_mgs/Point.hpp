#include <geometry_mgs/msg/Point.hpp>

namespace soar_ros {
namespace conversion {
namespace geometry_mgs {
void to_wme(const geometry_msgs::Point &pt, sml::Identifier *id) {
  id->CreateFloatWME("x", pt.x);
  id->CreateFloatWME("y", pt.y);
  id->CreateFloatWME("z", pt.z);
}

geometry_msgs::Point from_wme(sml::Identifier *root) {
  geometry_msgs::Point pt;
  for (int i = 0; i < root->GetNumberChildren(); ++i) {
    sml::WMElement *wme = root->GetChild(i);
    std::string attr = wme->GetAttribute();
    if (attr == "x")
      pt.x = wme->GetValueAsDouble();
    else if (attr == "y")
      pt.y = wme->GetValueAsDouble();
    else if (attr == "z")
      pt.z = wme->GetValueAsDouble();
  }
  return pt;
}
} // namespace geometry_mgs

} // namespace conversion
} // namespace soar_ros
