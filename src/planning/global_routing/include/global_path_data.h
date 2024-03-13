#pragma once

#include <string>
struct Point2d_s
{
  double x;
  double y;
};

struct Point3D_s
{
  double x;
  double y;
  double z;
};

struct Point4d_s
{
  double x;
  double y;
  double z;
  double o;
};

struct State_s
{
  double x;
  double y;
  double z;
  double pitch;
  double roll;
  double yaw;
};

//参考系
const std::string Frame_id = "map";