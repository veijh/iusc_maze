/* 该文件实现ENU坐标系和给定任务坐标系之间坐标转换 */
#ifndef IUSC_MAZE_GPS_COORD_TF_H
#define IUSC_MAZE_GPS_COORD_TF_H
#include "WGS84toCartesian.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Eigen>

#define PAI (3.1415926)

using namespace std;

class GPS_COORD_TF
{
  private:
  // MSN坐标系相对正北正东的偏移角，逆时针为正方向
    double MSN_head_rad;
  // 上电建立的ENU坐标系相对正北正东的偏移角，逆时针为正方向
    double ENU_head_rad;
  // 任务坐标系原点相对于GPS原点的偏移，GPS坐标系方向沿正北正东
    Eigen::Vector2d MSN_to_GPS_offset;
  // MSN相对正北正东的旋转矩阵
    Eigen::Matrix2d MSN_ref_GPS;
  // ENU相对正北正东的旋转矩阵
    Eigen::Matrix2d ENU_ref_GPS;
  public:
  // 构造函数，计算偏移量
  /* _ENU_head_deg: ENU坐标系建立时，相对正北正东的偏移角，逆时针为正方向；可从电子罗盘处获取
   * ENU_LALO：ENU原点的纬度和经度
   * MSN_LALO：参考点的纬度和经度
   * MSN_XY：参考点在任务坐标系下的坐标
   */
    GPS_COORD_TF(const double &_ENU_head_deg=0.0, const Eigen::Vector2d &ENU_LALO, const vector<Eigen::Vector2d> &MSN_LALO, const vector<Eigen::Vector2d> &MSN_XY);
  // 任务坐标系坐标 -> ENU坐标系坐标
    Eigen::Vector2d MSN_to_ENU(const Eigen::Vector2d &MSN_XY);
    Eigen::Vector2d MSN_to_ENU(const double &MSN_X, const double &MSN_Y);
    void MSN_to_ENU(const double &MSN_X, const double &MSN_Y, double &ENU_X, double &ENU_Y);
  // ENU坐标系坐标 -> 任务坐标系坐标
    Eigen::Vector2d ENU_to_MSN(const Eigen::Vector2d &ENU_XY);
    Eigen::Vector2d ENU_to_MSN(const double &ENU_X, const double &ENU_Y);
    void ENU_to_MSN(const double &ENU_X, const double &ENU_Y, double &MSN_X, double &MSN_Y);

    /* 
      abort
    /*
    Eigen::Vector2d MSN_to_ENU(double &offset_x, double &offset_y,
                        const double &msn_lat, const double &msn_lon, const double &msn_x, const double &msn_y,
                        const double &enu_lat, const double &enu_lon, const double &enu_x = 0.0, const double &enu_y = 0.0)
    {
      std::array<double, 2> delta_EN;
      delta_EN = wgs84::toCartesian({msn_lat, msn_lon}, {enu_lat, enu_lon});
      offset_x = enu_x - delta_EN[0] - msn_x;
      offset_y = enu_y - delta_EN[1] - msn_y;
    }
    */
};
#endif