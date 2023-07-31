#ifndef __MAP_PATH_H__
#define __MAP_PATH_H__

struct MapPath {
  std::string path;
  std::string occupancy_grid_map;
  std::string map_RGBA;
  std::string way_point;
  std::string likelyhood_field;
  std::string mapInfo;
  double init_x;
  double init_y;
  double init_a;

  MapPath(
      std::string path_,
      std::string occupancy_grid_map_,
      std::string map_RGBA_,
      std::string way_point_,
      std::string likelyhood_field_,
      std::string mapInfo_,
      double init_x, double init_y, double init_a) {
    path = path_;
    occupancy_grid_map = occupancy_grid_map_;
    map_RGBA = map_RGBA_;
    way_point = way_point_;
    likelyhood_field = likelyhood_field_;
    mapInfo = mapInfo_;
    this->init_x = init_x;
    this->init_y = init_y;
    this->init_a = init_a;
  };
};
#endif
