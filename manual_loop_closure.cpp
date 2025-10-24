/**
 * manual_loop_closure.cpp
 * 
 * このプログラムは、slam.cppによって生成された部分地図データを読み込み、
 * ユーザーが手動で地図を移動・回転させてループクロージャを行うためのツールです。
 */

#include <iostream>
#include <vector>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <iomanip> 
#include <string>
#include <algorithm>

namespace fs = std::filesystem;
using OccupancyGrid = std::vector<std::vector<double>>;

// slam.cppから移植した定義
#define GREEN cv::Scalar(118,209,173)
#define BACK_BLUE cv::Scalar(47,35,30)
#define BLUE_LIGHT cv::Scalar(255,218,0)
#define WHITE cv::Scalar(198,204,203)
#define RED cv::Scalar(0,31,245)

// slam.cppから移植した構造体
struct Point {
  double x, y;
  Point() : x(0), y(0) {};
  Point(double x, double y) : x(x), y(y) {}
};

struct Pose {
  long long ts;
  double x, y, a;
  Pose() : ts(0), x(0), y(0), a(0) {};
  Pose(long long ts, double x, double y, double a) : ts(ts), x(x), y(y), a(a) {}
};

struct SubMap {
  int submap_id;
  Pose start_pose;  // グローバル座標系での部分地図の基準姿勢
  OccupancyGrid local_gmap; // 部分地図内のローカルな占有格子地図

  // 地図の物理的・ピクセル的情報
  double min_x, max_x, min_y, max_y; // ローカル座標系でのバウンディングボックス
  double LOCAL_CSIZE;
  int LOCAL_WIDTH, LOCAL_HEIGHT, LOCAL_ORIGIN_X, LOCAL_ORIGIN_Y;

  // 読み込み成功フラグ
  bool is_valid = false;
};

// slam.cppから移植したユーティリティ関数
constexpr double deg2rad(double deg) { return deg * M_PI / 180.0; }
constexpr double rad2deg(double rad) { return rad * 180.0 / M_PI; }
double normalize_th(double ra) {
  while(ra > M_PI) ra -= 2*M_PI;
  while(ra < -M_PI) ra += 2*M_PI;
  return ra;
}

// 関数のプロトタイプ宣言
std::vector<SubMap> load_all_submaps(const std::string& root_dir);
void visualize_maps(const std::vector<SubMap>& submaps, int selected_id);


/******************************************************
* MAIN
*******************************************************/
int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << "使用法: " << argv[0] << " <slam_result_directory>" << std::endl;
    return 1;
  }
  std::string result_dir = argv[1];

  // 1. 全ての部分地図データをファイルから読み込む
  std::vector<SubMap> submaps = load_all_submaps(result_dir);
  if (submaps.empty()) {
    std::cerr << "部分地図を読み込めませんでした。ディレクトリを確認してください: " << result_dir << std::endl;
    return 1;
  }

  int selected_submap_id = 0; // 現在選択されている部分地図のID
  bool needs_redraw = true;   // 再描画が必要かどうかのフラグ
  bool id_selection_mode = false; // ID選択モードのフラグ
  int temp_highlighted_id = 0; // ID選択モード中にハイライトされるID

  std::cout << "操作方法:" << std::endl;
  std::cout << "  - i: ID選択モードに入る (n/pで移動, Enterで決定, Escでキャンセル)" << std::endl;
  std::cout << "  - 矢印キー: 選択した地図以降を平行移動" << std::endl;
  std::cout << "  - r/t: 選択した地図以降を回転 (r:反時計回り, t:時計回り)" << std::endl;
  std::cout << "  - s: 現在の変更を保存（未実装）" << std::endl;
  std::cout << "  - q: 終了" << std::endl;

  // 2. インタラクションループ
  while (true) {
    int id_to_highlight = id_selection_mode ? temp_highlighted_id : selected_submap_id;
    if (needs_redraw) {
      visualize_maps(submaps, id_to_highlight);
      needs_redraw = false;
    }

    int key = cv::waitKey(0); // キー入力を待つ

    double dx = 0, dy = 0, da = 0; // 移動量

    if (id_selection_mode) {
      switch (key) {
        case 'n': // Next ID
          temp_highlighted_id = (temp_highlighted_id + 1) % submaps.size();
          std::cout << "ハイライトID: " << temp_highlighted_id << std::endl;
          needs_redraw = true;
          break;
        case 'p': // Previous ID
          temp_highlighted_id = (temp_highlighted_id - 1 + submaps.size()) % submaps.size();
          std::cout << "ハイライトID: " << temp_highlighted_id << std::endl;
          needs_redraw = true;
          break;
        case 13: // Enter key (ASCII for CR)
          selected_submap_id = temp_highlighted_id;
          id_selection_mode = false;
          std::cout << "部分地図 " << selected_submap_id << " を選択しました。" << std::endl;
          needs_redraw = true;
          break;
        case 27: // Escape key
          id_selection_mode = false;
          std::cout << "ID選択モードをキャンセルしました。" << std::endl;
          needs_redraw = true;
          break;
        default:
          // Ignore other keys in selection mode
          break;
      }
    } else { // Normal mode
      switch (key) {
        case 'i': // Enter ID selection mode
          id_selection_mode = true;
          temp_highlighted_id = selected_submap_id; // Start highlighting from current selection
          std::cout << "ID選択モードに入りました (n/pで移動, Enterで決定, Escでキャンセル)。" << std::endl;
          needs_redraw = true;
          break;

        // --- 移動・回転 ---
        case 82: dy = -0.1; break; // Up arrow
        case 84: dy = 0.1; break;  // Down arrow
        case 81: dx = -0.1; break; // Left arrow
        case 83: dx = 0.1; break;  // Right arrow
        case 'r': da = deg2rad(1.0); break; // 1度回転
        case 't': da = -deg2rad(1.0); break; // -1度回転

        // --- 終了 ---
        case 'q':
          std::cout << "終了します。" << std::endl;
          return 0;

        // --- 保存（未実装） ---
        case 's':
          std::cout << "保存機能は未実装です。" << std::endl;
          break;
      }
    }

    // 3. 連動ロジックの適用
    if (dx != 0 || dy != 0 || da != 0) {
      // 回転の中心を選択されたサブマップの原点に設定
      const double center_x = submaps[selected_submap_id].start_pose.x;
      const double center_y = submaps[selected_submap_id].start_pose.y;
      const double c = cos(da);
      const double s = sin(da);

      for (int i = std::max(selected_submap_id, 1); i < submaps.size(); ++i) {
        // 回転処理 (選択された地図の中心周り)
        double translated_x = submaps[i].start_pose.x - center_x;
        double translated_y = submaps[i].start_pose.y - center_y;

        double rotated_x = translated_x * c - translated_y * s;
        double rotated_y = translated_x * s + translated_y * c;

        submaps[i].start_pose.x = rotated_x + center_x;
        submaps[i].start_pose.y = rotated_y + center_y;

        // 平行移動処理
        submaps[i].start_pose.x += dx;
        submaps[i].start_pose.y += dy;

        // 角度の更新
        submaps[i].start_pose.a = normalize_th(submaps[i].start_pose.a + da);
      }
      needs_redraw = true;
    }
  }

  return 0;
}

// YAMLファイルから特定のキーの値を取得する、より堅牢なヘルパー関数
double get_yaml_value(const std::string& path, const std::string& key_to_find) {
    std::ifstream file(path);
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string key_part;
        ss >> key_part;
        if (key_part == key_to_find + ":") {
            double value;
            ss >> value;
            if (!ss.fail()) {
                return value;
            }
        }
    }
    return 0.0; // 見つからなかった、またはパース失敗
}

// 単一の部分地図を読み込むヘルパー関数
SubMap load_single_submap(const std::string& submap_dir) {
    SubMap submap;
    std::string map_info_path = submap_dir + "/mapInfo.yaml";
    std::string gmap_path = submap_dir + "/local_gmap.yml";

    if (!fs::exists(map_info_path) || !fs::exists(gmap_path)) {
        return submap; // 必要なファイルがない
    }

    // 1. mapInfo.yamlからメタデータを読み込む
    submap.submap_id = static_cast<int>(get_yaml_value(map_info_path, "submap_id"));
    submap.start_pose.x = get_yaml_value(map_info_path, "x");
    submap.start_pose.y = get_yaml_value(map_info_path, "y");
    submap.start_pose.a = get_yaml_value(map_info_path, "a");
    submap.min_x = get_yaml_value(map_info_path, "min_x");
    submap.max_x = get_yaml_value(map_info_path, "max_x");
    submap.min_y = get_yaml_value(map_info_path, "min_y");
    submap.max_y = get_yaml_value(map_info_path, "max_y");

    // 2. local_gmap.ymlから地図データを読み込む
    cv::FileStorage fs(gmap_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return submap;
    }
    cv::Mat gmap_mat;
    fs["local_gmap"] >> gmap_mat;
    fs["width"] >> submap.LOCAL_WIDTH;
    fs["height"] >> submap.LOCAL_HEIGHT;
    fs["origin_x"] >> submap.LOCAL_ORIGIN_X;
    fs["origin_y"] >> submap.LOCAL_ORIGIN_Y;
    fs["cell_size"] >> submap.LOCAL_CSIZE;
    fs.release();

    // cv::MatをOccupancyGridに変換
    submap.local_gmap.resize(gmap_mat.rows, std::vector<double>(gmap_mat.cols));
    for (int r = 0; r < gmap_mat.rows; ++r) {
        for (int c = 0; c < gmap_mat.cols; ++c) {
            submap.local_gmap[r][c] = gmap_mat.at<double>(r, c);
        }
    }

    submap.is_valid = true;
    return submap;
}

// 全ての部分地図を読み込むメイン関数
std::vector<SubMap> load_all_submaps(const std::string& root_dir) {
    std::string submaps_path = root_dir + "/submaps";
    std::vector<SubMap> submaps;

    if (!fs::exists(submaps_path) || !fs::is_directory(submaps_path)) {
        return submaps;
    }

    // ディレクトリ名をソートするために一時的なベクターに格納
    std::vector<std::string> dir_paths;
    for (const auto& entry : fs::directory_iterator(submaps_path)) {
        if (entry.is_directory()) {
            dir_paths.push_back(entry.path().string());
        }
    }
    std::sort(dir_paths.begin(), dir_paths.end());

    // ソートされたパスからサブマップを読み込む
    for (const auto& path : dir_paths) {
        std::cout << path << " を読み込み中..." << std::endl;
        SubMap submap = load_single_submap(path);
        if (submap.is_valid) {
            submaps.push_back(submap);
        }
    }

    return submaps;
}

void visualize_maps(const std::vector<SubMap>& submaps, int selected_id) {
    if (submaps.empty()) return;

    const double CSIZE = 0.05; // 解像度

    // 1. 全体地図の描画範囲を計算
    double global_min_x = std::numeric_limits<double>::max();
    double global_max_x = std::numeric_limits<double>::lowest();
    double global_min_y = std::numeric_limits<double>::max();
    double global_max_y = std::numeric_limits<double>::lowest();

    for (const auto& submap : submaps) {
        double cos_a = cos(submap.start_pose.a);
        double sin_a = sin(submap.start_pose.a);
        std::vector<Point> corners = {
            {submap.min_x, submap.min_y}, {submap.max_x, submap.min_y},
            {submap.max_x, submap.max_y}, {submap.min_x, submap.max_y}
        };
        for (const auto& corner : corners) {
            double rot_x = corner.x * cos_a - corner.y * sin_a;
            double rot_y = corner.x * sin_a + corner.y * cos_a;
            double glob_x = submap.start_pose.x + rot_x;
            double glob_y = submap.start_pose.y + rot_y;
            if (glob_x < global_min_x) global_min_x = glob_x;
            if (glob_x > global_max_x) global_max_x = glob_x;
            if (glob_y < global_min_y) global_min_y = glob_y;
            if (glob_y > global_max_y) global_max_y = glob_y;
        }
    }
    double margin = 10.0; // 10mマージン
    global_min_x -= margin; global_max_x += margin;
    global_min_y -= margin; global_max_y += margin;

    // 2. 描画用イメージを作成
    int img_w = static_cast<int>((global_max_x - global_min_x) / CSIZE);
    int img_h = static_cast<int>((global_max_y - global_min_y) / CSIZE);
    cv::Mat integrated_img = cv::Mat::zeros(img_h, img_w, CV_8UC3);
    integrated_img.setTo(BACK_BLUE);

    // 3. 各部分地図を描画
    for (const auto& submap : submaps) {
        double cos_a = cos(submap.start_pose.a);
        double sin_a = sin(submap.start_pose.a);
        for (int y = 0; y < submap.LOCAL_HEIGHT; ++y) {
            for (int x = 0; x < submap.LOCAL_WIDTH; ++x) {
                if (submap.local_gmap[y][x] > 0.4) { // 占有確率が高いグリッドのみ描画
                    double local_x = (x - submap.LOCAL_ORIGIN_X) * submap.LOCAL_CSIZE;
                    double local_y = -(y - submap.LOCAL_ORIGIN_Y) * submap.LOCAL_CSIZE;
                    double rot_x = local_x * cos_a - local_y * sin_a;
                    double rot_y = local_x * sin_a + local_y * cos_a;
                    double glob_x = submap.start_pose.x + rot_x;
                    double glob_y = submap.start_pose.y + rot_y;
                    int px = static_cast<int>((glob_x - global_min_x) / CSIZE);
                    int py = static_cast<int>((global_max_y - glob_y) / CSIZE);
                    if (px >= 0 && px < img_w && py >= 0 && py < img_h) {
                        integrated_img.at<cv::Vec3b>(py, px) = cv::Vec3b(198, 204, 203);
                    }
                }
            }
        }
    }

    // 4. 各部分地図の目印（ID付きの円）を描画
    for (const auto& submap : submaps) {
        double glob_x = submap.start_pose.x;
        double glob_y = submap.start_pose.y;
        int px = static_cast<int>((glob_x - global_min_x) / CSIZE);
        int py = static_cast<int>((global_max_y - glob_y) / CSIZE);

        cv::Scalar color = (submap.submap_id == selected_id) ? RED : BLUE_LIGHT;
        int radius = (submap.submap_id == selected_id) ? 10 : 5;

        if (px >= 0 && px < img_w && py >= 0 && py < img_h) {
            cv::circle(integrated_img, cv::Point(px, py), radius, color, -1); // 塗りつぶし
            cv::putText(integrated_img, std::to_string(submap.submap_id), cv::Point(px + 10, py + 10), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);
        }
    }

    // 5. ウィンドウに表示
    cv::imshow("Manual Loop Closure", integrated_img);
}

