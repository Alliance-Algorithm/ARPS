#include "../updater.hpp"

namespace radar {

void Updater::initialize_debug_plotter()
{
    minimap_image_ = cv::imread("./resources/images/RM2024Battlefield.png");
    labels_ = {
        { 0, "CantIdentfy" },
        { 1, "R_Hero" }, { 2, "R_Engineer" },
        { 3, "R_Solider_3" }, { 4, "R_Solider_4" }, { 5, "R_Solider_5" },
        { 7, "R_Sentry" },
        { 101, "B_Hero" }, { 102, "B_Engineer" },
        { 103, "B_Solider_3" }, { 104, "B_Solider_4" }, { 105, "B_Solider_5" },
        { 107, "B_Sentry" }
    };
    cv::namedWindow("minimap", 0);
    cv::resizeWindow("minimap", cv::Size(960, 540));
}
void Updater::plot()
{

    minimap_image_.copyTo(temp_img_);

    for (auto robot_position : radar_information_->enemy_robot_positions) {

        int car_position_x = int((robot_position.second.x / 28.0) * 1854.0);
        int car_position_y = int(((15.0 - robot_position.second.y) / 15.0) * 996.0);
        cv::Point2d car_point(car_position_x, car_position_y);
        cv::Point2d id_point(car_position_x + 30, car_position_y + 10);

        cv::Scalar color = int(robot_position.first) > 7 ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);

        cv::circle(temp_img_, car_point, 20, color, -1);
        cv::putText(temp_img_, labels_[int(robot_position.first)], id_point, 1, 2,
            cv::Scalar(255, 255, 255), 2);
    }

    cv::imshow("minimap", temp_img_);
    cv::waitKey(1);
}
}