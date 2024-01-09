#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

//cv::Point2f  float类型的二维点坐标
cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if(control_points.size() == 1) return control_points[0];
    std::vector<cv::Point2f> next_layer_control_points;
    for(int i = 0; i < control_points.size() - 1; i++){
        cv::Point2f p0 = control_points[i];
        cv::Point2f p1 = control_points[i+1];
        cv::Point2f p2 = p0 + t * (p1 - p0);
        next_layer_control_points.push_back(p2);
    }
    return recursive_bezier(next_layer_control_points, t);

}
double get_dist(cv::Point2f point1, cv::Point2f point2){//计算两点的哈夫曼距离
    return fabs(point1.x - point2.x) + fabs(point1.y - point2.y);
}

//cv::Mat &window：表示屏幕矩阵;矩阵内元素为CV_8UC3类型(无符号8位整数，RGB三通道，cv::Vec3b)
void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for(double t = 0.0; t < 1.0; t += 0.001){
        cv::Point2f point = recursive_bezier(control_points, t);
        cv::Point2f point0( std::min(floor(point.x), ceil(point.x)), std::min(floor(point.y), ceil(point.y)) );
        double dist;
        std::vector<double> bias{0.5, -0.5};
        for(int i = 0; i < 4; i++){
            cv::Point2f centerPoint(point0.x + bias[i % 2], point0.y + bias[i % 2]);//计算中心点
            dist = get_dist(point, centerPoint);
            window.at<cv::Vec3b>(centerPoint.y, centerPoint.x)[1] = 255 * (3 - dist) / 3;
        } 
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            naive_bezier(control_points, window);
            //   bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
