//
// Created by goksu on 4/6/19.
//

#pragma once

#include "Triangle.hpp"
#include <algorithm>
#include <eigen3/Eigen/Eigen>
using namespace Eigen;

namespace rst {
enum class Buffers
{   // Buffers bufferType = Buffers::Color;
    Color = 1, //关联的数值在实际编程中可以用于表示某种状态、选项或者标志位
    Depth = 2
};

inline Buffers operator|(Buffers a, Buffers b)
{
    return Buffers((int)a | (int)b);
}

inline Buffers operator&(Buffers a, Buffers b)
{
    return Buffers((int)a & (int)b);
}

enum class Primitive
{
    Line,
    Triangle
};

/*
 * For the curious : The draw function takes two buffer id's as its arguments.
 * These two structs make sure that if you mix up with their orders, the
 * compiler won't compile it. Aka : Type safety
 * */
struct pos_buf_id
{
    int pos_id = 0;
};

struct ind_buf_id
{
    int ind_id = 0;
};

class rasterizer// 光栅器
{
  public:
    rasterizer(int w, int h);
    pos_buf_id load_positions(const std::vector<Eigen::Vector3f>& positions);//加载当前位置的id编号
    ind_buf_id load_indices(const std::vector<Eigen::Vector3i>& indices);//ind是像素索引

    void set_model(const Eigen::Matrix4f& m);
    void set_view(const Eigen::Matrix4f& v);
    void set_projection(const Eigen::Matrix4f& p);
    //将屏幕像素点(x, y) 设为(r, g, b) 的颜色，并写入相应的帧缓冲区frame_buf位置。
    void set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color);

    void clear(Buffers buff);

    void draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, Primitive type);

    std::vector<Eigen::Vector3f>& frame_buffer() { return frame_buf; }

  private:
    void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end);
    void rasterize_wireframe(const Triangle& t);// 三角形外边框

  private:
    Eigen::Matrix4f model;// 模型变换矩阵
    Eigen::Matrix4f view;// 视图变换矩阵
    Eigen::Matrix4f projection;// 投影矩阵

    std::map<int, std::vector<Eigen::Vector3f>> pos_buf;// Vector3f: 浮点型3维向量
    std::map<int, std::vector<Eigen::Vector3i>> ind_buf;// Vector3i: 整型3维向量

    std::vector<Eigen::Vector3f> frame_buf;//帧缓冲对象，用于存储需要在屏幕上绘制的颜色数据。
    std::vector<float> depth_buf;// 深度缓冲对象
    int get_index(int x, int y);

    int width, height;

    int next_id = 0;
    int get_next_id() { return next_id++; }
};
} // namespace rst
