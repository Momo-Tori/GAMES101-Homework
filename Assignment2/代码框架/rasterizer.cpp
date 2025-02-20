// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include <math.h>

constexpr double MY_PI = 3.1415926;

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}



// inline float times(Vector3f const & v1,Vector3f const & v2)
// {
//     return v1.x()*v2.y()-v2.x()*v1.y();
// }


//叉乘判断
// static bool insideTriangle(float x, float y, const Vector4f* _vi)
// {   
//     Vector3f v[3],P;
//     P << x,y,1;
//     for (int i=0;i<3;i++)
//     {
//         v[i]<<_vi[i].x()/_vi[i].z(),_vi[i].y()/_vi[i].z(),1;
//     }
//     float
//     z11=times(P-v[0],v[1]-v[0]),z12=times(v[2]-v[0],v[1]-v[0]),
//     z21=times(P-v[1],v[2]-v[1]),z22=times(v[0]-v[1],v[2]-v[1]),
//     z31=times(P-v[2],v[0]-v[2]),z32=times(v[1]-v[2],v[0]-v[2]);
//     return z11*z12>=0
//         && z21*z22>=0
//         && z31*z32>=0;
// }

//利用三个矢量夹角之和为360度，经过测试速度基本相同，该算法较快
static bool insideTriangle(float x, float y, const Vector4f* _vi)
{   
    Vector3f _v[3];
    for (int i=0;i<3;i++)
    {
        _v[i]<<_vi[i].x()-x,_vi[i].y()-y,0;
        if(_v[i].norm()==0)
            return true;
    }
    double theta1=acos(_v[0].dot(_v[1]) /(_v[0].norm()*_v[1].norm()));
    double theta2=acos(_v[0].dot(_v[2]) /(_v[0].norm()*_v[2].norm()));
    double theta3=acos(_v[2].dot(_v[1]) /(_v[2].norm()*_v[1].norm()));
    return fabs(theta1+theta2+theta3-2*MY_PI)<1e-4;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}


template<class a>
inline const a& max(const a& l,const a& r)
{
    return (l>r)?l:r;
}

template<class a>
inline const a& max(const a& l,const a& m,const a& r,const a& rr)
{
    return max(max(l,m),max(r,rr));
}


template<class a>
inline const a& min(const a& l,const a& r)
{
    return (l<r)?l:r;
}

template<class a>
inline const a& min(const a& l,const a& m,const a& r,const a& rr)
{
    return min(min(l,m),min(r,rr));
}


void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

void rst::rasterizer::set(int x,int y,const Triangle& t,bool judge)
{
    auto v = t.toVector4();

    for(int i=0;i<4;i++)
    {
        float xf=x+0.25+0.5*(i&1);
        float yf=y+0.25+0.5*((i&2)>>1);
        if(judge||insideTriangle(xf,yf,v.data()))
        {
            auto[alpha, beta, gamma] = computeBarycentric2D(xf, yf, v.data());
            float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            z_interpolated *= w_reciprocal;
            int position=(height-1-y)*width*4+x*4+i;
            auto& z_b=z_buf[position];
            if(z_interpolated<z_b)
            {
                z_b = z_interpolated;
                Eigen::Vector3f line_color = t.getColor();
                color_buf[position]=line_color;
            }
        }
    }
    Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
    int position=(height-1-y)*width*4+x*4;
    Eigen::Vector3f line_color = (color_buf[position]
                                +color_buf[position+1]
                                +color_buf[position+2]
                                +color_buf[position+3])/4;
    set_pixel(point,line_color);

}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    // iterate through the pixel and find if the current pixel is inside the triangle

//SSAA实现

    Eigen::Vector4f left=v[0],middle=v[1],right=v[2];

    if(left.x()>middle.x())
        left.swap(middle);
    if(middle.x()>right.x())
        middle.swap(right);
    if(left.x()>middle.x())
        left.swap(middle);

    float kx1=(right.y()-left.y())/(right.x()-left.x());

    float kx2=std::numeric_limits<float>::max();
    if(middle.x()-left.x()!=0)
        kx2=(middle.y()-left.y())/(middle.x()-left.x());
    for(int x=left.x();x<middle.x();x++)
    {
        int y1=left.y()+kx1*(x-left.x()),y2=left.y()+kx2*(x-left.x());
        int i=min(y1,y2)-1;
        int end=max(y1,y2)-1;
        set(x,i,t,false);
        i++;
        set(x,i,t,false);
        i++;
        for (;i<end;i++)
        {
            set(x,i,t,true);
        }
        set(x,i,t,false);
        i++;
        set(x,i,t,false);
        i++;
        set(x,i,t,false);
    } 

    float kx3=std::numeric_limits<float>::max();
    if(middle.x()-right.x()!=0)
        kx3=(middle.y()-right.y())/(middle.x()-right.x());
    for(int x=middle.x();x<=right.x();x++)
    {
        int y1=right.y()+kx1*(x-right.x()),y2=right.y()+kx3*(x-right.x());
        int i=min(y1,y2)-1;
        int end=max(y1,y2)-1;
        set(x,i,t,false);
        i++;
        set(x,i,t,false);
        i++;
        for (;i<end;i++)
        {
            set(x,i,t,true);
        }
        set(x,i,t,false);
        i++;
        set(x,i,t,false);
        i++;
        set(x,i,t,false);
    } 

//一般的实现
/*     for (int x=xMin;x<=xMax;x++)
    {
        bool begin=false;
        for (int y=yMin;y<=yMax;y++)
        {
            if(insideTriangle(x+0.5,y+0.5,v.data()))
            {
                begin=true;
                // If so, use the following code to get the interpolated z value.
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, v.data());
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                auto& z_b=depth_buf[(height-1-y)*width + x];
                if(z_interpolated<z_b)
                {
                    z_b=z_interpolated;
                    Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
                    Eigen::Vector3f line_color = t.getColor();
                    set_pixel(point,line_color);
                }
            }
            else if(begin==true)
                break;
        }
    } */
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        color_buf=std::vector<Eigen::Vector3f>(width*height*4,Eigen::Vector3f(0,0,0));
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        z_buf=std::vector<float>(width*height*4,std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);

    z_buf.resize(w * h*4);
    color_buf.resize(w * h*4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on