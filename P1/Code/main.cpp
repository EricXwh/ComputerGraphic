#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 
                0, 1, 0, -eye_pos[1], 
                0, 0, 1,-eye_pos[2], 
                0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    double rad = rotation_angle / 180 * MY_PI;
    Eigen::Matrix4f rotation;
    rotation << std::cos(rad), -std::sin(rad), 0, 0,
                std::sin(rad), std::cos(rad), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
    model = rotation * model;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    float rad = eye_fov / 180 * MY_PI;
    float t = zNear * std::tan(rad/2);
    float r = aspect_ratio * t;
    float l = -r;
    float b = -t;
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f perspective;
    perspective << zNear, 0, 0, 0,
                    0, zNear, 0, 0,
                    0, 0, zNear + zFar, -zNear * zFar,
                    0, 0, 1, 0;
    Eigen::Matrix4f orthographic_trans;
    orthographic_trans << 1, 0, 0, -(r + l)/ 2,
                        0, 1, 0, -(t + b)/ 2,
                        0, 0, 1, -(zNear + zFar)/ 2,
                        0, 0, 0, 1;
    Eigen::Matrix4f orthographic_scale;
    orthographic_scale << 2 /(r - l), 0, 0, 0,
                        0, 2 /(t - b), 0, 0,
                        0, 0, 2 /(zNear - zFar), 0,
                        0, 0, 0, 1;
    Eigen::Matrix4f Mt;
    Mt << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1;
    perspective = perspective * Mt;
    projection = orthographic_scale * orthographic_trans * perspective * projection;
    return projection;
}

//Using Rodrigues' Rotation Formula
Eigen::Matrix4f get_rotation(Vector3f axis, float angle){
    axis.normalize();
    float theta = angle / 180 * MY_PI;
    Eigen::Matrix3f n;
    n << 0, -axis[2], axis[1],
        axis[2], 0, -axis[0],
        -axis[1], axis[0], 0;

    Eigen::Matrix3f rotatoinMatrix;
    rotatoinMatrix << std::cos(theta) * Eigen::Matrix3f::Identity() + 
                        (1 - std::cos(theta))* axis * axis.transpose() + 
                        std::sin(theta) * n;
    Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();
    transformMatrix.block<3,3>(0,0) = rotatoinMatrix;
    return transformMatrix;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";


    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }


    rst::rasterizer r(700, 700);


    Eigen::Vector3f eye_pos = {0, 0, 5};


    // Edit begin
    Eigen::Vector3f rotate_axis = {1,1,0};
    // Edit end


    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};


    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};


    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);


    int key = 0;
    int frame_count = 0;


    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);


        // Edit begin
        //围绕z轴旋转
        //r.set_model(get_model_matrix(angle));
        //围绕任意轴旋转
        r.set_model(get_rotation(rotate_axis,angle));
        // Edit end


        r.set_view(get_view_matrix(eye_pos));
        //注意这里写入的zNear和zFar是正数，代表着距离，但课程上推导的透视矩阵是坐标，且假定是朝向z负半轴的，所以透视矩阵是需要取反的
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));


        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);


        cv::imwrite(filename, image);


        return 0;
    }


    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);


        // Edit begin
        //围绕z轴旋转
        //r.set_model(get_model_matrix(angle));
        //围绕任意轴旋转
        r.set_model(get_rotation(rotate_axis,angle));
        // Edit end


        r.set_view(get_view_matrix(eye_pos));
        //注意这里写入的zNear和zFar是正数，代表着距离，但课程上推导的透视矩阵是坐标，且假定是朝向z负半轴的，所以透视矩阵是需要取反的
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));


        r.draw(pos_id, ind_id, rst::Primitive::Triangle);


        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);


        std::cout << "frame count: " << frame_count++ << '\n';


        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }


    return 0;
}



