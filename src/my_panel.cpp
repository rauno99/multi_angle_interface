#include <multi_angle_interface/my_panel.h>
#include <QPushButton>
#include <QVBoxLayout>
#include <view_controller_msgs/CameraPlacement.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

namespace rviz
{

ViewPanel::ViewPanel(QWidget* parent) : rviz::Panel(parent)
{
    QVBoxLayout* layout = new QVBoxLayout;
    QGridLayout* grid_layout = new QGridLayout;

    top_view_button_ = new QPushButton("Bird's-eye view");
    right_side_view_button_ = new QPushButton("Right side");
    left_side_view_button_ = new QPushButton("Left side");
    front_side_view_button_ = new QPushButton("Front side");
    back_side_view_button_ = new QPushButton("Back side");

    grid_layout->addWidget(front_side_view_button_, 0, 1);
    grid_layout->addWidget(left_side_view_button_, 1, 0);
    grid_layout->addWidget(top_view_button_, 1, 1);
    grid_layout->addWidget(right_side_view_button_, 1, 2);
    grid_layout->addWidget(back_side_view_button_, 2, 1);

    connect(top_view_button_, SIGNAL(clicked()), this, SLOT(setTopView()));
    connect(right_side_view_button_, SIGNAL(clicked()), this, SLOT(setRightSideView()));
    connect(left_side_view_button_, SIGNAL(clicked()), this, SLOT(setLeftSideView()));
    connect(front_side_view_button_, SIGNAL(clicked()), this, SLOT(setFrontView()));
    connect(back_side_view_button_, SIGNAL(clicked()), this, SLOT(setBackView()));

    layout->addLayout(grid_layout);
    setLayout(layout);
    camera_view_pub_ = nh_.advertise<view_controller_msgs::CameraPlacement>("rviz/camera_placement", 10);
}


void ViewPanel::setCameraView(const geometry_msgs::Point& eye,
                            const geometry_msgs::Point& focus,
                            const geometry_msgs::Vector3& up)
{
    view_controller_msgs::CameraPlacement cp;
    cp.target_frame = "base_link";

    cp.eye.point = eye;
    cp.eye.header.frame_id = "base_link";

    cp.focus.point = focus;
    cp.focus.header.frame_id = "base_link";

    cp.up.vector = up;
    cp.up.header.frame_id = "base_link";

    cp.time_from_start = ros::Duration(0.5);
    camera_view_pub_.publish(cp);
    ros::Duration(0.2).sleep();
    camera_view_pub_.publish(cp);
    ros::Duration(0.2).sleep();
}

void ViewPanel::setFrontView()
{
   geometry_msgs::Point eye;
    eye.x = -6.5;
    eye.y = -0.07;
    eye.z = 3.6;

    geometry_msgs::Point focus;
    focus.x = -2.2;
    focus.y = -0.1;
    focus.z = 1.5;

    geometry_msgs::Vector3 up;
    up.x = 0;
    up.y = 0;
    up.z = 1;

    setCameraView(eye, focus, up);

}

void ViewPanel::setBackView()
{
   geometry_msgs::Point eye;
    eye.x = 1.8;
    eye.y = -0.06;
    eye.z = 6.6;

    geometry_msgs::Point focus;
    focus.x = -2.2;
    focus.y = -0.1;
    focus.z = 1.5;

    geometry_msgs::Vector3 up;
    up.x = 0;
    up.y = 0;
    up.z = 1;

    setCameraView(eye, focus, up);

}

void ViewPanel::setTopView()
{
   geometry_msgs::Point eye;
    eye.x = -0.1;
    eye.y = 0;
    eye.z = 11;

    geometry_msgs::Point focus;
    focus.x = 0;
    focus.y = 0;
    focus.z = 5;

    geometry_msgs::Vector3 up;
    up.x = 0;
    up.y = 0;
    up.z = 1;

    setCameraView(eye, focus, up);

}

void ViewPanel::setLeftSideView()
{
    geometry_msgs::Point eye;
    eye.x = -0.24;
    eye.y = -5;
    eye.z = 5;

    geometry_msgs::Point focus;
    focus.x = -0.3;
    focus.y = 0.52;
    focus.z = 1.2;

    geometry_msgs::Vector3 up;
    up.x = 0;
    up.y = 0;
    up.z = 1;

    setCameraView(eye, focus, up);
}

void ViewPanel::setRightSideView()
{
    geometry_msgs::Point eye;
    eye.x = -0.34;
    eye.y = 5;
    eye.z = 5;

    geometry_msgs::Point focus;
    focus.x = -0.2;
    focus.y = 1.8;
    focus.z = 2.7;

    geometry_msgs::Vector3 up;
    up.x = 0;
    up.y = 0;
    up.z = 1;

    setCameraView(eye, focus, up);
}

} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::ViewPanel, rviz::Panel)
