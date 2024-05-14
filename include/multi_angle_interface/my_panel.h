#ifndef MY_PANEL_H
#define MY_PANEL_H

#include <rviz/panel.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

class QPushButton;
class QVBoxLayout;

namespace rviz
{
class RenderPanel;
class VisualizationManager;

class ViewPanel : public rviz::Panel
{
Q_OBJECT
public:
    ViewPanel(QWidget* parent = 0);

protected Q_SLOTS:
    void setTopView();
    void setRightSideView();
    void setLeftSideView();
    void setFrontView();
    void setBackView();

protected:
    QPushButton* top_view_button_;
    QPushButton* right_side_view_button_;
    QPushButton* left_side_view_button_;
    QPushButton* back_side_view_button_;
    QPushButton* front_side_view_button_;

private:
    void setCameraView(const geometry_msgs::Point& eye, const geometry_msgs::Point& focus, const geometry_msgs::Vector3& up);
    ros::Publisher camera_view_pub_;
    ros::NodeHandle nh_;
    
};
} // end namespace rviz

#endif // MY_PANEL_H
