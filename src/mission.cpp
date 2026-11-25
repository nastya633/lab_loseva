// mission: маршрут B -> C -> D -> A для апарата drone_0
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

struct Point { double x, y, z; };

// стан, який оновлює колбек одометрії
Point cur = {0, 0, 0};     // поточна позиція апарата
bool have_odom = false;    // чи прийшло хоч одне повідомлення
bool running = false;      // чи йде маршрут (тільки тоді рахуємо шлях)
double path_length = 0.0;  // пройдений шлях, м

double dist(const Point& a, const Point& b)
{
  return std::sqrt((a.x - b.x) * (a.x - b.x) +
                   (a.y - b.y) * (a.y - b.y) +
                   (a.z - b.z) * (a.z - b.z));
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  Point p = {msg->pose.pose.position.x,
             msg->pose.pose.position.y,
             msg->pose.pose.position.z};
  if (have_odom && running) path_length += dist(cur, p);
  cur = p;
  have_odom = true;
}

bool readPoint(ros::NodeHandle& nh, const std::string& name, Point& p)
{
  std::vector<double> v;
  if (!nh.getParam(name, v) || v.size() != 3)
  {
    ROS_ERROR("Параметр '%s' має бути списком [x, y, z]", name.c_str());
    return false;
  }
  p = {v[0], v[1], v[2]};
  return true;
}

void publishGoal(ros::Publisher& pub, const Point& g)
{
  geometry_msgs::PoseStamped goal;
  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = "world";
  goal.pose.position.x = g.x;
  goal.pose.position.y = g.y;
  goal.pose.position.z = g.z;
  goal.pose.orientation.w = 1.0;
  pub.publish(goal);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mission");
  ros::NodeHandle nh("~");

  // 1. параметри
  Point A, B, C, D;
  if (!readPoint(nh, "A", A) || !readPoint(nh, "B", B) ||
      !readPoint(nh, "C", C) || !readPoint(nh, "D", D))
    return 1;

  double radius, hold_time;
  std::string odom_topic, goal_topic, csv_path;
  nh.param("arrival_radius", radius, 0.5);
  nh.param("hold_time", hold_time, 1.0);
  nh.param<std::string>("odom_topic", odom_topic, "/drone_0_visual_slam/odom");
  nh.param<std::string>("goal_topic", goal_topic, "/move_base_simple/goal");
  nh.param<std::string>("csv_path", csv_path,
                        "/home/nastya/ego-planner-swarm/logs/mission_log.csv");

  // 2. зв'язок з планувальником
  ros::Subscriber sub = nh.subscribe(odom_topic, 1000, odomCallback);
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>(goal_topic, 1);

  // чекаємо одометрію і підписку планувальника
  ros::Rate wait(10);
  while (ros::ok() && (!have_odom || pub.getNumSubscribers() == 0))
  {
    ros::spinOnce();
    wait.sleep();
  }
  if (!ros::ok()) return 0;
  ros::Duration(1.0).sleep();

  // 3. лог
  std::ofstream csv(csv_path.c_str());
  if (!csv.is_open())
  {
    ROS_ERROR("Не вдалося відкрити %s", csv_path.c_str());
    return 1;
  }
  csv << "t,x,y,z,goal,dist\n";
  csv << std::fixed << std::setprecision(3);

  // 4. маршрут
  std::vector<Point> route = {B, C, D, A};
  const char* names[] = {"B", "C", "D", "A"};
  std::vector<double> seg_times;

  ros::Time t0 = ros::Time::now();
  ros::Time seg_start = t0;
  ros::Time hold_start = t0;
  bool holding = false;
  size_t idx = 0;

  running = true;
  publishGoal(pub, route[idx]);
  ROS_INFO("Ціль %s: (%.1f, %.1f, %.1f)", names[idx],
           route[idx].x, route[idx].y, route[idx].z);

  ros::Rate rate(10);  // 10 Гц = крок 100 мс
  while (ros::ok() && idx < route.size())
  {
    ros::spinOnce();
    ros::Time now = ros::Time::now();
    double d = dist(cur, route[idx]);

    csv << (now - t0).toSec() << "," << cur.x << "," << cur.y << ","
        << cur.z << "," << (idx + 1) << "," << d << "\n";

    if (d < radius)
    {
      if (!holding)
      {
        holding = true;
        hold_start = now;
      }
      else if ((now - hold_start).toSec() >= hold_time)
      {
        double seg = (now - seg_start).toSec();
        seg_times.push_back(seg);
        ROS_INFO("Ціль %s досягнуто за %.1f с", names[idx], seg);
        idx++;
        holding = false;
        if (idx < route.size())
        {
          publishGoal(pub, route[idx]);
          seg_start = now;
          ROS_INFO("Ціль %s: (%.1f, %.1f, %.1f)", names[idx],
                   route[idx].x, route[idx].y, route[idx].z);
        }
      }
    }
    else
    {
      holding = false;  // вилетів із зони, відлік утримання скидається
    }
    rate.sleep();
  }
  running = false;
  double total = (ros::Time::now() - t0).toSec();
  csv.close();

  // 5. підсумок
  if (idx == route.size())
    ROS_INFO("Маршрут завершено");
  else
    ROS_WARN("Маршрут перервано");
  ROS_INFO("Загальний час: %.1f с", total);
  ROS_INFO("Довжина шляху: %.1f м", path_length);
  for (size_t i = 0; i < seg_times.size(); i++)
    ROS_INFO("Відрізок до %s: %.1f с", names[i], seg_times[i]);
  return 0;
}
