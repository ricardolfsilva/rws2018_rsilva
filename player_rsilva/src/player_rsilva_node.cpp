#include <iostream>
#include <vector>

// Boost includes
#include <boost/shared_ptr.hpp>

// Ros includes
#include <ros/ros.h>
#include <rws2018_libs/team.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>  //Messages on sreen

#include <rws2018_msgs/MakeAPlay.h>

#define DEFAULT_TIME 0.05

using namespace ros;

using namespace std;

namespace rws_rsilva
{
class Player
{
public:
  // Constructor with the same name as the class
  Player(string name)
  {
    this->name = name;
  }

  int setTeamName(int team_index = 0 /*default value*/)
  {
    switch (team_index)
    {
      case 0:
        return setTeamName("red");
        break;
      case 1:
        return setTeamName("green");
        break;
      case 2:
        return setTeamName("blue");
        break;
      default:
        // cout << "wrong team index given. Cannot set team" << endl;
        ROS_ERROR("Wrong team index given. Cannot set team.");
        break;
    }
  }

  // Set team name, if given a correct team name (accessor)
  int setTeamName(string team)
  {
    if (team == "red" || team == "green" || team == "blue")
    {
      this->team = team;
      return 1;
    }
    else
    {
      this->team = "no team";
      // cout << "cannot set team name to " << team << endl;
      ROS_ERROR("Cannot set team name to %s.", team.c_str());
      return 0;
    }
  }

  // Gets team name (accessor)
  string getTeam(void)
  {
    return team;
  }

  string name;  // A public atribute

private:
  string team;
};

class MyPlayer : public Player
{
public:
  boost::shared_ptr<Team> red_team;
  boost::shared_ptr<Team> green_team;
  boost::shared_ptr<Team> blue_team;
  boost::shared_ptr<Team> my_team;
  boost::shared_ptr<Team> my_preys;
  boost::shared_ptr<Team> my_hunters;

  tf::TransformBroadcaster br;  // declare the broadcaster
  ros::NodeHandle n;
  boost::shared_ptr<ros::Subscriber> sub;  // declare the subscriver
  tf::Transform transform;                 // declare the transformation object (player's pose wrt world)
  boost::shared_ptr<ros::Publisher> pub;   // declare the publisher
  tf::TransformListener listener;          // declare listener

  MyPlayer(string argin_name, string argin_team) : Player(argin_name)
  {
    red_team = boost::shared_ptr<Team>(new Team("red"));
    green_team = boost::shared_ptr<Team>(new Team("green"));
    blue_team = boost::shared_ptr<Team>(new Team("blue"));

    if (red_team->playerBelongsToTeam(argin_name))
    {
      my_team = red_team;
      my_preys = green_team;
      my_hunters = blue_team;
      setTeamName("red");
    }
    if (green_team->playerBelongsToTeam(argin_name))
    {
      my_team = green_team;
      my_preys = blue_team;
      my_hunters = red_team;
      setTeamName("green");
    }
    if (blue_team->playerBelongsToTeam(argin_name))
    {
      my_team = blue_team;
      my_preys = red_team;
      my_hunters = green_team;
      setTeamName("blue");
    }

    setTeamName(argin_team);

    // Message subscriver
    sub = boost::shared_ptr<ros::Subscriber>(new ros::Subscriber());
    *sub = n.subscribe("/make_a_play", 100, &MyPlayer::move, this);

    // Message publisher
    pub = boost::shared_ptr<ros::Publisher>(new ros::Publisher());
    *pub = n.advertise<visualization_msgs::Marker>("/bocas", 0);

    struct timeval t1;
    gettimeofday(&t1, NULL);
    srand(t1.tv_usec);  // set the initial seed value
    double start_x = ((double)rand() / (double)RAND_MAX) * 10 - 5;
    double start_y = ((double)rand() / (double)RAND_MAX) * 10 - 5;
    printf("start_x=%f, start_y=%f\n", start_x, start_y);
    warp(start_x, start_y, M_PI / 2);

    PrintReport();
  }

  void warp(double x, double y, double alfa)
  {
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, alfa);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "rsilva"));
    ROS_INFO("Warping to x=%f y=%f a=%f", x, y, alfa);
  }

  // angle
  double getAngleToPLayer(string other_player, double time_to_wait = DEFAULT_TIME)
  {
    tf::StampedTransform t;  // The transform object
    // Time now = Time::now(); //get the time
    ros::Time now = Time(0);  // get the latest transform received

    try
    {
      listener.waitForTransform("rsilva", other_player, now, Duration(time_to_wait));
      listener.lookupTransform("rsilva", other_player, now, t);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
      return NAN;
    }

    return atan2(t.getOrigin().y(), t.getOrigin().x());
  }

  // distance
  float getDistanceToPlayer(string player_name, float time_to_wait = 0.1)
  {
    tf::StampedTransform trans;
    ros::Time now = Time(0);  // get the latest transform received

    try
    {
      listener.waitForTransform(name, player_name, now, Duration(time_to_wait));
      listener.lookupTransform(name, player_name, now, trans);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.01).sleep();
      return 10000;
    }

    float x = trans.getOrigin().x();
    float y = trans.getOrigin().y();
    return sqrt(x * x + y * y);
  }

  void move(const rws2018_msgs::MakeAPlay::ConstPtr& msg)
  {
    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    double a = 0;

    //----------- AI part ---------//
    double min_distance = 99999;
    string player_to_hunt = "no player";
    for (size_t i = 0; i < my_preys->player_names.size(); i++)
    {
      double dist = getDistanceToPlayer(my_preys->player_names[i]);
      if (isnan(dist))
      {
      }
      else if (dist < min_distance)
      {
        min_distance = dist;
        player_to_hunt = my_preys->player_names[i];
      }
    }
    double displacement = 1;  // max velocity for now

    double delta_alpha = getAngleToPLayer(player_to_hunt);
    if (isnan(delta_alpha))
      delta_alpha = 0;

    //----------- /BOCAS part ---------//
    visualization_msgs::Marker marker;
    marker.header.frame_id = "rsilva";  // referencial name
    marker.header.stamp = ros::Time();
    marker.ns = "rsilva";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.3;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.text = "K20 " + player_to_hunt;
    marker.lifetime = ros::Duration(2);
    pub->publish(marker);

    //----------- CONSTRAINS part ---------//
    double displacement_max = msg->dog;
    double displacement_with_constrains;
    displacement > displacement_max ? displacement = displacement_max : displacement = displacement;

    double delta_alpha_max = M_PI / 30;
    fabs(delta_alpha) > fabs(delta_alpha_max) ? delta_alpha = delta_alpha_max * delta_alpha / fabs(delta_alpha) :
                                                delta_alpha = delta_alpha;

    tf::Transform my_move_T;
    my_move_T.setOrigin(tf::Vector3(displacement, 0.0, 0.0));
    tf::Quaternion q1;
    q1.setRPY(0, 0, delta_alpha);
    my_move_T.setRotation(q1);

    transform = transform * my_move_T;
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "rsilva"));  // constroi e envia a mensagem
    ROS_INFO("My name is %s and I am moving.", name.c_str());
    // ROS_WARN("My name is %s and I am moving.", name.c_str());  //warnings
    // ROS_ERROR("My name is %s and I am moving.", name.c_str()); //errors
    // Message
    ROS_INFO("Moving to ");
  }

  void PrintReport()
  {
    // cout << "My name is " << name << " and my team is " << getTeam() << endl;
    ROS_INFO("My name is %s and my team is %s.", name.c_str(), getTeam().c_str());
  }
};
}

int main(int argc, char** argv)
{
  // Creating an instance of class Player
  ros::init(argc, argv, "rsilva");

  ros::NodeHandle n;

  rws_rsilva::MyPlayer my_player("rsilva", "green");

  if (my_player.red_team->playerBelongsToTeam("rsilva"))
  {
    // cout << "o ricardo esta na equipa certa" << endl;
    ROS_INFO("Ricardo is in the correct team.");
  };

  /*ros::Rate loop_rate(10);
  while (ros::ok())
  {
    my_player.move();
    ros::spinOnce();
    loop_rate.sleep();
  }*/
  ros::spin();
}
