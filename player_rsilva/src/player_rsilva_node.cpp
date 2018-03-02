#include <boost/shared_ptr.hpp>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"

#include <rws2018_libs/team.h>
#include <sstream>

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
        ROS_WARN("Wrong team index given. Cannot set team.");
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
      ROS_WARN("Cannot set team name to %s.", team.c_str());
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

    PrintReport();
  }

  void move()
  {
    static tf::TransformBroadcaster br;             // cria transform_broadcaster, permite o envio
    tf::Transform transform;                        // cria transformacao
    transform.setOrigin(tf::Vector3(-4, -4, 0.0));  // edita posicao
    tf::Quaternion q;
    q.setRPY(0, 0, M_PI / 4);  // edita rotacao
    transform.setRotation(q);
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "rsilva"));  // constroi e envia a mensagem
    ROS_INFO("My name is %s and I am moving.", name.c_str());
    // ROS_WARN("My name is %s and I am moving.", name.c_str());  //warnings
    // ROS_ERROR("My name is %s and I am moving.", name.c_str()); //errors
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
    ROS_INFO("O Ricardo está na equipa certa.");
  };

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    my_player.move();
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();
}
