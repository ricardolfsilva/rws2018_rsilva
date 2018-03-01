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
        cout << "wrong team index given. Cannot set team" << endl;
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
      cout << "cannot set team name to " << team << endl;
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

  MyPlayer(string argin_name, string argin_team) : Player(argin_name)
  {
    red_team = boost::shared_ptr<Team>(new Team("red"));
    green_team = boost::shared_ptr<Team>(new Team("green"));
    blue_team = boost::shared_ptr<Team>(new Team("blue"));

    setTeamName(argin_team);

    PrintReport();
  }
  
  void move()
  {
  	    static tf::TransformBroadcaster br; //cria transform_broadcaster, permite o envio
    	tf::Transform transform; //cria transformacao
    	transform.setOrigin( tf::Vector3(4, 4, 0.0)  ); //edita posicao
    	tf::Quaternion q;
    	q.setRPY(0, 0, M_PI/4); //edita rotacao
    	transform.setRotation(q);   
    	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "rsilva")); //constroi e envia a mensagem
  }

  void PrintReport()
  {
    cout << "My name is " << name << " and my team is " << getTeam() << endl;
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
    cout << "o ricardo esta na equipa certa" << endl;
  };

  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    my_player.move();
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();
}
