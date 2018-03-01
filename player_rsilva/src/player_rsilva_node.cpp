#include <iostream>

class Player
{
    public:

    //Constructor with the same name as the class
    Player(std::string name) { this->name = name; }
    
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
                std::cout << "wrong team index given. Cannot set team" << std::endl;
                break;
        }
     }                                                                                                                                                      //Set team name, if given a correct team name (accessor)
    int setTeamName(std::string team)
    {
        if (team =="red" || team =="green" || team =="blue")
        {
            this->team = team;
            return 1;
        }
        else                                                                        {
            std::cout << "cannot set team name to " << team << std::endl;
            return 0;
        }
    }
    
    //Gets team name (acessor)
    std::string getTeamName(void) {return team;}

    std::string name; //A public atribute

    private:
    std::string team;
};



int main()
{    
    //Creating an instance of class Player
    Player player("rsilva");
    player.setTeamName("red");
    player.setTeamName(2);
    std::cout << "player.name is" << player.name << std::endl;
    std::cout << "team is" << player.getTeamName() << std::endl;                                                      
}
