#include <vector>
#include <geometry_msgs/Pose.h>

#ifndef GOAL_GENERATOR_H
#define GOAL_GENERATOR_H

class GoalGenerator 
{
private:
    std::vector<geometry_msgs::Pose> poses_;
    uint32_t num_goals_requested; 
public:
    GoalGenerator(): num_goals_requested(0) {}
    ~GoalGenerator() {}
    bool readFromYaml(const std::string &fileName);
    void setNextGoal(move_base_msgs::MoveBaseGoal & goal);
    void setNextGoalCircular(move_base_msgs::MoveBaseGoal & goal);
};



#endif