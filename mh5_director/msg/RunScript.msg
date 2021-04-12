# name of script to run in format <portfolio_name.script_name>
string      script

# the callback will wait for the script to finish and report (LOG)
bool        wait

# the callback will register a FollowJointTrajectoryFeedback and report 
bool        feedback

# play speed factor; will default 1.0
float32     playback_speed
