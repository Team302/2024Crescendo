$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

// C++ Includes
#include <any>

// Team 302 Includes
#include <RobotDefinitions.h>

/*
    Will need to include all mechanism and sensor h files
    Will also need to include their builders
*/
$$_INCLUDES_$$

/*
    Here's what this should look like once generated: (teamNumber is passed in from BuildDetailsReader where GetRobotDefinition is called)

    switch(teamNumber){
        case $$_ROBOT_ID_$$:
            return Get$$_ROBOT_ID_$$Definition();  //separate functions will make the code more readable during review
        case $$_ROBOT_ID_$$:
            return Get$$_ROBOT_ID_$$Definition();
        default:
            return Get302Defition();  //this could return comp bot or simulation bot
    }
*/
static RobotDefinition *RobotDefinitions::GetRobotDefinition(int teamNumber){
    $$_ROBOT_DEFINITION_SWITCH_$$}

/*
    This is where all of the functions in the switch statement will be created
    Here's what one should look like once generated:

    RobotDefinition* Get302Definition()
    {
        /// NOTE switching to be one vector with type std::any for now, later on may go back to separate vectors if needed
        //std::vector<std::pair<RobotDefinitions::Components, Mechanism>> mechs = new std::vector<Mechanism>();
        //std::vector<std::pair<RobotDefinitions::Components, Sensor>> sensors = new std::vector<Sensor>();
        std::vector<std::pair<RobotDefinitions::Components, std::any>> components = new std::vector<std::pair<RobotDefinitions::Components, std::any>>();

        Mechanism intake = IntakeBuilder::GetBuilder()->CreateNewIntake(args); //or however the builders will be called to create mechs
        components.emplace_back(std::make_pair(RobotDefinitions::Components::Intake, intake));

        Mechanism shooter = ShooterBuilder::GetBuilder()->CreateNewShooter(args);
        components.emplace_back(std::make_pair(RobotDefinitions::Components::Shooter, shooter));

        Sensor intakeSensor = new BannerSensor(port);
        components.emplace_back(std::make_pair(RobotDefinitions::Components::IntakeSensor, intakeSensor));

        return new RobotDefinition(components);
    }
*/
$$_ROBOT_VARIANT_CREATION_FUNCTIONS_$$