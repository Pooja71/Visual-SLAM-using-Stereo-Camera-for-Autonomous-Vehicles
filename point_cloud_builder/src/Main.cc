#include "Rendering.h"

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"pcb_listening");
    ros::start();
    Rendering ren = Rendering();
    ren.run();
    ros::shutdown();
    return 0;
}
