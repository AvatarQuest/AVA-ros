#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>

namespace action_ns {

    class MyPositionController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
        // public:
            // MyPositionC
    };

}

int main() {
    return 0;
}
