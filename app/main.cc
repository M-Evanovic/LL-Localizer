#include <gflags/gflags.h>
#include <unistd.h>
#include <csignal>

#include "localizer.h"

DEFINE_string(traj_log_file, "./Log/traj.txt", "path to traj log file");
void SigHandle(int sig) {
    ll_localizer::options::FLAG_EXIT = true;
    ROS_WARN("catch sig %d", sig);
}

int main(int argc, char **argv) {
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "ll_localizer");
    ros::NodeHandle nh;

    auto ll_localizer = std::make_shared<ll_localizer::Localizer>();
    ll_localizer->InitROS(nh);

    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);

    while (ros::ok()) {
        if (ll_localizer::options::FLAG_EXIT) {
            break;
        }
        ros::spinOnce();
        ll_localizer->Run();
        rate.sleep();
    }

    LOG(INFO) << "finishing mapping";
    ll_localizer->Finish();

    ll_localizer::Timer::PrintAll();
    LOG(INFO) << "save trajectory to: " << FLAGS_traj_log_file;
    ll_localizer->Savetrajectory(FLAGS_traj_log_file);

    return 0;
}
