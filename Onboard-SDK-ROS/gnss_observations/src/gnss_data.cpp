
#include <gnss_data.h>

// main process
int main (int argc, char** argv)
{
    ros::init(argc, argv, "gnss_data");
    ros::NodeHandle nh;
    struct sigaction sigIntHandler;

    // create shutdown interrupt
    sigIntHandler.sa_handler = shutdown_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    // init gnss_data class 
    gnss_data = new gnss_data_t(nh);

    // init skdy service
    // skdy_CorrDataHndr_Initialization("DC1611231047000144", "xJXgTjqfo7");
    // skdy_CorrDataHndr_Initialization("DC1620268962000910", "yc39HOPIuQ");

    // spin
    ros::Rate loop_rate(0.2);
    while(ros::ok()){

        ros::spinOnce();

        // skdy_CorrDataHndr_ProcessPppRtk(current_position, &skdy_CorrData);

        loop_rate.sleep();

    }

    return 0;
}

/* handler for ctrl+c -------------------------------------------------------*/
void shutdown_handler(int sig)
{
    delete gnss_data;
    kill(0, SIGTERM);
    exit(0);
}

/* gnss data class ----------------------------------------------------------*/
gnss_data_t::gnss_data_t(ros::NodeHandle& nh) 
{
    // parameters 
    string ws_path, opt_file;
    // nh.param<string>("gnss_observations/ws_path", ws_path, "/home/ubuntu/catkin_ws/src/gnss_observations");
    // nh.param<string>("gnss_observations/opt_file", opt_file, 
    //                                         "/home/ubuntu/catkin_ws/src/gnss_observations/config/rtkrcv.conf");
    // nh.param<bool>("gnss_observations/post_proc", post_proc, false);
    ws_path = "/home/gnc/catkin_ws/src/Onboard-SDK-ROS/gnss_observations";
    opt_file = "/home/gnc/catkin_ws/src/Onboard-SDK-ROS/gnss_observations/config/rtkrcv.conf";
    post_proc = false;

    // publishers and subscribers
    obs_pub = nh.advertise<gnss_observations::EpochData>("obs", 1);
    rtk_pub = nh.advertise<sensor_msgs::NavSatFix>("rtk", 1);
    if (!post_proc) {
        for (int i = 0; i < 3; i++) {
            char str_str[10];
            sprintf(str_str, "stream/%1d", i);
            stream_pub.push_back(nh.advertise<std_msgs::String>(str_str, 1));
        }
    }
    else {
        for (int i = 0; i < 3; i++) {
            char str_str[10];
            sprintf(str_str, "stream/%1d", i);
            switch (i) {
                case 0: stream_sub.push_back(nh.subscribe(str_str, 10, stream_callback(0))); break;
                case 1: stream_sub.push_back(nh.subscribe(str_str, 10, stream_callback(1))); break;
                case 2: stream_sub.push_back(nh.subscribe(str_str, 10, stream_callback(2)));
            } 
        }
    }
    
    // start rtklib server for local processing 
    rtkrcv_start(ws_path.data(), opt_file.data());

}

gnss_data_t::~gnss_data_t() 
{
    // shutdown rtklib servers
    rtkrcv_stop();
}


// publish gnss stream
void gnss_stream_publish(const unsigned char *in, int len, int channel)
{
    std_msgs::String str;
    for (int i = 0; i < len; i++) {
        str.data.append(1,1);
        str.data[i] = *(in+i);
    }

    stream_pub[channel].publish(str);
}

// subscribe gnss stream 
void gnss_stream_subscribe(unsigned char *in, int* len, int channel)
{
    for (int i = 0; i < nstream[channel]; i++) {
            *(in+i) = stream[channel][i];
    }
    *len = nstream[channel];
    stream_updated[channel] = false;
}

// options
int is_post_proc(void)
{
    return post_proc;
}

// data availiable 
int is_data_avail(int channel)
{
    return stream_updated[channel];
}

// getRosTime
extern void getRosTime(gtime_t* ep)
{
    ros::Time tm = ros::Time::now();
    ep->time = tm.sec;
    ep->sec = (double)tm.nsec * 1.0e-9;
}

// publish raw GNSS data
extern void gnss_data_publish(const obsd_t *obs, int n, 
      const nav_t *nav, const prcopt_t *opt, const sol_t *sol, const ssat_t *ssat)
{
    return;
}

// publish RTKLIB solutions
extern void rtksol_publish(sol_t *sol)
{
    sensor_msgs::NavSatFix rtksol;
    gtime_t utct = gpst2utc(sol->time);
    double pos[3];
    int solq;
    static const int solq_nmea[]={  /* nmea quality flags to rtklib sol quality */
        /* nmea 0183 v.2.3 quality flags: */
        /*  0=invalid, 1=gps fix (sps), 2=dgps fix, 3=pps fix, 4=rtk, 5=float rtk */
        /*  6=estimated (dead reckoning), 7=manual input, 8=simulation */
        
        SOLQ_NONE ,SOLQ_SINGLE, SOLQ_DGPS, SOLQ_PPP , SOLQ_FIX,
        SOLQ_FLOAT,SOLQ_DR    , SOLQ_NONE, SOLQ_NONE, SOLQ_NONE
    };

    ecef2pos(sol->rr, pos);
    // rtksol.header.stamp.sec = utct.time;
    // rtksol.header.stamp.nsec = (time_t)(utct.sec*1e9);
    rtksol.header.stamp = ros::Time::now();
    for (solq=0;solq<8;solq++) if (solq_nmea[solq]==sol->stat) break;
    if (solq>=8) solq=0;
    rtksol.status.status = solq;
    rtksol.status.service = 45;
    rtksol.latitude = pos[0] * R2D;
    rtksol.longitude = pos[1] * R2D;
    double h = geoidh(pos);
    rtksol.altitude = pos[2] - h;
    rtksol.position_covariance[0] = sol->rr[3]; // use this message to store velocity
    rtksol.position_covariance[1] = sol->rr[4]; // (temporally use only)
    rtksol.position_covariance[2] = sol->rr[5];

    rtk_pub.publish(rtksol);
}
