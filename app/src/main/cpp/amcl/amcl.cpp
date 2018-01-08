//
// Created by shydh on 12/22/17.
//

#include <log_utils.h>
#include "amcl.h"

namespace amcl
{
    AMCL::AMCL(transform2::Transformer latest_tf_, transform2::Transformer tf_)
            : latest_tf_(latest_tf_), tf_(tf_),
              sent_first_transform_(false),
              latest_tf_valid_(false),
              map_(NULL),
              pf_(NULL),
              resample_count_(0),
              odom_(NULL),
              laser_(NULL),
              initial_pose_hyp_(NULL),
              first_map_received_(false),
              first_reconfigure_call_(true)
    {
        laser_min_range_ = -1.0;
        laser_max_range_ = -1.0;
        max_beams_ = 30;
        min_particles_ = 100;
        max_particles_ = 5000;
        pf_err_ = 0.01;
        pf_z_ = 0.99;
        alpha1_ = 0.2;
        alpha2_ = 0.2;
        alpha3_ = 0.2;
        alpha4_ = 0.2;
        alpha5_ = 0.2;

        do_beamskip_ = false;
        beam_skip_distance_ = 0.5;
        beam_skip_threshold_ = 0.3;
        beam_skip_error_threshold_ = 0.9;

        z_hit_ = 0.95;
        z_short_ = 0.1;
        z_max_ = 0.05;
        z_rand_ = 0.05;
        sigma_hit_ = 0.2;
        lambda_short_ = 0.1;
        laser_likelihood_max_dist_ = 2.0;

        laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;

        odom_model_type_ = ODOM_MODEL_DIFF;

        d_thresh_ = 0.2;
        a_thresh_ = M_PI / 6.0;
        odom_frame_id_ = 2;
        base_frame_id_ = 1;
        global_frame_id_ = 0;
        resample_interval_ = 2;

        alpha_slow_ = 0.001;
        alpha_fast_ = 0.1;
        tf_broadcast_ = true;

        updatePoseFromServer();

        if (map_ == NULL)
        {
            return;
        }

        LOGI("Initializing with uniform distribution");
        pf_init_model(pf_,
                      (pf_init_model_fn_t) uniformPoseGenerator,
                      (void *) map_);
        LOGI("Global initialisation done!");
        pf_init_ = false;
        return;
    }

    AMCL::~AMCL()
    {
        delete map_;
        //freeMapDependentMemory();
        delete pf_;
        delete odom_;
        delete laser_;
        delete initial_pose_hyp_;
        delete tf_;
    }

    int AMCL::process()
    {
        return 0;
    }

    void AMCL::updatePoseFromServer()
    {
        init_pose_[0] = 0.0;
        init_pose_[1] = 0.0;
        init_pose_[2] = 0.0;
        init_cov_[0] = 0.5 * 0.5;
        init_cov_[1] = 0.5 * 0.5;
        init_cov_[2] = (M_PI / 12.0) * (M_PI / 12.0);
    }

    pf_vector_t AMCL::uniformPoseGenerator(void *arg)
    {
        map_t *map = (map_t *) arg;

        double min_x, max_x, min_y, max_y;

        min_x = (map->size_x * map->scale) / 2.0 - map->origin_x;
        max_x = (map->size_x * map->scale) / 2.0 + map->origin_x;
        min_y = (map->size_y * map->scale) / 2.0 - map->origin_y;
        max_y = (map->size_y * map->scale) / 2.0 + map->origin_y;

        pf_vector_t p;

        LOGI("Generating new uniform sample");
        for (;;)
        {
            p.v[0] = min_x + drand48() * (max_x - min_x);
            p.v[1] = min_y + drand48() * (max_y - min_y);
            p.v[2] = drand48() * 2 * M_PI - M_PI;
            // Check that it's a free cell
            int i, j;
            i = (int) MAP_GXWX(map, p.v[0]);
            j = (int) MAP_GYWY(map, p.v[1]);
            if (MAP_VALID(map, i, j) &&
                (map->cells[MAP_INDEX(map, i, j)].occ_state == -1))
                break;
        }

        return p;
    }

    void AMCL::handleMapMessage(const base_info::OccupancyGrid &msg)
    {
        //boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

        LOGI("Received a %d X %d map @ %.3f m/pix\n",
             msg.info.width,
             msg.info.height,
             msg.info.resolution);

        freeMapDependentMemory();
        // Clear queued laser objects because they hold pointers to the existing
        // map, #5202.
        lasers_.clear();
        lasers_update_.clear();
        frame_to_laser_.clear();

        map_ = convertMap(msg);

        // Create the particle filter
        pf_ = pf_alloc(min_particles_, max_particles_,
                       alpha_slow_, alpha_fast_,
                       (pf_init_model_fn_t) uniformPoseGenerator,
                       (void *) map_);
        pf_->pop_err = pf_err_;
        pf_->pop_z = pf_z_;

        // Initialize the filter
        updatePoseFromServer();
        pf_vector_t pf_init_pose_mean = pf_vector_zero();
        pf_init_pose_mean.v[0] = init_pose_[0];
        pf_init_pose_mean.v[1] = init_pose_[1];
        pf_init_pose_mean.v[2] = init_pose_[2];
        pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
        pf_init_pose_cov.m[0][0] = init_cov_[0];
        pf_init_pose_cov.m[1][1] = init_cov_[1];
        pf_init_pose_cov.m[2][2] = init_cov_[2];
        pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
        pf_init_ = false;

        // Instantiate the sensor objects
        // Odometry
        delete odom_;
        odom_ = new AMCLOdom();
        odom_->SetModel(odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);
        // Laser
        delete laser_;
        laser_ = new AMCLLaser(max_beams_, map_);
        if (laser_model_type_ == LASER_MODEL_BEAM)
            laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_,
                                 sigma_hit_, lambda_short_, 0.0);
        else if (laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD_PROB)
        {
            LOGI("Initializing likelihood field model; "
                         "this can take some time on large maps...");
            laser_->SetModelLikelihoodFieldProb(z_hit_, z_rand_, sigma_hit_,
                                                laser_likelihood_max_dist_,
                                                do_beamskip_, beam_skip_distance_,
                                                beam_skip_threshold_,
                                                beam_skip_error_threshold_);
            LOGI("Done initializing likelihood field model.");
        } else
        {
            LOGI("Initializing likelihood field model; "
                         "this can take some time on large maps...");
            laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
                                            laser_likelihood_max_dist_);
            LOGI("Done initializing likelihood field model.");
        }

        // In case the initial pose message arrived before the first map,
        // try to apply the initial pose now that the map has arrived.
        applyInitialPose();
    }

    map_t *AMCL::convertMap(const base_info::OccupancyGrid &map_msg)
    {
        map_t *map = map_alloc();

        map->size_x = map_msg.info.width;
        map->size_y = map_msg.info.height;
        map->scale = map_msg.info.resolution;
        map->origin_x = map_msg.info.origin.x() + (map->size_x / 2) * map->scale;
        map->origin_y = map_msg.info.origin.y() + (map->size_y / 2) * map->scale;
        // Convert to player format
        map->cells = (map_cell_t *) malloc(sizeof(map_cell_t) * map->size_x * map->size_y);

        for (int i = 0; i < map->size_x * map->size_y; i++)
        {
            if (map_msg.data[i] == 0)
                map->cells[i].occ_state = -1;
            else if (map_msg.data[i] == 100)
                map->cells[i].occ_state = +1;
            else
                map->cells[i].occ_state = 0;
        }

        return map;
    }

    void AMCL::applyInitialPose()
    {
        if (initial_pose_hyp_ != NULL && map_ != NULL)
        {
            pf_init(pf_,
                    initial_pose_hyp_->pf_pose_mean,
                    initial_pose_hyp_->pf_pose_cov);
            pf_init_ = false;

            delete initial_pose_hyp_;
            initial_pose_hyp_ = NULL;
        }
    }

    bool AMCL::globalLocalizationCallback()
    {
        if (map_ == NULL)
            return false;

        //boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
        LOGI("Initializing with uniform distribution");
        pf_init_model(pf_,
                      (pf_init_model_fn_t) uniformPoseGenerator,
                      (void *) map_);
        LOGI("Global initialisation done!");
        pf_init_ = false;
        return true;
    }

    void AMCL::nomotionUpdateCallback()
    {
        m_force_update = true;
        //ROS_INFO("Requesting no-motion update");
        return;
    }

    void AMCL::setMapCallback()
    {
        //handleMapMessage(req.map);
        //handleInitialPoseMessage(req.initial_pose);
        return;
    }

    void AMCL::laserReceived(const base_info::LaserScan &laser_scan)
    {
        last_laser_received_ts_ = time(NULL);
        if (map_ == NULL)
            return;

        //boost::recursive_mutex::scoped_lock lr(configuration_mutex_);
        int laser_index = -1;

        // Do we have the base->base_laser Tx yet?
        if (frame_to_laser_.find(laser_scan.frame_id_) == frame_to_laser_.end())
        {
            LOGI("Setting up laser %d (frame_id=%d)",
                 (int) frame_to_laser_.size(),
                 laser_scan.frame_id_);
            lasers_.push_back(new AMCLLaser(*laser_));
            lasers_update_.push_back(true);
            laser_index = (int) frame_to_laser_.size();

            base_info::StampedPose ident;
            /*(tf::Transform(tf::createIdentityQuaternion(),
                                                       tf::Vector3(0,0,0)),
                                         ros::Time(), laser_scan->header.frame_id);*/
            base_info::StampedPose laser_pose;
            tf_.transformPose(base_frame_id_, ident, laser_pose);

            pf_vector_t laser_pose_v;
            laser_pose_v.v[0] = laser_pose.x();
            laser_pose_v.v[1] = laser_pose.y();
            // laser mounting angle gets computed later -> set to 0 here!
            laser_pose_v.v[2] = 0;
            lasers_[laser_index]->SetLaserPose(laser_pose_v);
            LOGI("Received laser's pose wrt robot: %.3f %.3f %.3f",
                 laser_pose_v.v[0],
                 laser_pose_v.v[1],
                 laser_pose_v.v[2]);

            frame_to_laser_[laser_scan.frame_id_] = laser_index;
        } else
        {
            // we have the laser pose, retrieve laser index
            laser_index = frame_to_laser_[laser_scan.frame_id_];
        }

        // Where was the robot when this scan was taken?
        pf_vector_t pose;
        if (!getOdomPose(latest_odom_pose_, pose.v[0], pose.v[1], pose.v[2],
                         laser_scan.frame_id_, base_frame_id_))
        {
            LOGI("Couldn't determine robot's pose associated with laser scan");
            return;
        }


        pf_vector_t delta = pf_vector_zero();

        if (pf_init_)
        {
            // Compute change in pose
            //delta = pf_vector_coord_sub(pose, pf_odom_pose_);
            delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
            delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
            delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);

            // See if we should update the filter
            bool update = fabs(delta.v[0]) > d_thresh_ ||
                          fabs(delta.v[1]) > d_thresh_ ||
                          fabs(delta.v[2]) > a_thresh_;
            update = update || m_force_update;
            m_force_update = false;

            // Set the laser update flags
            if (update)
                for (unsigned int i = 0; i < lasers_update_.size(); i++)
                    lasers_update_[i] = true;
        }

        bool force_publication = false;
        if (!pf_init_)
        {
            // Pose at last filter update
            pf_odom_pose_ = pose;

            // Filter is now initialized
            pf_init_ = true;

            // Should update sensor data
            for (unsigned int i = 0; i < lasers_update_.size(); i++)
                lasers_update_[i] = true;

            force_publication = true;

            resample_count_ = 0;
        }
            // If the robot has moved, update the filter
        else if (pf_init_ && lasers_update_[laser_index])
        {
            //printf("pose\n");
            //pf_vector_fprintf(pose, stdout, "%.3f");

            AMCLOdomData odata;
            odata.pose = pose;
            // HACK
            // Modify the delta in the action data so the filter gets
            // updated correctly
            odata.delta = delta;

            // Use the action data to update the filter
            odom_->UpdateAction(pf_, (AMCLSensorData *) &odata);

            // Pose at last filter update
            //pf_odom_pose = pose;
        }

        bool resampled = false;
        // If the robot has moved, update the filter
        if (lasers_update_[laser_index])
        {
            AMCLLaserData ldata;
            ldata.sensor = lasers_[laser_index];
            ldata.range_count = (int) laser_scan.range_max;

            // To account for lasers that are mounted upside-down, we determine the
            // min, max, and increment angles of the laser in the base frame.
            //
            // Construct min and max angles of laser, in the base_link frame.
            Eigen::Quaterniond q;
            double yaw = 0.0, pitching = 0.0, droll = laser_scan.angle_min;
            Eigen::Vector3d ea0(yaw, pitching, droll);
            Eigen::Matrix3d R;
            R = Eigen::AngleAxisd(ea0[0], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(ea0[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(ea0[2], Eigen::Vector3d::UnitX());
            q = R;

            base_info::StampedQuat min_q;
            min_q = q;
            min_q.stamp_ = laser_scan.stamp_;
            min_q.frame_id_ = laser_scan.frame_id_;

            droll = laser_scan.angle_min + laser_scan.angle_increment;
            Eigen::Vector3d ea1(yaw, pitching, droll);
            R = Eigen::AngleAxisd(ea1[0], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(ea1[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(ea1[2], Eigen::Vector3d::UnitX());
            q = R;

            base_info::StampedQuat inc_q;
            inc_q = q;
            inc_q.stamp_ = laser_scan.stamp_;
            inc_q.frame_id_ = laser_scan.frame_id_;

            tf_.transformQuaternion(base_frame_id_, min_q, min_q);
            tf_.transformQuaternion(base_frame_id_, inc_q, inc_q);

            double angle_min = tf::getYaw(min_q);
            double angle_increment = tf::getYaw(inc_q) - angle_min;

            // wrapping angle to [-pi .. pi]
            angle_increment = fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI;

            LOGI("Laser %d angles in base frame: min: %.3f inc: %.3f",
                 laser_index, angle_min,
                 angle_increment);

            // Apply range min/max thresholds, if the user supplied them
            if (laser_max_range_ > 0.0)
                ldata.range_max = std::min(laser_scan.range_max, (float) laser_max_range_);
            else
                ldata.range_max = laser_scan.range_max;

            double range_min;
            if (laser_min_range_ > 0.0)
                range_min = std::max(laser_scan.range_min, (float) laser_min_range_);
            else
                range_min = laser_scan.range_min;

            // The AMCLLaserData destructor will free this memory
            ldata.ranges = new double[ldata.range_count][2];

            for (int i = 0; i < ldata.range_count; i++)
            {
                // amcl doesn't (yet) have a concept of min range.  So we'll map short
                // readings to max range.
                if (laser_scan.ranges[i] <= range_min)
                    ldata.ranges[i][0] = ldata.range_max;
                else
                    ldata.ranges[i][0] = laser_scan.ranges[i];
                // Compute bearing
                ldata.ranges[i][1] = angle_min +
                                     (i * angle_increment);
            }

            lasers_[laser_index]->UpdateSensor(pf_, (AMCLSensorData *) &ldata);

            lasers_update_[laser_index] = false;

            pf_odom_pose_ = pose;

            // Resample the particles
            if (!(++resample_count_ % resample_interval_))
            {
                pf_update_resample(pf_);
                resampled = true;
            }

            pf_sample_set_t *set = pf_->sets + pf_->current_set;
            LOGI("Num samples: %d\n", set->sample_count);

            // Publish the resulting cloud
            // TODO: set maximum rate for publishing
            /*if (!m_force_update)
            {
                geometry_msgs::PoseArray cloud_msg;
                cloud_msg.header.stamp = ros::Time::now();
                cloud_msg.header.frame_id = global_frame_id_;
                cloud_msg.poses.resize(set->sample_count);
                for (int i = 0; i < set->sample_count; i++)
                {
                    tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(set->samples[i].pose.v[2]),
                                             tf::Vector3(set->samples[i].pose.v[0],
                                                         set->samples[i].pose.v[1], 0)),
                                    cloud_msg.poses[i]);
                }
                //particlecloud_pub_.publish(cloud_msg);
            }*/
        }

        if (resampled || force_publication)
        {
            // Read out the current hypotheses
            double max_weight = 0.0;
            int max_weight_hyp = -1;
            std::vector<amcl_hyp_t> hyps;
            hyps.resize((unsigned long) pf_->sets[pf_->current_set].cluster_count);
            for (int hyp_count = 0;
                 hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
            {
                double weight;
                pf_vector_t pose_mean;
                pf_matrix_t pose_cov;
                if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov))
                {
                    LOGI("Couldn't get stats on cluster %d", hyp_count);
                    break;
                }

                hyps[hyp_count].weight = weight;
                hyps[hyp_count].pf_pose_mean = pose_mean;
                hyps[hyp_count].pf_pose_cov = pose_cov;

                if (hyps[hyp_count].weight > max_weight)
                {
                    max_weight = hyps[hyp_count].weight;
                    max_weight_hyp = hyp_count;
                }
            }

            if (max_weight > 0.0)
            {
                LOGI("Max weight pose: %.3f %.3f %.3f",
                     hyps[max_weight_hyp].pf_pose_mean.v[0],
                     hyps[max_weight_hyp].pf_pose_mean.v[1],
                     hyps[max_weight_hyp].pf_pose_mean.v[2]);

                /*geometry_msgs::PoseWithCovarianceStamped p;
                // Fill in the header
                p.header.frame_id = global_frame_id_;
                p.header.stamp = laser_scan..stamp;
                // Copy in the pose
                p.pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
                p.pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
                tf::quaternionTFToMsg(
                        tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
                        p.pose.pose.orientation);*/
                // Copy in the covariance, converting from 3-D to 6-D
                pf_sample_set_t *set = pf_->sets + pf_->current_set;
                /*for (int i = 0; i < 2; i++)
                {
                    for (int j = 0; j < 2; j++)
                    {
                        // Report the overall filter covariance, rather than the
                        // covariance for the highest-weight cluster
                        //p.covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
                        p.pose.covariance[6 * i + j] = set->cov.m[i][j];
                    }
                }*/
                // Report the overall filter covariance, rather than the
                // covariance for the highest-weight cluster
                //p.covariance[6*5+5] = hyps[max_weight_hyp].pf_pose_cov.m[2][2];
                //p.pose.covariance[6 * 5 + 5] = set->cov.m[2][2];

                //pose_pub_.publish(p);
                //last_published_pose = p;

                LOGI("New pose: %6.3f %6.3f %6.3f",
                     hyps[max_weight_hyp].pf_pose_mean.v[0],
                     hyps[max_weight_hyp].pf_pose_mean.v[1],
                     hyps[max_weight_hyp].pf_pose_mean.v[2]);

                // subtracting base to odom from map to base and send map to odom instead
                base_info::StampedPose odom_to_map;

                /*tf::Transform tmp_tf(
                        tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
                        tf::Vector3(hyps[max_weight_hyp].pf_pose_mean.v[0],
                                    hyps[max_weight_hyp].pf_pose_mean.v[1],
                                    0.0));*/
                base_info::StampedPose tmp_tf_stamped;
                /*(tmp_tf.inverse(),
                  laser_scan.stamp_,
                  base_frame_id_);*/
                tf_.transformPose(odom_frame_id_,
                                  tmp_tf_stamped,
                                  odom_to_map);

                /*latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                           tf::Point(odom_to_map.getOrigin()));*/
                latest_tf_valid_ = true;

                if (tf_broadcast_ == true)
                {
                    // We want to send a transform that is good up until a
                    // tolerance time so that odom can be used
                    /*ros::Time transform_expiration = (laser_scan.stamp_ +
                                                      transform_tolerance_);
                    tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                                        transform_expiration,
                                                        global_frame_id_, odom_frame_id_);
                    tfb_->sendTransform(tmp_tf_stamped);*/
                    sent_first_transform_ = true;
                }
            }
        } else if (latest_tf_valid_)
        {
            if (tf_broadcast_)
            {
                // Nothing changed, so we'll just republish the last transform, to keep
                // everybody happy.
                /*ros::Time transform_expiration = (laser_scan->header.stamp +
                                                  transform_tolerance_);
                tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                                    transform_expiration,
                                                    global_frame_id_, odom_frame_id_);
                this->tfb_->sendTransform(tmp_tf_stamped);*/
            }

            // Is it time to save our last pose to the param server
            /*ros::Time now = ros::Time::now();
            if ((save_pose_period.toSec() > 0.0) &&
                (now - save_pose_last_time) >= save_pose_period)
            {
                this->savePoseToServer();
                save_pose_last_time = now;
            }*/
        }
    }

    bool AMCL::getOdomPose(base_info::StampedPose &odom_pose,
                           double &x, double &y, double &yaw,
                           const time_t &t, const int f)
    {
        // Get the robot's pose
        base_info::StampedPose ident;
        /*(tf::Transform(tf::createIdentityQuaternion(),
                                                   tf::Vector3(0,0,0)), t, f);*/
        tf_.transformPose(odom_frame_id_, ident, odom_pose);
        x = odom_pose.x();
        y = odom_pose.y();
        double pitch, roll;
        //odom_pose.getBasis().getEulerYPR(yaw, pitch, roll);

        return true;
    }

}
