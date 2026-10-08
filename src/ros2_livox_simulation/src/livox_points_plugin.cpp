#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/logging.hpp>

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>// Store the latest laser scans into laserMsg
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>
#include <chrono>
#include <algorithm>
#include <cmath>
#include "ros2_livox/livox_points_plugin.h"
#include "ros2_livox/csv_reader.hpp"
#include "ros2_livox/livox_ode_multiray_shape.h"
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <omp.h>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace gazebo
{

    GZ_REGISTER_SENSOR_PLUGIN(LivoxPointsPlugin)

    LivoxPointsPlugin::LivoxPointsPlugin() {}

    LivoxPointsPlugin::~LivoxPointsPlugin() {}

    void convertDataToRotateInfo(const std::vector<std::vector<double>> &datas, std::vector<AviaRotateInfo> &avia_infos) {
        avia_infos.reserve(datas.size());
        double deg_2_rad = M_PI / 180.0;
        for (auto &data : datas) {
            if (data.size() == 3) {
                avia_infos.emplace_back();
                avia_infos.back().time = data[0];
                avia_infos.back().azimuth = data[1] * deg_2_rad;
                avia_infos.back().zenith = data[2] * deg_2_rad - M_PI_2; //转化成标准的右手系角度
            } else {
            RCLCPP_ERROR(rclcpp::get_logger("convertDataToRotateInfo"), "data size is not 3!");
            }
        }
    }

    void LivoxPointsPlugin::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr sdf) {
        node_ = gazebo_ros::Node::Get(sdf);
        
        std::vector<std::vector<double>> datas;
        std::string file_name = sdf->Get<std::string>("csv_file_name");
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "load csv file name: %s", file_name.c_str());
        if (!CsvReader::ReadCsvFile(file_name, datas)) {   
            RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "cannot get csv file! %s will return !", file_name.c_str());
            return;
        }
        sdfPtr = sdf;
        auto rayElem = sdfPtr->GetElement("ray");
        auto scanElem = rayElem->GetElement("scan");
        auto rangeElem = rayElem->GetElement("range");


        raySensor = _parent;
        if (sdf->HasElement("scan_period_sec")) {
            scanPeriodNs = static_cast<int64_t>(sdf->Get<double>("scan_period_sec") * 1e9);
        }
        if (sdf->HasElement("raycast_update_rate")) {
            raycastUpdateRate = sdf->Get<int>("raycast_update_rate");
        }
        if (scanPeriodNs <= 0 || raycastUpdateRate <= 0) {
            RCLCPP_ERROR(rclcpp::get_logger("LivoxPointsPlugin"), "scan period and raycast update rate must be positive");
            return;
        }
        // The sensor callback is the firing clock. Publish only after the full
        // scan has been sampled across this interval.
        raySensor->SetUpdateRate(static_cast<double>(raycastUpdateRate));
        auto sensor_pose = raySensor->Pose();
        auto curr_scan_topic = sdf->Get<std::string>("topic");
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "ros topic name: %s", curr_scan_topic.c_str());

        child_name = raySensor->Name();
        parent_name = raySensor->ParentName();
        size_t delimiter_pos = parent_name.find("::");
        parent_name = parent_name.substr(delimiter_pos + 2);

        node = transport::NodePtr(new transport::Node());
        node->Init(raySensor->WorldName());
        // PointCloud2 publisher
        cloud2_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(curr_scan_topic, 10);

        scanPub = node->Advertise<msgs::LaserScanStamped>(curr_scan_topic+"laserscan", 50);

        aviaInfos.clear();
        convertDataToRotateInfo(datas, aviaInfos);
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "scan info size: %ld", aviaInfos.size());
        maxPointSize = aviaInfos.size();

        RayPlugin::Load(_parent, sdfPtr);
        laserMsg.mutable_scan()->set_frame(_parent->ParentName());
        // parentEntity = world->GetEntity(_parent->ParentName());
        parentEntity = this->world->EntityByName(_parent->ParentName());
        //SendRosTf(sensor_pose, raySensor->ParentName(), raySensor->Name());
        auto physics = world->Physics();
        laserCollision = physics->CreateCollision("multiray", _parent->ParentName());
        laserCollision->SetName("ray_sensor_collision");
        laserCollision->SetRelativePose(_parent->Pose());
        laserCollision->SetInitialRelativePose(_parent->Pose());
        rayShape.reset(new gazebo::physics::LivoxOdeMultiRayShape(laserCollision));
        laserCollision->SetShape(rayShape);
        samplesStep = sdfPtr->Get<int>("samples");
        downSample = sdfPtr->Get<int>("downsample");
        if (downSample < 1) {
            downSample = 1;
        }
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "sample: %ld", samplesStep);
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "downsample: %ld", downSample);
        if (samplesStep <= 0 || maxPointSize <= 0) {
            RCLCPP_ERROR(rclcpp::get_logger("LivoxPointsPlugin"), "samples and CSV pattern must be non-empty");
            return;
        }
        pointsPerScan = (samplesStep + downSample - 1) / downSample;
        const double physicsStep = std::max(0.0, world->Physics()->GetMaxStepSize());
        const double callbackStep = std::max(physicsStep, 1.0 / static_cast<double>(raycastUpdateRate));
        rayPoolSize = std::min(pointsPerScan,
            static_cast<size_t>(std::ceil(pointsPerScan * callbackStep / (scanPeriodNs * 1e-9))) + 4);
        framePoints.resize(pointsPerScan);
        rayShape->RayShapes().reserve(rayPoolSize);
        rayShape->Load(sdfPtr);
        rayShape->Init();
        minDist = rangeElem->Get<double>("min");
        maxDist = rangeElem->Get<double>("max");
        auto offset = laserCollision->RelativePose();
        ignition::math::Vector3d start_point, end_point;
        for (size_t j = 0; j < rayPoolSize; ++j) {
            size_t index = j % maxPointSize;
            auto &rotate_info = aviaInfos[index];
            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            start_point = minDist * axis + offset.Pos();
            end_point = maxDist * axis + offset.Pos();
            rayShape->AddRay(start_point, end_point);
        }
    }

    void LivoxPointsPlugin::OnNewLaserScans() {
        if (!rayShape || !raySensor || pointsPerScan == 0) {
            return;
        }
        const auto simTime = raySensor->LastMeasurementTime();
        const int64_t nowNs = static_cast<int64_t>(simTime.sec) * 1000000000LL + simTime.nsec;

        if (!frameStarted) {
            frameStartNs = nowNs;
            frameStarted = true;
        }

        // Finish each scan at its 100 ms boundary. All samples due in the last
        // physics interval are raycast at this callback's simulation pose.
        while (nowNs >= frameStartNs + scanPeriodNs) {
            // A delayed Gazebo sensor callback can leave more due rays than fit
            // in the reusable ray pool. Drain them in batches at this actual
            // callback time so every emitted point has a matching pose/time.
            while (nextPointIndex < pointsPerScan) {
                const size_t previousIndex = nextPointIndex;
                SampleDueRays(nowNs);
                if (nextPointIndex == previousIndex) {
                    RCLCPP_ERROR(rclcpp::get_logger("LivoxPointsPlugin"),
                                 "Unable to drain due rays at scan boundary");
                    break;
                }
            }
            PublishScan(frameStartNs);
            frameStartNs += scanPeriodNs;
            nextPointIndex = 0;
            framePoints.assign(pointsPerScan, SampledPoint{});
            currStartIndex += samplesStep;
        }
        SampleDueRays(nowNs);
        if (nextPointIndex == pointsPerScan) {
            PublishScan(frameStartNs);
            frameStartNs += scanPeriodNs;
            nextPointIndex = 0;
            framePoints.assign(pointsPerScan, SampledPoint{});
            currStartIndex += samplesStep;
        }
    }

    void LivoxPointsPlugin::SampleDueRays(int64_t now_ns) {
        struct DueRay { size_t output_index; size_t slot; AviaRotateInfo info; };
        const int64_t elapsedNs = std::max<int64_t>(0, now_ns - frameStartNs);
        auto &rays = rayShape->RayShapes();
        std::vector<DueRay> due;
        due.reserve(rayPoolSize);
        const auto offset = laserCollision->RelativePose();
        ignition::math::Vector3d start, end;
        ignition::math::Quaterniond rotation;
        while (nextPointIndex < pointsPerScan) {
            const int64_t scheduledNs = static_cast<int64_t>(nextPointIndex) * scanPeriodNs / pointsPerScan;
            if (scheduledNs > elapsedNs) break;
            const size_t csvIndex = static_cast<size_t>(
                (currStartIndex + static_cast<int64_t>(nextPointIndex) * downSample) % maxPointSize);
            const auto &info = aviaInfos[csvIndex];
            rotation.Euler(ignition::math::Vector3d(0.0, info.zenith, info.azimuth));
            const auto axis = offset.Rot() * rotation * ignition::math::Vector3d(1.0, 0.0, 0.0);
            start = minDist * axis + offset.Pos();
            end = maxDist * axis + offset.Pos();
            const size_t slot = due.size();
            if (slot >= rays.size()) {
                // The caller can invoke this method again to drain the due
                // points in another batch using the same ray pool.
                break;
            }
            rays[slot]->SetPoints(start, end);
            due.push_back({nextPointIndex, slot, info});
            ++nextPointIndex;
        }
        if (due.empty()) return;

        rayShape->Update();
        const uint32_t actualOffsetNs = static_cast<uint32_t>(
            std::min<int64_t>(elapsedNs, scanPeriodNs));
        for (const auto &sample : due) {
            double range = rayShape->GetRange(sample.slot);
            if (range >= RangeMax() || range <= RangeMin()) range = 0.0;
            rotation.Euler(ignition::math::Vector3d(0.0, sample.info.zenith, sample.info.azimuth));
            const auto point = range * (rotation * ignition::math::Vector3d(1.0, 0.0, 0.0));
            auto &output = framePoints[sample.output_index];
            output.x = static_cast<float>(point.X());
            output.y = static_cast<float>(point.Y());
            output.z = static_cast<float>(point.Z());
            output.intensity = static_cast<float>(rayShape->GetRetro(sample.slot));
            output.csv_time_raw = sample.info.time;
            output.offset_time_ns = actualOffsetNs;
        }
    }

    void LivoxPointsPlugin::PublishScan(int64_t stamp_ns) {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
        cloud.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
        cloud.header.frame_id = raySensor->Name();
        cloud.height = 1;
        cloud.width = static_cast<uint32_t>(pointsPerScan);
        cloud.is_dense = false;
        cloud.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(9,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
            "tag", 1, sensor_msgs::msg::PointField::UINT16,
            "line", 1, sensor_msgs::msg::PointField::UINT16,
            "csv_time_raw", 1, sensor_msgs::msg::PointField::FLOAT64,
            "scan_sequence", 1, sensor_msgs::msg::PointField::UINT32,
            "offset_time", 1, sensor_msgs::msg::PointField::UINT32);
        modifier.resize(pointsPerScan);

        sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> intensity(cloud, "intensity");
        sensor_msgs::PointCloud2Iterator<uint16_t> tag(cloud, "tag"), line(cloud, "line");
        sensor_msgs::PointCloud2Iterator<double> csvTime(cloud, "csv_time_raw");
        sensor_msgs::PointCloud2Iterator<uint32_t> sequence(cloud, "scan_sequence"), offsetTime(cloud, "offset_time");
        for (const auto &point : framePoints) {
            *x = point.x; *y = point.y; *z = point.z;
            *intensity = point.intensity;
            *tag = 0; *line = 0;
            *csvTime = point.csv_time_raw;
            *sequence = scanSequence;
            *offsetTime = point.offset_time_ns;
            ++x; ++y; ++z; ++intensity; ++tag; ++line; ++csvTime; ++sequence; ++offsetTime;
        }
        cloud2_pub->publish(cloud);
        ++scanSequence;
    }

    void LivoxPointsPlugin::InitializeScan(msgs::LaserScan *&scan) {
        // Store the latest laser scans into laserMsg
        msgs::Set(scan->mutable_world_pose(), raySensor->Pose() + parentEntity->WorldPose());
        scan->set_angle_min(AngleMin().Radian());
        scan->set_angle_max(AngleMax().Radian());
        scan->set_angle_step(AngleResolution());
        scan->set_count(RangeCount());

        scan->set_vertical_angle_min(VerticalAngleMin().Radian());
        scan->set_vertical_angle_max(VerticalAngleMax().Radian());
        scan->set_vertical_angle_step(VerticalAngleResolution());
        scan->set_vertical_count(VerticalRangeCount());

        scan->set_range_min(RangeMin());
        scan->set_range_max(RangeMax());

        scan->clear_ranges();
        scan->clear_intensities();

        unsigned int rangeCount = RangeCount();
        unsigned int verticalRangeCount = VerticalRangeCount();

        for (unsigned int j = 0; j < verticalRangeCount; ++j)
        {
            for (unsigned int i = 0; i < rangeCount; ++i)
            {
                scan->add_ranges(0);
                scan->add_intensities(0);
            }
        }
    }

    ignition::math::Angle LivoxPointsPlugin::AngleMin() const
    {
        if (rayShape)
            return rayShape->MinAngle();
        else
            return -1;
    }

    ignition::math::Angle LivoxPointsPlugin::AngleMax() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->MaxAngle().Radian());
        }
        else
            return -1;
    }

    double LivoxPointsPlugin::GetRangeMin() const { return RangeMin(); }

    double LivoxPointsPlugin::RangeMin() const
    {
        if (rayShape)
            return rayShape->GetMinRange();
        else
            return -1;
    }

    double LivoxPointsPlugin::GetRangeMax() const { return RangeMax(); }

    double LivoxPointsPlugin::RangeMax() const
    {
        if (rayShape)
            return rayShape->GetMaxRange();
        else
            return -1;
    }

    double LivoxPointsPlugin::GetAngleResolution() const { return AngleResolution(); }

    double LivoxPointsPlugin::AngleResolution() const { return (AngleMax() - AngleMin()).Radian() / (RangeCount() - 1); }

    double LivoxPointsPlugin::GetRangeResolution() const { return RangeResolution(); }

    double LivoxPointsPlugin::RangeResolution() const
    {
        if (rayShape)
            return rayShape->GetResRange();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetRayCount() const { return RayCount(); }

    int LivoxPointsPlugin::RayCount() const
    {
        if (rayShape)
            return rayShape->GetSampleCount();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetRangeCount() const { return RangeCount(); }

    int LivoxPointsPlugin::RangeCount() const
    {
        if (rayShape)
            return rayShape->GetSampleCount() * rayShape->GetScanResolution();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetVerticalRayCount() const { return VerticalRayCount(); }

    int LivoxPointsPlugin::VerticalRayCount() const
    {
        if (rayShape)
            return rayShape->GetVerticalSampleCount();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetVerticalRangeCount() const { return VerticalRangeCount(); }

    int LivoxPointsPlugin::VerticalRangeCount() const
    {
        if (rayShape)
            return rayShape->GetVerticalSampleCount() * rayShape->GetVerticalScanResolution();
        else
            return -1;
    }

    ignition::math::Angle LivoxPointsPlugin::VerticalAngleMin() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->VerticalMinAngle().Radian());
        }
        else
            return -1;
    }

    ignition::math::Angle LivoxPointsPlugin::VerticalAngleMax() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->VerticalMaxAngle().Radian());
        }
        else
            return -1;
    }

    double LivoxPointsPlugin::GetVerticalAngleResolution() const { return VerticalAngleResolution(); }

    double LivoxPointsPlugin::VerticalAngleResolution() const
    {
        return (VerticalAngleMax() - VerticalAngleMin()).Radian() / (VerticalRangeCount() - 1);
    }


}
