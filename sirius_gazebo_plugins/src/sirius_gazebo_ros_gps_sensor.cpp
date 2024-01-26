/*
 * Copyright 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * This code is based on the Gazebo ROS Magnetometer Sensor plugin.
 * Original code repository: https://github.com/Darkproduct/gazebo_ros_magnetometer_sensor
 *
 * Modified versions of the original code are licensed under the same
 * Apache License, Version 2.0.
 */

#include <sirius_gazebo_plugins/sirius_gazebo_ros_gps_sensor.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

GZ_REGISTER_SENSOR_PLUGIN(gazebo::GazeboRosGpsSensor)

namespace gazebo
{
GazeboRosGpsSensor::GazeboRosGpsSensor() : SensorPlugin()
{
}

GazeboRosGpsSensor::~GazeboRosGpsSensor()
{
  if (node != nullptr)
  {
    node->shutdown();
  }
}

void GazeboRosGpsSensor::Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_)
{
  sdf = sdf_;
  sensor = std::dynamic_pointer_cast<sensors::GpsSensor>(sensor_);

  if (!sensor)
  {
    ROS_FATAL("Error: Sensor pointer is NULL!");
    return;
  }

  sensor->SetActive(true);

  if (!LoadParameters())
  {
    ROS_FATAL("Error Loading Parameters!");
    return;
  }

  if (!ros::isInitialized())
  {
    ROS_FATAL("ROS has not been initialized!");
    return;
  }

  node = std::make_unique<ros::NodeHandle>(robot_namespace);

  gps_data_publisher = node->advertise<sensor_msgs::NavSatFix>(topic_name, 1);
  gps_velocity_data_publisher = node->advertise<geometry_msgs::Vector3Stamped>(topic_name + "_velocity", 1);
  connection =
      event::Events::ConnectWorldUpdateBegin([this](const gazebo::common::UpdateInfo& info) { UpdateChild(info); });

  last_time = sensor->LastUpdateTime();
}

void GazeboRosGpsSensor::UpdateChild(const gazebo::common::UpdateInfo& /*_info*/)
{
  common::Time current_time = sensor->LastUpdateTime();

  if (update_rate > 0 && (current_time - last_time).Double() < 1.0 / update_rate)
  {
    return;
  }

  if (gps_data_publisher.getNumSubscribers() > 0 || gps_velocity_data_publisher.getNumSubscribers() > 0)
  {
    gps_msg.latitude = sensor->Latitude().Degree();
    gps_msg.longitude = sensor->Longitude().Degree();
    gps_msg.altitude = sensor->Altitude();
    gps_velocity_msg.vector.x = sensor->VelocityEast();
    gps_velocity_msg.vector.y = sensor->VelocityNorth();
    gps_velocity_msg.vector.z = sensor->VelocityUp();

    gps_msg.position_covariance[0] = position_covariance[0];
    gps_msg.position_covariance[1] = position_covariance[1];
    gps_msg.position_covariance[2] = position_covariance[2];
    gps_msg.position_covariance[3] = position_covariance[3];
    gps_msg.position_covariance[4] = position_covariance[4];
    gps_msg.position_covariance[5] = position_covariance[5];
    gps_msg.position_covariance[6] = position_covariance[6];
    gps_msg.position_covariance[7] = position_covariance[7];
    gps_msg.position_covariance[8] = position_covariance[8];

    gps_msg.header.frame_id = body_name;
    gps_msg.header.stamp.sec = current_time.sec;
    gps_msg.header.stamp.nsec = current_time.nsec;

    gps_velocity_msg.header.frame_id = body_name;
    gps_velocity_msg.header.stamp.sec = current_time.sec;
    gps_velocity_msg.header.stamp.nsec = current_time.nsec;

    gps_data_publisher.publish(gps_msg);
    gps_velocity_data_publisher.publish(gps_velocity_msg);

    ros::spinOnce();
  }

  last_time = current_time;
}

bool GazeboRosGpsSensor::LoadParameters()
{
  if (sdf->HasElement("robotNamespace"))
  {
    robot_namespace = sdf->Get<std::string>("robotNamespace") + "/";
    ROS_INFO_STREAM("<robotNamespace> set to: " << robot_namespace);
  }
  else
  {
    std::string scoped_name = sensor->ParentName();
    std::size_t it = scoped_name.find("::");

    robot_namespace = "/" + scoped_name.substr(0, it) + "/";
    ROS_WARN_STREAM("missing <robotNamespace>, set to: " << robot_namespace);
  }

  if (sdf->HasElement("topicName"))
  {
    topic_name = robot_namespace + sdf->Get<std::string>("topicName");
    ROS_INFO_STREAM("<topicName> set to: " << topic_name);
  }
  else
  {
    topic_name = robot_namespace + "/gps";
    ROS_WARN_STREAM("missing <topicName>, set to: " << topic_name);
  }

  if (sdf->HasElement("frameName"))
  {
    body_name = sdf->Get<std::string>("frameName");
    ROS_INFO_STREAM("<frameName> set to: " << body_name);
  }
  else
  {
    ROS_FATAL("missing <frameName>, cannot proceed");
    return false;
  }

  if (sdf->HasElement("updateRateHZ"))
  {
    update_rate = sdf->Get<double>("updateRateHZ");
    ROS_INFO_STREAM("<updateRateHZ> set to: " << update_rate);
  }
  else
  {
    update_rate = 1.0;
    ROS_WARN_STREAM("missing <updateRateHZ>, set to: " << update_rate);
  }
  if (sdf->HasElement("positionCovariance"))
  {
    auto position_covariance_stream = std::stringstream(sdf->Get<std::string>("positionCovariance"));
    position_covariance_stream.setf(std::ios_base::skipws);
    position_covariance_stream >> position_covariance[0] >> position_covariance[1] >> position_covariance[2] >>
        position_covariance[3] >> position_covariance[4] >> position_covariance[5] >> position_covariance[6] >>
        position_covariance[7] >> position_covariance[8];
    ROS_INFO_STREAM("Position covariance: "
                    << position_covariance[0] << " " << position_covariance[1] << " " << position_covariance[2] << " "
                    << position_covariance[3] << " " << position_covariance[4] << " " << position_covariance[5] << " "
                    << position_covariance[6] << " " << position_covariance[7] << " " << position_covariance[8]);
  }
  if (sdf->HasElement("positionCovarianceType"))
  {
    auto position_covariance_type = sdf->Get<std::string>("positionCovarianceType");
    std::transform(position_covariance_type.begin(), position_covariance_type.end(), position_covariance_type.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    if (position_covariance_type == "unknown")
    {
      gps_msg.position_covariance_type = 0;
      ROS_INFO_STREAM("Position covariance type: UNKNOWN");
    }
    else if (position_covariance_type == "approximated")
    {
      gps_msg.position_covariance_type = 1;
      ROS_INFO_STREAM("Position covariance type: APPROXIMATED");
    }
    else if (position_covariance_type == "diagonal_known")
    {
      gps_msg.position_covariance_type = 2;
      ROS_INFO_STREAM("Position covariance type: DIAGONAL_KNOWN");
    }
    else if (position_covariance_type == "known")
    {
      gps_msg.position_covariance_type = 3;
      ROS_INFO_STREAM("Position covariance type: KNOWN");
    }
    else
    {
      gps_msg.position_covariance_type = 0;
      ROS_WARN_STREAM("missing or unknown positionCovarianceType: " << position_covariance_type << ". Set to UNKNOWN");
    }
  }

  // Load spherical_coordinates parameters
  if (sdf->HasElement("sphericalCoordinates"))
  {
    // Set Gazebo world origin to the spherical_coordinates values
    gazebo::physics::WorldPtr world = gazebo::physics::get_world(sensor->WorldName());
    if (world)
    {
      sdf::ElementPtr spherical_coordinates = sdf->GetElement("sphericalCoordinates");

      if (spherical_coordinates->HasElement("latitudeDeg"))
      {
        auto latitude = spherical_coordinates->Get<double>("latitudeDeg");
        world->SphericalCoords()->SetLatitudeReference(latitude * M_PI / 180.0);
        ROS_INFO_STREAM("Latitude: " << latitude);
      }

      if (spherical_coordinates->HasElement("longitudeDeg"))
      {
        auto longitude = spherical_coordinates->Get<double>("longitudeDeg");
        world->SphericalCoords()->SetLongitudeReference(longitude * M_PI / 180.0);
        ROS_INFO_STREAM("Longitude: " << longitude);
      }

      if (spherical_coordinates->HasElement("elevation"))
      {
        auto elevation = spherical_coordinates->Get<double>("elevation");
        world->SphericalCoords()->SetElevationReference(elevation);
        ROS_INFO_STREAM("Elevation: " << elevation);
      }

      if (spherical_coordinates->HasElement("headingDeg"))
      {
        auto heading = spherical_coordinates->Get<double>("headingDeg");
        // We rotate the heading by 180 degrees because the gazebo returns (-E)(-N)U, instead of ENU
        // This is a bug in gazebo. See: https://github.com/ros-simulation/gazebo_ros_pkgs/pull/982
        world->SphericalCoords()->SetHeadingOffset(M_PI - heading * M_PI / 180.0);
        ROS_INFO_STREAM("Heading: " << heading);
      }
    }
  }
  return true;
}
}  // namespace gazebo
