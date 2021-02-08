#ifndef ROSOUTPUTWRAPPER_H
#define ROSOUTPUTWRAPPER_H

#pragma once
#include "IOWrapper/Output3DWrapper.h"
#include "boost/thread.hpp"
#include "util/MinimalImage.h"

#include "FullSystem/HessianBlocks.h"
#include "util/FrameShell.h"

#include "geometry_msgs/Pose.h"
#include "pcl_ros/point_cloud.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/tfMessage.h"
#include "tf/transform_broadcaster.h"

namespace dso {

class FrameHessian;
class CalibHessian;
class FrameShell;

namespace IOWrap {

class ROSOutputWrapper : public Output3DWrapper {
   public:
    inline ROSOutputWrapper() {
        cloud_pub = nh.advertise<sensor_msgs::PointCloud>("dso_point_cloud", 1);
        calib_pub = nh.advertise<sensor_msgs::CameraInfo>("dso_camera_info", 1);
        pose_pub = nh.advertise<geometry_msgs::TransformStamped>("dso_camera_pose", 1);
        printf("OUT: Created SampleOutputWrapper\n");
    }

    virtual ~ROSOutputWrapper() { printf("OUT: Destroyed SampleOutputWrapper\n"); }

    virtual void publishGraph(
        const std::map<uint64_t, Eigen::Vector2i, std::less<uint64_t>,
                       Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i>>>&
            connectivity) override {
        printf("OUT: got graph with %d edges\n", (int)connectivity.size());

        int maxWrite = 5;

        for (const std::pair<const uint64_t, Eigen::Vector2i>& p : connectivity) {
            int idHost = p.first >> 32;
            int idTarget = p.first & ((uint64_t)0xFFFFFFFF);
            printf("OUT: Example Edge %d -> %d has %d active and %d marg residuals\n", idHost,
                   idTarget, p.second[0], p.second[1]);
            maxWrite--;
            if (maxWrite == 0) break;
        }
    }

    virtual void publishKeyframes(std::vector<FrameHessian*>& frames, bool final,
                                  CalibHessian* HCalib) override {
        if (!final) {
            return;
        }
        sensor_msgs::PointCloud cloud;
        auto first_frame = frames[0];
        cloud.header.frame_id = "map";
        cloud.header.stamp = ros::Time(first_frame->shell->timestamp);
        cloud.header.seq = first_frame->shell->incoming_id;

        float fx, fy, cx, cy;
        float fxi, fyi, cxi, cyi;
        // float colorIntensity = 1.0f;
        fx = HCalib->fxl();
        fy = HCalib->fyl();
        cx = HCalib->cxl();
        cy = HCalib->cyl();
        fxi = 1 / fx;
        fyi = 1 / fy;
        cxi = -cx / fx;
        cyi = -cy / fy;

        size_t points_num = 0;
        for (FrameHessian* f : frames) {
            auto const& m = f->shell->camToWorld.matrix3x4();
            if (f->shell->poseValid) {
                for (PointHessian* p : f->pointHessiansMarginalized) {
                    float depth = 1.0f / p->idepth;
                    auto const x = (p->u * fxi + cxi) * depth;
                    auto const y = (p->v * fyi + cyi) * depth;
                    auto const z = depth * (1 + 2 * fxi);
                    Eigen::Vector4d camPoint(x, y, z, 1.f);
                    Eigen::Vector3d worldPoint = m * camPoint;

                    geometry_msgs::Point32 h;
                    h.x = worldPoint[0];
                    h.y = worldPoint[1];
                    h.z = worldPoint[2];
                    cloud.points.push_back(h);
                    points_num++;
                }
            }
        }
        cloud_pub.publish(cloud);
        printf("points number: %lu\n", points_num);
        printf("frame timestamp: %f\n", first_frame->shell->timestamp);
        printf("frame timestamp in ros time: %f\n", ros::Time(first_frame->shell->timestamp));
        printf("cloud timestamp: %f\n", cloud.header.stamp);

        last_kf_id++;

        sensor_msgs::CameraInfo cam_info;
        cam_info.width = wG[0];
        cam_info.height = hG[0];
        cam_info.distortion_model = "plumb_bob";

        //        cam_info.D[0] = 0;
        //        cam_info.D[1] = 0;
        //        cam_info.D[2] = 0;
        //        cam_info.D[3] = 0;
        //        cam_info.D[4] = 0;
        cam_info.D = {0, 0, 0, 0, 0};

        printf("hello there");

        cam_info.K[0] = fx;
        cam_info.K[1] = 0;
        cam_info.K[2] = cx;
        cam_info.K[3] = 0;
        cam_info.K[4] = fy;
        cam_info.K[5] = cy;
        cam_info.K[6] = 0;
        cam_info.K[7] = 0;
        cam_info.K[8] = 1;

        cam_info.R[0] = 1;
        cam_info.R[1] = 0;
        cam_info.R[2] = 0;
        cam_info.R[3] = 0;
        cam_info.R[4] = 1;
        cam_info.R[5] = 0;
        cam_info.R[6] = 0;
        cam_info.R[7] = 0;
        cam_info.R[8] = 1;

        cam_info.P[0] = fx;
        cam_info.P[1] = 0;
        cam_info.P[2] = cx;
        cam_info.P[3] = 0;
        cam_info.P[4] = 0;
        cam_info.P[5] = fy;
        cam_info.P[6] = cy;
        cam_info.P[7] = 0;
        cam_info.P[8] = 0;
        cam_info.P[9] = 0;
        cam_info.P[10] = 1;
        cam_info.P[11] = 0;

        calib_pub.publish(cam_info);
    }

    virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib) override {
        printf("publish camera pose timestamp: %f\n", frame->timestamp);

        geometry_msgs::TransformStamped pose_msg;
        pose_msg.header.seq = frame->incoming_id;
        pose_msg.header.stamp = ros::Time(frame->timestamp);
        pose_msg.header.frame_id = "map";
        pose_msg.transform.translation.x = frame->camToWorld.translation().x();
        pose_msg.transform.translation.y = frame->camToWorld.translation().y();
        pose_msg.transform.translation.z = frame->camToWorld.translation().z();

        Eigen::Quaterniond quat(frame->camToWorld.rotationMatrix());
        pose_msg.transform.rotation.x = quat.x();
        pose_msg.transform.rotation.y = quat.y();
        pose_msg.transform.rotation.z = quat.z();
        pose_msg.transform.rotation.w = quat.w();
        pose_msg.child_frame_id = "dso_cam";

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(frame->camToWorld.translation().x(),
                                        frame->camToWorld.translation().y(),
                                        frame->camToWorld.translation().z()));
        transform.setRotation(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
        tf::StampedTransform stamped_transform(transform, ros::Time(frame->timestamp), "map",
                                               "dso_cam");
        broadcaster.sendTransform(stamped_transform);

        pose_pub.publish(pose_msg);
    }

    virtual void pushLiveFrame(FrameHessian* image) override {
        // can be used to get the raw image / intensity pyramid.
    }

    virtual void pushDepthImage(MinimalImageB3* image) override {
        // can be used to get the raw image with depth overlay.
    }
    virtual bool needPushDepthImage() override { return false; }

    virtual void pushDepthImageFloat(MinimalImageF* image, FrameHessian* KF) override {
        printf(
            "OUT: Predicted depth for KF %d (id %d, time %f, internal frame-ID %d). "
            "CameraToWorld:\n",
            KF->frameID, KF->shell->incoming_id, KF->shell->timestamp, KF->shell->id);
        std::cout << KF->shell->camToWorld.matrix3x4() << "\n";

        int maxWrite = 5;
        for (int y = 0; y < image->h; y++) {
            for (int x = 0; x < image->w; x++) {
                if (image->at(x, y) <= 0) continue;

                printf("OUT: Example Idepth at pixel (%d,%d): %f.\n", x, y, image->at(x, y));
                maxWrite--;
                if (maxWrite == 0) break;
            }
            if (maxWrite == 0) break;
        }
    }

    ros::NodeHandle nh;
    ros::Publisher cloud_pub;
    ros::Publisher calib_pub;
    ros::Publisher pose_pub;
    tf::TransformBroadcaster broadcaster;
};

}  // namespace IOWrap

}  // namespace dso

#endif  // ROSOUTPUTWRAPPER_H
