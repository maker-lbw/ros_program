#include <gtest/gtest.h>

#include <nav_msgs/Path.h>

#include "smooth_waypoint_planner/path_processing.h"

namespace smooth_waypoint_planner
{
namespace
{

geometry_msgs::PoseStamped MakePose(double x, double y)
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;
  return pose;
}

TEST(PathProcessingTest, SelectWaypointSourcePrefersLiveWhenConfigured)
{
  EXPECT_EQ(
    SelectWaypointSource({true, true, true, true}),
    WaypointSource::kLiveTopic);
  EXPECT_EQ(
    SelectWaypointSource({true, false, false, true}),
    WaypointSource::kFile);
  EXPECT_EQ(
    SelectWaypointSource({false, true, true, true}),
    WaypointSource::kFile);
  EXPECT_EQ(
    SelectWaypointSource({false, true, true, false}),
    WaypointSource::kLiveTopic);
  EXPECT_EQ(
    SelectWaypointSource({true, false, false, false}),
    WaypointSource::kNone);
}

TEST(PathProcessingTest, SelectWaypointSourceFallsBackToFileWhenLiveTopicIsEmpty)
{
  EXPECT_EQ(
    SelectWaypointSource({true, true, false, true}),
    WaypointSource::kFile);
  EXPECT_EQ(
    SelectWaypointSource({true, true, false, false}),
    WaypointSource::kNone);
}

TEST(PathProcessingTest, ConvertPathToPointsPreservesOrder)
{
  nav_msgs::Path path;
  path.poses.push_back(MakePose(0.0, 0.0));
  path.poses.push_back(MakePose(1.0, 2.0));
  path.poses.push_back(MakePose(-0.5, 4.0));

  const std::vector<Point2D> points = ConvertPathToPoints(path);

  ASSERT_EQ(points.size(), 3U);
  EXPECT_DOUBLE_EQ(points[0].x, 0.0);
  EXPECT_DOUBLE_EQ(points[0].y, 0.0);
  EXPECT_DOUBLE_EQ(points[1].x, 1.0);
  EXPECT_DOUBLE_EQ(points[1].y, 2.0);
  EXPECT_DOUBLE_EQ(points[2].x, -0.5);
  EXPECT_DOUBLE_EQ(points[2].y, 4.0);
}

TEST(PathProcessingTest, BuildWaypointTraversalKeepsLinearOrderByDefault)
{
  const std::vector<Point2D> points{
    {0.0, 0.0},
    {1.0, 0.0},
    {2.0, 0.0},
    {3.0, 0.0},
  };

  const std::vector<Point2D> traversal = BuildWaypointTraversal(points, 2U, false);

  ASSERT_EQ(traversal.size(), 2U);
  EXPECT_DOUBLE_EQ(traversal[0].x, 2.0);
  EXPECT_DOUBLE_EQ(traversal[1].x, 3.0);
}

TEST(PathProcessingTest, BuildWaypointTraversalWrapsWhenLoopEnabled)
{
  const std::vector<Point2D> points{
    {0.0, 0.0},
    {1.0, 0.0},
    {2.0, 0.0},
    {3.0, 0.0},
  };

  const std::vector<Point2D> traversal = BuildWaypointTraversal(points, 2U, true);

  ASSERT_EQ(traversal.size(), 4U);
  EXPECT_DOUBLE_EQ(traversal[0].x, 2.0);
  EXPECT_DOUBLE_EQ(traversal[1].x, 3.0);
  EXPECT_DOUBLE_EQ(traversal[2].x, 0.0);
  EXPECT_DOUBLE_EQ(traversal[3].x, 1.0);
}

TEST(PathProcessingTest, BuildWaypointTraversalReturnsSingleGoalWhenTargetBehindAndNoLoop)
{
  const std::vector<Point2D> points{
    {0.0, 0.0},
    {1.0, 0.0},
    {2.0, 0.0},
    {3.0, 0.0},
  };

  const std::vector<Point2D> traversal = BuildWaypointTraversal(points, 3U, false);

  ASSERT_EQ(traversal.size(), 1U);
  EXPECT_DOUBLE_EQ(traversal[0].x, 3.0);
  EXPECT_DOUBLE_EQ(traversal[0].y, 0.0);
}

TEST(PathProcessingTest, PrepareWaypointsForSplineRoundsSharpCorner)
{
  const std::vector<Point2D> points{
    {0.0, 0.0},
    {1.0, 0.0},
    {1.0, 1.0},
  };

  const std::vector<Point2D> rounded = PrepareWaypointsForSpline(
    points,
    {0.01, 30.0, 0.25});

  ASSERT_EQ(rounded.size(), 4U);
  EXPECT_DOUBLE_EQ(rounded.front().x, 0.0);
  EXPECT_DOUBLE_EQ(rounded.front().y, 0.0);
  EXPECT_NEAR(rounded[1].x, 0.75, 1e-9);
  EXPECT_NEAR(rounded[1].y, 0.0, 1e-9);
  EXPECT_NEAR(rounded[2].x, 1.0, 1e-9);
  EXPECT_NEAR(rounded[2].y, 0.25, 1e-9);
  EXPECT_DOUBLE_EQ(rounded.back().x, 1.0);
  EXPECT_DOUBLE_EQ(rounded.back().y, 1.0);
}

TEST(PathProcessingTest, PrepareWaypointsForSplineKeepsStraightSegments)
{
  const std::vector<Point2D> points{
    {0.0, 0.0},
    {1.0, 0.0},
    {2.0, 0.0},
  };

  const std::vector<Point2D> processed = PrepareWaypointsForSpline(
    points,
    {0.01, 30.0, 0.25});

  ASSERT_EQ(processed.size(), 3U);
  EXPECT_DOUBLE_EQ(processed[1].x, 1.0);
  EXPECT_DOUBLE_EQ(processed[1].y, 0.0);
}

}  // namespace
}  // namespace smooth_waypoint_planner
