#ifndef SMPL_MARKER_CONVERSIONS_H
#define SMPL_MARKER_CONVERSIONS_H

// standard includes
#include <vector>

// system includes
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// project includes
#include <smpl/debug/marker.h>

namespace smpl {
namespace visual {

void ConvertMarkerMsgToMarker(
    const visualization_msgs::Marker& mm,
    Marker& m);

void ConvertMarkerToMarkerMsg(
    const Marker& m,
    visualization_msgs::Marker& mm);

auto ConvertMarkersToMarkerArray(const std::vector<Marker>& markers)
    -> visualization_msgs::MarkerArray;

auto ConvertMarkerArrayToMarkers(const visualization_msgs::MarkerArray& ma)
    -> std::vector<smpl::visual::Marker>;

} // namespace visual
} // namespace smpl

#endif
