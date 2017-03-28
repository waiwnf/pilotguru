#include "MapPointersIndex.h"
#include "MapPoint.h"
#include "KeyFrame.h"

namespace ORB_SLAM2 {

MapPointersIndex IndexReachableMapPointers(MapPointersQueue* queue) {
	CHECK_NOTNULL(queue);
	MapPointersIndex result;

	while (!queue->nodes.empty()) {
		const MapPointersNodeExpander* const node = *(queue->nodes.begin());
		queue->nodes.erase(queue->nodes.begin());
		node->ExpandNode(&result, queue);
	}
	return result;
}

IndexedNodes DeserializeNodes(const SerializedNodes& serialized) {
	IndexedNodes result;

	std::map<size_t, const SerializedMapPoint*> pointsPtrIndex;
	std::map<size_t, const SerializedKeyFrame*> framesPtrIndex;

	// Allocate the nodes.
	for (const auto& indexedPoint : serialized.points()) {
		result.points.insert(
				std::make_pair(indexedPoint.index(),
						new MapPoint(indexedPoint.point())));
		pointsPtrIndex.insert(std::make_pair(indexedPoint.index(), &indexedPoint.point()));
	}

	for (const auto& indexedFrame : serialized.frames()) {
		result.frames.insert(
				std::make_pair(indexedFrame.index(),
						new KeyFrame(indexedFrame.frame())));
		framesPtrIndex.insert(std::make_pair(indexedFrame.index(), &indexedFrame.frame()));
	}

	// Restore the inside pointers in MapPoints.
	for (auto& indexedPoint : result.points) {
		indexedPoint.second->RestorePointers(*LookupOrDie(indexedPoint.first, pointsPtrIndex), result);
	}
	for (auto& indexedFrame : result.frames) {
		indexedFrame.second->RestorePointers(*LookupOrDie(indexedFrame.first, framesPtrIndex), result);
	}

	return result;
}

}
