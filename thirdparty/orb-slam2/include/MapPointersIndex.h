#ifndef INCLUDE_MAPPOINTERSINDEX_H_
#define INCLUDE_MAPPOINTERSINDEX_H_

#include <map>
#include <set>

#include <glog/logging.h>

#include "SerializedNodes.pb.h"

namespace ORB_SLAM2 {

class MapPoint;
class KeyFrame;

struct IndexedNodes {
	std::map<size_t, MapPoint*> points;
	std::map<size_t, KeyFrame*> frames;

};

template<typename K, typename V> const V& LookupOrDie(const K& key,
		const std::map<K, V>& collection) {
	const auto it = collection.find(key);
	CHECK(it != collection.end()) << "Key not found: " << key;
	return it->second;
}

template<typename K, typename V> void InsertOrDie(const K& key,
		const V& value, std::map<K, V>* collection) {
	const auto it = collection->find(key);
	CHECK(it == collection->end()) << "Key is already present: " << key;
	collection->insert(std::make_pair(key,value));
}


template<typename T> class IndexBimap {
public:
	// Add a new element to the map. Dies if the element is already present.
	// Returns the integer index of freshly inserted element.
	int AddOrDie(const T& t) {
		const size_t index = toIndex.size();
		InsertOrDie(index, t, &fromIndex);
		InsertOrDie(t, index, &toIndex);
		return index;
	}

	const T& ValueOrDie(size_t index) const {
		return LookupOrDie(index, fromIndex);
	}

	size_t IndexOrDie(const T& t) const {
		return LookupOrDie(t, toIndex);
	}

	bool Contains(const T& t) const {
		return toIndex.find(t) != toIndex.end();
	}

	const std::map<T, size_t>& GetToIndex() const {
		return toIndex;
	}
private:
	std::map<size_t, T> fromIndex;
	std::map<T, size_t> toIndex;
};

class MapPointersNodeExpander;  // Defined below.

template<typename T>
void InsertIfNotInIndex(const T* t, const IndexBimap<const T*>& index,
		std::set<const MapPointersNodeExpander*>* target) {
	CHECK_NOTNULL(target);
	if (!index.Contains(t)) {
		target->insert(t);
	}
}

struct MapPointersIndex {
	IndexBimap<const MapPoint*> points;
	IndexBimap<const KeyFrame*> frames;
};

struct MapPointersQueue {
	std::set<const MapPointersNodeExpander*> nodes;
};

class MapPointersPopulator {
public:
	virtual ~MapPointersPopulator() {
	}
	virtual void Populate(MapPointersQueue* queue) const = 0;
};

class MapPointersNodeExpander {
public:
	virtual ~MapPointersNodeExpander() {
	}
	virtual void ExpandNode(MapPointersIndex* alreadyExpanded,
			MapPointersQueue* queue) const = 0;
};

MapPointersIndex IndexReachableMapPointers(MapPointersQueue* queue);

IndexedNodes DeserializeNodes(const SerializedNodes& serialized);
}

#endif /* INCLUDE_MAPPOINTERSINDEX_H_ */
