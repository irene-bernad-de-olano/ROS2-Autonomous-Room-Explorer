#!/usr/bin/env python3
"""Classifies rooms as Kitchen / Bedroom / Bathroom / Empty.

The arena shows a printed picture inside each occupied room. Since we run a
stock COCO-trained YOLO model (no custom training required by the
challenge), we don't recognize the picture itself - we look for objects
typically visible in a photo of that room type. This matches the object
list already used for the human-readable report in room_explorer_node.py.
"""

# Objects (COCO class names, lowercase) associated with each room type.
ROOM_TYPE_OBJECTS = {
    "Bathroom": {"toilet"},
    "Bedroom": {"bed", "potted plant"},
    "Kitchen": {"oven", "microwave", "refrigerator"},
}


def get_evidence(observations, min_confidence):
    """Sum confidences of relevant-object detections, per room, per type.

    observations: {room_id: [{"object": str, "confidence": float, ...}, ...]}
    returns: {room_id: {"Kitchen": score, "Bedroom": score, "Bathroom": score}}
    """

    evidence = {}

    for room_id, detections in observations.items():

        scores = {
            room_type: 0.0
            for room_type in ROOM_TYPE_OBJECTS
        }

        for detection in detections:

            confidence = float(detection.get("confidence", 0.0))

            if confidence < min_confidence:
                continue

            object_name = str(detection.get("object", "")).lower()

            for room_type, objects in ROOM_TYPE_OBJECTS.items():

                if object_name in objects:
                    scores[room_type] += confidence

        evidence[room_id] = scores

    return evidence


def classify_rooms(observations, min_confidence):
    """Assign each room exactly one label: Kitchen, Bedroom, Bathroom or Empty.

    Each room type can be assigned to at most one room, matching the arena
    rule that the three occupied rooms each show a different type and one
    room is always empty. Assignment is greedy by descending evidence score.
    """

    evidence = get_evidence(observations, min_confidence)

    candidates = []

    for room_id, scores in evidence.items():
        for room_type, score in scores.items():
            if score > 0.0:
                candidates.append((score, room_id, room_type))

    # Highest-evidence (room, type) pairs win first.
    candidates.sort(key=lambda item: item[0], reverse=True)

    assignments = {
        room_id: "Empty"
        for room_id in observations
    }

    used_types = set()
    used_rooms = set()

    for score, room_id, room_type in candidates:

        if room_id in used_rooms or room_type in used_types:
            continue

        assignments[room_id] = room_type

        used_rooms.add(room_id)
        used_types.add(room_type)

    return assignments, evidence
