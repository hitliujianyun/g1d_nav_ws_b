"""DDS discovery helpers (CycloneDDS builtin topics)."""
from __future__ import annotations

import time
from typing import List


def find_writers(topic: str, domain: int, wait_s: float = 1.0) -> List[str]:
    """Participant keys of remote/local writers on `topic` seen within `wait_s`."""
    from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication
    from cyclonedds.domain import DomainParticipant

    dp = DomainParticipant(domain)
    reader = BuiltinDataReader(dp, BuiltinTopicDcpsPublication)
    found = {}
    t_end = time.monotonic() + wait_s
    while time.monotonic() < t_end:
        for s in reader.take(N=256):
            if getattr(s, "topic_name", None) == topic:
                found[str(s.key)] = str(s.participant_key)
        time.sleep(0.05)
    del reader
    return sorted(found.values())
