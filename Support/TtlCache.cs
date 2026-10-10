using System.Collections.Generic;
using UnityEngine;

namespace AutoTOT
{
    /// <summary>
    /// Small TTL cache clocked by <see cref="Time.unscaledTime"/> (real seconds, unaffected by pause and time
    /// compression), for per-frame UI paths such as flight-time ETAs and launcher facts. Inserting a new key
    /// into a full cache drops expired entries first, then the oldest quarter if it is still full.
    /// Allocation-free apart from two scratch lists.
    /// </summary>
    internal sealed class TtlCache<TKey, TValue>
    {
        private struct Entry
        {
            // When the entry was written, which its TTL runs from.
            public float StampUnscaled;
            // When the value's inputs were taken, which its reported age runs from. Equal to the stamp unless
            // the writer supplied an earlier origin, as a worker's answer does.
            public float OriginUnscaled;
            public TValue Value;
        }

        private readonly Dictionary<TKey, Entry> _map = new Dictionary<TKey, Entry>();
        private readonly List<TKey> _evictScratch = new List<TKey>();
        private readonly float _ttlSeconds;
        private readonly int _capacity;
        private long _hits;
        private long _misses;
        private long _evictionsTtl;
        private long _evictionsCapacity;

        public TtlCache(float ttlSeconds, int capacity = 512)
        {
            _ttlSeconds = ttlSeconds;
            _capacity = capacity;
        }

        /// <summary>Live (non-expired) value for the key, if any.</summary>
        public bool TryGet(TKey key, out TValue value)
        {
            if (_map.TryGetValue(key, out Entry entry) && (Time.unscaledTime - entry.StampUnscaled) < _ttlSeconds)
            {
                value = entry.Value;
                _hits++;
                return true;
            }
            value = default;
            _misses++;
            return false;
        }

        /// <summary>
        /// Live value plus its age in the TTL's real seconds, for a caller that records staleness itself: a
        /// shared entry may have been computed up to a whole TTL ago, and one written with an origin is older
        /// still by the time between its origin and its write.
        /// </summary>
        public bool TryGet(TKey key, out TValue value, out float ageSeconds)
        {
            if (_map.TryGetValue(key, out Entry entry))
            {
                float now = Time.unscaledTime;
                if (now - entry.StampUnscaled < _ttlSeconds)
                {
                    value = entry.Value;
                    ageSeconds = now - entry.OriginUnscaled;
                    _hits++;
                    return true;
                }
            }
            value = default;
            ageSeconds = 0f;
            _misses++;
            return false;
        }

        /// <summary>Insert or overwrite; a new key in a full cache evicts first.</summary>
        public void Set(TKey key, TValue value) => Set(key, value, Time.unscaledTime);

        /// <summary>
        /// Insert or overwrite a value whose inputs were taken at <paramref name="originUnscaled"/> (on this
        /// cache's clock). The TTL still runs from now; only the age <see cref="TryGet(TKey, out TValue, out float)"/>
        /// reports starts at the origin.
        /// </summary>
        public void Set(TKey key, TValue value, float originUnscaled)
        {
            float now = Time.unscaledTime;
            if (_map.Count >= _capacity && !_map.ContainsKey(key)) Evict(now);
            _map[key] = new Entry { StampUnscaled = now, OriginUnscaled = Mathf.Min(originUnscaled, now), Value = value };
        }

        /// <summary>
        /// The last value written for the key, expired or not, for a display that would rather show a slightly old
        /// number than recompute it this frame. False once the entry has been evicted. Not counted as a hit or miss.
        /// </summary>
        public bool TryGetStale(TKey key, out TValue value)
        {
            if (_map.TryGetValue(key, out Entry entry))
            {
                value = entry.Value;
                return true;
            }
            value = default;
            return false;
        }

        /// <summary>
        /// Make room: drop expired entries, then the oldest quarter if the cache is still full of live ones.
        /// Clearing everything would make every key miss at once and re-solve a large strike on the main
        /// thread in one frame.
        /// </summary>
        private void Evict(float now)
        {
            _evictScratch.Clear();
            foreach (KeyValuePair<TKey, Entry> kv in _map)
                if ((now - kv.Value.StampUnscaled) >= _ttlSeconds) _evictScratch.Add(kv.Key);
            for (int i = 0; i < _evictScratch.Count; i++) _map.Remove(_evictScratch[i]);
            _evictionsTtl += _evictScratch.Count;
            if (_map.Count < _capacity) return;

            _ageScratch.Clear();
            foreach (KeyValuePair<TKey, Entry> kv in _map)
                _ageScratch.Add(new KeyValuePair<TKey, float>(kv.Key, kv.Value.StampUnscaled));
            _ageScratch.Sort(OldestFirst);
            int drop = _map.Count - (_capacity * 3) / 4;
            for (int i = 0; i < drop; i++) _map.Remove(_ageScratch[i].Key);
            _evictionsCapacity += drop;
        }

        private static readonly System.Comparison<KeyValuePair<TKey, float>> OldestFirst =
            (a, b) => a.Value.CompareTo(b.Value);
        private readonly List<KeyValuePair<TKey, float>> _ageScratch = new List<KeyValuePair<TKey, float>>();

        public long HitCount => _hits;
        public long MissCount => _misses;
        public int Count => _map.Count;
        public long EvictionsTtl => _evictionsTtl;
        public long EvictionsCapacity => _evictionsCapacity;
        public void ResetStats() { _hits = 0; _misses = 0; _evictionsTtl = 0; _evictionsCapacity = 0; }
        public void Clear() { _map.Clear(); _hits = 0; _misses = 0; _evictionsTtl = 0; _evictionsCapacity = 0; }
    }
}
