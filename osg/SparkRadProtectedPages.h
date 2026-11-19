#ifndef SPARK_RAD_PROTECTED_PAGES_H
#define SPARK_RAD_PROTECTED_PAGES_H

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

// Owner-thread cache for the pages referenced by the active and displayed
// point lists. Sorting only permutes a list and does not change its pages.
//
// Caller contract: activeVersion and displayVersion identify the membership
// of their respective lists. The old complete display can outlive an active
// transition while its replacement is being sorted. An accepted replacement
// changes displayVersion, whereas camera-only sorts do not. Callers using the
// default displayVersion must advance activeVersion for either membership.
// Equal nonzero active/display versions identify the same membership, allowing
// a sorted display to reuse the active page set without scanning its indices.
// residencyRevision must change whenever any slot-to-chunk mapping changes,
// including page reuse.
// The lists and mapping must not be mutated concurrently with update().
// A producer may acknowledge a single unreferenced chunk change through
// noteChunkMappingChange() to retain the cached membership across that revision.
// Optional certifiedActivePages must contain exactly the pages referenced by
// active (duplicates are allowed), and certifies that every active index maps
// to a valid page. The caller must establish the same active membership and
// residencyRevision as this update; a stale summary must not be supplied.
// Bounds validation here rejects invalid page IDs but cannot detect an omitted
// page, an unresolved active index or a summary from an older residency state.
class SparkRadProtectedPages {
public:
    const std::vector<int>& update(const std::vector<int>& active,
                                  const std::vector<int>& display,
                                  const std::vector<int>& slotToChunk,
                                  std::size_t numChunks,
                                  unsigned int activeVersion,
                                  std::uint64_t residencyRevision,
                                  unsigned int displayVersion = 0,
                                  const std::vector<int>* certifiedActivePages = nullptr)
    {
        if (_valid && _activeVersion == activeVersion &&
            _displayVersion == displayVersion &&
            _residencyRevision == residencyRevision && _numChunks == numChunks)
            return _pages;

        // A mapping change not acknowledged below can affect either list.
        // Membership changes alone leave the other list's page set reusable.
        if (!_valid || _residencyRevision != residencyRevision || _numChunks != numChunks) {
            _activePagesValid = false;
            _displayPagesValid = false;
        }
        auto mark = [&](const std::vector<int>& indices,
                        std::vector<unsigned char>& seen, bool& unresolved) {
            seen.assign(numChunks, 0);
            unresolved = false;
            ++_membershipScanCount;
            for (int index : indices) {
                if (index < 0 || std::size_t(index) >= slotToChunk.size()) {
                    unresolved = true;
                    continue;
                }
                const int chunk = slotToChunk[index];
                if (chunk >= 0 && std::size_t(chunk) < numChunks)
                    seen[chunk] = 1;
                else
                    unresolved = true;
            }
        };
        if (!_activePagesValid || _activeVersion != activeVersion) {
            bool useSummary = certifiedActivePages != nullptr;
            if (useSummary) {
                for (int chunk : *certifiedActivePages) {
                    if (chunk < 0 || std::size_t(chunk) >= numChunks) {
                        useSummary = false;
                        break;
                    }
                }
            }
            if (useSummary) {
                _activeSeen.assign(numChunks, 0);
                for (int chunk : *certifiedActivePages)
                    _activeSeen[chunk] = 1;
                _activeUnresolved = false;
                ++_summaryUseCount;
            } else {
                mark(active, _activeSeen, _activeUnresolved);
            }
        }
        const bool implicitDisplayVersion = displayVersion == 0;
        const unsigned int displayMembershipVersion = implicitDisplayVersion ? activeVersion : displayVersion;
        if (!implicitDisplayVersion && displayVersion == activeVersion) {
            // Sorting permutes the active list. Its pages (including unresolved
            // references) are identical, even if the old display was different.
            _displaySeen = _activeSeen;
            _displayUnresolved = _activeUnresolved;
        } else if (!_displayPagesValid || _displayMembershipVersion != displayMembershipVersion ||
                   (_displayVersion == 0) != implicitDisplayVersion) {
            mark(display, _displaySeen, _displayUnresolved);
        }
        _activePagesValid = _displayPagesValid = true;
        _displayMembershipVersion = displayMembershipVersion;
        _hasUnresolvedIndices = _activeUnresolved || _displayUnresolved;
        _seen.resize(numChunks);
        _pages.clear();
        for (std::size_t chunk = 0; chunk < numChunks; ++chunk) {
            _seen[chunk] = _activeSeen[chunk] | _displaySeen[chunk];
            if (_seen[chunk])
                _pages.push_back(static_cast<int>(chunk));
        }

        _activeVersion = activeVersion;
        _displayVersion = displayVersion;
        _residencyRevision = residencyRevision;
        _numChunks = numChunks;
        _valid = true;
        ++_rebuildCount;
        return _pages;
    }

    // Owner-thread notification after one mapping mutation increments its
    // revision. Every changed slot must belong to changedChunk or be unmapped
    // both before and after this mutation; overwriting another chunk requires
    // its own invalidation/notification. In particular, page reuse must notify
    // the old chunk's eviction and the new chunk's application separately.
    //
    // Unknown, skipped or repeated revisions fail closed. Unresolved indices
    // also require a rescan because applying/growing a page could make a
    // formerly invalid active/display reference valid.
    bool noteChunkMappingChange(int changedChunk, std::uint64_t newResidencyRevision)
    {
        if (!_valid || _hasUnresolvedIndices ||
            _residencyRevision == std::numeric_limits<std::uint64_t>::max() ||
            newResidencyRevision != _residencyRevision + 1 ||
            changedChunk < 0 || std::size_t(changedChunk) >= _seen.size() ||
            _seen[changedChunk]) {
            _valid = false;
            return false;
        }
        _residencyRevision = newResidencyRevision;
        return true;
    }

    // Call when loading/resetting a dataset, even if its versions start again
    // at the same values as the previous dataset. Keep capacity for reuse.
    void clear()
    {
        _valid = false;
        _hasUnresolvedIndices = false;
        _pages.clear();
    }

    std::uint64_t rebuildCount() const { return _rebuildCount; }
    std::uint64_t membershipScanCount() const { return _membershipScanCount; }
    std::uint64_t summaryUseCount() const { return _summaryUseCount; }

private:
    bool _valid = false;
    bool _activePagesValid = false;
    bool _displayPagesValid = false;
    bool _activeUnresolved = false;
    bool _displayUnresolved = false;
    bool _hasUnresolvedIndices = false;
    unsigned int _activeVersion = 0;
    unsigned int _displayVersion = 0;
    unsigned int _displayMembershipVersion = 0;
    std::uint64_t _residencyRevision = 0;
    std::size_t _numChunks = 0;
    std::uint64_t _rebuildCount = 0;
    std::uint64_t _membershipScanCount = 0;
    std::uint64_t _summaryUseCount = 0;
    std::vector<unsigned char> _activeSeen;
    std::vector<unsigned char> _displaySeen;
    std::vector<unsigned char> _seen;
    std::vector<int> _pages;
};

#endif
