// Parallel VSA: results must not depend on the thread count, regions must stay connected, and the
// parallel bucket flood must be as good as the exact sequential one.
#include "core/MeshPrep.h"
#include "core/Operations.h"
#include "core/Smoothing.h"
#include "param/ThreadPool.h"
#include "TestUtil.h"

#include <cstdio>

// Number of connected components of every region (through shared edges).
static std::vector<int> regionComponents(const TriMesh& m, const std::vector<int>& label, int k)
{
    std::vector<int> comps(k, 0);
    std::vector<char> seen(m.n_faces(), 0);
    for (size_t f0 = 0; f0 < m.n_faces(); ++f0) {
        if (seen[f0] || label[f0] < 0) continue;
        ++comps[label[f0]];
        std::vector<int> stack{int(f0)};
        seen[f0] = 1;
        while (!stack.empty()) {
            const int f = stack.back();
            stack.pop_back();
            for (auto nb : m.ff_range(TriMesh::FaceHandle(f)))
                if (!seen[nb.idx()] && label[nb.idx()] == label[f0]) { seen[nb.idx()] = 1; stack.push_back(nb.idx()); }
        }
    }
    return comps;
}

static VSAState run(const VSAInput& in, int k, ThreadPool* pool, size_t parallelMin, int iterations)
{
    VSAState s;
    s.metric = Metric::L21;
    s.smooth.lambda = 0.1;
    s.pool = pool;
    s.parallelMinFaces = parallelMin;
    initFarthestSeeds(in, s, k, 1);
    for (int i = 0; i < iterations; ++i) lloydStep(in, s);
    return s;
}

int main(int argc, char** argv)
{
    TriMesh m;
    CHECK(loadMesh(argc > 1 ? argv[1] : "data/bunny.ply", m));
    normalizeToUnitBox(m);
    const FaceData fd = computeFaceData(m);
    const std::vector<Moments> fm = computeFaceMoments(m, fd);
    const VSAInput in{m, fd, fm};
    const int k = 30, iterations = 6;

    ThreadPool one(1), three(3), many(8);
    const VSAState a = run(in, k, &one, 0, iterations);
    const VSAState b = run(in, k, &three, 0, iterations);
    const VSAState c = run(in, k, &many, 0, iterations);
    CHECK(a.label == b.label);
    CHECK(a.label == c.label);
    CHECK(a.energyHistory == b.energyHistory);
    CHECK(a.energyHistory == c.energyHistory);

    // every face labeled, every region one connected piece
    int unlabeled = 0;
    for (int l : a.label) unlabeled += l < 0;
    CHECK(unlabeled == 0);
    int split = 0;
    for (int n : regionComponents(m, a.label, a.k())) split += n != 1;
    CHECK(split == 0);

    // quality: the bucket flood against the exact sequential priority queue
    const VSAState seq = run(in, k, &one, size_t(-1), iterations);
    const double eSeq = seq.energyHistory.back(), ePar = a.energyHistory.back();
    std::printf("energy: sequential %.5f, parallel %.5f\n", eSeq, ePar);
    CHECK(ePar < eSeq * 1.15);
    split = 0;
    for (int n : regionComponents(m, seq.label, seq.k())) split += n != 1;
    CHECK(split == 0);

    // smoothing alone: same result for any thread count, and it must not break the invariants
    VSAState x = a, y = a;
    x.pool = &one;
    y.pool = &many;
    for (VSAState* s : {&x, &y}) {
        for (int& l : s->label) if (l >= 0 && (&l - &s->label[0]) % 7 == 0) l = (l + 1) % s->k();   // salt
        s->smooth.lambda = 0.3;
        smoothLabels(in, *s);
    }
    CHECK(x.label == y.label);
    CHECK(x.regions.size() == y.regions.size());

    // partition from the same labels: identical for any thread count
    VSAState p = a, q = a;
    p.pool = &one;
    q.pool = &many;
    partition(in, p);
    partition(in, q);
    CHECK(p.label == q.label);

    if (failures) { std::printf("%d failures\n", failures); return 1; }
    std::printf("parallel VSA: OK\n");
    return 0;
}
